use super::predicted_next_fill;
use super::{
    RetuneOutcome, SinkDir as GenericSinkDir, SinkPortExt, consume_freewheel_input,
    detect_underrun, finish_input_sequence, follower_servo, level_correct, pending_write_offset,
    prepare_pending_write, prime_playback, recover_or_hold, release_input, reset_unpreserved_pause,
    resume_playback, retain_partial_write, retune_period, settle_target, write_retained_tail,
};
use crate::backend::{
    self, IoStatus, StreamWake, WriteOutcome,
    fake::{FakeBackend, FakeProperties, FakeStream},
    test_transport::{drain, fill_pipe, free_space, pattern, pipe_pair},
};
use crate::spa::{IoArea, Log};

use super::super::{
    Direction, NodeShared, Port, PortConfig, RateLimit, RebuildContext, RebuildWork,
    RebuildWorkSlot, latch_rebuild_required, queue_port_rebuild, reset_stream_epoch,
    take_polled_xruns, take_wake_xruns, wake_queue_fill,
};
use libspa::sys::SPA_IO_CLOCK_FLAG_XRUN_RECOVER;
use std::ffi::c_int;

type SinkDir = GenericSinkDir<FakeBackend>;

fn test_port_with_stream(dsp: FakeStream, target_delay: u32, period: u32) -> Port<SinkDir> {
    Port {
        config: None,
        buffers: vec![],
        io: IoArea::null(),
        rate_match: IoArea::null(),
        dsp,
        dll: Default::default(),
        setup_period: period,
        bw_adapt: Default::default(),
        delivery_quantum_bytes: 1024,
        rebuild_pending: false,
        generation: 0,
        stream_token: backend::StreamToken::for_port(0),
        was_matching: false,
        warn_limit: RateLimit::new(),
        pending_xrun: None,
        stream_wake: None,
        rebuild_required: false,
        xrun_tracker: backend::XrunTracker::default(),
        ext: SinkPortExt {
            target_delay,
            target_goal: target_delay,
            minimum_fill: period.saturating_add((period / 4).max(1024)),
            ..Default::default()
        },
    }
}

// The fd-backed port exercises native nonblocking I/O. Tests that require an
// exact short write use the buffered fake because pipe capacity semantics vary
// across kernels.
fn test_port(write_fd: c_int, target_delay: u32, period: u32) -> Port<SinkDir> {
    test_port_with_stream(FakeStream::test_on_fd(write_fd, 8), target_delay, period)
}

fn buffered_test_port(capacity: usize, target_delay: u32, period: u32) -> Port<SinkDir> {
    let mut dsp = FakeStream::test_buffered(8);
    dsp.set_capacity(capacity);
    test_port_with_stream(dsp, target_delay, period)
}

#[test]
fn playback_wake_supplies_fill_and_xrun_deltas() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    port.stream_wake = Some(StreamWake {
        stream: port.stream_identity(),
        timing: backend::WakeTiming::Readiness,
        ready_bytes: Some(1234),
        queue: Some(backend::QueueObservation {
            fill_bytes: 4096,
            quality: backend::ObservationQuality::Exact,
        }),
        clock: None,
        xruns: Some(backend::XrunObservation::cumulative_events(3)),
        state: backend::StreamWakeState::Active,
    });

    assert_eq!(wake_queue_fill(&port), Some(4096));
    assert_eq!(take_wake_xruns(&mut port).unwrap().events, 3);
    assert_eq!(take_wake_xruns(&mut port).unwrap().events, 0);
    assert_eq!(
        take_polled_xruns(&mut port, backend::XrunObservation::resetting_events(5)).events,
        2
    );
    port.stream_wake.as_mut().unwrap().xruns = Some(backend::XrunObservation::cumulative_events(1));
    assert_eq!(take_wake_xruns(&mut port).unwrap().events, 1);
    port.rebuild_required = true;
    reset_stream_epoch(&mut port);
    assert!(port.stream_wake.is_none());
    assert!(!port.rebuild_required);
    unsafe { libc::close(r) };
}

#[test]
fn playback_wake_reports_the_live_buffer_state() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 20_480, 16_384);
    port.ext.buffer_size = 65_536;
    assert_eq!(
        <SinkDir as Direction>::wake_buffer_state(&port),
        backend::WakeBufferState {
            frame_stride: 1,
            period_bytes: 16_384,
            quantum_bytes: 1_024,
            capacity_bytes: 65_536,
            target_fill_bytes: 20_480,
        }
    );

    // The backend maps this live target to native readiness units.
    port.ext.target_delay = 65_536;
    assert_eq!(
        <SinkDir as Direction>::wake_buffer_state(&port).target_fill_bytes,
        65_536
    );
    port.ext.target_delay = 8_192;
    assert_eq!(
        <SinkDir as Direction>::wake_buffer_state(&port).target_fill_bytes,
        8_192
    );
    unsafe { libc::close(r) };
}

#[test]
fn disconnected_playback_transport_queues_a_rebuild() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(w, 4_096, 2_048);
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    unsafe { libc::close(r) };

    let outcome = port.dsp.write(&[0; 1_024]);
    assert_eq!(outcome.status, IoStatus::Disconnected);
    latch_rebuild_required(&mut port, outcome.status);
    assert!(port.rebuild_required);

    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new());
    let endpoint = RebuildWorkSlot::<SinkDir>::new();
    let mut deferred = None;
    assert!(queue_port_rebuild(
        &mut port,
        0,
        RebuildContext {
            path: "test://playback",
            backend_properties: FakeProperties::new(true),
            log: &Log::test_null(),
            shared: &shared,
            endpoint: &endpoint,
            deferred: &mut deferred,
        },
    ));
    assert!(port.rebuild_pending);
    assert!(matches!(endpoint.take(), Some(RebuildWork::Rebuild(_))));
    assert!(deferred.is_none());
}

// On the first data cycle past an underrun, re-prime the fill before writing
// that cycle's data. Both writes may be short against a near-full ring, so
// preserve frame alignment and leave the untouched tail with the caller.
#[test]
fn recovery_reprimes_then_writes_into_a_near_full_ring() {
    let mut port = buffered_test_port(4096 + 1024, 4096, 2048);
    port.dsp.write_silence(0); // a recovering channel is already running
    port.ext.xrun_timestamp = 1_000;

    let data = pattern(2048, 1);
    let n = recover_or_hold(&mut port, 2_000, 0, &data);

    // The hold cleared and only the prefix that fit after the re-prime was
    // consumed; process_ports retains the returned tail in production.
    assert_eq!(port.ext.xrun_timestamp, 0);
    assert_eq!(n.bytes, 1024);
    let out = port.dsp.test_take_playback();
    assert_eq!(out.len(), 5120);
    assert!(
        out[..4096].iter().all(|&b| b == 0),
        "the re-prime must precede the data"
    );
    assert_eq!(&out[4096..], &data[..1024]);
}

// the skip-buffer hold: until the driver clock passes the event (and the
// host isn't in its own recovery window), buffers are consumed unwritten
// and the hold stays armed
#[test]
fn recovery_holds_buffers_until_the_clock_passes_the_event() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    port.ext.xrun_timestamp = 5_000;

    let data = pattern(2048, 2);

    // same-cycle clock: not past the event yet
    let n = recover_or_hold(&mut port, 5_000, 0, &data);
    assert_eq!(n.bytes, 2048);
    assert_eq!(port.ext.xrun_timestamp, 5_000);
    assert!(
        drain(r).is_empty(),
        "a held buffer must not reach the device"
    );

    // past the event, but the host flags its own xrun recovery: still held
    let n = recover_or_hold(&mut port, 6_000, SPA_IO_CLOCK_FLAG_XRUN_RECOVER, &data);
    assert_eq!(n.bytes, 2048);
    assert_eq!(port.ext.xrun_timestamp, 5_000);
    assert!(drain(r).is_empty());

    // past the event with no host recovery: re-primes and writes
    let n = recover_or_hold(&mut port, 6_000, 0, &data);
    assert_eq!(n.bytes, 2048);
    assert_eq!(port.ext.xrun_timestamp, 0);
    let out = drain(r);
    assert_eq!(out.len(), 4096 + 2048);
    assert!(out[..4096].iter().all(|&b| b == 0));
    assert_eq!(&out[4096..], &data[..]);
    unsafe { libc::close(r) };
}

#[test]
fn recovery_restores_safe_headroom_without_filling_the_latency_goal() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 1024, 2048);
    port.ext.target_goal = 8192;
    port.dsp.write_silence(0);
    port.ext.xrun_timestamp = 1_000;
    let data = pattern(2048, 9);

    assert_eq!(recover_or_hold(&mut port, 2_000, 0, &data).bytes, 2048);
    // fill_floor(2048, 1024) = 3072: enough safe headroom, not the 8192 goal.
    assert_eq!(port.ext.target_delay, 3072);
    assert_eq!(port.ext.target_goal, 8192);
    let out = drain(r);
    assert_eq!(out.len(), 3072 + data.len());
    assert!(out[..3072].iter().all(|&b| b == 0));
    assert_eq!(&out[3072..], data);
    unsafe { libc::close(r) };
}

#[test]
fn steady_recovery_restores_the_full_live_target() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    port.ext.xrun_timestamp = 1_000;
    let data = pattern(2048, 19);

    assert_eq!(recover_or_hold(&mut port, 2_000, 0, &data).bytes, 2048);
    assert_eq!(port.ext.target_delay, 4096);
    let out = drain(r);
    assert_eq!(out.len(), 4096 + data.len());
    assert!(out[..4096].iter().all(|&b| b == 0));
    assert_eq!(&out[4096..], data);
    unsafe { libc::close(r) };
}

#[test]
fn resume_writes_the_first_buffer_without_an_xrun_hold() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    port.ext.resuming = true;
    let data = pattern(2048, 6);

    // The backend restored its preserved queue: append only real data, even if
    // its error counter observed synthetic audio draining while paused.
    assert_eq!(
        resume_playback(&mut port, 1024, 3, &data, &Log::test_null()).bytes,
        data.len()
    );
    assert!(!port.ext.resuming);
    assert_eq!(port.ext.target_delay, 1024);
    assert_eq!(drain(r), data);

    // A queue observation need not include backend-internal staging. With no
    // driver xrun, zero observed fill still continues without inserting silence.
    port.ext.resuming = true;
    assert_eq!(
        resume_playback(&mut port, 0, 0, &data, &Log::test_null()).bytes,
        data.len()
    );
    assert_eq!(port.ext.target_delay, 0);
    assert_eq!(drain(r), data);
    unsafe { libc::close(r) };
}

#[test]
fn full_resume_write_remains_retryable() {
    let mut port = buffered_test_port(0, 4096, 2048);
    port.dsp.write_silence(0);
    let data = pattern(2048, 10);
    let log = Log::test_null();

    port.ext.resuming = true;
    let blocked = resume_playback(&mut port, 1024, 0, &data, &log);
    assert!(blocked.would_block());
    assert!(port.ext.resuming);
    assert_eq!(port.ext.target_delay, 4096);

    port.dsp.set_capacity(data.len());
    let written = resume_playback(&mut port, 1024, 0, &data, &log);
    assert_eq!(written.bytes, data.len());
    assert_eq!(written.status, IoStatus::Progress);
    assert!(!port.ext.resuming);
    assert_eq!(port.dsp.test_take_playback(), data);
}

#[test]
fn short_resume_seeds_from_bytes_the_device_accepted() {
    let mut port = buffered_test_port(1024, 4096, 2048);
    port.dsp.write_silence(0);
    let data = pattern(2048, 18);

    port.ext.resuming = true;
    let written = resume_playback(&mut port, 2048, 0, &data, &Log::test_null());
    assert_eq!(written.bytes, 1024);
    assert_eq!(port.ext.target_delay, 1024);
    assert!(!port.ext.resuming);
    assert_eq!(port.dsp.test_take_playback(), data[..1024]);
}

#[test]
fn partial_writes_advance_only_within_the_same_host_buffer() {
    let mut ext = SinkPortExt::default();

    assert_eq!(pending_write_offset(&mut ext, 7, 16384), 0);
    assert!(retain_partial_write(
        &mut ext,
        16384,
        WriteOutcome {
            bytes: 8192,
            status: IoStatus::Progress,
        },
    ));
    assert_eq!(pending_write_offset(&mut ext, 7, 16384), 8192);

    assert!(retain_partial_write(
        &mut ext,
        8192,
        WriteOutcome {
            bytes: 4096,
            status: IoStatus::WouldBlock,
        },
    ));
    assert_eq!(pending_write_offset(&mut ext, 7, 16384), 12288);

    assert!(retain_partial_write(
        &mut ext,
        4096,
        WriteOutcome {
            bytes: 2048,
            status: IoStatus::WouldBlock,
        },
    ));
    assert_eq!(pending_write_offset(&mut ext, 7, 16384), 14336);

    // Buffer ids may be reused after NEED_DATA, so a different current id
    // always starts at the beginning of its own chunk.
    assert_eq!(pending_write_offset(&mut ext, 8, 16384), 0);

    // A non-retryable device error is not mistaken for ordinary backpressure.
    assert!(!retain_partial_write(
        &mut ext,
        16384,
        WriteOutcome {
            bytes: 4096,
            status: IoStatus::Fatal(backend::StreamError::from_native_code(libc::EIO)),
        },
    ));
    assert_eq!(ext.pending_offset, 0);
}

#[test]
fn retryable_zero_write_retains_the_entire_host_buffer() {
    let mut ext = SinkPortExt::default();
    assert_eq!(pending_write_offset(&mut ext, 7, 16384), 0);
    assert!(retain_partial_write(
        &mut ext,
        16384,
        WriteOutcome {
            bytes: 0,
            status: IoStatus::WouldBlock,
        },
    ));
    assert_eq!(pending_write_offset(&mut ext, 7, 16384), 0);
    assert_eq!(ext.pending_buffer, Some(7));
}

#[test]
fn sequence_reset_preserves_a_fatal_rebuild_latch() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.rebuild_required = true;
    port.stream_wake = Some(StreamWake {
        stream: port.stream_identity(),
        timing: backend::WakeTiming::Readiness,
        ready_bytes: Some(0),
        queue: Some(backend::QueueObservation {
            fill_bytes: 0,
            quality: backend::ObservationQuality::Exact,
        }),
        clock: None,
        xruns: Some(backend::XrunObservation::cumulative_events(1)),
        state: backend::StreamWakeState::Active,
    });
    port.ext.pending_buffer = Some(7);
    port.ext.pending_offset = 3;

    finish_input_sequence(&mut port, true);

    assert!(port.rebuild_required);
    assert!(port.stream_wake.is_none());
    assert_eq!(port.ext.pending_buffer, None);
    assert_eq!(port.ext.pending_offset, 0);
    unsafe {
        libc::close(r);
    }
}

#[test]
fn pause_reset_retries_a_partially_accepted_host_buffer_from_byte_zero() {
    let mut port = buffered_test_port(5, 0, 8);
    port.dsp.write_silence(0);
    let data = pattern(8, 23);

    assert_eq!(pending_write_offset(&mut port.ext, 7, data.len()), 0);
    let first = port.dsp.write(&data);
    assert_eq!(first.bytes, 5);
    assert!(retain_partial_write(
        &mut port.ext,
        data.len() as u32,
        first
    ));
    assert_eq!(port.ext.pending_offset, 5);

    assert!(reset_unpreserved_pause(&mut port));
    assert_eq!(port.ext.pending_buffer, None);
    assert_eq!(port.ext.pending_offset, 0);
    assert!(port.dsp.test_take_playback().is_empty());

    port.dsp.set_capacity(data.len());
    let retry = pending_write_offset(&mut port.ext, 7, data.len());
    assert_eq!(retry, 0);
    assert_eq!(port.dsp.write(&data[retry..]).bytes, data.len());
    assert_eq!(port.dsp.test_take_playback(), data);
}

#[test]
fn zero_byte_write_is_retryable_backpressure() {
    let write = WriteOutcome {
        bytes: 0,
        status: IoStatus::WouldBlock,
    };
    assert!(write.would_block());
}

#[test]
fn retained_tail_precedes_fill_correction() {
    let mut port = buffered_test_port(2048, 4096, 2048);
    port.dsp.write_silence(0);

    let data = pattern(4096, 21);
    let first = port.dsp.write(&data);
    assert_eq!(first.bytes, 2048);
    assert!(retain_partial_write(
        &mut port.ext,
        data.len() as u32,
        first
    ));
    assert_eq!(port.dsp.test_take_playback(), data[..2048]);

    // Conditions that would normally hold or correct a new graph buffer must
    // not discard or splice this accepted buffer's remaining bytes.
    port.ext.xrun_timestamp = 1;
    let tail = &data[port.ext.pending_offset as usize..];
    let written = write_retained_tail(&mut port, tail).unwrap();
    assert_eq!(written.bytes, tail.len());
    assert_eq!(port.ext.xrun_timestamp, 1);

    assert_eq!(port.dsp.test_take_playback(), tail);
}

#[test]
fn releasing_a_partial_write_closes_its_open_frame() {
    let mut port = buffered_test_port(2048, 4096, 2048);
    port.dsp.set_maximum_io(2046);
    port.dsp.write_silence(0);

    let data = pattern(4096, 22);
    let first = port.dsp.write(&data);
    assert_eq!(first.bytes, 2046);
    assert!(retain_partial_write(
        &mut port.ext,
        data.len() as u32,
        first
    ));

    // A hard-error/drop path releases this host buffer. Its six accepted
    // bytes of the final frame are completed with format silence, not bytes
    // from the next graph buffer.
    let mut result = 0;
    release_input(&mut port, &mut result);
    assert_eq!(port.ext.pending_offset, 0);

    let queued = port.dsp.test_take_playback();
    assert_eq!(&queued[queued.len() - 2..], &[0, 0]);
}

#[test]
fn changed_retained_buffer_closes_its_open_frame() {
    let mut port = buffered_test_port(2048, 4096, 2048);
    port.dsp.set_maximum_io(2046);
    port.dsp.write_silence(0);
    let data = pattern(4096, 24);

    for (old_id, new_id, new_size) in [(7, 8, data.len()), (9, 9, 1024)] {
        assert_eq!(prepare_pending_write(&mut port, old_id, data.len()), 0);
        let first = port.dsp.write(&data);
        assert_eq!(first.bytes, 2046);
        assert!(retain_partial_write(
            &mut port.ext,
            data.len() as u32,
            first
        ));

        assert_eq!(prepare_pending_write(&mut port, new_id, new_size), 0);
        assert_eq!(port.ext.pending_buffer, Some(new_id));
        assert_eq!(port.ext.pending_offset, 0);

        let queued = port.dsp.test_take_playback();
        assert_eq!(&queued[queued.len() - 2..], &[0, 0]);
    }
}

#[test]
fn freewheel_closes_an_abandoned_partial_frame() {
    let mut port = buffered_test_port(2048, 4096, 2048);
    port.dsp.set_maximum_io(2046);
    port.dsp.write_silence(0);

    let data = pattern(4096, 23);
    let first = port.dsp.write(&data);
    assert_eq!(first.bytes, 2046);
    assert!(retain_partial_write(
        &mut port.ext,
        data.len() as u32,
        first
    ));

    consume_freewheel_input(&mut port);
    assert_eq!(port.ext.pending_offset, 0);
    let queued = port.dsp.test_take_playback();
    assert_eq!(&queued[queued.len() - 2..], &[0, 0]);
}

#[test]
fn drained_resume_reprimes_and_writes_in_the_same_cycle() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    port.ext.resuming = true;
    let data = pattern(2048, 7);

    assert_eq!(
        resume_playback(&mut port, 0, 1, &data, &Log::test_null()).bytes,
        data.len()
    );
    assert!(!port.ext.resuming);
    let out = drain(r);
    assert_eq!(out.len(), 4096 + data.len());
    assert!(out[..4096].iter().all(|&b| b == 0));
    assert_eq!(&out[4096..], data);
    unsafe { libc::close(r) };
}

// the underrun gate arms the recovery hold once per event: the driver
// clock is snapshotted on the first detection and held cycles must not
// re-stamp it (odelay reads 0 on a pipe - a truly empty ring)
#[test]
fn underrun_detection_arms_the_hold_once() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0); // the gate runs on a running channel
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    let log = Log::test_null();

    detect_underrun(
        &mut port,
        2048,
        backend::XrunDelta {
            events: 3,
            quality: Some(backend::ObservationQuality::Exact),
            ..Default::default()
        },
        1_000_000,
        500_000,
        &log,
    );
    assert_eq!(port.ext.xrun_timestamp, 500_000);
    // Deposit one xrun event for process() to notify.
    assert_eq!(
        port.pending_xrun.take().map(|report| report.trigger_us),
        Some(1_000)
    );

    // a later cycle's count must not move the armed snapshot (and must
    // not deposit a second event for the same hold)
    detect_underrun(
        &mut port,
        2048,
        backend::XrunDelta {
            events: 5,
            quality: Some(backend::ObservationQuality::Exact),
            ..Default::default()
        },
        2_000_000,
        700_000,
        &log,
    );
    assert_eq!(port.ext.xrun_timestamp, 500_000);
    assert_eq!(port.pending_xrun, None);
    unsafe { libc::close(r) };
}

// the follower fill snap: a level error past one period refills to target
// on underfill and skips the cycle's buffer on overfill; in-band errors go
// to the DLL instead
#[test]
fn fill_snap_refills_underfill_and_skips_overfill() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);

    // underfill past one period: refill to target, don't skip
    let (corr, skip) = follower_servo(&mut port, 1024, 8, 0);
    assert_eq!(corr, 1.0);
    assert!(!skip);
    let out = drain(r);
    assert_eq!(out.len(), 4096 - 1024);
    assert!(out.iter().all(|&b| b == 0));

    // overfill past one period: skip the buffer, write nothing (the device
    // drains toward target meanwhile)
    // target + one period + one frame: just past the snap threshold
    let (corr, skip) = follower_servo(&mut port, 4096 + 2048 + 8, 8, 0);
    assert_eq!(corr, 1.0);
    assert!(skip);
    assert!(drain(r).is_empty());

    // in-band error: no snap, the DLL absorbs it. With a negotiated
    // config the geometry latches and the DLL engages: the first update
    // cold-starts the gains, the second produces a real correction
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    super::commit_geometry(
        &mut port,
        2048,
        backend::PlaybackBufferGeometry {
            capacity_bytes: 65536,
            quantum_bytes: 1024,
            target_fill_bytes: 4096,
            target_goal_bytes: 4096,
            minimum_fill_bytes: 3072,
            required_capacity_bytes: 0,
            delay_capped: false,
        },
    );
    port.setup_period = 2048;
    port.ext.target_delay = 4096;
    follower_servo(&mut port, 4096 + 512, 8, 1);
    port.was_matching = true; // the caller latches this after each cycle
    let (corr, skip) = follower_servo(&mut port, 4096 + 512, 8, 2);
    assert!(!skip);
    assert!((0.9..=1.1).contains(&corr));
    assert!(corr != 1.0, "the DLL never engaged");
    assert!(drain(r).is_empty());
    unsafe { libc::close(r) };
}

// the same-device follower's level correction snaps the same way, without
// a DLL to relock
#[test]
fn same_device_level_correct_snaps_the_fill() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);

    assert!(!level_correct(&mut port, 4096)); // on target: nothing to do
    assert!(drain(r).is_empty());
    assert!(level_correct(&mut port, 4096 + 2049)); // overfill: drain a cycle
    assert!(drain(r).is_empty());
    assert!(!level_correct(&mut port, 1024)); // underfill: refill to target
    let out = drain(r);
    assert_eq!(out.len(), 4096 - 1024);
    assert!(out.iter().all(|&b| b == 0));

    // A retune can leave a lower reachable live target and a larger geometry
    // goal. With no rate actuator, preserve that target instead of walking it
    // upward and inserting silence.
    port.ext.target_delay = 1024;
    port.ext.target_goal = 4096;
    assert!(!level_correct(&mut port, 1024));
    assert_eq!(port.ext.target_delay, 1024);
    assert!(drain(r).is_empty());
    unsafe { libc::close(r) };
}

// A live in-place retune recommits geometry without writing silence into the
// queued audio. The live target then approaches the geometry goal through the
// existing rate servo.
#[test]
fn retune_recommits_in_place_without_splicing_silence() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0); // a retuning channel is running
    port.ext.buffer_size = 16384;
    port.dsp.test_set_buffer_geometry(2048, 1024, 16384);
    port.dsp.test_hold_retunes(1);
    let log = Log::test_null();

    // one flip is debounced: write at the old geometry for a cycle
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert_eq!(port.setup_period, 2048);
    assert!(drain(r).is_empty());

    // Sustained: retune in place. A pipe cannot report queued bytes, so the
    // transition target starts at zero while the geometry goal is retained.
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Retuned
    );
    assert_eq!(port.setup_period, 4096);
    assert_eq!(port.ext.target_delay, 0);
    assert_eq!(port.ext.target_goal, 5120); // fill_floor(4096, 1024) binds
    assert!(!port.ext.retune_pending);
    assert!(
        drain(r).is_empty(),
        "retuning a live stream must not insert silence"
    );

    // The target advances only a bounded amount ahead of real measured fill.
    settle_target(&mut port, 4096, 8);
    assert_eq!(port.ext.target_delay, 5120);
    unsafe { libc::close(r) };
}

#[test]
fn retune_target_stays_bounded_ahead_of_real_fill() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 4096);
    port.ext.target_goal = 16384;

    settle_target(&mut port, 4096, 8);
    assert_eq!(port.ext.target_delay, 5120); // one quarter-period lead
    settle_target(&mut port, 4096, 8);
    assert_eq!(port.ext.target_delay, 5120); // no synthetic progress
    settle_target(&mut port, 4608, 8);
    assert_eq!(port.ext.target_delay, 5632); // follows accumulated real fill
    unsafe { libc::close(r) };
}

#[test]
fn short_retune_write_keeps_its_seed_for_the_current_cycle() {
    let period = 4096;
    let fill = 8192;
    let write_now = 0;
    let seed = predicted_next_fill(fill, write_now, period);
    assert_eq!(seed, 4096);

    // Settling from the pre-write fill in this cycle would move the target to
    // 9216. After the short write and one new-period drain, that is more than a
    // period above the real fill and would trigger the silence snap.
    let premature = fill + period / 4;
    assert!(premature - seed > period);

    // Production receives this explicit outcome and skips follower correction
    // until the next cycle can measure the real post-write fill.
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, fill, 2048);
    port.dsp.write_silence(0);
    port.ext.buffer_size = 16384;
    port.dsp.test_set_buffer_geometry(2048, 1024, 16384);
    port.dsp.test_hold_retunes(1);
    let log = Log::test_null();
    assert_eq!(
        retune_period(&mut port, period, 8, write_now, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert_eq!(
        retune_period(&mut port, period, 8, write_now, 0, &log),
        RetuneOutcome::Retuned
    );
    assert!(drain(r).is_empty());
    unsafe { libc::close(r) };
}

// A near-full live ring is left untouched by retune, so the current real
// buffer gets all available capacity instead of competing with a silence snap.
#[test]
fn retune_preserves_capacity_for_real_audio() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0); // a retuning channel is running
    port.ext.buffer_size = 16384;
    port.dsp.test_set_buffer_geometry(2048, 1024, 16384);
    port.dsp.test_hold_retunes(1);
    let log = Log::test_null();

    // debounce cycle: no writes yet
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert!(drain(r).is_empty());

    let capacity = fill_pipe(w);
    free_space(r, 1024);

    // Sustained: retune commits without consuming the remaining capacity.
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Retuned
    );
    assert_eq!(port.setup_period, 4096);
    assert_eq!(drain(r).len(), capacity - 1024); // retune added nothing

    // Retuning consumed no capacity, so the real audio is accepted in full.
    let data = pattern(2048, 3);
    let n = port.dsp.write(&data);
    assert_eq!(n.bytes, 2048);
    assert_eq!(n.bytes % 8, 0);
    assert_eq!(drain(r), data);
    unsafe { libc::close(r) };
}

// A ring too small for the new period wants a stream suspend; the pipe-backed
// transport refuses that native operation, so retune asks for a rebuild and
// keeps the debounce counter armed for an immediate retry.
#[test]
fn retune_requests_rebuild_when_the_suspend_is_refused() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.test_set_buffer_geometry(2048, 1024, 0);
    port.dsp.test_hold_retunes(1);
    port.dsp.write_silence(0);
    let log = Log::test_null();

    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Rebuild
    );
    assert_eq!(port.setup_period, 2048); // untouched; the rebuild replaces the device
    assert!(port.ext.retune_pending);
    assert!(drain(r).is_empty());
    unsafe { libc::close(r) };
}

// A healthy stream whose granted ring cannot hold the larger period can
// suspend in place. The old queue and its measurement epoch are discarded so
// the caller can prime the new geometry in the same process cycle.
#[test]
fn retune_reprime_discards_the_queue_and_resets_node_state() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp = FakeStream::new("fake://playback-reprime");
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    port.dsp.test_set_buffer_geometry(2048, 1024, 4096);
    port.dsp.test_hold_retunes(1);
    port.ext.buffer_size = 4096;
    assert_eq!(port.dsp.write(&[1, 2, 3, 4]).bytes, 4);
    assert!(port.dsp.is_running());
    assert_eq!(port.dsp.queued_playback_bytes(), 4);

    assert_eq!(
        take_polled_xruns(&mut port, backend::XrunObservation::cumulative_events(5)).events,
        5
    );
    port.ext.xrun_timestamp = 123_456;
    port.was_matching = true;
    port.stream_wake = Some(StreamWake {
        stream: port.stream_identity(),
        timing: backend::WakeTiming::Readiness,
        ready_bytes: Some(4096),
        queue: Some(backend::QueueObservation {
            fill_bytes: 4,
            quality: backend::ObservationQuality::Exact,
        }),
        clock: None,
        xruns: Some(backend::XrunObservation::cumulative_events(5)),
        state: backend::StreamWakeState::Active,
    });
    let log = Log::test_null();

    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert!(port.ext.retune_pending);
    assert_eq!(port.ext.xrun_timestamp, 123_456);
    assert!(port.was_matching);

    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Retuned
    );
    assert!(!port.dsp.is_running());
    assert_eq!(port.dsp.queued_playback_bytes(), 0);
    assert_eq!(port.setup_period, 2048);
    assert!(!port.ext.retune_pending);
    assert_eq!(port.ext.xrun_timestamp, 0);
    assert!(!port.was_matching);
    assert!(port.stream_wake.is_none());
    assert_eq!(
        take_polled_xruns(&mut port, backend::XrunObservation::cumulative_events(5)).events,
        5,
        "the re-prime starts a fresh native counter epoch"
    );
    prime_playback(&mut port, 4096, 48_000, &FakeProperties::new(true), &log);
    assert!(port.dsp.is_running());
    assert_eq!(port.setup_period, 4096);
    unsafe { libc::close(r) };
}

// A Start command makes the node runnable but does not trigger the OSS
// channel. The first data cycle therefore retunes while the stream is still
// in Setup, then primes it. Keep that pre-prime retune away from Running-only
// queue and xrun observations.
#[test]
fn first_start_cycle_retunes_before_prime_without_sampling_setup_state() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp = FakeStream::new("fake://first-start");
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    let log = Log::test_null();

    assert!(!port.dsp.is_running());
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert!(!port.dsp.is_running());

    prime_playback(&mut port, 4096, 48_000, &FakeProperties::new(true), &log);
    assert!(port.dsp.is_running());
    assert_eq!(port.setup_period, 4096);
    unsafe { libc::close(r) };
}

// zero-period geometry is never committed: setup_period == 0
// short-circuits retune_period, so a commit here would wedge the
// geometry until a full rebuild (the prime gate in process_ports is
// the first line of defense; this is the backstop)
#[test]
fn zero_period_is_never_committed() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    assert!(!super::commit_geometry(
        &mut port,
        0,
        backend::PlaybackBufferGeometry::default(),
    ));
    assert_eq!(port.setup_period, 2048); // untouched
    assert_eq!(port.ext.target_delay, 4096); // untouched
    assert!(drain(r).is_empty());
    unsafe { libc::close(r) };
}
