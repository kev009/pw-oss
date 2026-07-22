use super::{
    RetuneOutcome, SinkDir, SinkPortExt, consume_freewheel_input, detect_underrun, follower_servo,
    level_correct, pending_write_offset, prepare_pending_write, recover_or_hold, release_input,
    resume_playback, retain_partial_write, retune_period, settle_target, write_retained_tail,
};
use super::{
    buffer_request, buffer_required, desired_delay, fill_floor, predicted_next_fill, retune_seed,
    target_delay,
};
use crate::backend::{
    self, DeviceEvent, IoStatus, PlaybackStream, WriteOutcome,
    test_transport::{drain, fill_pipe, free_space, pattern, pipe_pair},
};
use crate::spa::{IoArea, Log};

use super::super::{
    Direction, Port, PortConfig, RateLimit, device_event_fill, reset_device_event,
    take_device_event_xruns, take_fallback_xruns,
};
use libspa::sys::SPA_IO_CLOCK_FLAG_XRUN_RECOVER;
use std::ffi::c_int;

// a Port on a pipe-backed device: the pipe's buffer plays the OSS ring
// (byte-exact accounting, short writes on a full ring), GETODELAY reads 0
// (the ioctl fails on a pipe), so the phase functions get the fill level
// passed explicitly where a decision needs it
fn test_port(write_fd: c_int, target_delay: u32, period: u32) -> Port<SinkDir> {
    Port {
        config: None,
        buffers: vec![],
        io: IoArea::null(),
        rate_match: IoArea::null(),
        dsp: PlaybackStream::test_on_fd(write_fd, 8),
        dll: Default::default(),
        setup_period: period,
        bw_adapt: Default::default(),
        setup_blocksize: 1024,
        rebuild_pending: false,
        generation: 0,
        was_matching: false,
        warn_limit: RateLimit::new(),
        pending_xrun: None,
        device_event: None,
        device_eof: false,
        event_xruns_seen: 0,
        wake_threshold: 0,
        ext: SinkPortExt {
            target_delay,
            target_goal: target_delay,
            ..Default::default()
        },
    }
}

#[test]
fn target_matches_live_geometry() {
    // the production log shape: granted 65536, blocksize 2048, period 16384
    // -> target delay 20480 (fill_floor binds: period + period/4)
    assert_eq!(target_delay(65536, 16384, 2048, 16384, 0), (20480, false));
    // a fragment wider than the jitter margin takes over the floor
    assert_eq!(fill_floor(16384, 8192), 16384 + 8192);
}

#[test]
fn playback_kevent_supplies_fill_and_xrun_deltas() {
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
    port.device_event = Some(DeviceEvent {
        fd: port.dsp.fd().unwrap(),
        available_bytes: 1234,
        queued_frames: Some(512),
        xruns: 3,
        eof: false,
    });

    assert_eq!(device_event_fill(&port), Some(4096));
    assert_eq!(take_device_event_xruns(&mut port), Some(3));
    assert_eq!(take_device_event_xruns(&mut port), Some(0));
    assert_eq!(take_fallback_xruns(&mut port, 5), 2);
    assert_eq!(port.event_xruns_seen, 0);
    port.device_event.as_mut().unwrap().xruns = 1; // a new kernel counter epoch
    assert_eq!(take_device_event_xruns(&mut port), Some(1));
    port.wake_threshold = 4096;
    port.device_eof = true;
    reset_device_event(&mut port);
    assert!(port.device_event.is_none());
    assert!(!port.device_eof);
    assert_eq!(port.event_xruns_seen, 0);
    assert_eq!(port.wake_threshold, 0);
    unsafe { libc::close(r) };
}

#[test]
fn playback_kevent_wakes_at_the_live_fill_target() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 20_480, 16_384);
    port.ext.buffer_size = 65_536;
    assert_eq!(<SinkDir as Direction>::wake_threshold(&port), 45_056);

    // LOW_WATER is clamped away from zero, and otherwise follows the live
    // target directly even while an in-place retune settles it.
    port.ext.target_delay = 65_536;
    assert_eq!(<SinkDir as Direction>::wake_threshold(&port), 1);
    port.ext.target_delay = 8_192;
    assert_eq!(<SinkDir as Direction>::wake_threshold(&port), 57_344);
    unsafe { libc::close(r) };
}

// "buffer_required() and target_delay() must derive this identically": any
// grant that passes the retune gate (buffer_size >= required) must yield a
// fill target at or above the floor (no starvation) with a full write plus
// one fragment of wander of headroom above it (no short-write drops)
#[test]
fn granted_at_required_never_starves_or_drops() {
    for period in [1024u32, 4096, 16384, 65536] {
        for blocksize in [512u32, 1024, 2047, 2048, 16384, 65536] {
            for write_max in [period, period * 2, period * 4] {
                for playback_delay_eighths in [0u32, 4, 32, 1024] {
                    let desired = desired_delay(period, playback_delay_eighths);
                    let required = buffer_required(period, desired, blocksize, write_max);
                    for granted in [required, required + 1, required.saturating_mul(2)] {
                        let (target, _) =
                            target_delay(granted, period, blocksize, write_max, desired);
                        assert!(
                            target >= fill_floor(period, blocksize),
                            "starved: target {} < floor {} (granted {}, period {}, blocksize {}, write_max {}, desired {})",
                            target,
                            fill_floor(period, blocksize),
                            granted,
                            period,
                            blocksize,
                            write_max,
                            desired
                        );
                        assert!(
                            target.saturating_add(write_max).saturating_add(blocksize) <= granted,
                            "will drop: target {target} + write_max {write_max} + blocksize {blocksize} > granted {granted} (period {period}, desired {desired})"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn small_grant_is_best_effort_half() {
    // under two quanta there is no workable geometry; half the ring, and the
    // caller warns
    assert_eq!(target_delay(8192, 16384, 1024, 16384, 0), (4096, false));
}

#[test]
fn oversized_delay_is_capped_and_reported() {
    // platform::PLAYBACK_DELAY pushing past the ceiling: clamp and flag it
    let (target, capped) = target_delay(65536, 4096, 1024, 4096, u32::MAX);
    assert_eq!(target, 65536 - 4096 - 1024);
    assert!(capped);
}

// On the first data cycle past an underrun, re-prime the fill before writing
// that cycle's data. Both writes may be short against a near-full ring, so
// preserve frame alignment and leave the untouched tail with the caller.
#[test]
fn recovery_reprimes_then_writes_into_a_near_full_ring() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0); // a recovering channel is already running
    port.ext.xrun_timestamp = 1_000;

    // near-full ring: room for the full re-prime (odelay reads 0 on a pipe,
    // so the refill is the whole target) but only half this cycle's buffer
    let capacity = fill_pipe(w);
    free_space(r, 4096 + 1024);

    let data = pattern(2048, 1);
    let n = recover_or_hold(&mut port, 2_000, 0, &data);

    // The hold cleared and only the prefix that fit after the re-prime was
    // consumed; process_ports retains the returned tail in production.
    assert_eq!(port.ext.xrun_timestamp, 0);
    assert_eq!(n.bytes, 1024);
    let out = drain(r);
    assert_eq!(out.len(), capacity); // filler + re-prime silence + data head
    let tail = &out[out.len() - 5120..];
    assert!(
        tail[..4096].iter().all(|&b| b == 0),
        "the re-prime must precede the data"
    );
    assert_eq!(&tail[4096..], &data[..1024]);
    unsafe { libc::close(r) };
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

    // SKIP restored queued audio from the kernel shadow: append only real data,
    // even if GETERROR counted the silence buffer draining while paused.
    assert_eq!(
        resume_playback(&mut port, 1024, 3, &data, &Log::test_null()).bytes,
        data.len()
    );
    assert!(!port.ext.resuming);
    assert_eq!(port.ext.target_delay, 1024);
    assert_eq!(drain(r), data);

    // GETODELAY does not include the hardware buffer. With no driver xrun,
    // zero soft fill still continues directly rather than inserting silence.
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
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    let capacity = fill_pipe(w);
    let data = pattern(2048, 10);
    let log = Log::test_null();

    port.ext.resuming = true;
    let blocked = resume_playback(&mut port, 1024, 0, &data, &log);
    assert!(blocked.would_block());
    assert!(port.ext.resuming);
    assert_eq!(port.ext.target_delay, 4096);

    free_space(r, data.len());
    let written = resume_playback(&mut port, 1024, 0, &data, &log);
    assert_eq!(written.bytes, data.len());
    assert_eq!(written.status, IoStatus::Progress);
    assert!(!port.ext.resuming);
    let queued = drain(r);
    assert_eq!(queued.len(), capacity);
    assert_eq!(&queued[queued.len() - data.len()..], data);
    unsafe { libc::close(r) };
}

#[test]
fn short_resume_seeds_from_bytes_the_device_accepted() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    fill_pipe(w);
    free_space(r, 1024);
    let data = pattern(2048, 18);

    port.ext.resuming = true;
    let written = resume_playback(&mut port, 2048, 0, &data, &Log::test_null());
    assert_eq!(written.bytes, 1024);
    assert_eq!(port.ext.target_delay, 1024);
    assert!(!port.ext.resuming);
    unsafe { libc::close(r) };
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

    // Buffer ids may be reused after NEED_DATA, so a different current id
    // always starts at the beginning of its own chunk.
    assert_eq!(pending_write_offset(&mut ext, 8, 16384), 0);

    // A non-retryable device error is not mistaken for ordinary backpressure.
    assert!(!retain_partial_write(
        &mut ext,
        16384,
        WriteOutcome {
            bytes: 4096,
            status: IoStatus::Failed,
        },
    ));
    assert_eq!(ext.pending_offset, 0);
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
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    let capacity = fill_pipe(w);
    free_space(r, 2048);

    let data = pattern(4096, 21);
    let first = port.dsp.write(&data);
    assert_eq!(first.bytes, 2048);
    assert!(retain_partial_write(
        &mut port.ext,
        data.len() as u32,
        first
    ));

    // Conditions that would normally hold or correct a new graph buffer must
    // not discard or splice this accepted buffer's remaining bytes.
    port.ext.xrun_timestamp = 1;
    free_space(r, data.len());
    let tail = &data[port.ext.pending_offset as usize..];
    let written = write_retained_tail(&mut port, tail).unwrap();
    assert_eq!(written.bytes, tail.len());
    assert_eq!(port.ext.xrun_timestamp, 1);

    let queued = drain(r);
    assert_eq!(queued.len(), capacity - 2048);
    assert_eq!(&queued[queued.len() - tail.len()..], tail);
    unsafe { libc::close(r) };
}

#[test]
fn releasing_a_partial_write_closes_its_open_frame() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    fill_pipe(w);
    free_space(r, 2046);

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
    free_space(r, 2);
    let mut result = 0;
    release_input(&mut port, &mut result);
    assert_eq!(port.ext.pending_offset, 0);

    let queued = drain(r);
    assert_eq!(&queued[queued.len() - 2..], &[0, 0]);
    unsafe { libc::close(r) };
}

#[test]
fn changed_retained_buffer_closes_its_open_frame() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    let data = pattern(4096, 24);

    for (old_id, new_id, new_size) in [(7, 8, data.len()), (9, 9, 1024)] {
        fill_pipe(w);
        free_space(r, 2046);
        assert_eq!(prepare_pending_write(&mut port, old_id, data.len()), 0);
        let first = port.dsp.write(&data);
        assert_eq!(first.bytes, 2046);
        assert!(retain_partial_write(
            &mut port.ext,
            data.len() as u32,
            first
        ));

        free_space(r, 2);
        assert_eq!(prepare_pending_write(&mut port, new_id, new_size), 0);
        assert_eq!(port.ext.pending_buffer, Some(new_id));
        assert_eq!(port.ext.pending_offset, 0);

        let queued = drain(r);
        assert_eq!(&queued[queued.len() - 2..], &[0, 0]);
    }
    unsafe { libc::close(r) };
}

#[test]
fn freewheel_closes_an_abandoned_partial_frame() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    fill_pipe(w);
    free_space(r, 2046);

    let data = pattern(4096, 23);
    let first = port.dsp.write(&data);
    assert_eq!(first.bytes, 2046);
    assert!(retain_partial_write(
        &mut port.ext,
        data.len() as u32,
        first
    ));

    free_space(r, 2);
    consume_freewheel_input(&mut port);
    assert_eq!(port.ext.pending_offset, 0);
    let queued = drain(r);
    assert_eq!(&queued[queued.len() - 2..], &[0, 0]);
    unsafe { libc::close(r) };
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

// the "genuinely low" threshold behind the underrun gate: healthy
// sawtooth fills sit above it, lateness lowers it by what drained, and
// the floor keeps a truly empty ring detectable at any lateness
#[test]
fn underrun_threshold_tracks_lateness() {
    use super::underrun_low;
    // on-time wakeup, roomy target: one period binds
    assert_eq!(underrun_low(20480, 2048, 16384, 0), 16384);
    // a healthy sawtooth fill (target minus a fragment) is NOT low
    assert!(20480 - 2048 >= underrun_low(20480, 2048, 16384, 0));
    // a fragment wider than the period caps at the sawtooth floor
    assert_eq!(underrun_low(20480, 18432, 16384, 0), 20480 - 18432);
    // lateness lowers the threshold by what drained (plus wander)...
    assert_eq!(underrun_low(20480, 2048, 16384, 8192), 20480 - 8192 - 4096);
    // ...but a truly empty ring stays detectable at any lateness
    assert_eq!(underrun_low(20480, 2048, 16384, 1 << 30), 16384 / 16);
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

    detect_underrun(&mut port, 2048, 3, 1_000_000, 500_000, &log);
    assert_eq!(port.ext.xrun_timestamp, 500_000);
    // Deposit one xrun event for process() to notify.
    assert_eq!(port.pending_xrun.take(), Some(1_000));

    // a later cycle's count must not move the armed snapshot (and must
    // not deposit a second event for the same hold)
    detect_underrun(&mut port, 2048, 5, 2_000_000, 700_000, &log);
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
    super::commit_geometry(&mut port, 65536, 2048, 1024, 2048, 4096);
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
    let log = Log::test_null();

    // one flip is debounced: write at the old geometry for a cycle
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert_eq!(port.setup_period, 2048);
    assert!(drain(r).is_empty());

    // Sustained: retune in place. A pipe cannot report GETODELAY, so the
    // transition target starts at zero while the geometry goal is retained.
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, 0, &log),
        RetuneOutcome::Retuned
    );
    assert_eq!(port.setup_period, 4096);
    assert_eq!(port.ext.target_delay, 0);
    assert_eq!(port.ext.target_goal, 5120); // fill_floor(4096, 1024) binds
    assert_eq!(port.ext.period_mismatch, 0);
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
    assert_eq!(retune_seed(12_288, fill, write_now, period), seed);
    assert_eq!(retune_seed(12_288, fill, 2048, period), 6144);
    assert_eq!(retune_seed(4096, fill, 4096, period), 4096);

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
    let log = Log::test_null();
    assert_eq!(
        retune_period(&mut port, period, 8, write_now, 0, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert_eq!(
        retune_period(&mut port, period, 8, write_now, 0, 0, &log),
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
    let log = Log::test_null();

    // debounce cycle: no writes yet
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert!(drain(r).is_empty());

    let capacity = fill_pipe(w);
    free_space(r, 1024);

    // Sustained: retune commits without consuming the remaining capacity.
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, 0, &log),
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

// a ring too small for the new period wants a trigger suspend; the pipe
// refuses the ioctl (the dying-fd model), so retune asks for a rebuild
// and keeps the debounce counter armed for an immediate retry
#[test]
fn retune_requests_rebuild_when_the_suspend_is_refused() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_silence(0);
    let log = Log::test_null();

    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, 0, &log),
        RetuneOutcome::Unchanged
    );
    assert_eq!(
        retune_period(&mut port, 4096, 8, 4096, 0, 0, &log),
        RetuneOutcome::Rebuild
    );
    assert_eq!(port.setup_period, 2048); // untouched; the rebuild replaces the device
    assert!(port.ext.period_mismatch >= 2);
    assert!(drain(r).is_empty());
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
    assert!(!super::commit_geometry(&mut port, 65536, 0, 1024, 2048, 0));
    assert_eq!(port.setup_period, 2048); // untouched
    assert_eq!(port.ext.target_delay, 4096); // untouched
    assert!(drain(r).is_empty());
    unsafe { libc::close(r) };
}

#[test]
fn request_covers_the_largest_negotiable_quantum() {
    // the prime-time request holds the stable floor so later period changes
    // retune in place; the kernel cap always wins
    let cap = backend::buffer_capacity_limit(8, 48000);
    let req = buffer_request(4096, 16384, 8, 48000, 0, 2048, 4096, 4);
    assert!(req >= buffer_required(16384, desired_delay(16384, 4), 2048, 16384));
    assert!(req >= 65_536.min(cap));
    assert!(req <= cap);
}
