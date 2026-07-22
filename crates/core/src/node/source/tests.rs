use super::{
    SourceDir as GenericSourceDir, SourcePortExt, bounded_read, follower_servo, prime_capture,
    recover_overrun, retune_period,
};
use crate::backend::{
    self, StreamWake,
    fake::{FakeBackend, FakeProperties, FakeStream},
    test_transport::{pattern, pipe_pair},
};
use crate::spa::{IoArea, Log};

use super::super::{
    NodeShared, Port, PortConfig, RateLimit, RebuildContext, RebuildWork, RebuildWorkSlot,
    queue_port_rebuild, take_polled_xruns, wake_queue_fill,
};
use std::ffi::c_int;

type SourceDir = GenericSourceDir<FakeBackend>;

fn exact_xruns(events: u32) -> backend::XrunDelta {
    backend::XrunDelta {
        events,
        quality: Some(backend::ObservationQuality::Exact),
        ..Default::default()
    }
}

// A Port on a pipe-backed device: the pipe provides byte-exact capture
// accounting but no queue/capacity report, so phase helpers receive queued
// fill explicitly, as their callers do.
fn test_port(read_fd: c_int, period: u32, read_peak: u32) -> Port<SourceDir> {
    Port {
        config: None,
        buffers: vec![],
        io: IoArea::null(),
        rate_match: IoArea::null(),
        dsp: FakeStream::test_on_fd(read_fd, 8),
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
        ext: SourcePortExt {
            read_peak,
            ..Default::default()
        },
    }
}

// the read tail on a pipe-backed device: catch-up reads only the backlog
// beyond the healthy peak, and a late cycle (nothing queued) still hands
// the graph a whole period of silence
#[test]
fn bounded_read_caps_catchup_and_pads_late_cycles() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 2048);
    let mut buf = vec![0xaau8; 8192];

    // backlog past the peak: one period plus the excess is drained, no more
    let s = pattern(4096, 4);
    assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 4096) }, 4096);
    let n = bounded_read(&mut port, 4096, &mut buf, 8);
    assert_eq!(n, 1024 + (4096 - 2048));
    assert_eq!(&buf[..n as usize], &s[..n as usize]);

    // late cycle: nothing queued, so nothing is read from the blocking fd
    // and the graph still gets a whole period of silence
    let n = bounded_read(&mut port, 0, &mut buf, 8);
    assert_eq!(n, 1024);
    assert!(buf[..1024].iter().all(|&b| b == 0));
    unsafe { libc::close(w) };
}

#[test]
fn disconnected_capture_transport_queues_a_rebuild() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1_024, 2_048);
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    let mut buffer = vec![0xaau8; 1_024];
    unsafe { libc::close(w) };

    assert_eq!(bounded_read(&mut port, 1_024, &mut buffer, 8), 1_024);
    assert!(port.rebuild_required);
    assert!(buffer.iter().all(|&byte| byte == 0));

    let shared = std::sync::Arc::new(NodeShared::<SourceDir>::new());
    let endpoint = RebuildWorkSlot::<SourceDir>::new();
    let mut deferred = None;
    assert!(queue_port_rebuild(
        &mut port,
        0,
        RebuildContext {
            path: "test://capture",
            backend_properties: FakeProperties::new(false),
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

#[test]
fn capture_wake_uses_ready_bytes_without_rounding_to_frames() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 2048);
    port.stream_wake = Some(StreamWake {
        stream: port.stream_identity(),
        timing: backend::WakeTiming::Readiness,
        ready_bytes: Some(1027),
        queue: Some(backend::QueueObservation {
            fill_bytes: 1027,
            quality: backend::ObservationQuality::Exact,
        }),
        clock: None,
        xruns: None,
        state: backend::StreamWakeState::Active,
    });
    assert_eq!(wake_queue_fill(&port), Some(1027));
    unsafe { libc::close(w) };
}

#[test]
fn bounded_read_uses_biased_u8_silence() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 16, 32);
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::U8,
        rate: 48000,
        channels: 8,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    let mut buf = [0u8; 16];
    assert_eq!(bounded_read(&mut port, 0, &mut buf, 8), 16);
    assert_eq!(buf, [0x80; 16]);
    unsafe { libc::close(w) };
}

// the in-place retune: enough ring for the new period recommits the
// fill geometry without touching the device
#[test]
fn retune_recommits_in_place() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    port.ext.primed = true;
    port.ext.ring_size = 8192;
    port.dsp.test_set_buffer_geometry(1024, 1024, 8192);
    port.dsp.test_hold_retunes(1);
    let log = Log::test_null();

    assert!(!retune_period(&mut port, 2048, &log)); // debounced
    assert_eq!(port.setup_period, 1024);
    assert!(!retune_period(&mut port, 2048, &log)); // sustained: retune
    assert_eq!(port.setup_period, 2048);
    assert_eq!(port.ext.target_fill, 2048 + 512); // period + half an arrival
    assert_eq!(port.ext.read_peak, 4096);
    assert!(port.ext.primed);
    unsafe { libc::close(w) };
}

#[test]
fn zero_period_is_never_committed() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 2048);
    port.ext.ring_size = 8192;
    port.ext.target_fill = 3072;

    assert!(!super::commit_geometry(
        &mut port,
        0,
        backend::CaptureBufferGeometry::default(),
    ));
    assert_eq!(port.setup_period, 1024);
    assert_eq!(port.ext.ring_size, 8192);
    assert_eq!(port.ext.target_fill, 3072);
    assert_eq!(port.ext.read_peak, 2048);
    unsafe { libc::close(w) };
}

// A ring the new period outgrew wants a stream suspend; the pipe-backed
// transport refuses that native operation, so retune asks for a rebuild.
#[test]
fn retune_requests_rebuild_when_the_suspend_is_refused() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    port.ext.primed = true;
    port.ext.ring_size = 1024;
    port.dsp.test_set_buffer_geometry(1024, 1024, 1024);
    port.dsp.test_hold_retunes(1);
    // A read transitions the device to running, so suspend reaches the
    // failing native operation instead of short-circuiting from setup.
    let s = pattern(8, 5);
    assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 8) }, 8);
    let mut buf = [0u8; 8];
    assert_eq!(port.dsp.read(&mut buf).bytes, 8);
    let log = Log::test_null();

    assert!(!retune_period(&mut port, 2048, &log));
    assert!(retune_period(&mut port, 2048, &log));
    assert!(port.ext.primed); // not re-primed; the rebuild replaces the device
    assert_eq!(port.setup_period, 1024);
    // armed for an immediate retry if the rebuild can't be queued (the
    // sink's refused-suspend arm keeps the counter the same way)
    assert!(port.ext.retune_pending);
    unsafe { libc::close(w) };
}

#[test]
fn retune_reprime_discards_capture_backlog_and_resets_the_epoch() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.dsp = FakeStream::new("fake://capture-reprime");
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48_000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    port.ext.primed = true;
    port.ext.ring_size = 1024;
    port.dsp.test_set_buffer_geometry(1024, 1024, 1024);
    port.dsp.test_hold_retunes(1);
    port.dsp.push_capture(&[1; 16]);
    let mut first_frame = [0u8; 8];
    assert_eq!(port.dsp.read(&mut first_frame).bytes, 8);
    assert!(port.dsp.is_running());
    assert_eq!(
        <FakeStream as backend::CaptureOperations>::queued_bytes(&port.dsp),
        8
    );

    assert_eq!(
        take_polled_xruns(&mut port, backend::XrunObservation::cumulative_events(7)).events,
        7
    );
    port.stream_wake = Some(StreamWake {
        stream: port.stream_identity(),
        timing: backend::WakeTiming::Readiness,
        ready_bytes: Some(1024),
        queue: Some(backend::QueueObservation {
            fill_bytes: 8,
            quality: backend::ObservationQuality::Exact,
        }),
        clock: None,
        xruns: Some(backend::XrunObservation::cumulative_events(7)),
        state: backend::StreamWakeState::Active,
    });
    let log = Log::test_null();

    assert!(!retune_period(&mut port, 2048, &log));
    assert!(port.ext.retune_pending);
    assert!(port.ext.primed);

    assert!(!retune_period(&mut port, 2048, &log));
    assert!(!port.dsp.is_running());
    assert_eq!(port.setup_period, 1024);
    assert!(!port.ext.retune_pending);
    assert!(!port.ext.primed);
    assert!(port.stream_wake.is_none());
    assert_eq!(
        take_polled_xruns(&mut port, backend::XrunObservation::cumulative_events(7)).events,
        7,
        "the re-prime starts a fresh native counter epoch"
    );
    let mut cycle = [0xaau8; 2048];
    assert_eq!(
        prime_capture(
            &mut port,
            2048,
            48_000,
            &FakeProperties::new(false),
            &mut cycle,
            &log,
        ),
        2048
    );
    assert!(port.dsp.is_running());
    assert!(port.ext.primed);
    assert_eq!(port.setup_period, 2048);
    assert_eq!(
        <FakeStream as backend::CaptureOperations>::queued_bytes(&port.dsp),
        0,
        "the suspend-based re-prime discarded the old capture backlog"
    );
    assert!(cycle.iter().all(|&byte| byte == 0));
    unsafe { libc::close(w) };
}

// The node honors a backend which delays recovery across observations and
// clears that streak when an observation-free cycle intervenes.
#[test]
fn overrun_recovery_waits_for_the_backend_verdict() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.ext.primed = true;
    port.ext.ring_size = 8192;
    port.dsp.test_set_buffer_geometry(1024, 1024, 8192);
    port.dsp.test_recover_overrun_after(3);
    let log = Log::test_null();

    // Two observations are counted without selecting recovery yet.
    recover_overrun(&mut port, exact_xruns(4), Some(8000), 0, &log);
    recover_overrun(&mut port, exact_xruns(4), Some(8000), 0, &log);
    assert_eq!(port.dsp.test_overrun_observations(), 2);
    assert!(port.ext.primed);
    assert_eq!(port.pending_xrun, None, "ignored ticks deposit no event");

    // An observation-free cycle clears backend recovery state.
    <FakeStream as backend::CaptureOperations>::clear_overrun_observation(&mut port.dsp);
    assert_eq!(port.dsp.test_overrun_observations(), 0);
    assert!(port.ext.primed);

    // The third consecutive observation selects recovery and re-primes.
    for _ in 0..3 {
        recover_overrun(&mut port, exact_xruns(4), Some(8000), 0, &log);
    }
    assert_eq!(port.dsp.test_overrun_observations(), 0);
    assert!(!port.ext.primed);
    // Recovery deposits the xrun event for process() to notify.
    assert_eq!(
        port.pending_xrun.take().map(|report| report.trigger_us),
        Some(0)
    );
    unsafe { libc::close(w) };
}

#[test]
fn overrun_epoch_reset_preserves_a_fatal_rebuild_latch() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.ext.primed = true;
    port.ext.ring_size = 8192;
    port.dsp.test_set_buffer_geometry(1024, 1024, 8192);
    port.dsp.test_recover_overrun_after(3);
    let log = Log::test_null();

    // Reach the final scripted observation, then model a fatal read and a
    // queued native event from the same process cycle.
    recover_overrun(&mut port, exact_xruns(4), Some(8000), 0, &log);
    recover_overrun(&mut port, exact_xruns(4), Some(8000), 0, &log);
    port.rebuild_required = true;
    port.stream_wake = Some(StreamWake {
        stream: port.stream_identity(),
        timing: backend::WakeTiming::Readiness,
        ready_bytes: Some(8000),
        queue: Some(backend::QueueObservation {
            fill_bytes: 8000,
            quality: backend::ObservationQuality::Exact,
        }),
        clock: None,
        xruns: Some(backend::XrunObservation::cumulative_events(4)),
        state: backend::StreamWakeState::Active,
    });

    recover_overrun(&mut port, exact_xruns(4), Some(8000), 0, &log);

    assert!(port.rebuild_required);
    assert!(port.stream_wake.is_none());
    unsafe { libc::close(w) };
}

// the follower servo: in-band errors feed the DLL, a level error past
// the snap threshold (period.max(blocksize/2 + period/2)) relocks
// instead of winding the integrator
#[test]
fn follower_servo_locks_in_band_and_relocks_on_snap() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.ext.target_fill = 2560;

    let corr = follower_servo(&mut port, 2560 + 1500, 0, 8);
    assert_eq!(corr, 1.0); // snapped: relock only

    // with a negotiated config the geometry latches and the DLL engages:
    // the first in-band update cold-starts the gains, the second must
    // produce a real (non-unity) correction
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S32LE,
        rate: 48000,
        channels: 2,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    super::commit_geometry(
        &mut port,
        1024,
        backend::CaptureBufferGeometry {
            capacity_bytes: 8192,
            quantum_bytes: 1024,
            target_fill_bytes: 1536,
            peak_fill_bytes: 3072,
            required_capacity_bytes: 4096,
            device_lost: false,
        },
    );
    port.ext.target_fill = 2560;
    follower_servo(&mut port, 2560 - 512, 1, 8);
    port.was_matching = true; // the caller latches this after each cycle
    let corr = follower_servo(&mut port, 2560 - 512, 2, 8);
    assert!((0.9..=1.1).contains(&corr)); // in-band: the DLL absorbs it
    assert!(corr != 1.0, "the DLL never engaged");
    unsafe { libc::close(w) };
}
