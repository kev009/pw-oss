use super::{
    SourceDir, SourcePortExt, bounded_read, fill_targets, follower_servo, recover_overrun,
    retune_period, ring_request, ring_required,
};
use crate::backend::{
    self, CaptureStream, DeviceEvent,
    test_transport::{pattern, pipe_pair},
};
use crate::spa::{IoArea, Log};

use super::super::{
    NodeShared, Port, PortConfig, RateLimit, RebuildContext, RebuildWork, RebuildWorkSlot,
    device_event_fill, queue_port_rebuild,
};
use std::ffi::c_int;

// a Port on a pipe-backed device: the pipe plays the capture ring
// (byte-exact accounting), GETISPACE fails on a pipe, so the phase
// functions get the queued fill passed explicitly (as the callers do)
fn test_port(read_fd: c_int, period: u32, read_peak: u32) -> Port<SourceDir> {
    Port {
        config: None,
        buffers: vec![],
        io: IoArea::null(),
        rate_match: IoArea::null(),
        dsp: CaptureStream::test_on_fd(read_fd, 8),
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
    assert!(port.device_eof);
    assert!(buffer.iter().all(|&byte| byte == 0));

    let shared = std::sync::Arc::new(NodeShared::<SourceDir>::new());
    let endpoint = RebuildWorkSlot::<SourceDir>::new();
    let mut deferred = None;
    assert!(queue_port_rebuild(
        &mut port,
        0,
        RebuildContext {
            path: "/dev/dsp",
            fragment_bytes: 0,
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
fn capture_kevent_uses_ready_bytes_without_rounding_to_frames() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 2048);
    port.device_event = Some(DeviceEvent {
        fd: port.dsp.fd().unwrap(),
        available_bytes: 1027,
        queued_frames: Some(128),
        xruns: 0,
        eof: false,
    });
    assert_eq!(device_event_fill(&port), Some(1027));
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

#[test]
fn fill_targets_track_arrival_granularity() {
    for period in [1024u32, 4096, 16384, 65536] {
        for blocksize in [512u32, 1024, 2047, 2048, 16384, 65536] {
            // unbounded: target sits half an arrival over one period, peak half
            // an arrival plus half a period over that
            let (target, peak) = fill_targets(period, blocksize, 0);
            assert_eq!(target, period + blocksize / 2);
            assert_eq!(peak, target + blocksize / 2 + period / 2);

            // an adequate ring keeps a catch-up band (one arrival above target)
            // and one arrival of top headroom - the read never goes dead
            let ring = ring_required(period, blocksize);
            let (target2, peak2) = fill_targets(period, blocksize, ring);
            assert_eq!(target2, target);
            assert!(
                peak2 >= target + blocksize,
                "catch-up band lost: peak {peak2} < target {target} + arrival {blocksize} (ring {ring})"
            );
            assert!(
                peak2 <= ring - blocksize,
                "undrainable: peak {peak2} past ring {ring} - arrival {blocksize}"
            );

            // a degenerate ring still pins the peak inside it
            let (_, peak3) = fill_targets(period, blocksize, period);
            assert!(peak3 <= period.saturating_sub(blocksize));
        }
    }
}

// the in-place retune: enough ring for the new period recommits the
// fill geometry without touching the device
#[test]
fn retune_recommits_in_place() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.ext.primed = true;
    port.ext.ring_size = 8192;
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

// a ring the new period outgrew wants a trigger suspend; the pipe
// refuses the ioctl (the dying-fd model), so retune asks for a rebuild
#[test]
fn retune_requests_rebuild_when_the_suspend_is_refused() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.ext.primed = true;
    port.ext.ring_size = 1024;
    // a read transitions the device to running, so suspend really issues
    // the (failing) SETTRIGGER instead of short-circuiting from setup
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
    assert!(port.ext.period_mismatch >= 2);
    unsafe { libc::close(w) };
}

// the overrun gate: ticks with a drainable ring are ignored, and only a
// ring PINNED at the ceiling across PINNED_CYCLE_LIMIT consecutive
// cycles triggers the re-prime recovery
#[test]
fn overruns_recover_only_when_the_ring_stays_pinned() {
    let (r, w) = pipe_pair(false, false);
    let mut port = test_port(r, 1024, 0);
    port.ext.primed = true;
    port.ext.ring_size = 8192; // blocksize 1024: pinned above 7168
    let log = Log::test_null();

    // two pinned cycles: counted, no recovery yet
    recover_overrun(&mut port, 4, Some(8000), 0, &log);
    recover_overrun(&mut port, 4, Some(8000), 0, &log);
    assert_eq!(port.ext.pinned_cycles, 2);
    assert!(port.ext.primed);
    assert_eq!(port.pending_xrun, None, "ignored ticks deposit no event");

    // a drainable fill resets the pin streak (kernel disposed upstream)
    recover_overrun(&mut port, 4, Some(100), 0, &log);
    assert_eq!(port.ext.pinned_cycles, 0);
    assert!(port.ext.primed);

    // three consecutive pinned cycles: recovery re-primes
    for _ in 0..3 {
        recover_overrun(&mut port, 4, Some(8000), 0, &log);
    }
    assert_eq!(port.ext.pinned_cycles, 0);
    assert!(!port.ext.primed);
    // Recovery deposits the xrun event for process() to notify.
    assert_eq!(port.pending_xrun.take(), Some(0));
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
    super::commit_geometry(&mut port, 1024, 1024);
    port.ext.target_fill = 2560;
    follower_servo(&mut port, 2560 - 512, 1, 8);
    port.was_matching = true; // the caller latches this after each cycle
    let corr = follower_servo(&mut port, 2560 - 512, 2, 8);
    assert!((0.9..=1.1).contains(&corr)); // in-band: the DLL absorbs it
    assert!(corr != 1.0, "the DLL never engaged");
    unsafe { libc::close(w) };
}

#[test]
fn ring_request_floors_and_caps() {
    // four periods of the largest negotiable quantum, floored at the byte
    // budget, and the kernel cap always wins
    assert_eq!(ring_request(1024, 16384, 8, 48_000), 16384 * 4);
    assert_eq!(ring_request(32768, 16384, 8, 48_000), 32768 * 4);
    assert!(ring_request(64, 64, 8, 48_000) >= 65_536);
    assert_eq!(
        ring_request(65536, 65536, 8, 48_000),
        backend::buffer_capacity_limit(8, 48_000)
    );
}
