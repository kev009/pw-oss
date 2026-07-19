use super::{
    SinkDir, SinkPortExt, detect_underrun, follower_servo, level_correct, recover_or_hold,
    retune_period,
};
use super::{buffer_request, buffer_required, desired_delay, fill_floor, target_delay};
use crate::node::PortConfig;
use crate::oss::test_util::{drain, fill_pipe, free_space, pattern, pipe_pair};
use libspa::sys::SPA_IO_CLOCK_FLAG_XRUN_RECOVER;

// a Port on a pipe-backed device: the pipe's buffer plays the OSS ring
// (byte-exact accounting, short writes on a full ring), GETODELAY reads 0
// (the ioctl fails on a pipe), so the phase functions get the fill level
// passed explicitly where a decision needs it
fn test_port(write_fd: libc::c_int, target_delay: u32, period: u32) -> crate::node::Port<SinkDir> {
    crate::node::Port {
        config: None,
        buffers: vec![],
        io: crate::spa::IoArea::null(),
        rate_match: crate::spa::IoArea::null(),
        dsp: crate::oss::DspWriter::test_on_fd(write_fd, 8),
        dll: Default::default(),
        setup_period: period,
        bw_adapt: Default::default(),
        setup_blocksize: 1024,
        rebuild_pending: false,
        generation: 0,
        was_matching: false,
        warn_limit: crate::node::RateLimit::new(),
        pending_xrun: None,
        ext: SinkPortExt {
            target_delay,
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

// "buffer_required() and target_delay() must derive this identically": any
// grant that passes the retune gate (buffer_size >= required) must yield a
// fill target at or above the floor (no starvation) with a full write plus
// one fragment of wander of headroom above it (no short-write drops)
#[test]
fn granted_at_required_never_starves_or_drops() {
    for period in [1024u32, 4096, 16384, 65536] {
        for blocksize in [512u32, 1024, 2047, 2048, 16384, 65536] {
            for write_max in [period, period * 2, period * 4] {
                for oss_delay in [0u32, 4, 32, 1024] {
                    let desired = desired_delay(period, oss_delay);
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
    // oss.delay pushing past the ceiling: clamped to it, flagged for the log
    let (target, capped) = target_delay(65536, 4096, 1024, 4096, u32::MAX);
    assert_eq!(target, 65536 - 4096 - 1024);
    assert!(capped);
}

// The recovery sequencing behind the 0.9.7 underrun fix: on the first
// data cycle past the event, the fill re-primes to target FIRST and the
// cycle's data follows in the SAME cycle - into a ring that is already
// near-full, so both writes short-write and must stay frame-aligned
// while the tail drops as whole frames.
#[test]
fn recovery_reprimes_then_writes_into_a_near_full_ring() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_zeroes(0); // a recovering channel is already running
    port.ext.xrun_timestamp = 1_000;

    // near-full ring: room for the full re-prime (odelay reads 0 on a pipe,
    // so the refill is the whole target) but only half this cycle's buffer
    let capacity = fill_pipe(w);
    free_space(r, 4096 + 1024);

    let data = pattern(2048, 1);
    let n = recover_or_hold(&mut port, 2_000, 0, &data);

    // the hold cleared and the overfull ring dropped the tail: only the
    // frames that fit after the re-prime were consumed
    assert_eq!(port.ext.xrun_timestamp, 0);
    assert_eq!(n, 1024);
    let out = drain(r);
    assert_eq!(out.len(), capacity); // filler + re-prime zeroes + data head
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
    port.dsp.write_zeroes(0);
    port.ext.xrun_timestamp = 5_000;

    let data = pattern(2048, 2);

    // same-cycle clock: not past the event yet
    let n = recover_or_hold(&mut port, 5_000, 0, &data);
    assert_eq!(n, 2048);
    assert_eq!(port.ext.xrun_timestamp, 5_000);
    assert!(
        drain(r).is_empty(),
        "a held buffer must not reach the device"
    );

    // past the event, but the host flags its own xrun recovery: still held
    let n = recover_or_hold(&mut port, 6_000, SPA_IO_CLOCK_FLAG_XRUN_RECOVER, &data);
    assert_eq!(n, 2048);
    assert_eq!(port.ext.xrun_timestamp, 5_000);
    assert!(drain(r).is_empty());

    // past the event with no host recovery: re-primes and writes
    let n = recover_or_hold(&mut port, 6_000, 0, &data);
    assert_eq!(n, 2048);
    assert_eq!(port.ext.xrun_timestamp, 0);
    let out = drain(r);
    assert_eq!(out.len(), 4096 + 2048);
    assert!(out[..4096].iter().all(|&b| b == 0));
    assert_eq!(&out[4096..], &data[..]);
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
    port.dsp.write_zeroes(0); // the gate runs on a running channel
    port.config = Some(PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48000,
        channels: 4,
        positions: vec![],
        flags: 0,
        stride: 8,
    });
    let log = crate::spa::Log::test_null();

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
    unsafe { libc::close(r) };
}

// the in-place retune: a sustained period change with enough ring
// headroom recommits the geometry and snaps the fill to the new target
#[test]
fn retune_recommits_in_place_and_snaps_the_fill() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_zeroes(0); // a retuning channel is running
    port.ext.buffer_size = 16384;
    let log = crate::spa::Log::test_null();

    // one flip is debounced: write at the old geometry for a cycle
    assert!(!retune_period(&mut port, 4096, 8, 0, 0, &log));
    assert_eq!(port.setup_period, 2048);
    assert!(drain(r).is_empty());

    // sustained: retune in place and fill to the new target (odelay
    // reads 0 on a pipe, so the snap writes the whole target)
    assert!(!retune_period(&mut port, 4096, 8, 0, 0, &log));
    assert_eq!(port.setup_period, 4096);
    assert_eq!(port.ext.target_delay, 5120); // fill_floor(4096, 1024) binds
    assert_eq!(port.ext.period_mismatch, 0);
    let out = drain(r);
    assert_eq!(out.len(), 5120);
    assert!(out.iter().all(|&b| b == 0));
    unsafe { libc::close(r) };
}

// The in-place retune against a near-full ring: the gate math (granted >=
// target + write_max + blocksize post-snap) makes drops impossible when
// odelay reads true, but a vchan can under-read odelay and the snap then
// overfills - so the write path is the backstop and its drops must stay
// frame-aligned. Model the worst case directly: a ring with room for the
// whole snap (odelay reads 0 on a pipe, so the snap writes the full new
// target) but only half the cycle's data behind it.
#[test]
fn retune_snap_then_data_drops_whole_frames_on_a_near_full_ring() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_zeroes(0); // a retuning channel is running
    port.ext.buffer_size = 16384;
    let log = crate::spa::Log::test_null();

    // debounce cycle: no writes yet
    assert!(!retune_period(&mut port, 4096, 8, 0, 0, &log));
    assert!(drain(r).is_empty());

    let capacity = fill_pipe(w);
    free_space(r, 5120 + 1024);

    // sustained: the retune commits and the snap fills to the new target
    assert!(!retune_period(&mut port, 4096, 8, 0, 0, &log));
    assert_eq!(port.setup_period, 4096);
    assert_eq!(port.ext.target_delay, 5120);

    // the cycle's data write against the 1024 bytes left: a frame-aligned
    // short write, the tail dropped as whole frames
    let data = pattern(2048, 3);
    let n = port.dsp.write(&data);
    assert_eq!(n, 1024);
    assert_eq!(n % 8, 0);

    let out = drain(r);
    assert_eq!(out.len(), capacity); // filler + snap zeroes + data head
    let tail = &out[out.len() - (5120 + 1024)..];
    assert!(
        tail[..5120].iter().all(|&b| b == 0),
        "the snap must precede the data"
    );
    assert_eq!(&tail[5120..], &data[..1024]);
    unsafe { libc::close(r) };
}

// a ring too small for the new period wants a trigger suspend; the pipe
// refuses the ioctl (the dying-fd model), so retune asks for a rebuild
// and keeps the debounce counter armed for an immediate retry
#[test]
fn retune_requests_rebuild_when_the_suspend_is_refused() {
    let (r, w) = pipe_pair(true, true);
    let mut port = test_port(w, 4096, 2048);
    port.dsp.write_zeroes(0);
    let log = crate::spa::Log::test_null();

    assert!(!retune_period(&mut port, 4096, 8, 0, 0, &log));
    assert!(retune_period(&mut port, 4096, 8, 0, 0, &log));
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
    let cap = crate::oss::ring_byte_cap(8, 48000);
    let req = buffer_request(4096, 16384, cap, 0, 2048, 4096, 4);
    assert!(req >= buffer_required(16384, desired_delay(16384, 4), 2048, 16384));
    assert!(req >= crate::oss::MIN_RING_BYTES.min(cap));
    assert!(req <= cap);
}
