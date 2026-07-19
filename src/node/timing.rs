use super::*;

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time); both directions share
// the skeleton, with the fill measurement and error sign supplied through
// the Direction servo_* hooks. Returns (corr, delay) for the clock.
pub(super) fn timeout_servo<D: Direction>(
    state: &mut DataState<D>,
    nsec: u64,
    rate: u32,
) -> (f64, i64) {
    let mut corr: f64 = 1.0;
    let mut delay: i64 = 0;
    for port in &mut state.ports {
        let Some((stride, device_rate)) = port.stride_rate() else {
            continue;
        };
        let device_rate = device_rate.max(1);
        if !port.dsp.is_running()
            || port.setup_period == 0
            || port.rebuild_pending
            || !D::servo_ready(port)
        {
            continue;
        }

        let fill = D::servo_fill(port);
        // device frames scale to the graph rate; the resampler queue is already
        // graph-side (audioconvert reports it unscaled, like ALSA adds it)
        let resamp = port.rate_match.with_ref(|rm| rm.delay as i64).unwrap_or(0);
        delay = (fill as i64 / stride as i64) * rate as i64 / device_rate as i64 + resamp;

        if D::servo_hold(port) {
            continue; // recovering; process() is discarding buffers, hold the servo
        }

        // clamp the error so a wakeup-jitter spike can't wind up the integrator
        // against an actuator that moves slowly (ALSA clamps to max_error too)
        let err_raw = D::servo_err(port, fill);
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = err_raw.clamp(-max_err, max_err);
        corr = port.dll.update(err);
        port.bw_adapt.update(&mut port.dll, err, nsec);

        // a diverged servo must not wedge the graph clock
        if !(0.5..=2.0).contains(&corr) {
            crate::warn!(
                state.log,
                "{}: DLL diverged (corr {}); relocking",
                port.dsp.path(),
                corr
            );
            port.dll.init();
            port.bw_adapt.reset();
            corr = 1.0;
        }

        #[cfg(debug_assertions)]
        eprintln!("{}: corr = {}, err = {}", port.dsp.path(), corr, err_raw);
    }
    (corr, delay)
}

// ALSA adapts the DLL bandwidth continuously from the error variance
// (alsa-pcm.c, BW_PERIOD); we approximate with two stages: a fast lock at
// BW_MAX after (re)start, then the low steady-state bandwidth
pub(super) unsafe extern "C" fn on_timeout<D: Direction>(source: *mut spa_source) {
    // the timer source we registered in init; its data points at our State
    let root: *mut State<D> = unsafe { (*source).data.cast() };
    assert!(!root.is_null(), "(*source).data is not supposed to be null");

    // Phase 1, under a scoped borrow: drain the timer, run the servo and
    // publish the clock (every early exit arms or parks the timer itself).
    // Collect the ready notification here as a copied hook: pw
    // runs process() inline from ready() on this same thread, conjuring a
    // fresh &mut DataState, so the callback must not run under this borrow.
    // SAFETY: the registered source data points at our live State (the
    // add_source contract); the borrow ends before the notify call below.
    let notify = unsafe { with_data_mut(root, timeout_cycle) };

    let Some(hook) = notify else {
        return; // early exit; the timer was armed or parked inside
    };
    if let Some((cb, data)) = hook {
        if let Some(ready_fun) = cb.ready {
            // no State borrow is live here; sound per NodeCallbacks::hook
            let err = unsafe { ready_fun(data, D::READY_STATUS) };
            #[cfg(debug_assertions)]
            unsafe {
                with_data_mut(root, |state| crate::trace!(state.log, "ready -> {}", err));
            };
            #[cfg(not(debug_assertions))]
            let _ = err;
        }
    }

    // Phase 2: re-borrow to arm the timer for the deadline the cycle
    // computed. SAFETY: the callback returned, so no reentrant borrow is
    // live; the source stays registered while the node lives. The callback
    // may have synchronously paused the node or cleared its IO, so do not
    // undo the timer park that transition just installed.
    unsafe {
        with_data_mut(root, |state| {
            if state.started
                && !state.following
                && !state.position.is_null()
                && !state.clock.is_null()
            {
                set_timeout(state, state.next_time);
            } else {
                set_timeout(state, 0);
            }
        });
    };
}

// the on_timeout cycle body, run under one scoped &mut DataState borrow. None =
// early exit (the timer was armed/parked as needed); Some(hook) = the full
// cycle ran, the clock is published, and the caller must invoke the ready
// hook (when present) and then arm the timer for state.next_time.
#[allow(clippy::type_complexity)] // the copied C (table, data) pair
pub(super) fn timeout_cycle<D: Direction>(
    state: &mut DataState<D>,
) -> Option<Option<(spa_node_callbacks, *mut c_void)>> {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "on_timeout");

    let mut expirations = 0;
    if state
        .timer_fd
        .as_ref()
        .expect("the node timer lives until clear")
        .read(&mut expirations)
        < 0
    {
        // disarmed (Pause/Suspend) in this same wakeup; nothing to read
        return None;
    }

    // after the drain: the source is level-triggered, so bailing with the fd
    // readable would busy-spin the loop; the one-shot timer is only re-armed
    // by set_timeout below, so returning here really does park it
    // stopped between the timer firing and this callback; don't signal ready()
    // into a node being reconfigured, and don't re-arm
    if !state.started || state.following {
        return None;
    }

    if state.position.is_null() || state.clock.is_null() {
        return None; // ios cleared while the timer was armed; skip the cycle
    }

    // A failed clock read must not abort the data loop, but a bare return
    // would park the one-shot timer until the next external transition
    // (only set_timeout re-arms it): retry on a RELATIVE ~10 ms one-shot.
    // next_time deliberately does not advance - it re-anchors only from a
    // successful read (the stall resync below); an absolute re-arm computed
    // from a stale deadline would fire immediately and busy-spin the loop
    // until the synthetic deadline caught up with wall time.
    let Some(now) = crate::node::try_now_ns(&state.data_system) else {
        set_timeout_rel(state, SPA_NSEC_PER_SEC as u64 / 100);
        return None;
    };

    // resync after a long stall instead of replaying a burst of stale cycles
    // (ALSA snaps when more than a second behind)
    if now.saturating_sub(state.next_time) > SPA_NSEC_PER_SEC as u64 {
        crate::warn!(
            state.log,
            "timer stalled ({} ns behind); resyncing",
            now - state.next_time
        );
        state.next_time = now;
    }

    let nsec = state.next_time;

    D::debug_cycle(state, now, nsec);

    // position and clock were null-checked above and stay set for the cycle
    let (duration, rate) = state
        .position
        .with_ref(|p| (p.clock.target_duration, p.clock.target_rate.denom))
        .unwrap_or((0, 0));
    if duration == 0 || rate == 0 {
        // malformed position: idle-tick, and advance next_time so the deadline
        // isn't stale when the position recovers
        state.next_time = nsec + SPA_NSEC_PER_SEC as u64 / 100;
        set_timeout(state, state.next_time);
        return None;
    }

    let (corr, delay) = timeout_servo(state, nsec, rate);

    // steer the timer by the correction so the published clock genuinely follows
    // the device (ALSA warps next_time the same way); this also closes the loop
    // in passthrough setups where no resampler consumes a rate_match
    state.next_time =
        nsec + (duration as f64 * SPA_NSEC_PER_SEC as f64 / (rate as f64 * corr)) as u64;

    let next_time = state.next_time;
    state.clock.with(|c| {
        c.nsec = nsec;
        c.rate = c.target_rate;
        c.position += c.duration;
        c.duration = duration;
        c.delay = delay;
        c.rate_diff = corr;
        c.next_nsec = next_time;
    });

    // hand the copied hook out (None inside = no callbacks yet, or cleared;
    // the caller keeps the clock ticking either way)
    Some(state.callbacks.hook())
}

// Data loop only. Arm the wakeup timer from now when this node drives the
// graph (started, not following, position present); park it otherwise. A
// failed clock read must not park a node that wants to run (nothing but
// another external transition would ever re-arm it): retry on a relative
// ~10 ms one-shot without touching next_time - it re-anchors only from a
// successful read (here or on_timeout's stall resync; an absolute arm from
// a stale next_time would busy-spin) - and let on_timeout take over from
// there; nothing aborts the data loop (the sink's former copy assert!()ed).
pub(crate) fn update_timers<D: Direction>(state: &mut DataState<D>) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "update_timers");

    if !(state.started && !state.following && !state.position.is_null()) {
        set_timeout(state, 0); // park
        return;
    }
    match crate::node::try_now_ns(&state.data_system) {
        Some(now) => {
            state.next_time = now;
            #[cfg(debug_assertions)]
            crate::trace!(state.log, "next time {}", now);
            set_timeout(state, now); // immediate fire from a fresh anchor
        }
        None => set_timeout_rel(state, SPA_NSEC_PER_SEC as u64 / 100),
    }
}

pub(crate) fn set_timeout<D: Direction>(state: &DataState<D>, next_time: u64) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "set_timeout {}", next_time);

    // absolute one-shot on the loop clock; 0 disarms (parks)
    arm_timer(state, next_time, SPA_FD_TIMER_ABSTIME as i32);
}

// Relative one-shot: the clock-read failure paths' retry. They have no
// trustworthy "now" to anchor an absolute deadline on, and an absolute arm
// from a stale next_time fires immediately - a busy-spin for as long as the
// clock keeps failing. `delay_ns` must be nonzero (zero would disarm).
pub(crate) fn set_timeout_rel<D: Direction>(state: &DataState<D>, delay_ns: u64) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "set_timeout_rel {}", delay_ns);

    debug_assert!(delay_ns > 0);
    arm_timer(state, delay_ns, 0);
}

pub(super) fn arm_timer<D: Direction>(state: &DataState<D>, value_ns: u64, flags: i32) {
    let timerspec = itimerspec {
        it_value: timespec {
            tv_sec: (value_ns / SPA_NSEC_PER_SEC as u64) as i64,
            tv_nsec: (value_ns % SPA_NSEC_PER_SEC as u64) as i64,
        },
        it_interval: timespec {
            tv_sec: 0,
            tv_nsec: 0,
        },
    };

    state
        .timer_fd
        .as_ref()
        .expect("the node timer lives until clear")
        .settime(flags, &timerspec);
}

// identify our device clock (spa_io_clock.name) so consumers can tell whether
// two nodes tick from the same hardware
pub(crate) fn set_clock_name(clock: &mut libspa::sys::spa_io_clock, name: &std::ffi::CStr) {
    // at most 63 bytes plus the forced terminator fit the 64-byte name field
    let bytes = name.to_bytes_with_nul();
    for (dst, &src) in clock.name.iter_mut().take(63).zip(bytes.iter()) {
        *dst = src as _;
    }
    clock.name[63] = 0;
}

// does the driver's clock in `position` carry our clock name? (then we tick
// from the same device and rate matching is pointless - ALSA does the same
// clock-name comparison)
pub(crate) fn same_clock(position: &libspa::sys::spa_io_position, name: &std::ffi::CStr) -> bool {
    let theirs = &position.clock.name;
    let ours = name.to_bytes();
    if ours.is_empty() || ours.len() >= theirs.len() || theirs[0] == 0 {
        return false;
    }
    for (i, &b) in ours.iter().enumerate() {
        if theirs[i] as u8 != b {
            return false;
        }
    }
    theirs[ours.len()] == 0
}

// one graph cycle expressed in device bytes; the device rate can differ from
// the graph rate (the adapter's resampler makes up the difference)
pub(crate) fn device_period_bytes(
    target_duration: u64,
    device_rate: u32,
    graph_rate: u32,
    stride: u32,
) -> u32 {
    if graph_rate == 0 {
        return 0;
    }
    // saturate: a corrupt duration must not wrap (or panic in debug builds)
    (target_duration.saturating_mul(device_rate as u64) / graph_rate as u64)
        .saturating_mul(stride as u64)
        .min(u32::MAX as u64) as u32
}

// a nanosecond interval (hardware drain quantum, elapsed time) expressed in
// device bytes; saturating and clamped - the inputs are device- or
// clock-provided and an overflow here would abort the data loop
pub(crate) fn ns_to_bytes(ns: u64, rate: u32, stride: u32) -> u32 {
    ((ns as u128)
        .saturating_mul(rate as u128)
        .saturating_mul(stride as u128)
        / 1_000_000_000)
        .min(u32::MAX as u128) as u32
}

// ns_to_bytes rounded up to a whole frame (the division floors: a 2048-byte
// hardware quantum reads as 2047); a saturated conversion stays saturated at
// the largest frame multiple instead of overflowing the round-up
pub(crate) fn ns_to_frame_bytes(ns: u64, rate: u32, stride: u32) -> u32 {
    let stride = stride.max(1);
    ns_to_bytes(ns, rate, stride)
        .checked_next_multiple_of(stride)
        .unwrap_or(u32::MAX - u32::MAX % stride)
}

// at most one message a second from a per-cycle warn site, with a count of
// what went unsaid (ALSA uses spa_ratelimit for the same purpose)
pub(crate) struct RateLimit {
    last: u64,
    suppressed: u32,
}

impl Default for RateLimit {
    fn default() -> Self {
        Self::new()
    }
}

impl RateLimit {
    pub(crate) const fn new() -> Self {
        Self {
            last: 0,
            suppressed: 0,
        }
    }

    // Some(previously suppressed count) when the caller may log now
    pub(crate) fn check(&mut self, now: u64) -> Option<u32> {
        if now.saturating_sub(self.last) >= 1_000_000_000 {
            self.last = now;
            Some(std::mem::take(&mut self.suppressed))
        } else {
            self.suppressed += 1;
            None
        }
    }
}

// CLOCK_MONOTONIC through the host system vtable; None when the read fails.
// Fallible on purpose: every caller runs on the data loop under extern "C",
// where an assert would abort the whole daemon - each caller has a soft
// path (park the timer, reuse the previous stamp, skip a cycle).
pub(crate) fn try_now_ns(system: &crate::spa::System) -> Option<u64> {
    let mut now = libspa::sys::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now);
    if err < 0 {
        return None;
    }
    Some((now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64)
}

#[cfg(test)]
mod clock_tests {
    use super::*;

    #[test]
    fn frame_bytes_round_up_and_stay_saturated() {
        // the floored hardware quantum comes back up to a whole frame
        assert_eq!(ns_to_frame_bytes(5_333_333, 48000, 8), 2048);
        assert_eq!(ns_to_frame_bytes(0, 48000, 8), 0);
        // a saturated conversion must not overflow the round-up (debug builds
        // would abort the data loop; release would wrap the chunk to zero) -
        // it stays pinned at the largest frame multiple
        assert_eq!(
            ns_to_frame_bytes(u64::MAX, u32::MAX, 8),
            u32::MAX - u32::MAX % 8
        );
        assert_eq!(ns_to_frame_bytes(u64::MAX, u32::MAX, 1), u32::MAX);
    }
    #[test]
    fn ns_to_bytes_floors() {
        // the production hw-quantum shape: 256 frames @ 48k S32 stereo is
        // 5333333 ns, which floors to 2047 - call sites round back up to the
        // stride and rely on this direction; a rounding change here silently
        // shifts every geometry derived from it
        assert_eq!(ns_to_bytes(5_333_333, 48000, 8), 2047);
        assert_eq!(ns_to_bytes(5_333_334, 48000, 8), 2048);
        assert_eq!(ns_to_bytes(0, 48000, 8), 0);
        assert_eq!(ns_to_bytes(1_000_000_000, 48000, 8), 384_000);
        // saturates instead of wrapping
        assert_eq!(ns_to_bytes(u64::MAX, u32::MAX, u32::MAX), u32::MAX);
    }

    #[test]
    fn device_period_scales_with_rate_ratio() {
        assert_eq!(device_period_bytes(2048, 48000, 48000, 8), 16384);
        assert_eq!(device_period_bytes(2048, 96000, 48000, 8), 32768);
        assert_eq!(device_period_bytes(2048, 44100, 48000, 8), 15048); // floors
        assert_eq!(device_period_bytes(2048, 48000, 0, 8), 0);
        assert_eq!(
            device_period_bytes(u64::MAX, u32::MAX, 1, u32::MAX),
            u32::MAX
        );
    }
}
