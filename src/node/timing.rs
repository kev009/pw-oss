use super::*;
use crate::backend;
use crate::spa::System;

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time); both directions share
// the skeleton, with the fill measurement and error sign supplied through
// the Direction servo_* hooks. Returns (corr, delay) for the clock.
fn driver_servo<D: Direction>(
    state: &mut DataState<D>,
    nsec: u64,
    rate: u32,
    wake: WakeKind,
) -> (f64, i64) {
    let mut corr: f64 = 1.0;
    let mut delay: i64 = 0;
    let expected_nsec = state.next_time;
    for port in &mut state.ports {
        let Some((stride, device_rate)) = port.stride_rate() else {
            continue;
        };
        let device_rate = device_rate.max(1);
        if !port.dsp.is_running()
            || port.setup_period == 0
            || port.rebuild_pending
            || port.device_eof
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

        // An enriched sound event is an IRQ-style clock observation. Its fill
        // is fragment-quantized and can be consistently one fragment below
        // the native wake threshold; steering from that biased level winds the
        // DLL forever.
        // ALSA likewise uses wake-time error in IRQ mode and buffer-level error
        // only for timer scheduling. The timer fallback still needs the latter
        // because it has no device-clock observation of its own.
        let err_raw = match wake {
            WakeKind::Device => timing_error_bytes(nsec, expected_nsec, device_rate, stride),
            WakeKind::Timer => D::servo_err(port, fill),
        };
        // Clamp the error so a wakeup-jitter spike can't wind up the integrator
        // against an actuator that moves slowly (ALSA clamps to max_error too).
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = err_raw.clamp(-max_err, max_err);
        corr = port.dll.update(err);
        match wake {
            WakeKind::Device => port.bw_adapt.update_timing(&mut port.dll, err, nsec),
            WakeKind::Timer => port.bw_adapt.update_fill(&mut port.dll, err, nsec),
        }

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
pub(super) unsafe extern "C" fn on_wake<D: Direction>(source: *mut spa_source) {
    // The timer/kqueue source registered in init; its data points at State.
    let root: *mut State<D> = unsafe { (*source).data.cast() };
    assert!(!root.is_null(), "(*source).data is not supposed to be null");

    // Phase 1, under a scoped borrow: drain the timer, run the servo and
    // publish the clock (every early exit arms or parks the timer itself).
    // Collect the ready notification here as a copied hook: pw
    // runs process() inline from ready() on this same thread, conjuring a
    // fresh &mut DataState, so the callback must not run under this borrow.
    // SAFETY: the registered source data points at our live State (the
    // add_source contract); the borrow ends before the notify call below.
    let notify = unsafe { with_data_mut(root, wake_cycle) };

    let Some(hook) = notify else {
        return; // early exit; the timer was armed or parked inside
    };
    if let Some((cb, data)) = hook
        && let Some(ready_fun) = cb.ready
    {
        unsafe {
            with_data_mut(root, |state| state.ready_dispatching = true);
        };
        // no State borrow is live here; sound per NodeCallbacks::hook
        let err = unsafe { ready_fun(data, D::READY_STATUS) };
        unsafe {
            with_data_mut(root, |state| state.ready_dispatching = false);
        };
        #[cfg(debug_assertions)]
        unsafe {
            with_data_mut(root, |state| crate::trace!(state.log, "ready -> {}", err));
        };
        #[cfg(not(debug_assertions))]
        let _ = err;
    }

    // Phase 2: re-borrow to arm the timer for the deadline the cycle
    // computed. SAFETY: the callback returned, so no reentrant borrow is
    // live; the source stays registered while the node lives. The callback
    // may have synchronously paused the node or cleared its IO, so do not
    // undo the timer park that transition just installed.
    unsafe {
        with_data_mut(root, select_next_wake);
    };
}

// The driver wake body, run under one scoped &mut DataState borrow. None =
// early exit (the timer was armed/parked as needed); Some(hook) = the full
// cycle ran, the clock is published, and the caller must invoke the ready
// hook (when present) and then select the next device or timer wake.
pub(super) fn wake_cycle<D: Direction>(
    state: &mut DataState<D>,
) -> Option<Option<(spa_node_callbacks, *mut c_void)>> {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "on_wake");

    let Some(wake) = drain_wake(state) else {
        // The loop may have observed readability just before a synchronous
        // transition removed its event, and a failed kevent read must not
        // consume the only one-shot timer arm. Restore the current
        // device/deadline selection before returning.
        select_next_wake(state);
        return None;
    };

    // A stop or role transition can land after the descriptor became ready
    // but before this callback. Do not signal ready() into a node being
    // reconfigured; explicitly remove the device knote and park its timer.
    if !state.started || state.following {
        select_next_wake(state);
        return None;
    }

    if state.position.is_null() || state.clock.is_null() {
        select_next_wake(state);
        return None; // ios cleared while the descriptor was ready
    }

    // EV_EOF is an ownership transition, not merely an I/O hint. Claim the
    // rebuild before ready(): a conforming host may defer process() until
    // after this callback, and the deadline watchdog could otherwise replace
    // the EOF snapshot with a timer cycle before process() observes it.
    super::process::queue_device_eof_rebuilds(state);

    // A failed clock read must not abort the data loop, but a bare return
    // would park the one-shot timer until the next external transition
    // (only set_timeout re-arms it): retry on a RELATIVE ~10 ms one-shot.
    // next_time deliberately does not advance. A later successful read can
    // re-anchor it through the stall path below; an absolute re-arm computed
    // from a stale deadline would fire immediately and busy-spin the loop
    // until the synthetic deadline caught up with wall time.
    let Some(now) = try_now_ns(&state.data_system) else {
        disable_device_wake(state);
        set_timeout_rel(state, SPA_NSEC_PER_SEC as u64 / 100);
        return None;
    };

    let (duration, rate) = state
        .position
        .with_ref(|p| (p.clock.target_duration, p.clock.target_rate.denom))
        .unwrap_or((0, 0));
    if wake_is_from_previous_cycle(now, state.next_time, duration, rate) {
        // A timer/device pair can straddle the nonblocking kevent read by a
        // few microseconds. The first event already advanced next_time and
        // drove this graph cycle; consume the late half without starting a
        // second cycle while followers are still processing the first.
        for port in &mut state.ports {
            port.device_event = None;
        }
        select_next_wake(state);
        return None;
    }

    // Both backends advance the phase accumulator from its prior deadline.
    // Re-anchor only after a genuine stall; replaying stale deadlines would
    // otherwise make either backend busy-spin until the timeline caught up.
    if now.saturating_sub(state.next_time) > SPA_NSEC_PER_SEC as u64 {
        crate::warn!(
            state.log,
            "driver wake stalled ({} ns behind); resyncing",
            now - state.next_time
        );
        state.next_time = now;
    }
    let nsec = match wake {
        // As in ALSA's IRQ path, a device readiness event is the published
        // clock observation, while the servo compares it with the accumulated
        // deadline from the previous cycle.
        WakeKind::Device => now,
        // Timer wakes publish the requested deadline, not scheduling latency.
        WakeKind::Timer => state.next_time,
    };

    D::debug_cycle(state, now, nsec);

    // position and clock were null-checked above and stay set for the cycle
    if duration == 0 || rate == 0 {
        // malformed position: idle-tick, and advance next_time so the deadline
        // isn't stale when the position recovers
        state.next_time = nsec + SPA_NSEC_PER_SEC as u64 / 100;
        disable_device_wake(state);
        set_timeout(state, state.next_time);
        return None;
    }

    let (corr, delay) = driver_servo(state, nsec, rate, wake);

    // Keep phase error cumulative as ALSA does: advancing from the previous
    // prediction lets a residual rate mismatch grow into a corrective signal.
    // Re-anchoring every device cycle would reduce it to a tiny one-period
    // interval error and effectively freeze the DLL at low bandwidth.
    state.next_time = advance_deadline(state.next_time, duration, rate, corr);

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

// Drain the descriptor that made wake_source readable. A timer cycle has no
// atomic device snapshot; a sound event deposits its pre-I/O snapshot on the
// matching port for both the servo and the inline process() call.
#[derive(Clone, Copy)]
enum WakeKind {
    Timer,
    Device,
}

fn drain_wake<D: Direction>(state: &mut DataState<D>) -> Option<WakeKind> {
    if let Some(queue) = &state.sound_queue {
        match queue.next_event() {
            Ok(Some(backend::WakeEvent::Timer)) => {
                for port in &mut state.ports {
                    port.device_event = None;
                }
                Some(WakeKind::Timer)
            }
            Ok(Some(backend::WakeEvent::Device(event))) => {
                let Some(port) = state
                    .ports
                    .iter_mut()
                    .find(|port| port.dsp.fd() == Some(event.fd))
                else {
                    // Device replacement normally deletes its old knote.
                    // If an already-queued event survives, consume it and
                    // let wake_cycle restore the current wake selection.
                    crate::debug!(state.log, "ignoring stale OSS event for fd {}", event.fd);
                    return None;
                };
                port.device_eof |= event.eof;
                port.device_event = Some(event);
                Some(WakeKind::Device)
            }
            Ok(None) => None,
            Err(err) => {
                crate::warn!(state.log, "reading the OSS kqueue event: {}", err);
                None
            }
        }
    } else {
        let mut expirations = 0;
        if state
            .timer_fd
            .as_ref()
            .expect("the timer backend lives until clear")
            .read(&mut expirations)
            < 0
        {
            // disarmed (Pause/Suspend) in this same wakeup; nothing to read
            return None;
        }
        for port in &mut state.ports {
            port.device_event = None;
        }
        Some(WakeKind::Timer)
    }
}

fn disable_device_wake<D: Direction>(state: &mut DataState<D>) {
    if let Some(queue) = &mut state.sound_queue
        && let Err(err) = queue.unregister_device()
    {
        crate::warn!(state.log, "removing the OSS kqueue device: {}", err);
    }
    for port in &mut state.ports {
        port.device_event = None;
    }
}

// A close/open can reuse the same integer descriptor after the kernel has
// silently removed the old fd's knote. Clear both our registration cache and
// its failure cache whenever device ownership changes.
pub(crate) fn invalidate_device_wake<D: Direction>(state: &mut DataState<D>) {
    disable_device_wake(state);
    state.sound_failed_fd = None;
}

// Keep the sound knote aligned with the one running driver device. true means
// the knote is registered; the independent deadline watchdog remains armed.
// The node exposes one port; the fd match rejects stale events after device
// replacement.
pub(crate) fn refresh_device_wake<D: Direction>(state: &mut DataState<D>) -> bool {
    refresh_device_wake_inner(state, false)
}

fn refresh_device_wake_inner<D: Direction>(state: &mut DataState<D>, retry_failed: bool) -> bool {
    if state.sound_queue.is_none() {
        return false;
    }

    let candidate =
        if state.started && !state.following && !state.position.is_null() && !state.clock.is_null()
        {
            state.ports.iter().find_map(|port| {
                (port.dsp.is_running()
                    && !port.rebuild_pending
                    && !port.device_eof
                    && port.setup_period != 0
                    && port.dsp.fd().is_some())
                .then(|| {
                    (
                        port.dsp.fd().expect("checked above"),
                        D::wake_threshold(port),
                    )
                })
            })
        } else {
            None
        };

    let Some((fd, threshold)) = candidate else {
        disable_device_wake(state);
        return false;
    };

    if state.sound_failed_fd == Some(fd) && !retry_failed {
        return false;
    }
    if state.sound_failed_fd.is_some_and(|failed| failed != fd) {
        state.sound_failed_fd = None;
    }

    let port_idx = state
        .ports
        .iter()
        .position(|port| port.dsp.fd() == Some(fd))
        .expect("the candidate came from this port set");
    let needs_threshold = wake_threshold_changed(
        state.ports[port_idx].wake_threshold,
        threshold,
        state.ports[port_idx].setup_blocksize,
    );
    if needs_threshold
        && !state.ports[port_idx]
            .dsp
            .configure_wake_threshold(threshold)
    {
        if let Some(queue) = &mut state.sound_queue {
            let _ = queue.unregister_device();
        }
        let first_failure = state.sound_failed_fd != Some(fd);
        state.sound_failed_fd = Some(fd);
        if first_failure {
            crate::warn!(
                state.log,
                "{}: can't set the OSS kqueue wake threshold {}; using the timer",
                state.ports[port_idx].dsp.path(),
                threshold
            );
        }
        return false;
    }
    if needs_threshold {
        state.ports[port_idx].wake_threshold = threshold;
    }

    let queue = state
        .sound_queue
        .as_mut()
        .expect("checked at the start of the function");
    if let Err(err) = queue.register_device(fd, D::PLAYBACK) {
        let first_failure = state.sound_failed_fd != Some(fd);
        state.sound_failed_fd = Some(fd);
        if first_failure {
            crate::warn!(
                state.log,
                "{}: can't register OSS kqueue events: {}; using the timer",
                state.ports[port_idx].dsp.path(),
                err
            );
        }
        return false;
    }
    // A successful explicit retry starts a new failure episode: if this fd
    // later loses kqueue support again, report that transition once.
    state.sound_failed_fd = None;
    true
}

fn wake_threshold_changed(current: u32, desired: u32, blocksize: u32) -> bool {
    current == 0 || current.abs_diff(desired) >= blocksize.max(1)
}

fn driver_wake_deadline(
    started: bool,
    following: bool,
    has_position: bool,
    has_clock: bool,
    next_time: u64,
) -> u64 {
    if started && !following && has_position && has_clock {
        next_time
    } else {
        0
    }
}

fn watchdog_grace(duration: u64, rate: u32) -> u64 {
    if duration == 0 || rate == 0 {
        return 0;
    }
    // Leave the normal device edge a small part of one graph period to win
    // the race. This remains a prompt liveness fallback at low latency while
    // keeping its expiration away from ordinary interrupt jitter.
    ((duration as u128 * SPA_NSEC_PER_SEC as u128 / rate as u128) / 8)
        .max(1)
        .min(u64::MAX as u128) as u64
}

fn watchdog_deadline(deadline: u64, device_armed: bool, duration: u64, rate: u32) -> u64 {
    if deadline == 0 || !device_armed {
        deadline
    } else {
        deadline.saturating_add(watchdog_grace(duration, rate))
    }
}

fn wake_is_from_previous_cycle(now: u64, deadline: u64, duration: u64, rate: u32) -> bool {
    if deadline == 0 || duration == 0 || rate == 0 {
        return false;
    }
    // Real device wakes can lead their prediction slightly. Only reject an
    // event more than one eighth-period early: a just-consumed timer/device
    // partner is nearly a full period early after next_time advances.
    now.saturating_add(watchdog_grace(duration, rate)) < deadline
}

fn timing_error_bytes(actual: u64, expected: u64, rate: u32, stride: u32) -> f64 {
    let magnitude =
        actual.abs_diff(expected) as f64 * rate as f64 * stride as f64 / SPA_NSEC_PER_SEC as f64;
    if actual >= expected {
        magnitude
    } else {
        -magnitude
    }
}

fn advance_deadline(deadline: u64, duration: u64, rate: u32, corr: f64) -> u64 {
    deadline
        .saturating_add((duration as f64 * SPA_NSEC_PER_SEC as f64 / (rate as f64 * corr)) as u64)
}

// Keep the enriched knote and the predicted graph deadline armed together.
// EV_CLEAR needs a later sound-buffer transition to reactivate; when playback
// drains empty without receiving another graph buffer, that transition stops.
// The one-shot deadline is therefore a liveness watchdog as well as the
// fallback for a failed knote. Re-arming it after a device cycle replaces an
// undelivered expiration from the preceding deadline.
pub(crate) fn select_next_wake<D: Direction>(state: &mut DataState<D>) {
    let mut deadline = driver_wake_deadline(
        state.started,
        state.following,
        !state.position.is_null(),
        !state.clock.is_null(),
        state.next_time,
    );
    let device_armed = refresh_device_wake(state);
    let (duration, rate) = state
        .position
        .with_ref(|p| (p.clock.target_duration, p.clock.target_rate.denom))
        .unwrap_or((0, 0));
    deadline = watchdog_deadline(deadline, device_armed, duration, rate);
    set_timeout(state, deadline);
}

// Data loop only. Arm the wakeup timer from now when this node drives the
// graph (started, not following, position present); park it otherwise. A
// failed clock read must not park a node that wants to run (nothing but
// another external transition would ever re-arm it): retry on a relative
// ~10 ms one-shot without touching next_time - it re-anchors only from a
// successful read (here or on_wake's stall resync; an absolute arm from
// a stale next_time would busy-spin) - and let on_wake take over from
// there; nothing aborts the data loop (the sink's former copy assert!()ed).
pub(crate) fn update_driver_wake<D: Direction>(state: &mut DataState<D>) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "update_driver_wake");

    if !(state.started && !state.following && !state.position.is_null()) {
        refresh_device_wake(state);
        set_timeout(state, 0); // park
        return;
    }

    // This path runs for explicit state changes (Start, role/configuration
    // changes and device replacement), never for ordinary audio cycles. Give
    // a transient native-threshold/registration failure one bounded retry.
    // Preserve its marker during the attempt so a permanent failure warns once
    // per failure episode rather than once per transition.
    refresh_device_wake_inner(state, true);

    match try_now_ns(&state.data_system) {
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
    if let Some(queue) = &state.sound_queue {
        // EVFILT_TIMER NOTE_ABSTIME is a wall-clock epoch, unlike SPA's
        // CLOCK_MONOTONIC deadline. Convert absolute monotonic deadlines to
        // a relative interval; on a second clock-read failure, retain a
        // retry instead of turning a stale deadline into a busy loop.
        let delay_ns = if value_ns == 0 || flags == 0 {
            value_ns
        } else {
            try_now_ns(&state.data_system)
                .map(|now| deadline_delay(value_ns, now))
                .unwrap_or(SPA_NSEC_PER_SEC as u64 / 100)
        };
        if let Err(err) = queue.arm_timer(delay_ns) {
            crate::warn!(state.log, "arming the OSS kqueue timer: {}", err);
        }
        return;
    }
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
        .expect("the timer backend lives until clear")
        .settime(flags, &timerspec);
}

fn deadline_delay(deadline: u64, now: u64) -> u64 {
    deadline.saturating_sub(now).max(1)
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
pub(crate) fn try_now_ns(system: &System) -> Option<u64> {
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

    #[test]
    fn monotonic_deadlines_become_nonzero_relative_timers() {
        assert_eq!(deadline_delay(1_500, 1_000), 500);
        assert_eq!(deadline_delay(1_000, 1_000), 1);
        assert_eq!(deadline_delay(500, 1_000), 1);
    }

    #[test]
    fn live_driver_keeps_its_deadline_watchdog() {
        assert_eq!(
            driver_wake_deadline(true, false, true, true, 12_345),
            12_345
        );
        assert_eq!(driver_wake_deadline(false, false, true, true, 12_345), 0);
        assert_eq!(driver_wake_deadline(true, true, true, true, 12_345), 0);
        assert_eq!(driver_wake_deadline(true, false, false, true, 12_345), 0);
        assert_eq!(driver_wake_deadline(true, false, true, false, 12_345), 0);
    }

    #[test]
    fn device_watchdog_trails_the_expected_edge() {
        let period = 2048 * SPA_NSEC_PER_SEC as u64 / 48_000;
        let grace = period / 8;
        assert_eq!(watchdog_grace(2048, 48_000), grace);
        assert_eq!(
            watchdog_deadline(1_000_000_000, true, 2048, 48_000),
            1_000_000_000 + grace
        );
        assert_eq!(
            watchdog_deadline(1_000_000_000, false, 2048, 48_000),
            1_000_000_000
        );
        assert_eq!(watchdog_deadline(0, true, 2048, 48_000), 0);
    }

    #[test]
    fn wake_pair_cannot_start_two_graph_cycles() {
        let period = 2048 * SPA_NSEC_PER_SEC as u64 / 48_000;
        let next = 1_000_000_000 + period;
        assert!(wake_is_from_previous_cycle(
            1_000_010_000,
            next,
            2048,
            48_000
        ));
        assert!(!wake_is_from_previous_cycle(
            next - period / 16,
            next,
            2048,
            48_000
        ));
        assert!(!wake_is_from_previous_cycle(next, next, 2048, 48_000));
    }

    #[test]
    fn device_wake_error_tracks_time_without_fragment_bias() {
        assert_eq!(
            timing_error_bytes(1_001_000_000, 1_000_000_000, 48_000, 8),
            384.0
        );
        assert_eq!(
            timing_error_bytes(999_000_000, 1_000_000_000, 48_000, 8),
            -384.0
        );
        assert_eq!(
            timing_error_bytes(1_000_000_000, 1_000_000_000, 48_000, 8),
            0.0
        );
    }

    #[test]
    fn device_deadline_accumulates_from_the_previous_prediction() {
        let previous = 1_000_000_000;
        let period = 512 * SPA_NSEC_PER_SEC as u64 / 48_000;
        assert_eq!(
            advance_deadline(previous, 512, 48_000, 1.0),
            previous + period
        );

        // A late device arrival is the phase-error observation, not the next
        // deadline's anchor. Its lateness therefore remains visible on the
        // following cycle instead of being discarded every wake.
        let arrival = previous + 20_000;
        assert_ne!(
            advance_deadline(previous, 512, 48_000, 1.0),
            arrival + period
        );
    }

    #[test]
    fn accumulated_phase_servo_converges_inside_the_partner_guard() {
        let duration = 512u64;
        let rate = 48_000u32;
        let stride = 8u32;
        let ideal_period = duration as f64 * SPA_NSEC_PER_SEC as f64 / rate as f64;
        // Exercise both signs at a device-rate offset much larger than normal
        // crystal tolerance. The DLL must correct before a legitimate early
        // device edge reaches the duplicate-partner rejection window.
        for actual_period in [
            (ideal_period * 0.9995) as u64,
            (ideal_period * 1.0005) as u64,
        ] {
            let mut dll = SpaDLL::default();
            let mut bw = BwAdapt::default();
            bw.configure(stride, 1024, duration as u32 * stride, rate * stride);
            let mut deadline = 1_000_000_000u64;
            let mut actual = deadline;
            let mut corr = 1.0;
            let mut phase_error = 0.0;
            for _ in 0..10_000 {
                actual = actual.saturating_add(actual_period);
                deadline = advance_deadline(deadline, duration, rate, corr);
                assert!(
                    !wake_is_from_previous_cycle(actual, deadline, duration, rate),
                    "normal phase convergence reached the duplicate-wake guard"
                );
                phase_error = timing_error_bytes(actual, deadline, rate, stride);
                corr = dll.update(phase_error);
                bw.update_timing(&mut dll, phase_error, actual);
            }

            let expected = ideal_period / actual_period as f64;
            assert!(
                (corr - expected).abs() < 0.0001,
                "corr {corr} did not approach device ratio {expected}"
            );
            assert!(
                phase_error.abs() < 8.0 * stride as f64,
                "phase error did not settle: {phase_error} bytes"
            );
        }
    }

    #[test]
    fn wake_threshold_moves_at_fragment_granularity() {
        assert!(wake_threshold_changed(0, 16_384, 2_048));
        assert!(!wake_threshold_changed(16_384, 17_407, 2_048));
        assert!(wake_threshold_changed(16_384, 18_432, 2_048));
        assert!(wake_threshold_changed(18_432, 16_384, 2_048));
        assert!(wake_threshold_changed(8, 9, 0));
    }
}
