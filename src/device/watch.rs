use super::*;
use crate::platform;

// Tell the session manager to push the new hardware state into the child
// node's Props (channelVolumes/softVolumes or mute/softMute), keeping
// audioconvert at unity - the anti-double-attenuation mechanism
// (pod shape: alsa-acp-device.c:1015-1084).
pub(super) fn queue_object_config(
    state: &Runtime,
    pos: usize,
    volume: bool,
    notifications: &mut Vec<DeviceNotification>,
) {
    let route = &state.routes[pos];
    let (node_id, levels, mute) = (route.node_id, route.levels, route.mute);
    let hw = route.control.is_some();

    let buffer = if volume {
        build_object_config(node_id, Some((levels, hw)), None)
    } else {
        build_object_config(node_id, None, Some(mute))
    };

    notifications.push(DeviceNotification::Event(buffer));
}

// announce a Route change: flip the serial so consumers re-read the param
pub(super) fn queue_route_change(state: &Runtime, notifications: &mut Vec<DeviceNotification>) {
    state.events.with_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Route);
    });
    notifications.push(DeviceNotification::Info(state.events.take_info()));
}

// The ~1 Hz external-change poll: on a modify_counter tick, value-diff the
// levels and mute against the shadow and re-emit only on a real change. The
// counter is only a hint (it misses RECSRC changes and writes-to-muted); the
// value diff is what prevents spurious re-emissions either way.
// re-resolve a recsrc-derived capture control (RECSRC changes never tick the
// modify counter, and the write path must not adjust the OLD source)
pub(super) fn resolve_recsrc(state: &mut Runtime, pos: usize) {
    if !state.routes[pos].follows_recsrc {
        return;
    }
    let mi = state.routes[pos].mixer;
    if let Some((control, true)) = state.mixers[mi].mixer.input_control() {
        state.routes[pos].control = Some(control);
    }
}

// pull the hardware state into a route's shadow (no emissions)
pub(super) fn refresh_route_shadow(state: &mut Runtime, pos: usize) {
    resolve_recsrc(state, pos);
    let mi = state.routes[pos].mixer;
    let Some(control) = state.routes[pos].control else {
        return; // nothing to shadow for a control-less source route
    };
    if let Some(levels) = state.mixers[mi].mixer.level(control) {
        state.routes[pos].levels = levels;
    }
    if let Some(mute) = state.mixers[mi].mixer.muted(control) {
        state.routes[pos].mute = mute;
    }
}

// Value-poll RECSRC and move the active flag to the route backing the
// current source; the kernel never ticks modify_counter for RECSRC writes
// (mixer_setrecsrc, mixer.c:334-361), so external mixer(8) changes are only
// visible this way. Multiple set bits collapse to the lowest (the v1
// single-route convention). Returns the newly active route when it moved.
pub(super) fn sync_recsrc(state: &mut Runtime, mi: usize) -> Option<usize> {
    if !state
        .routes
        .iter()
        .any(|r| r.mixer == mi && r.source.is_some())
    {
        return None;
    }
    let recsrc = state.mixers[mi].mixer.recsrc()?;
    if recsrc == state.mixers[mi].recsrc {
        return None;
    }
    state.mixers[mi].recsrc = recsrc;
    let masked = recsrc & state.mixers[mi].mixer.recmask();
    if masked == 0 {
        return None; // keep the current selection rather than guessing
    }
    let bit = masked.trailing_zeros();
    let pos = state
        .routes
        .iter()
        .position(|r| r.mixer == mi && r.source == Some(bit))?;
    if state.routes[pos].active {
        return None; // an extra bit appeared; the winning source is unchanged
    }
    for route in state.routes.iter_mut() {
        if route.mixer == mi && route.source.is_some() {
            route.active = route.source == Some(bit);
        }
    }
    refresh_route_shadow(state, pos);
    Some(pos)
}

pub(super) fn poll_mixers(state: &mut Runtime) -> Vec<DeviceNotification> {
    let mut notifications = Vec::new();
    if state.profile == 0 {
        return notifications; // nodes are retracted under Off
    }

    let mut changed: Vec<(usize, bool, bool)> = vec![]; // (route, volume, mute)
    let mut switched: Vec<usize> = vec![];

    for mi in 0..state.mixers.len() {
        let Some(counter) = state.mixers[mi].mixer.modify_counter() else {
            continue; // the device may be mid-detach; the node teardown handles it
        };
        // Diff by VALUE every tick, not only when the counter moved: the kernel
        // doesn't bump it for writes to a muted control (mixer.c early-returns
        // into level_muted), and an external write landing inside our own
        // write-then-refresh window is swallowed by the baseline. The counter is
        // still tracked for log/debug value.
        state.mixers[mi].counter = counter;

        // recsrc first: it refreshes the new active route's shadow, so the
        // value diff below won't double-report the same movement
        if let Some(pos) = sync_recsrc(state, mi) {
            crate::info!(
                state.log,
                "recording source changed externally: route {}",
                state.routes[pos].name
            );
            switched.push(pos);
        }

        for pos in 0..state.routes.len() {
            if state.routes[pos].mixer != mi {
                continue;
            }
            resolve_recsrc(state, pos);
            let Some(control) = state.routes[pos].control else {
                continue; // control-less source routes carry no volume state
            };
            let mut vol_changed = false;
            let mut mute_changed = false;
            if let Some(levels) = state.mixers[mi].mixer.level(control)
                && levels != state.routes[pos].levels
            {
                state.routes[pos].levels = levels;
                vol_changed = true;
            }
            if let Some(mute) = state.mixers[mi].mixer.muted(control)
                && mute != state.routes[pos].mute
            {
                state.routes[pos].mute = mute;
                mute_changed = true;
            }
            // inactive routes still track the hardware (their level shows again on
            // the next switch), but a change there is observable in no pod
            if (vol_changed || mute_changed) && state.routes[pos].active {
                crate::info!(
                    state.log,
                    "route {} changed externally: levels {:?}, mute {}",
                    state.routes[pos].name,
                    state.routes[pos].levels,
                    state.routes[pos].mute
                );
                changed.push((pos, vol_changed, mute_changed));
            }
        }
    }

    if changed.is_empty() && switched.is_empty() {
        return notifications;
    }

    queue_route_change(state, &mut notifications);

    for pos in switched {
        // the node's effective input volume is the new source's control now
        if state.routes[pos].control.is_some() {
            queue_object_config(state, pos, true, &mut notifications);
            queue_object_config(state, pos, false, &mut notifications);
        }
    }

    for (pos, vol_changed, mute_changed) in changed {
        if vol_changed {
            queue_object_config(state, pos, true, &mut notifications);
        }
        if mute_changed {
            queue_object_config(state, pos, false, &mut notifications);
        }
    }
    notifications
}

pub(super) unsafe extern "C" fn on_mixer_timeout(source: *mut spa_source) {
    let state: *mut State = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        // Scoped runtime borrow: all mixer mutations and payload construction
        // finish before arbitrary listener code runs below.
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let timer_fd = state.timer_fd.as_ref()?;
                let mut expirations = 0;
                (timer_fd.read(&mut expirations) >= 0)
                    .then(|| (state.events.clone(), poll_mixers(state)))
            })
        }) else {
            return;
        };
        result
    };
    // SAFETY: the scoped State borrow ended above.
    unsafe { events.dispatch_all(notifications) };
}

// devd "SND CONN" watcher. What the kernel actually emits (verified against
// 14.4+ /usr/src) is "!system=SND subsystem=CONN type={IN,OUT} cdev=dspN"
// (type=NODEV without a cdev when the last device goes):
//  - sound.c:81-97 (pcm_hotswap) fires it when hw.snd.default_unit moves -
//    not jack state at all;
//  - hdaa.c:566-592 (hdaa_presence_handler) fires it on a pin-sense change,
//    but only when the codec owns the default unit, never for headphone
//    redirect associations (hdaa.c:572 returns first - the common laptop
//    jack), and cdev names the device the kernel now PREFERS: the plugged
//    association on connect, the first enabled same-direction association
//    on disconnect. Connect and disconnect messages are indistinguishable.
// No other sound driver emits it. The payload therefore identifies a pcm
// unit but carries no jack state, so per-route available yes/no cannot be
// derived and availability stays a constant "yes" (see build_route_info).
// What a jack event DOES change kernel-side is the recording source
// (hdaa_autorecsrc_handler, hdaa.c:562) and pin mutes, so the one sound
// reaction is nudging the mixer poll instead of waiting out the 1 Hz tick.
pub(super) unsafe extern "C" fn on_hotplug_event(source: *mut spa_source) {
    let state: *mut State = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let hotplug_monitor = state.hotplug_monitor.as_mut()?;

                let (alive, unit) = hotplug_monitor.read_mixer_event();
                let nudged = unit.is_some_and(|unit| {
                    state.pcm_devices.iter().any(|device| device.index == unit)
                });

                let notifications = if nudged {
                    crate::debug!(state.log, "SND CONN event; re-polling the mixers");
                    poll_mixers(state)
                } else {
                    Vec::new()
                };

                if !alive {
                    // devd restarted or dropped us; deregister or the level-triggered fd
                    // spins the main loop forever. The 1 Hz poll still covers changes.
                    crate::warn!(
                        state.log,
                        "devd connection lost; falling back to the mixer poll alone"
                    );
                    // SAFETY: this callback runs on the registered main loop.
                    if state.hotplug_source.unregister() < 0 {
                        eprintln!(
                            "{}: can't detach the devd source; aborting",
                            platform::DIAGNOSTIC_TAG
                        );
                        std::process::abort();
                    }
                    state.hotplug_monitor = None;
                    state.hotplug_source.set_fd(-1);
                }
                Some((state.events.clone(), notifications))
            })
        }) else {
            return;
        };
        result
    };
    // SAFETY: the scoped State borrow ended above.
    unsafe { events.dispatch_all(notifications) };
}

// Arm the external-change watchers: the ~1 Hz mixer poll timer and the devd
// socket (jack sense / recording-source flips). Both are best-effort - a
// failure only costs noticing external changes - and only worth arming
// when something is routed.
pub(super) unsafe fn arm_mixer_watch(state: &mut Runtime) {
    if state.routes.is_empty() {
        return;
    }

    if let (Some(main_loop), Some(system)) = (&state.main_loop, &state.system) {
        match system.timerfd_create(
            libc::CLOCK_MONOTONIC,
            (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as c_int,
        ) {
            Err(_) => {
                crate::warn!(
                    state.log,
                    "can't create the mixer poll timer; external volume changes won't be noticed"
                );
            }
            Ok(timer_fd) => {
                let timerspec = itimerspec {
                    it_value: timespec {
                        tv_sec: 1,
                        tv_nsec: 0,
                    },
                    it_interval: timespec {
                        tv_sec: 1,
                        tv_nsec: 0,
                    },
                };
                if timer_fd.settime(0, &timerspec) < 0 {
                    crate::warn!(state.log, "can't arm the mixer poll timer");
                }
                state.timer_source.set_fd(timer_fd.raw());
                state.timer_fd = Some(timer_fd);
                // SAFETY: init runs in the host context accepted by add_source;
                // the pinned source remains alive until clear unregisters it.
                if unsafe { state.timer_source.register(main_loop) } < 0 {
                    crate::warn!(
                        state.log,
                        "can't watch the mixer; external volume changes won't be noticed"
                    );
                    drop(state.timer_fd.take());
                    state.timer_source.set_fd(-1);
                }
            }
        }
    }

    // devd's SND CONN notifications (jack sense, default-unit moves) nudge
    // the same poll so kernel-side recording-source flips show up right
    // away; losing devd only costs that immediacy (jails, minimal systems)
    if let Some(main_loop) = &state.main_loop {
        match platform::HotplugMonitor::open() {
            Ok(socket) => {
                state.hotplug_source.set_fd(socket.fd());
                state.hotplug_monitor = Some(socket);
                // SAFETY: as for the mixer source above.
                if unsafe { state.hotplug_source.register(main_loop) } < 0 {
                    crate::warn!(
                        state.log,
                        "can't watch devd; jack events will wait for the mixer poll"
                    );
                    state.hotplug_monitor = None;
                    state.hotplug_source.set_fd(-1);
                }
            }
            Err(err) => {
                crate::info!(
                    state.log,
                    "no devd connection ({}); jack events will wait for the mixer poll",
                    err
                );
            }
        }
    }
}
