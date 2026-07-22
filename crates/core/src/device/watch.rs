use super::*;
use crate::backend::{self, HotplugMonitor as _};

pub(super) fn log_route_diagnostic(log: &crate::spa::Log, diagnostic: &backend::RouteDiagnostic) {
    match diagnostic.level {
        backend::RouteDiagnosticLevel::Info => crate::info!(log, "{}", diagnostic.message),
        backend::RouteDiagnosticLevel::Warning => crate::warn!(log, "{}", diagnostic.message),
    }
}

// Tell the session manager to push the new hardware state into the child
// node's Props (channelVolumes/softVolumes or mute/softMute), keeping
// audioconvert at unity. This is the anti-double-attenuation mechanism.
pub(super) fn queue_object_config<B: backend::Backend>(
    state: &Runtime<B>,
    pos: usize,
    emit_volume: bool,
    notifications: &mut Vec<DeviceNotification>,
) {
    let route = &state.routes[pos];
    let (node_id, volume, mute) = (route.node_id, &route.volume, route.mute.value);

    let buffer = if emit_volume {
        build_object_config(
            node_id,
            Some((&volume.values, volume.hardware, &volume.channels)),
            None,
        )
    } else {
        build_object_config(node_id, None, Some(mute))
    };

    notifications.push(DeviceNotification::Event(buffer));
}

// announce a Route change: flip the serial so consumers re-read the param
pub(super) fn queue_route_change<B: backend::Backend>(
    state: &Runtime<B>,
    notifications: &mut Vec<DeviceNotification>,
) {
    state.events.with_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Route);
    });
    notifications.push(DeviceNotification::Info(state.events.take_info()));
}

pub(super) fn poll_routes<B: backend::Backend>(state: &mut Runtime<B>) -> Vec<DeviceNotification> {
    let mut notifications = Vec::new();
    if state.profile == 0 {
        return notifications;
    }
    let changes = state.route_controller.poll(&mut state.routes);
    if changes.is_empty() {
        return notifications;
    }
    queue_route_change(state, &mut notifications);
    for change in changes {
        let Some(key) = change.key else { continue };
        let Some(pos) = state.routes.iter().position(|route| route.key == key) else {
            continue;
        };
        if let Some(diagnostic) = &change.diagnostic {
            log_route_diagnostic(&state.log, diagnostic);
        }
        if change.volume {
            queue_object_config(state, pos, true, &mut notifications);
        }
        if change.mute {
            queue_object_config(state, pos, false, &mut notifications);
        }
    }
    notifications
}

pub(super) unsafe extern "C" fn on_route_timeout<B: backend::Backend>(source: *mut spa_source) {
    let state: *mut State<B> = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        // Scoped runtime borrow: route refresh and payload construction finish
        // before arbitrary listener code runs below.
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let timer_fd = state.timer_fd.as_ref()?;
                let mut expirations = 0;
                (timer_fd.read(&mut expirations) >= 0)
                    .then(|| (state.events.clone(), poll_routes(state)))
            })
        }) else {
            return;
        };
        result
    };
    // SAFETY: the scoped State borrow ended above.
    unsafe { events.dispatch_all(notifications) };
}

pub(super) unsafe extern "C" fn on_hotplug_event<B: backend::Backend>(source: *mut spa_source) {
    let state: *mut State<B> = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let hotplug_monitor = state.hotplug_monitor.as_mut()?;

                let (alive, nudged) = state
                    .route_controller
                    .read_hotplug(&state.routes, hotplug_monitor);

                let notifications = if nudged {
                    crate::debug!(
                        state.log,
                        "{}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteNudge)
                    );
                    poll_routes(state)
                } else {
                    Vec::new()
                };

                if !alive {
                    // A closed level-triggered source must be deregistered or
                    // it spins the main loop forever. Periodic polling remains.
                    crate::warn!(
                        state.log,
                        "{}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteLost)
                    );
                    // SAFETY: this callback runs on the registered main loop.
                    if state.hotplug_source.unregister() < 0 {
                        eprintln!(
                            "{}: {}",
                            B::DIAGNOSTIC_TAG,
                            B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteDetachAbort)
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

// Arm the periodic and event-driven external-change watchers. Both are
// best-effort and only worth arming when something is routed.
pub(super) unsafe fn arm_route_watch<B: backend::Backend>(state: &mut Runtime<B>) {
    if state.routes.is_empty() {
        return;
    }

    let policy = state.route_controller.watch_policy();

    if let (Some(interval_ns), Some(main_loop), Some(system)) =
        (policy.poll_interval_ns, &state.main_loop, &state.system)
    {
        match system.timerfd_create(
            libc::CLOCK_MONOTONIC,
            (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as c_int,
        ) {
            Err(_) => {
                crate::warn!(
                    state.log,
                    "{}",
                    B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteTimerCreate)
                );
            }
            Ok(timer_fd) => {
                let timerspec = itimerspec {
                    it_value: timespec {
                        tv_sec: (interval_ns / 1_000_000_000) as _,
                        tv_nsec: (interval_ns % 1_000_000_000) as _,
                    },
                    it_interval: timespec {
                        tv_sec: (interval_ns / 1_000_000_000) as _,
                        tv_nsec: (interval_ns % 1_000_000_000) as _,
                    },
                };
                if timer_fd.settime(0, &timerspec) < 0 {
                    crate::warn!(
                        state.log,
                        "{}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteTimerArm)
                    );
                }
                state.timer_source.set_fd(timer_fd.raw());
                state.timer_fd = Some(timer_fd);
                // SAFETY: init runs in the host context accepted by add_source;
                // the pinned source remains alive until clear unregisters it.
                if unsafe { state.timer_source.register(main_loop) } < 0 {
                    crate::warn!(
                        state.log,
                        "{}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteTimerWatch)
                    );
                    drop(state.timer_fd.take());
                    state.timer_source.set_fd(-1);
                }
            }
        }
    }

    // Platform events nudge the same poll so route changes show up promptly;
    // losing the event service costs only that immediacy.
    if policy.event_driven
        && let Some(main_loop) = &state.main_loop
    {
        match B::Hotplug::open() {
            Ok(socket) => {
                state.hotplug_source.set_fd(socket.fd());
                state.hotplug_monitor = Some(socket);
                // SAFETY: as for the route timer source above.
                if unsafe { state.hotplug_source.register(main_loop) } < 0 {
                    crate::warn!(
                        state.log,
                        "{}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteWatch)
                    );
                    state.hotplug_monitor = None;
                    state.hotplug_source.set_fd(-1);
                }
            }
            Err(err) => {
                crate::info!(
                    state.log,
                    "{} ({}); {}",
                    B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteOpen),
                    err,
                    B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteOpenFallback)
                );
            }
        }
    }
}
