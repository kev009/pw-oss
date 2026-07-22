#[cfg(debug_assertions)]
use crate::spa::dump_spa_dict;
use crate::spa::{
    ListenerList, Log, Loop, LoopSource, ParamStep, System, TimerFd, enum_params_loop,
    for_each_dict_item, key,
};
use libspa::sys::*;
use std::ffi::{c_char, c_int, c_void};

use crate::backend::{self, DeviceInit as _, RouteController as _};

mod events;
mod params;
mod watch;

use events::{DeviceEvents, DeviceNotification, build_object_config, object_events};
use params::{build_profile_info, build_route_info, set_param};
type RouteState = backend::RouteSnapshot;
use watch::{
    arm_route_watch, log_route_diagnostic, on_hotplug_event, on_route_timeout, queue_object_config,
    queue_route_change,
};

// repr(C): the host casts spa_handle* to State*, so `handle` must stay
// the first field at offset 0.
#[repr(C)]
struct State<B: backend::Backend> {
    handle: spa_handle,
    device: spa_device,
    runtime: Runtime<B>,
}

struct Runtime<B: backend::Backend> {
    events: std::rc::Rc<DeviceEvents<B>>,
    snapshot: backend::DeviceSnapshot,
    profile: u32,       // 0 = off, 1 = default
    profile_save: bool, // echoed back in the Profile pod
    routes: Vec<RouteState>,
    route_controller: B::Routes,
    main_loop: Option<Loop>,   // for the route-control poll timer
    system: Option<System>,    // ditto
    timer_fd: Option<TimerFd>, // owns the LoopSource fd mirror
    timer_source: LoopSource,
    hotplug_monitor: Option<B::Hotplug>, // jack/default-unit nudges
    hotplug_source: LoopSource,
    log: Log,
}

// Project only the mutable runtime payload. The host-visible handle and
// interface stay outside every callback borrow, so listener reentry cannot
// overlap a broad &mut State.
unsafe fn with_runtime_mut<B: backend::Backend, R>(
    state: *mut State<B>,
    apply: impl for<'a> FnOnce(&'a mut Runtime<B>) -> R,
) -> R {
    assert!(!state.is_null(), "state is not supposed to be null");
    let runtime = unsafe { (&raw mut (*state).runtime).as_mut_unchecked() };
    apply(runtime)
}

unsafe fn with_runtime_ref<B: backend::Backend, R>(
    state: *const State<B>,
    apply: impl for<'a> FnOnce(&'a Runtime<B>) -> R,
) -> R {
    assert!(!state.is_null(), "state is not supposed to be null");
    let runtime = unsafe { (&raw const (*state).runtime).as_ref_unchecked() };
    apply(runtime)
}

unsafe extern "C" fn add_listener<B: backend::Backend>(
    object: *mut c_void,
    listener: *mut spa_hook,
    events: *const spa_device_events,
    data: *mut c_void,
) -> c_int {
    let state: *mut State<B> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let (device_events, objects) = {
        unsafe {
            with_runtime_ref(state, |state| {
                (
                    state.events.clone(),
                    object_events(&state.snapshot, &state.routes, state.profile != 0),
                )
            })
        }
    };

    let initial = |hooks: &ListenerList<spa_device_events>| {
        // The initial emissions only reach the newly added listener (the list
        // is isolated). One method per traversal, mirroring C's
        // spa_hook_list_call: a listener that removes and frees its hook
        // inside a callback must not be read for the next method.
        let info = device_events.initial_info();
        let dispatch_guard = device_events.begin_dispatch();
        // SAFETY: all State-backed object data was copied above.
        unsafe { device_events.emit_info_on(hooks, &info) };
        for object in &objects {
            unsafe { device_events.emit_object_on(hooks, object) };
        }
        dispatch_guard
    };
    let dispatch_guard =
        unsafe { device_events.with_new_listener(listener, events, data, initial) };
    if let Some(guard) = dispatch_guard {
        // Nested profile/route changes queued during the initial transaction
        // are delivered only after every initial snapshot, and after the full
        // listener list has been restored.
        unsafe { device_events.drain(guard) };
    }
    0
}

unsafe extern "C" fn sync<B: backend::Backend>(object: *mut c_void, seq: c_int) -> c_int {
    let state: *mut State<B> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { with_runtime_ref(state, |state| state.events.clone()) };
    // SAFETY: only the independent endpoint remains borrowed. Done joins the
    // same FIFO as info/object transactions, so reentrant sync cannot overtake
    // already-produced state notifications.
    unsafe { events.dispatch_all(vec![DeviceNotification::Done(seq)]) };

    0
}

unsafe extern "C" fn enum_params<B: backend::Backend>(
    object: *mut c_void,
    seq: c_int,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state: *mut State<B> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { with_runtime_ref(state, |state| state.events.clone()) };
    let runtime = unsafe { &raw mut (*state).runtime };

    unsafe {
        enum_params_loop(
            runtime,
            (start, max),
            filter,
            |state, index| {
                use ParamStep;
                // Only the active route becomes a Route pod; inactive
                // selectable sources remain EnumRoute choices. Consumers use
                // that single active index as the device's current route.
                if id == SPA_PARAM_Route
                    && (index as usize) < state.routes.len()
                    && !state.routes[index as usize].active
                {
                    return ParamStep::Skip;
                }

                #[expect(non_upper_case_globals)]
                match (id, index) {
                    (SPA_PARAM_EnumProfile, 0 | 1) => ParamStep::Built(build_profile_info(
                        SPA_PARAM_EnumProfile,
                        index,
                        &state.snapshot,
                        state.profile_save,
                        false,
                    )),
                    (SPA_PARAM_Profile, 0) => ParamStep::Built(build_profile_info(
                        SPA_PARAM_Profile,
                        state.profile,
                        &state.snapshot,
                        state.profile_save,
                        true,
                    )),
                    (SPA_PARAM_EnumRoute, i) if (i as usize) < state.routes.len() => {
                        ParamStep::Built(build_route_info(
                            SPA_PARAM_EnumRoute,
                            &state.routes[i as usize],
                            i as usize,
                            state.profile,
                            false,
                        ))
                    }
                    // no Route pods while Off is active: there is nothing routed
                    (SPA_PARAM_Route, i)
                        if state.profile != 0 && (i as usize) < state.routes.len() =>
                    {
                        ParamStep::Built(build_route_info(
                            SPA_PARAM_Route,
                            &state.routes[i as usize],
                            i as usize,
                            state.profile,
                            true,
                        ))
                    }
                    // a known id whose indices are exhausted ends the enumeration
                    (
                        SPA_PARAM_EnumProfile
                        | SPA_PARAM_Profile
                        | SPA_PARAM_EnumRoute
                        | SPA_PARAM_Route,
                        _,
                    ) => ParamStep::Stop(0),
                    _ => ParamStep::Stop(-libc::ENOENT),
                }
            },
            |index, param| {
                let result = spa_result_device_params {
                    id,
                    index,
                    next: index + 1,
                    param,
                };
                // SAFETY: enum_params_loop ended its per-step runtime borrow
                // before invoking this closure.
                events.emit_result(seq, &result);
            },
        )
    }
}

struct DeviceMethods<B>(std::marker::PhantomData<B>);

impl<B: backend::Backend> DeviceMethods<B> {
    const METHODS: spa_device_methods = spa_device_methods {
        version: SPA_VERSION_DEVICE_METHODS,
        add_listener: Some(add_listener::<B>),
        sync: Some(sync::<B>),
        enum_params: Some(enum_params::<B>),
        set_param: Some(set_param::<B>),
    };
}

unsafe extern "C" fn get_interface<B: backend::Backend>(
    handle: *mut spa_handle,
    type_: *const c_char,
    interface: *mut *mut c_void,
) -> c_int {
    let state = handle.cast::<State<B>>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    assert!(!interface.is_null());
    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Device.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe {
            *interface = (&raw mut (*state).device).cast::<c_void>();
        }
    } else {
        return -libc::ENOENT;
    }
    0
}

unsafe extern "C" fn clear<B: backend::Backend>(handle: *mut spa_handle) -> c_int {
    let state = handle.cast::<State<B>>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    unsafe {
        with_runtime_mut(state, |runtime| {
            // clear runs on the main loop's thread, so detach both sources there.
            if runtime.timer_source.is_registered() {
                if runtime.timer_source.unregister() < 0 {
                    eprintln!(
                        "{}: {}",
                        B::DIAGNOSTIC_TAG,
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteTimerDetachAbort)
                    );
                    std::process::abort();
                }
                drop(runtime.timer_fd.take());
                runtime.timer_source.set_fd(-1);
            }
            if runtime.hotplug_source.is_registered() && runtime.hotplug_source.unregister() < 0 {
                eprintln!(
                    "{}: {}",
                    B::DIAGNOSTIC_TAG,
                    B::hotplug_diagnostic(backend::HotplugDiagnostic::RouteDetachAbort)
                );
                std::process::abort();
            }
            if !runtime.hotplug_source.is_registered() {
                runtime.hotplug_source.set_fd(-1);
            }
        });
    }
    // the host frees the memory after clear; drop the fields exactly once here
    unsafe { std::ptr::drop_in_place(state) };
    0
}

extern "C" fn get_size<B: backend::Backend>(
    _factory: *const spa_handle_factory,
    _params: *const spa_dict,
) -> usize {
    size_of::<State<B>>()
}

unsafe fn parse_device_dict<B: backend::Backend>(
    info: *const spa_dict,
) -> <B as backend::Backend>::DeviceInit {
    let mut init = B::DeviceInit::default();

    if let Some(info) = unsafe { info.as_ref() } {
        #[cfg(debug_assertions)]
        unsafe {
            dump_spa_dict(info);
        }

        unsafe {
            for_each_dict_item(info, |key, value| init.parse(key, value));
        }
    }

    init
}

unsafe extern "C" fn init<B: backend::Backend>(
    _factory: *const spa_handle_factory,
    handle: *mut spa_handle,
    info: *const spa_dict,
    support: *const spa_support,
    n_support: u32,
) -> c_int {
    // the support array is the host's init contract: n_support valid entries
    let log = unsafe {
        spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast())
            .cast::<spa_log>()
    };
    let Some(log) = (unsafe { Log::wrap(log, Some(B::device_log_topic())) }) else {
        return -libc::EINVAL;
    };

    // The main loop and system drive the route-control poll timer; both are
    // optional, so external changes go unnoticed when they are absent.
    let main_loop = unsafe {
        spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast())
            .cast::<spa_loop>()
    };
    let system = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_System.as_ptr().cast(),
        )
        .cast::<spa_system>()
    };
    let main_loop = if main_loop.is_null() {
        None
    } else {
        Some(unsafe { Loop::wrap(main_loop) })
    };
    let system = if system.is_null() {
        None
    } else {
        Some(unsafe { System::wrap(system) })
    };

    let state = handle.cast::<State<B>>();
    assert!(!state.is_null(), "handle is not supposed to be null");

    let device_init = unsafe { parse_device_dict::<B>(info) };

    if !device_init.is_complete() {
        crate::error!(log, "{}", B::DeviceInit::missing_selector_diagnostic());
        return -libc::EINVAL;
    }

    let Some(snapshot) = device_init.snapshot() else {
        crate::error!(log, "{}", B::DeviceInit::snapshot_diagnostic());
        return -libc::EINVAL;
    };

    let (route_controller, routes) = B::Routes::probe(&snapshot);
    let events = std::rc::Rc::new(DeviceEvents::new());

    // the host hands us uninitialized memory of get_size() bytes; write the
    // whole State without dropping the garbage "old" value
    unsafe {
        std::ptr::write(
            state,
            State {
                handle: spa_handle {
                    version: SPA_VERSION_HANDLE,
                    get_interface: Some(get_interface::<B>),
                    clear: Some(clear::<B>),
                },

                device: spa_device {
                    iface: spa_interface {
                        type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
                        version: SPA_VERSION_DEVICE,
                        cb: spa_callbacks {
                            funcs: std::ptr::from_ref(&DeviceMethods::<B>::METHODS).cast(),
                            data: state.cast(),
                        },
                    },
                },

                runtime: Runtime {
                    events,

                    snapshot,
                    profile: 1, // default on until a session manager decides otherwise
                    profile_save: false,

                    routes,
                    route_controller,

                    main_loop,
                    system,

                    timer_fd: None,
                    timer_source: LoopSource::new(
                        spa_source {
                            loop_: std::ptr::null_mut(),
                            func: Some(on_route_timeout::<B>),
                            data: state.cast::<c_void>(),
                            fd: -1,
                            mask: SPA_IO_IN,
                            rmask: 0,
                            priv_: std::ptr::null_mut(),
                        },
                        B::DIAGNOSTIC_TAG,
                    ),

                    hotplug_monitor: None,
                    hotplug_source: LoopSource::new(
                        spa_source {
                            loop_: std::ptr::null_mut(),
                            func: Some(on_hotplug_event::<B>),
                            data: state.cast::<c_void>(),
                            fd: -1,
                            mask: SPA_IO_IN,
                            rmask: 0,
                            priv_: std::ptr::null_mut(),
                        },
                        B::DIAGNOSTIC_TAG,
                    ),

                    log,
                },
            },
        );
    }

    unsafe {
        with_runtime_mut(state, |state| {
            let description = state.snapshot.description.clone();
            state.events.with_info(|info| {
                info.fix_pointers();
                info.add_prop(key(SPA_KEY_DEVICE_API), B::DEVICE_API);
                info.add_prop(key(SPA_KEY_MEDIA_CLASS), "Audio/Device");
                if let Some(parent_name) = device_init.parent_name() {
                    info.add_prop(key(SPA_KEY_DEVICE_NAME), parent_name);
                }
                info.add_prop(key(SPA_KEY_DEVICE_DESCRIPTION), description.as_str());
                info.add_param(SPA_PARAM_EnumProfile, SPA_PARAM_INFO_READ);
                info.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READWRITE);
                info.add_param(SPA_PARAM_EnumRoute, SPA_PARAM_INFO_READ);
                info.add_param(SPA_PARAM_Route, SPA_PARAM_INFO_READWRITE);
            });

            arm_route_watch::<B>(state);
        });
    }

    0
}

const INTERFACE_INFO: [spa_interface_info; 1] = [spa_interface_info {
    type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
}];

unsafe extern "C" fn enum_interface_info(
    _factory: *const spa_handle_factory,
    info: *mut *const spa_interface_info,
    index: *mut u32,
) -> c_int {
    assert!(!info.is_null());
    assert!(!index.is_null());
    // non-null asserted above; the caller contract makes both valid and writable
    unsafe {
        match *index {
            0 => {
                *info = &INTERFACE_INFO[0];
                *index += 1;
                1
            }
            _ => 0,
        }
    }
}

const DEVICE_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub const fn factory<B: backend::Backend>(name: *const c_char) -> spa_handle_factory {
    spa_handle_factory {
        version: SPA_VERSION_HANDLE_FACTORY,
        name,
        info: &DEVICE_FACTORY_INFO,
        get_size: Some(get_size::<B>),
        init: Some(init::<B>),
        enum_interface_info: Some(enum_interface_info),
    }
}
