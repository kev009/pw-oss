use libspa::sys::*;
use std::os::raw::{c_char, c_int, c_uint, c_void};

mod events;
mod params;
mod routes;
mod watch;

use events::{DeviceEvents, DeviceNotification, build_object_config, object_events};
use params::{build_profile_info, build_route_info, set_param};
use routes::{
    MixerHandle, ROUTE_CHANNELS, ROUTE_MAP, RouteState, common_description, linear_to_oss,
    oss_to_linear, probe_routes,
};
use watch::{
    arm_mixer_watch, on_devd_event, on_mixer_timeout, queue_object_config, queue_route_change,
    refresh_route_shadow, resolve_recsrc, sync_recsrc,
};

// repr(C): the host casts spa_handle* to State*, so `handle` must stay
// the first field at offset 0.
#[repr(C)]
struct State {
    handle: spa_handle,
    device: spa_device,
    runtime: Runtime,
}

struct Runtime {
    events: std::rc::Rc<DeviceEvents>,
    pcm_devices: Vec<crate::oss::PcmDevice>,
    description: String,
    profile: u32,       // 0 = off, 1 = default
    profile_save: bool, // echoed back in the Profile pod
    routes: Vec<RouteState>,
    mixers: Vec<MixerHandle>,
    main_loop: Option<crate::spa::Loop>, // for the mixer poll timer
    system: Option<crate::spa::System>,  // ditto
    timer_fd: Option<crate::spa::TimerFd>, // owns the LoopSource fd mirror
    timer_source: crate::spa::LoopSource,
    devd_socket: Option<crate::freebsd::DevdSocket>, // jack/default-unit nudges; None = poll only
    devd_source: crate::spa::LoopSource,
    log: crate::spa::Log,
}

// Project only the mutable runtime payload. The host-visible handle and
// interface stay outside every callback borrow, so listener reentry cannot
// overlap a broad &mut State.
unsafe fn with_runtime_mut<R>(
    state: *mut State,
    apply: impl for<'a> FnOnce(&'a mut Runtime) -> R,
) -> R {
    assert!(!state.is_null(), "state is not supposed to be null");
    let runtime = unsafe { &mut *std::ptr::addr_of_mut!((*state).runtime) };
    apply(runtime)
}

unsafe fn with_runtime_ref<R>(
    state: *const State,
    apply: impl for<'a> FnOnce(&'a Runtime) -> R,
) -> R {
    assert!(!state.is_null(), "state is not supposed to be null");
    let runtime = unsafe { &*std::ptr::addr_of!((*state).runtime) };
    apply(runtime)
}

unsafe extern "C" fn add_listener(
    object: *mut c_void,
    listener: *mut spa_hook,
    events: *const spa_device_events,
    data: *mut c_void,
) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let (device_events, objects) = {
        unsafe {
            with_runtime_ref(state, |state| {
                (
                    state.events.clone(),
                    object_events(
                        &state.pcm_devices,
                        &state.routes,
                        &state.description,
                        state.profile != 0,
                    ),
                )
            })
        }
    };

    let initial = |hooks: &crate::spa::ListenerList<spa_device_events>| {
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

unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { with_runtime_ref(state, |state| state.events.clone()) };
    // SAFETY: only the independent endpoint remains borrowed. Done joins the
    // same FIFO as info/object transactions, so reentrant sync cannot overtake
    // already-produced state notifications.
    unsafe { events.dispatch_all(vec![DeviceNotification::Done(seq)]) };

    0
}

unsafe extern "C" fn enum_params(
    object: *mut c_void,
    seq: c_int,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { with_runtime_ref(state, |state| state.events.clone()) };
    let runtime = unsafe { std::ptr::addr_of_mut!((*state).runtime) };

    unsafe {
        crate::spa::enum_params_loop(
            runtime,
            (start, max),
            filter,
            |state, index| {
                use crate::spa::ParamStep;
                // only the active route becomes a Route pod; inactive selectable sources
                // exist as EnumRoute only (acp emits one Route per device with the
                // active port's index, alsa-acp-device.c:582-600)
                if id == SPA_PARAM_Route
                    && (index as usize) < state.routes.len()
                    && !state.routes[index as usize].active
                {
                    return ParamStep::Skip;
                }

                #[allow(non_upper_case_globals)]
                match (id, index) {
                    (SPA_PARAM_EnumProfile, 0 | 1) => ParamStep::Built(build_profile_info(
                        SPA_PARAM_EnumProfile,
                        index,
                        &state.pcm_devices,
                        state.profile_save,
                        false,
                    )),
                    (SPA_PARAM_Profile, 0) => ParamStep::Built(build_profile_info(
                        SPA_PARAM_Profile,
                        state.profile,
                        &state.pcm_devices,
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

const DEVICE_IMPL: spa_device_methods = spa_device_methods {
    version: SPA_VERSION_DEVICE_METHODS,
    add_listener: Some(add_listener),
    sync: Some(sync),
    enum_params: Some(enum_params),
    set_param: Some(set_param),
};

unsafe extern "C" fn get_interface(
    handle: *mut spa_handle,
    type_: *const c_char,
    interface: *mut *mut c_void,
) -> c_int {
    let state = handle.cast::<State>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    assert!(!interface.is_null());
    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Device.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe {
            *interface = std::ptr::addr_of_mut!((*state).device).cast::<c_void>();
        }
    } else {
        return -libc::ENOENT;
    }
    0
}

unsafe extern "C" fn clear(handle: *mut spa_handle) -> c_int {
    let state = handle.cast::<State>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    unsafe {
        with_runtime_mut(state, |runtime| {
            // clear runs on the main loop's thread, so detach both sources there.
            if runtime.timer_source.is_registered() {
                if runtime.timer_source.unregister() < 0 {
                    eprintln!("freebsd-oss: can't detach the mixer timer source; aborting");
                    std::process::abort();
                }
                drop(runtime.timer_fd.take());
                runtime.timer_source.set_fd(-1);
            }
            if runtime.devd_source.is_registered() && runtime.devd_source.unregister() < 0 {
                eprintln!("freebsd-oss: can't detach the devd source; aborting");
                std::process::abort();
            }
            if !runtime.devd_source.is_registered() {
                runtime.devd_source.set_fd(-1);
            }
        });
    }
    // the host frees the memory after clear; drop the fields exactly once here
    unsafe { std::ptr::drop_in_place(state) };
    0
}

extern "C" fn get_size(_factory: *const spa_handle_factory, _params: *const spa_dict) -> usize {
    std::mem::size_of::<State>()
}

// loosely mirror acp's analog input ordering: mic on top, then line, then
// the rest in a stable bit-derived order

unsafe fn parse_device_dict(info: *const spa_dict) -> (Option<String>, Vec<u32>) {
    let mut pcm_parent_device = None;
    let mut pcm_device_indexes = vec![];

    if let Some(info) = unsafe { info.as_ref() } {
        #[cfg(debug_assertions)]
        unsafe {
            crate::spa::dump_spa_dict(info);
        }

        unsafe {
            crate::spa::for_each_dict_item(info, |key, value| match key {
                crate::keys::PCM_PARENT_DEVICE => {
                    pcm_parent_device = Some(value.to_string());
                }
                crate::keys::PCM_DEVICE_INDEXES => {
                    for part in value.split(',') {
                        if let Ok(index) = part.parse::<u32>() {
                            pcm_device_indexes.push(index);
                        }
                    }
                }
                _ => (),
            });
        }
    }

    (pcm_parent_device, pcm_device_indexes)
}

// the device description shared by every aggregated pcm unit: the longest
// common prefix of their descriptions, trimmed of a dangling " (" tail.
// `pcm_devices` must be non-empty (init rejects an empty list first).
unsafe extern "C" fn init(
    _factory: *const spa_handle_factory,
    handle: *mut spa_handle,
    info: *const spa_dict,
    support: *const spa_support,
    n_support: u32,
) -> c_int {
    // the support array is the host's init contract: n_support valid entries
    let log =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast()) }
            as *mut spa_log;
    let log =
        unsafe { crate::spa::Log::wrap(log, std::ptr::NonNull::new(&raw mut OSS_DEVICE_TOPIC)) };

    // the main loop and system drive the mixer poll timer; both are optional -
    // without them external mixer changes just go unnoticed
    let main_loop =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast()) }
            as *mut spa_loop;
    let system = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_System.as_ptr().cast(),
        )
    } as *mut spa_system;
    let main_loop = if main_loop.is_null() {
        None
    } else {
        Some(unsafe { crate::spa::Loop::wrap(main_loop) })
    };
    let system = if system.is_null() {
        None
    } else {
        Some(unsafe { crate::spa::System::wrap(system) })
    };

    let state = handle.cast::<State>();
    assert!(!state.is_null(), "handle is not supposed to be null");

    let (pcm_parent_device, pcm_device_indexes) = unsafe { parse_device_dict(info) };

    if pcm_device_indexes.is_empty() {
        crate::error!(
            log,
            "{} should contain pcm device indexes",
            crate::keys::PCM_DEVICE_INDEXES
        );
        return -libc::EINVAL;
    }

    let pcm_devices = crate::oss::list_pcm_devices(&pcm_device_indexes);

    if pcm_devices.is_empty() {
        crate::error!(log, "can't retrieve pcm device information");
        return -libc::EINVAL;
    }

    let (routes, mixers) = probe_routes(&pcm_devices);
    let common_desc = common_description(&pcm_devices);
    let events = std::rc::Rc::new(DeviceEvents::new());

    // the host hands us uninitialized memory of get_size() bytes; write the
    // whole State without dropping the garbage "old" value
    unsafe {
        std::ptr::write(
            state,
            State {
                handle: spa_handle {
                    version: SPA_VERSION_HANDLE,
                    get_interface: Some(get_interface),
                    clear: Some(clear),
                },

                device: spa_device {
                    iface: spa_interface {
                        type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
                        version: SPA_VERSION_DEVICE,
                        cb: spa_callbacks {
                            funcs: &DEVICE_IMPL as *const _ as *const c_void,
                            data: state as *mut _ as *mut c_void,
                        },
                    },
                },

                runtime: Runtime {
                    events,

                    pcm_devices,
                    description: common_desc,
                    profile: 1, // default on until a session manager decides otherwise
                    profile_save: false,

                    routes,
                    mixers,

                    main_loop,
                    system,

                    timer_fd: None,
                    timer_source: crate::spa::LoopSource::new(spa_source {
                        loop_: std::ptr::null_mut(),
                        func: Some(on_mixer_timeout),
                        data: state.cast::<c_void>(),
                        fd: -1,
                        mask: SPA_IO_IN,
                        rmask: 0,
                        priv_: std::ptr::null_mut(),
                    }),

                    devd_socket: None,
                    devd_source: crate::spa::LoopSource::new(spa_source {
                        loop_: std::ptr::null_mut(),
                        func: Some(on_devd_event),
                        data: state.cast::<c_void>(),
                        fd: -1,
                        mask: SPA_IO_IN,
                        rmask: 0,
                        priv_: std::ptr::null_mut(),
                    }),

                    log,
                },
            },
        );
    }

    unsafe {
        with_runtime_mut(state, |state| {
            let description = state.description.clone();
            state.events.with_info(|info| {
                info.fix_pointers();
                info.add_prop(crate::spa::key(SPA_KEY_DEVICE_API), "freebsd-oss");
                info.add_prop(crate::spa::key(SPA_KEY_MEDIA_CLASS), "Audio/Device");
                if let Some(pcm_parent_device) = pcm_parent_device {
                    info.add_prop(crate::spa::key(SPA_KEY_DEVICE_NAME), pcm_parent_device);
                }
                info.add_prop(
                    crate::spa::key(SPA_KEY_DEVICE_DESCRIPTION),
                    description.as_str(),
                );
                info.add_param(SPA_PARAM_EnumProfile, SPA_PARAM_INFO_READ);
                info.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READWRITE);
                info.add_param(SPA_PARAM_EnumRoute, SPA_PARAM_INFO_READ);
                info.add_param(SPA_PARAM_Route, SPA_PARAM_INFO_READWRITE);
            });

            arm_mixer_watch(state);
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

const OSS_DEVICE_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub(crate) const OSS_DEVICE_FACTORY: spa_handle_factory = spa_handle_factory {
    version: SPA_VERSION_HANDLE_FACTORY,
    name: c"freebsd-oss.device".as_ptr(),
    info: &OSS_DEVICE_FACTORY_INFO,
    get_size: Some(get_size),
    init: Some(init),
    enum_interface_info: Some(enum_interface_info),
};

// mut: the host logger writes level/has_custom_level back after registration
pub(crate) static mut OSS_DEVICE_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: c"spa.oss.device".as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
