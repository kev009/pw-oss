use libspa::sys::*;
use std::collections::BTreeMap;
use std::os::raw::{c_char, c_int, c_void};
use std::string::String;
use std::vec::Vec;

struct MonitorEvents {
    hooks: crate::spa::ListenerList<spa_device_events>,
    pending: crate::spa::LocalNotificationQueue<MonitorNotification>,
}

enum MonitorObjectEvent {
    Added {
        id: u32,
        driver: String,
        indexes: Vec<u32>,
    },
    Removed {
        id: u32,
    },
}

enum MonitorNotification {
    Object(MonitorObjectEvent),
    ActivateListeners(std::rc::Rc<crate::spa::ListenerList<spa_device_events>>),
}

impl MonitorEvents {
    fn new() -> Self {
        Self {
            hooks: crate::spa::ListenerList::new(),
            pending: crate::spa::LocalNotificationQueue::new(),
        }
    }

    // SAFETY: no reference into the associated State may be live; listeners
    // may synchronously re-enter monitor methods. `hooks` is the active list
    // or an isolated activation batch with the same event-table type.
    unsafe fn emit_info_on(&self, hooks: &crate::spa::ListenerList<spa_device_events>) {
        let info = spa_device_info {
            version: SPA_VERSION_DEVICE_INFO,
            change_mask: crate::spa::SPA_DEVICE_CHANGE_MASK_ALL as u64,
            flags: 0,
            props: &DEV_INFO_PROPS,
            params: std::ptr::null_mut(),
            n_params: 0,
        };
        hooks.emit(|f, data| {
            if let Some(device_info) = f.info {
                unsafe { device_info(data, &info) };
            }
        });
    }

    // SAFETY: as emit_info_on().
    unsafe fn emit_object(&self, event: &MonitorObjectEvent) {
        unsafe { self.emit_object_on(&self.hooks, event) };
    }

    // SAFETY: as emit_info_on().
    unsafe fn emit_object_on(
        &self,
        hooks: &crate::spa::ListenerList<spa_device_events>,
        event: &MonitorObjectEvent,
    ) {
        match event {
            MonitorObjectEvent::Removed { id } => hooks.emit(|f, data| {
                if let Some(object_info) = f.object_info {
                    unsafe { object_info(data, *id, std::ptr::null()) };
                }
            }),
            MonitorObjectEvent::Added {
                id,
                driver,
                indexes,
            } => {
                let indexes_str = indexes
                    .iter()
                    .map(|i| format!("{i}"))
                    .collect::<Vec<_>>()
                    .join(",");
                let mut dict = crate::spa::Dictionary::new();
                dict.add_item(crate::keys::PCM_PARENT_DEVICE, driver.as_str());
                dict.add_item(crate::keys::PCM_DEVICE_INDEXES, indexes_str);
                let info = spa_device_object_info {
                    version: SPA_VERSION_DEVICE_OBJECT_INFO,
                    type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
                    factory_name: c"freebsd-oss.device".as_ptr(),
                    change_mask: crate::spa::SPA_DEVICE_OBJECT_CHANGE_MASK_ALL as u64,
                    flags: 0,
                    props: dict.raw(),
                };
                hooks.emit(|f, data| {
                    if let Some(object_info) = f.object_info {
                        unsafe { object_info(data, *id, &info) };
                    }
                });
            }
        }
    }

    // SAFETY: as emit_info_on().
    unsafe fn emit_objects_on(
        &self,
        hooks: &crate::spa::ListenerList<spa_device_events>,
        events: &[MonitorObjectEvent],
    ) {
        for event in events {
            unsafe { self.emit_object_on(hooks, event) };
        }
    }

    // Enqueue activation before synchronous initial callbacks whenever older
    // FIFO work exists. Those object notifications stay ahead of the listener;
    // callback-generated changes land after its barrier.
    unsafe fn with_new_listener<R>(
        &self,
        listener: *mut spa_hook,
        events: *const spa_device_events,
        data: *mut c_void,
        initial: impl FnOnce(&crate::spa::ListenerList<spa_device_events>) -> R,
    ) -> R {
        let deferred = self.pending.defer_when_busy(|| {
            let hooks = std::rc::Rc::new(crate::spa::ListenerList::new());
            (MonitorNotification::ActivateListeners(hooks.clone()), hooks)
        });
        let hooks = deferred.as_deref().unwrap_or(&self.hooks);
        unsafe { hooks.with_isolated_listener(listener, events, data, || initial(hooks)) }
    }

    fn begin_dispatch(&self) -> Option<crate::spa::LocalDispatchGuard<'_, MonitorNotification>> {
        self.pending.begin_dispatch()
    }

    // SAFETY: no State reference may be live; only the begin_dispatch owner
    // calls this, and the RefCell borrow ends before listener dispatch.
    unsafe fn drain(&self, guard: crate::spa::LocalDispatchGuard<'_, MonitorNotification>) {
        self.pending.drain(guard, |notification| {
            match notification {
                MonitorNotification::Object(event) => unsafe {
                    self.emit_object(&event);
                },
                MonitorNotification::ActivateListeners(hooks) => {
                    // SAFETY: barriers are drained between traversals.
                    unsafe { self.hooks.append_from(&hooks) };
                }
            }
        });
    }

    // SAFETY: no associated State reference may be live.
    unsafe fn dispatch_all(&self, notifications: Vec<MonitorObjectEvent>) {
        self.pending.dispatch_all(
            notifications.into_iter().map(MonitorNotification::Object),
            |notification| match notification {
                MonitorNotification::Object(event) => unsafe {
                    self.emit_object(&event);
                },
                MonitorNotification::ActivateListeners(hooks) => {
                    unsafe { self.hooks.append_from(&hooks) };
                }
            },
        );
    }
}

// repr(C): the host casts spa_handle* to State*, so `handle` must stay
// the first field at offset 0
#[repr(C)]
struct State {
    handle: spa_handle,
    device: spa_device,
    events: std::rc::Rc<MonitorEvents>,
    pcm_indexes: BTreeMap<String, Vec<u32>>,
    main_loop: crate::spa::Loop,
    devd_socket: Option<crate::utils::DevdSocket>, // None when devd isn't running (no hotplug)
    devd_source: spa_source,
    log: crate::spa::Log,
}

unsafe extern "C" fn add_listener(
    object: *mut c_void,
    listener: *mut spa_hook,
    events: *const spa_device_events,
    data: *mut c_void,
) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let (monitor_events, objects) = {
        let state = unsafe { &*state };
        (
            state.events.clone(),
            state
                .pcm_indexes
                .iter()
                .filter_map(|(driver, indexes)| {
                    Some(MonitorObjectEvent::Added {
                        id: *indexes.first()?,
                        driver: driver.clone(),
                        indexes: indexes.clone(),
                    })
                })
                .collect::<Vec<_>>(),
        )
    };

    let initial = |hooks: &crate::spa::ListenerList<spa_device_events>| {
        // The initial emissions only reach the newly added listener (the list
        // is isolated). One method per traversal, mirroring C's
        // spa_hook_list_call: a listener that removes and frees its hook
        // inside a callback must not be read for the next method.
        let dispatch_guard = monitor_events.begin_dispatch();
        // SAFETY: the State snapshot borrow ended before dispatch.
        unsafe {
            monitor_events.emit_info_on(hooks);
            monitor_events.emit_objects_on(hooks, &objects);
        }
        dispatch_guard
    };
    let dispatch_guard =
        unsafe { monitor_events.with_new_listener(listener, events, data, initial) };
    if let Some(guard) = dispatch_guard {
        unsafe { monitor_events.drain(guard) };
    }
    0
}

const DEVICE_IMPL: spa_device_methods = spa_device_methods {
    version: SPA_VERSION_DEVICE_METHODS,
    add_listener: Some(add_listener),
    sync: None,
    enum_params: None,
    set_param: None,
};

unsafe extern "C" fn get_interface(
    handle: *mut spa_handle,
    type_: *const c_char,
    interface: *mut *mut c_void,
) -> c_int {
    let state =
        unsafe { handle.cast::<State>().as_mut() }.expect("handle is not supposed to be null");

    assert!(!interface.is_null());

    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Device.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe { *interface = &mut state.device as *mut _ as *mut c_void };
    } else {
        return -libc::ENOENT;
    }

    0
}

unsafe extern "C" fn clear(handle: *mut spa_handle) -> c_int {
    let state =
        unsafe { handle.cast::<State>().as_mut() }.expect("handle is not supposed to be null");
    // clear runs on the main loop's thread, so detach the devd source directly;
    // the socket fd itself is closed by the DevdSocket drop below
    if state.devd_socket.is_some() {
        unsafe { state.main_loop.remove_source(&mut state.devd_source) };
    }
    // the host frees the memory after clear; drop the fields exactly once here
    unsafe { std::ptr::drop_in_place(state) };
    0
}

extern "C" fn get_size(_factory: *const spa_handle_factory, _params: *const spa_dict) -> usize {
    std::mem::size_of::<State>()
}

const DEV_INFO_PROPS: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

unsafe extern "C" fn on_devd_event(source: *mut spa_source) {
    let state: *mut State = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        let state = unsafe { &mut *state };
        let Some(devd_socket) = state.devd_socket.as_mut() else {
            return;
        };

        // Any pcm (or its uaudio parent) attach/detach can change the device
        // set. Resync and build owned notifications before listener dispatch.
        let mut resync = false;
        let mut detached: Vec<String> = Vec::new();
        let alive = devd_socket.read_event(|line| {
            static RE: std::sync::OnceLock<regex::Regex> = std::sync::OnceLock::new();
            let re = RE.get_or_init(|| regex::Regex::new(r"^([\+-])((?:pcm|uaudio)\d+)").unwrap());
            if let Some(groups) = re.captures(line) {
                resync = true;
                if groups.get(1).unwrap().as_str() == "-" {
                    detached.push(groups.get(2).unwrap().as_str().to_string());
                }
            }
        });

        let notifications = if resync {
            resync_devices(state, &detached)
        } else {
            Vec::new()
        };

        if !alive {
            // devd restarted or dropped us; deregister or the level-triggered
            // fd spins the main loop forever.
            crate::warn!(state.log, "devd connection lost; hotplug disabled");
            unsafe { state.main_loop.remove_source(&mut state.devd_source) };
            state.devd_socket = None;
        }
        (state.events.clone(), notifications)
    };
    // SAFETY: the scoped State mutation ended above.
    unsafe { events.dispatch_all(notifications) };
}

// re-read sndstat, diff the parent->indexes map, and retract/emit whatever
// changed; a parent whose index set changed is retracted and re-emitted so a
// reused unit number never leaves a node bound to the wrong hardware
fn resync_devices(state: &mut State, detached: &[String]) -> Vec<MonitorObjectEvent> {
    let mut notifications = Vec::new();
    // Force-retract every group a detach event names BEFORE diffing: a fast
    // replug can land the '-' event after the replacement re-attached with the
    // same nameunit and index set, leaving the maps identical while the
    // underlying hardware changed; the diff below then re-emits them fresh.
    for subject in detached {
        let key = if let Some(unit) = subject
            .strip_prefix("pcm")
            .and_then(|u| u.parse::<u32>().ok())
        {
            state
                .pcm_indexes
                .iter()
                .find(|(_, v)| v.contains(&unit))
                .map(|(k, _)| k.clone())
        } else if state.pcm_indexes.contains_key(subject) {
            Some(subject.clone())
        } else {
            None
        };
        if let Some(key) = key {
            if let Some(indexes) = state.pcm_indexes.remove(&key) {
                crate::info!(state.log, "removing {} ({:?}) on detach", key, indexes);
                if let Some(&id) = indexes.first() {
                    notifications.push(MonitorObjectEvent::Removed { id });
                }
            }
        }
    }

    let indexes = match crate::sound::read_sndstat() {
        Ok(indexes) => indexes,
        Err(err) => {
            crate::warn!(state.log, "can't re-read sndstat: {}", err);
            return notifications;
        }
    };
    let new_map = crate::sound::group_pcm_devices_by_parent(&indexes);
    let old_map = std::mem::replace(&mut state.pcm_indexes, new_map);

    for (driver, old_indexes) in &old_map {
        if state.pcm_indexes.get(driver) != Some(old_indexes) {
            crate::info!(state.log, "removing {} ({:?})", driver, old_indexes);
            if let Some(&id) = old_indexes.first() {
                notifications.push(MonitorObjectEvent::Removed { id });
            }
        }
    }
    for (driver, new_indexes) in &state.pcm_indexes {
        if old_map.get(driver) != Some(new_indexes) {
            crate::info!(state.log, "registering {} ({:?})", driver, new_indexes);
            if let Some(&id) = new_indexes.first() {
                notifications.push(MonitorObjectEvent::Added {
                    id,
                    driver: driver.clone(),
                    indexes: new_indexes.clone(),
                });
            }
        }
    }
    notifications
}

unsafe extern "C" fn init(
    _factory: *const spa_handle_factory,
    handle: *mut spa_handle,
    _info: *const spa_dict,
    support: *const spa_support,
    n_support: u32,
) -> c_int {
    let log =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast()) }
            as *mut spa_log;
    let log =
        unsafe { crate::spa::Log::wrap(log, std::ptr::NonNull::new(&raw mut OSS_MONITOR_TOPIC)) };

    let main_loop =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast()) }
            as *mut spa_loop;

    if main_loop.is_null() {
        return -libc::EINVAL;
    }

    let main_loop = unsafe { crate::spa::Loop::wrap(main_loop) };

    let state =
        unsafe { handle.cast::<State>().as_mut() }.expect("handle is not supposed to be null");

    let pcm_indexes = match crate::sound::read_sndstat() {
        Ok(indexes) => crate::sound::group_pcm_devices_by_parent(&indexes),
        Err(err) => {
            crate::error!(log, "Can't open /dev/sndstat: {}", err);
            return -(err as c_int);
        }
    };

    // no devd (jails, minimal systems) just means no hotplug
    let devd_socket = match crate::utils::DevdSocket::open() {
        Ok(socket) => Some(socket),
        Err(err) => {
            crate::warn!(log, "can't connect to devd, hotplug disabled: {}", err);
            None
        }
    };

    let devd_source = spa_source {
        loop_: std::ptr::null_mut(),
        func: Some(on_devd_event),
        data: state as *mut _ as *mut c_void,
        fd: devd_socket.as_ref().map(|s| s.fd()).unwrap_or(-1),
        mask: SPA_IO_IN,
        rmask: 0,
        priv_: std::ptr::null_mut(),
    };
    let events = std::rc::Rc::new(MonitorEvents::new());

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

                events,

                pcm_indexes,

                main_loop,
                devd_socket,
                devd_source,

                log,
            },
        );
    }

    if state.devd_socket.is_some() {
        let err = unsafe { state.main_loop.add_source(&mut state.devd_source) };
        if err < 0 {
            // no hotplug then; enumeration still works
            crate::warn!(state.log, "can't watch devd: {}", err);
            state.devd_socket = None;
        }
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

pub(crate) const OSS_MONITOR_FACTORY: spa_handle_factory = spa_handle_factory {
    version: SPA_VERSION_HANDLE_FACTORY,
    name: c"freebsd-oss.monitor".as_ptr(),
    info: std::ptr::null(),
    get_size: Some(get_size),
    init: Some(init),
    enum_interface_info: Some(enum_interface_info),
};

// mut: the host logger writes level/has_custom_level back after registration
pub(crate) static mut OSS_MONITOR_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: c"spa.oss.monitor".as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};

#[cfg(test)]
mod tests {
    use super::*;

    struct LateMonitorListener {
        seen: Vec<u32>,
    }

    unsafe extern "C" fn record_late_monitor_object(
        data: *mut c_void,
        id: u32,
        _info: *const spa_device_object_info,
    ) {
        unsafe { &mut *data.cast::<LateMonitorListener>() }
            .seen
            .push(id);
    }

    struct AddMonitorListenerContext {
        events: *const MonitorEvents,
        late_hook: *mut spa_hook,
        late_table: *const spa_device_events,
        late_data: *mut c_void,
        seen: Vec<u32>,
    }

    unsafe extern "C" fn add_monitor_listener_during_dispatch(
        data: *mut c_void,
        id: u32,
        _info: *const spa_device_object_info,
    ) {
        let context = unsafe { &mut *data.cast::<AddMonitorListenerContext>() };
        context.seen.push(id);
        if context.seen.len() != 1 {
            return;
        }
        let events = unsafe { &*context.events };
        let initial = |hooks: &crate::spa::ListenerList<spa_device_events>| unsafe {
            events.emit_object_on(hooks, &MonitorObjectEvent::Removed { id: 3 });
        };
        unsafe {
            events.with_new_listener(
                context.late_hook,
                context.late_table,
                context.late_data,
                initial,
            );
            events.dispatch_all(vec![MonitorObjectEvent::Removed { id: 4 }]);
        }
    }

    #[test]
    fn monitor_listener_added_during_dispatch_starts_at_its_barrier() {
        let events = MonitorEvents::new();
        let mut late = LateMonitorListener { seen: Vec::new() };
        let mut late_table: spa_device_events = unsafe { std::mem::zeroed() };
        late_table.version = SPA_VERSION_DEVICE_EVENTS;
        late_table.object_info = Some(record_late_monitor_object);
        let mut late_hook: spa_hook = unsafe { std::mem::zeroed() };
        let mut context = AddMonitorListenerContext {
            events: &events,
            late_hook: &mut late_hook,
            late_table: &late_table,
            late_data: (&raw mut late).cast(),
            seen: Vec::new(),
        };
        let mut table: spa_device_events = unsafe { std::mem::zeroed() };
        table.version = SPA_VERSION_DEVICE_EVENTS;
        table.object_info = Some(add_monitor_listener_during_dispatch);
        let mut hook: spa_hook = unsafe { std::mem::zeroed() };
        unsafe {
            events.with_new_listener(
                &mut hook,
                &raw const table,
                (&raw mut context).cast(),
                |_hooks| {},
            );
            events.dispatch_all(vec![
                MonitorObjectEvent::Removed { id: 1 },
                MonitorObjectEvent::Removed { id: 2 },
            ]);
            events.dispatch_all(vec![MonitorObjectEvent::Removed { id: 5 }]);
        }

        assert_eq!(context.seen, [1, 2, 4, 5]);
        assert_eq!(late.seen, [3, 4, 5]);
    }
}
