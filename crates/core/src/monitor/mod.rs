use crate::spa::{
    Dictionary, ListenerList, LocalDispatchGuard, LocalNotificationQueue, Log, Loop, LoopSource,
    SPA_DEVICE_CHANGE_MASK_ALL, SPA_DEVICE_OBJECT_CHANGE_MASK_ALL,
};
use libspa::sys::*;
use std::ffi::{c_char, c_int, c_void};
use std::vec::Vec;

use crate::backend::{self, DeviceCatalog as _, HotplugMonitor as _};

struct MonitorEvents<B: backend::Backend> {
    hooks: ListenerList<spa_device_events>,
    pending: LocalNotificationQueue<MonitorNotification>,
    backend: std::marker::PhantomData<B>,
}

enum MonitorObjectEvent {
    Added(backend::CatalogGroupSnapshot),
    Removed { id: u32 },
}

enum MonitorNotification {
    Object(MonitorObjectEvent),
    ActivateListeners(std::rc::Rc<ListenerList<spa_device_events>>),
}

impl<B: backend::Backend> MonitorEvents<B> {
    fn new() -> Self {
        Self {
            hooks: ListenerList::new(),
            pending: LocalNotificationQueue::new(),
            backend: std::marker::PhantomData,
        }
    }

    // SAFETY: no reference into the associated State may be live; listeners
    // may synchronously re-enter monitor methods. `hooks` is the active list
    // or an isolated activation batch with the same event-table type.
    unsafe fn emit_info_on(&self, hooks: &ListenerList<spa_device_events>) {
        let info = spa_device_info {
            version: SPA_VERSION_DEVICE_INFO,
            change_mask: SPA_DEVICE_CHANGE_MASK_ALL as u64,
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
        hooks: &ListenerList<spa_device_events>,
        event: &MonitorObjectEvent,
    ) {
        match event {
            MonitorObjectEvent::Removed { id } => hooks.emit(|f, data| {
                if let Some(object_info) = f.object_info {
                    unsafe { object_info(data, *id, std::ptr::null()) };
                }
            }),
            MonitorObjectEvent::Added(snapshot) => {
                let mut dict = Dictionary::new();
                for (key, value) in &snapshot.properties {
                    dict.add_item(key.as_str(), value.as_str());
                }
                let info = spa_device_object_info {
                    version: SPA_VERSION_DEVICE_OBJECT_INFO,
                    type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
                    factory_name: B::DEVICE_FACTORY_NAME.as_ptr(),
                    change_mask: SPA_DEVICE_OBJECT_CHANGE_MASK_ALL as u64,
                    flags: 0,
                    props: dict.raw(),
                };
                hooks.emit(|f, data| {
                    if let Some(object_info) = f.object_info {
                        unsafe { object_info(data, snapshot.object_id, &info) };
                    }
                });
            }
        }
    }

    // SAFETY: as emit_info_on().
    unsafe fn emit_objects_on(
        &self,
        hooks: &ListenerList<spa_device_events>,
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
        initial: impl FnOnce(&ListenerList<spa_device_events>) -> R,
    ) -> R {
        let deferred = self.pending.defer_when_busy(|| {
            let hooks = std::rc::Rc::new(ListenerList::new());
            (MonitorNotification::ActivateListeners(hooks.clone()), hooks)
        });
        let hooks = deferred.as_deref().unwrap_or(&self.hooks);
        unsafe { hooks.with_isolated_listener(listener, events, data, || initial(hooks)) }
    }

    fn begin_dispatch(&self) -> Option<LocalDispatchGuard<'_, MonitorNotification>> {
        self.pending.begin_dispatch()
    }

    // SAFETY: no State reference may be live; only the begin_dispatch owner
    // calls this, and the RefCell borrow ends before listener dispatch.
    unsafe fn drain(&self, guard: LocalDispatchGuard<'_, MonitorNotification>) {
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
struct State<B: backend::Backend> {
    handle: spa_handle,
    device: spa_device,
    runtime: Runtime<B>,
}

struct Runtime<B: backend::Backend> {
    events: std::rc::Rc<MonitorEvents<B>>,
    catalog: B::Catalog,
    main_loop: Loop,
    hotplug_monitor: Option<B::Hotplug>,
    hotplug_source: LoopSource,
    log: Log,
}

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
    let (monitor_events, objects) = {
        unsafe {
            with_runtime_ref(state, |state| {
                (
                    state.events.clone(),
                    state
                        .catalog
                        .snapshots()
                        .into_iter()
                        .map(MonitorObjectEvent::Added)
                        .collect::<Vec<_>>(),
                )
            })
        }
    };

    let initial = |hooks: &ListenerList<spa_device_events>| {
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

struct MonitorMethods<B>(std::marker::PhantomData<B>);

impl<B: backend::Backend> MonitorMethods<B> {
    const METHODS: spa_device_methods = spa_device_methods {
        version: SPA_VERSION_DEVICE_METHODS,
        add_listener: Some(add_listener::<B>),
        sync: None,
        enum_params: None,
        set_param: None,
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
            // clear runs on the registered main loop.
            if runtime.hotplug_source.is_registered() && runtime.hotplug_source.unregister() < 0 {
                eprintln!(
                    "{}: {}",
                    B::DIAGNOSTIC_TAG,
                    B::hotplug_diagnostic(backend::HotplugDiagnostic::MonitorDetachAbort)
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

const DEV_INFO_PROPS: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

unsafe extern "C" fn on_hotplug_event<B: backend::Backend>(source: *mut spa_source) {
    let state: *mut State<B> = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let hotplug_monitor = state.hotplug_monitor.as_mut()?;

                // The selected backend owns native event decoding and
                // replacement matching. Build owned notifications from its
                // neutral catalog diff before listener dispatch.
                let (alive, rescan) = hotplug_monitor.read_catalog_rescan(&mut state.catalog);
                let notifications = rescan
                    .map(|rescan| catalog_notifications::<B>(state, rescan))
                    .unwrap_or_default();

                if !alive {
                    // A closed level-triggered source must be deregistered or
                    // it spins the main loop forever.
                    crate::warn!(
                        state.log,
                        "{}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::MonitorLost)
                    );
                    // SAFETY: this callback runs on the registered main loop.
                    if state.hotplug_source.unregister() < 0 {
                        eprintln!(
                            "{}: {}",
                            B::DIAGNOSTIC_TAG,
                            B::hotplug_diagnostic(backend::HotplugDiagnostic::MonitorDetachAbort)
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
    // SAFETY: the scoped State mutation ended above.
    unsafe { events.dispatch_all(notifications) };
}

// Diff the selected backend catalog and retract/emit whatever changed. A
// parent whose endpoint set changed is retracted and re-emitted so a reused
// endpoint never leaves a node bound to the wrong hardware.
fn catalog_notifications<B: backend::Backend>(
    state: &Runtime<B>,
    rescan: backend::CatalogRescan,
) -> Vec<MonitorObjectEvent> {
    let backend::CatalogRescan { changes, error } = rescan;
    if let Some(err) = error {
        crate::warn!(
            state.log,
            "{}: {}",
            B::Catalog::refresh_error_context(),
            err
        );
    }
    changes
        .into_iter()
        .map(|change| match change {
            backend::CatalogChange::Added {
                snapshot,
                diagnostic,
            } => {
                crate::info!(state.log, "registering {}", diagnostic);
                MonitorObjectEvent::Added(snapshot)
            }
            backend::CatalogChange::Removed {
                object_id,
                diagnostic,
            } => {
                crate::info!(state.log, "removing {}", diagnostic);
                MonitorObjectEvent::Removed { id: object_id }
            }
        })
        .collect()
}

unsafe extern "C" fn init<B: backend::Backend>(
    _factory: *const spa_handle_factory,
    handle: *mut spa_handle,
    _info: *const spa_dict,
    support: *const spa_support,
    n_support: u32,
) -> c_int {
    let log = unsafe {
        spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast())
            .cast::<spa_log>()
    };
    let Some(log) = (unsafe { Log::wrap(log, Some(B::monitor_log_topic())) }) else {
        return -libc::EINVAL;
    };

    let main_loop = unsafe {
        spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast())
            .cast::<spa_loop>()
    };

    if main_loop.is_null() {
        return -libc::EINVAL;
    }

    let main_loop = unsafe { Loop::wrap(main_loop) };

    let state = handle.cast::<State<B>>();
    assert!(!state.is_null(), "handle is not supposed to be null");

    let catalog = match B::Catalog::scan() {
        Ok(catalog) => catalog,
        Err(err) => {
            crate::error!(log, "{}: {}", B::Catalog::open_error_context(), err);
            return -err.code();
        }
    };

    // A missing native event service disables only hotplug; initial
    // enumeration still succeeds.
    let hotplug_monitor = match B::Hotplug::open() {
        Ok(socket) => Some(socket),
        Err(err) => {
            crate::warn!(
                log,
                "{}: {}",
                B::hotplug_diagnostic(backend::HotplugDiagnostic::MonitorOpen),
                err
            );
            None
        }
    };

    let hotplug_source = LoopSource::new(
        spa_source {
            loop_: std::ptr::null_mut(),
            func: Some(on_hotplug_event::<B>),
            data: state.cast::<c_void>(),
            fd: hotplug_monitor
                .as_ref()
                .map(|monitor| monitor.fd())
                .unwrap_or(-1),
            mask: SPA_IO_IN,
            rmask: 0,
            priv_: std::ptr::null_mut(),
        },
        B::DIAGNOSTIC_TAG,
    );
    let events = std::rc::Rc::new(MonitorEvents::new());

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
                            funcs: std::ptr::from_ref(&MonitorMethods::<B>::METHODS).cast(),
                            data: state.cast(),
                        },
                    },
                },

                runtime: Runtime {
                    events,

                    catalog,

                    main_loop,
                    hotplug_monitor,
                    hotplug_source,

                    log,
                },
            },
        );
    }

    unsafe {
        with_runtime_mut(state, |state| {
            if state.hotplug_monitor.is_some() {
                let main_loop = state.main_loop;
                // SAFETY: init uses the host context accepted by add_source; clear
                // unregisters the pinned source before State is dropped.
                let err = state.hotplug_source.register(&main_loop);
                if err < 0 {
                    // no hotplug then; enumeration still works
                    crate::warn!(
                        state.log,
                        "{}: {}",
                        B::hotplug_diagnostic(backend::HotplugDiagnostic::MonitorWatch),
                        err
                    );
                    state.hotplug_monitor = None;
                    state.hotplug_source.set_fd(-1);
                }
            }
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

pub const fn factory<B: backend::Backend>(name: *const c_char) -> spa_handle_factory {
    spa_handle_factory {
        version: SPA_VERSION_HANDLE_FACTORY,
        name,
        info: std::ptr::null(),
        get_size: Some(get_size::<B>),
        init: Some(init::<B>),
        enum_interface_info: Some(enum_interface_info),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::fake::FakeBackend;

    type MonitorEvents = super::MonitorEvents<FakeBackend>;

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
        let initial = |hooks: &ListenerList<spa_device_events>| unsafe {
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
