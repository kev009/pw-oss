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
