use super::*;
use crate::spa::ListenerList;

struct ReentrantDeviceInfoContext {
    events: *const DeviceEvents,
    seen: Vec<u32>,
}

unsafe extern "C" fn reentrant_device_info(data: *mut c_void, info: *const spa_device_info) {
    let context = unsafe { &mut *data.cast::<ReentrantDeviceInfoContext>() };
    let info = unsafe { &*info };
    let params = unsafe { std::slice::from_raw_parts(info.params, info.n_params as usize) };
    context.seen.push(
        params
            .iter()
            .find(|param| param.id == SPA_PARAM_Profile)
            .expect("Profile is published")
            .flags,
    );
    if context.seen.len() == 1 {
        let events = unsafe { &*context.events };
        events.with_info(|info| info.bump_param(SPA_PARAM_Profile));
        let nested = DeviceNotification::Info(events.take_info());
        // SAFETY: the test owns no State; the endpoint queues this behind
        // the remaining outer notification.
        unsafe { events.dispatch_all(vec![nested]) };
    }
}

#[test]
fn device_notifications_preserve_fifo_order_under_reentry() {
    let events = DeviceEvents::new();
    events.with_info(|info| {
        info.fix_pointers();
        info.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READ);
    });

    let mut context = ReentrantDeviceInfoContext {
        events: &events,
        seen: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.info = Some(reentrant_device_info);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || {
        events.with_info(|info| info.bump_param(SPA_PARAM_Profile));
        let first = DeviceNotification::Info(events.take_info());
        events.with_info(|info| info.bump_param(SPA_PARAM_Profile));
        let second = DeviceNotification::Info(events.take_info());
        unsafe { events.dispatch_all(vec![first, second]) };
    };
    unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        );
    }

    assert_eq!(
        context.seen,
        [
            SPA_PARAM_INFO_READ | SPA_PARAM_INFO_SERIAL,
            SPA_PARAM_INFO_READ,
            SPA_PARAM_INFO_READ | SPA_PARAM_INFO_SERIAL,
        ]
    );
}

struct InitialDeviceContext {
    events: *const DeviceEvents,
    sequence: Vec<&'static str>,
}

unsafe extern "C" fn initial_device_info(data: *mut c_void, _info: *const spa_device_info) {
    let context = unsafe { &mut *data.cast::<InitialDeviceContext>() };
    context.sequence.push("info");
    let events = unsafe { &*context.events };
    unsafe {
        events.dispatch_all(vec![DeviceNotification::Object(
            DeviceObjectEvent::Removed { id: 2 },
        )]);
    }
}

unsafe extern "C" fn initial_device_object(
    data: *mut c_void,
    _id: u32,
    info: *const spa_device_object_info,
) {
    let context = unsafe { &mut *data.cast::<InitialDeviceContext>() };
    context
        .sequence
        .push(if info.is_null() { "removed" } else { "added" });
}

#[test]
fn initial_device_transaction_finishes_before_reentrant_changes() {
    let events = DeviceEvents::new();
    events.with_info(|info| info.fix_pointers());
    let info = events.initial_info();
    let initial_object = DeviceObjectEvent::Added {
        id: 2,
        rec: false,
        description: "Playback".into(),
        route_count: 0,
    };
    let mut context = InitialDeviceContext {
        events: &events,
        sequence: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.info = Some(initial_device_info);
    table.object_info = Some(initial_device_object);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || {
        let dispatch_guard = events.begin_dispatch().expect("the test owns dispatch");
        unsafe {
            events.emit_info(&info);
            events.emit_object(&initial_object);
        }
        dispatch_guard
    };
    let dispatch_guard = unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        )
    };
    unsafe {
        events.drain(dispatch_guard);
    }
    assert_eq!(context.sequence, ["info", "added", "removed"]);
}

struct LateDeviceListener {
    seen: Vec<u32>,
    results: Vec<c_int>,
}

unsafe extern "C" fn record_late_device_object(
    data: *mut c_void,
    id: u32,
    _info: *const spa_device_object_info,
) {
    unsafe { &mut *data.cast::<LateDeviceListener>() }
        .seen
        .push(id);
}

unsafe extern "C" fn record_late_device_result(
    data: *mut c_void,
    seq: c_int,
    _res: c_int,
    _type: u32,
    _result: *const c_void,
) {
    unsafe { &mut *data.cast::<LateDeviceListener>() }
        .results
        .push(seq);
}

struct AddDeviceListenerContext {
    events: *const DeviceEvents,
    late_hook: *mut spa_hook,
    late_table: *const spa_device_events,
    late_data: *mut c_void,
    seen: Vec<u32>,
}

unsafe extern "C" fn add_device_listener_during_dispatch(
    data: *mut c_void,
    id: u32,
    _info: *const spa_device_object_info,
) {
    let context = unsafe { &mut *data.cast::<AddDeviceListenerContext>() };
    context.seen.push(id);
    if context.seen.len() != 1 {
        return;
    }
    let events = unsafe { &*context.events };
    let initial = |hooks: &ListenerList<spa_device_events>| {
        unsafe { events.emit_object_on(hooks, &DeviceObjectEvent::Removed { id: 3 }) };
        let result = spa_result_device_params {
            id: SPA_PARAM_Profile,
            index: 0,
            next: 1,
            param: std::ptr::null_mut(),
        };
        // A synchronous enum_params from this initial callback emits through
        // the endpoint, not the cohort passed to the callback.
        unsafe { events.emit_result(41, &result) };
    };
    unsafe {
        events.with_new_listener(
            context.late_hook,
            context.late_table,
            context.late_data,
            initial,
        );
        events.dispatch_all(vec![DeviceNotification::Object(
            DeviceObjectEvent::Removed { id: 4 },
        )]);
    }
}

#[test]
fn device_listener_added_during_dispatch_starts_at_its_barrier() {
    let events = DeviceEvents::new();
    let mut late = LateDeviceListener {
        seen: Vec::new(),
        results: Vec::new(),
    };
    let mut late_table: spa_device_events = unsafe { std::mem::zeroed() };
    late_table.version = SPA_VERSION_DEVICE_EVENTS;
    late_table.object_info = Some(record_late_device_object);
    late_table.result = Some(record_late_device_result);
    let mut late_hook: spa_hook = unsafe { std::mem::zeroed() };
    let mut context = AddDeviceListenerContext {
        events: &events,
        late_hook: &mut late_hook,
        late_table: &late_table,
        late_data: (&raw mut late).cast(),
        seen: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.object_info = Some(add_device_listener_during_dispatch);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    unsafe {
        events.with_new_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            |_hooks| {},
        );
        events.dispatch_all(vec![
            DeviceNotification::Object(DeviceObjectEvent::Removed { id: 1 }),
            DeviceNotification::Object(DeviceObjectEvent::Removed { id: 2 }),
        ]);
        events.dispatch_all(vec![DeviceNotification::Object(
            DeviceObjectEvent::Removed { id: 5 },
        )]);
    }

    assert_eq!(context.seen, [1, 2, 4, 5]);
    assert_eq!(late.seen, [3, 4, 5]);
    assert_eq!(late.results, [41]);
}

struct DoneBarrierContext {
    events: *const DeviceEvents,
    sequence: Vec<&'static str>,
}

unsafe extern "C" fn done_barrier_info(data: *mut c_void, _info: *const spa_device_info) {
    let context = unsafe { &mut *data.cast::<DoneBarrierContext>() };
    context.sequence.push("info");
    if context.sequence.len() == 1 {
        let events = unsafe { &*context.events };
        unsafe { events.dispatch_all(vec![DeviceNotification::Done(7)]) };
    }
}

unsafe extern "C" fn done_barrier_result(
    data: *mut c_void,
    seq: c_int,
    _res: c_int,
    _type: u32,
    _result: *const c_void,
) {
    assert_eq!(seq, 7);
    unsafe { &mut *data.cast::<DoneBarrierContext>() }
        .sequence
        .push("done");
}

#[test]
fn device_done_does_not_overtake_an_active_transaction() {
    let events = DeviceEvents::new();
    events.with_info(|info| info.fix_pointers());
    let first = DeviceNotification::Info(events.take_info());
    let second = DeviceNotification::Info(events.take_info());
    let mut context = DoneBarrierContext {
        events: &events,
        sequence: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.info = Some(done_barrier_info);
    table.result = Some(done_barrier_result);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || unsafe { events.dispatch_all(vec![first, second]) };
    unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        );
    }
    assert_eq!(context.sequence, ["info", "info", "done"]);
}
