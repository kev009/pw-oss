use super::*;
use crate::node::sink::SinkDir;

struct ReentrantInfoContext {
    events: *const NodeEvents<SinkDir>,
    seen: Vec<u32>,
}

unsafe extern "C" fn reentrant_info(data: *mut c_void, info: *const spa_node_info) {
    let context = unsafe { &mut *data.cast::<ReentrantInfoContext>() };
    context.seen.push(unsafe { (*info).max_input_ports });
    if context.seen.len() == 1 {
        let events = unsafe { &*context.events };
        events.with_node_info(|info| info.set_max_input_ports(3));
        events.queue_node_info();
        // SAFETY: this callback holds no State reference; it simulates a
        // reentrant node method flushing its independently owned event.
        unsafe { events.flush() };
    }
}

#[test]
fn node_notifications_preserve_fifo_order_under_reentrant_flush() {
    let events = NodeEvents::<SinkDir>::new();
    events.with_info(|node, port| {
        node.fix_pointers();
        node.add_param(SPA_PARAM_Props, SPA_PARAM_INFO_READ);
        port.fix_pointers();
    });
    let mut context = ReentrantInfoContext {
        events: std::rc::Rc::as_ptr(&events),
        seen: Vec::new(),
    };
    let mut table: spa_node_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_NODE_EVENTS;
    table.info = Some(reentrant_info);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || {
        events.with_node_info(|info| info.set_max_input_ports(1));
        events.queue_node_info();
        events.with_node_info(|info| info.set_max_input_ports(2));
        events.queue_node_info();
        // SAFETY: the test owns no State; only the independent endpoint
        // is live during outer and nested callbacks.
        unsafe { events.flush() };
    };
    unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        );
    }
    assert_eq!(context.seen, [1, 2, 3]);
}

struct LateNodeListener {
    seen: Vec<u32>,
}

unsafe extern "C" fn record_late_node_info(data: *mut c_void, info: *const spa_node_info) {
    unsafe { &mut *data.cast::<LateNodeListener>() }
        .seen
        .push(unsafe { (*info).max_input_ports });
}

struct AddNodeListenerContext {
    events: *const NodeEvents<SinkDir>,
    late_hook: *mut spa_hook,
    late_table: *const spa_node_events,
    late_data: *mut c_void,
    seen: Vec<u32>,
}

unsafe extern "C" fn add_node_listener_during_dispatch(
    data: *mut c_void,
    info: *const spa_node_info,
) {
    let context = unsafe { &mut *data.cast::<AddNodeListenerContext>() };
    context.seen.push(unsafe { (*info).max_input_ports });
    if context.seen.len() != 1 {
        return;
    }
    let events = unsafe { &*context.events };
    events.with_node_info(|info| info.set_max_input_ports(3));
    let initial = |hooks: &crate::spa::ListenerList<spa_node_events>| {
        let (node, _port) = events.initial_snapshots();
        hooks.emit(|f, data| {
            if let Some(info) = f.info {
                unsafe { info(data, node.raw()) };
            }
        });
    };
    unsafe {
        events.with_new_listener(
            context.late_hook,
            context.late_table,
            context.late_data,
            initial,
        );
    }
    // This notification was created after the activation barrier and must
    // reach the new listener; the already-queued value 2 must not.
    events.with_node_info(|info| info.set_max_input_ports(4));
    events.queue_node_info();
}

#[test]
fn node_listener_added_during_dispatch_starts_at_its_barrier() {
    let events = NodeEvents::<SinkDir>::new();
    events.with_info(|node, port| {
        node.fix_pointers();
        port.fix_pointers();
    });
    let mut late = LateNodeListener { seen: Vec::new() };
    let mut late_table: spa_node_events = unsafe { std::mem::zeroed() };
    late_table.version = SPA_VERSION_NODE_EVENTS;
    late_table.info = Some(record_late_node_info);
    let mut late_hook: spa_hook = unsafe { std::mem::zeroed() };
    let mut context = AddNodeListenerContext {
        events: std::rc::Rc::as_ptr(&events),
        late_hook: &mut late_hook,
        late_table: &late_table,
        late_data: (&raw mut late).cast(),
        seen: Vec::new(),
    };
    let mut table: spa_node_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_NODE_EVENTS;
    table.info = Some(add_node_listener_during_dispatch);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    unsafe {
        events.with_new_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            |_hooks| {},
        );
    }

    events.with_node_info(|info| info.set_max_input_ports(1));
    events.queue_node_info();
    events.with_node_info(|info| info.set_max_input_ports(2));
    events.queue_node_info();
    unsafe { events.flush() };
    events.with_node_info(|info| info.set_max_input_ports(5));
    events.queue_node_info();
    unsafe { events.flush() };

    assert_eq!(context.seen, [1, 2, 4, 5]);
    assert_eq!(
        late.seen,
        [3, 4, 5],
        "initial state, post-barrier change, then later change"
    );
}

struct ReentrantDoneContext {
    events: *const NodeEvents<SinkDir>,
    order: Vec<i32>,
}

unsafe extern "C" fn info_queues_done(data: *mut c_void, info: *const spa_node_info) {
    let context = unsafe { &mut *data.cast::<ReentrantDoneContext>() };
    context
        .order
        .push(unsafe { (*info).max_input_ports } as i32);
    if context.order.len() == 1 {
        let events = unsafe { &*context.events };
        // Reentrant sync: it must append behind the already queued second
        // info snapshot, not overtake the active transaction.
        unsafe { events.emit_done(7) };
    }
}

unsafe extern "C" fn record_done(
    data: *mut c_void,
    seq: c_int,
    _res: c_int,
    _type: u32,
    _result: *const c_void,
) {
    unsafe { &mut *data.cast::<ReentrantDoneContext>() }
        .order
        .push(-seq);
}

#[test]
fn node_done_does_not_overtake_an_active_transaction() {
    let events = NodeEvents::<SinkDir>::new();
    events.with_node_info(|node| node.fix_pointers());
    let mut context = ReentrantDoneContext {
        events: std::rc::Rc::as_ptr(&events),
        order: Vec::new(),
    };
    let mut table: spa_node_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_NODE_EVENTS;
    table.info = Some(info_queues_done);
    table.result = Some(record_done);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || {
        events.with_node_info(|info| info.set_max_input_ports(1));
        events.queue_node_info();
        events.with_node_info(|info| info.set_max_input_ports(2));
        events.queue_node_info();
        unsafe { events.flush() };
    };
    unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        );
    }
    assert_eq!(context.order, [1, 2, -7]);
}

#[test]
fn node_dispatch_claim_releases_on_unwind() {
    let events = NodeEvents::<SinkDir>::new();
    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _claim = events.begin_dispatch().expect("first claim");
        panic!("injected dispatch panic");
    }));
    assert!(panicked.is_err());
    assert!(
        events.begin_dispatch().is_some(),
        "the unwind guard must release dispatch ownership"
    );
}

fn published_port_param_flags(events: &NodeEvents<SinkDir>, id: u32) -> u32 {
    let (_node, port) = events.initial_snapshots();
    let raw = unsafe { &*port.raw() };
    let params = unsafe { std::slice::from_raw_parts(raw.params, raw.n_params as usize) };
    params
        .iter()
        .find(|param| param.id == id)
        .map_or(0, |param| param.flags)
}

#[test]
fn format_loss_epoch_rejects_stale_delivery_but_survives_suspend() {
    let events = NodeEvents::<SinkDir>::new();
    events.with_port_info(|port| {
        port.fix_pointers();
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
        port.add_param(SPA_PARAM_Buffers, 0);
        port.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
        port.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
    });
    events.advance_format_publication_epoch();
    let failed_epoch = events.format_publication_epoch();
    let shared = NodeShared::<SinkDir>::new();
    let target = MainEventTarget::new(&events, shared.alive_token());

    // A newer successful format publication retires the delayed loss.
    events.advance_format_publication_epoch();
    unsafe {
        target.deliver_on_main(MainEvent::FormatLost {
            expected_publication_epoch: failed_epoch,
        });
    }
    assert_eq!(
        published_port_param_flags(&events, SPA_PARAM_Format),
        SPA_PARAM_INFO_READWRITE
    );
    assert_eq!(
        published_port_param_flags(&events, SPA_PARAM_Buffers),
        SPA_PARAM_INFO_READ
    );

    // Suspend changes the device generation but not this publication
    // epoch. A current loss must therefore still be applied.
    let current_epoch = events.format_publication_epoch();
    unsafe {
        target.deliver_on_main(MainEvent::FormatLost {
            expected_publication_epoch: current_epoch,
        });
    }
    assert_eq!(
        published_port_param_flags(&events, SPA_PARAM_Format),
        SPA_PARAM_INFO_WRITE
    );
    assert_eq!(published_port_param_flags(&events, SPA_PARAM_Buffers), 0);
}

#[test]
fn synchronous_format_loss_retires_a_same_epoch_deferred_loss() {
    let events = NodeEvents::<SinkDir>::new();
    events.with_port_info(|port| {
        port.fix_pointers();
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
        port.add_param(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
    });
    events.advance_format_publication_epoch();
    let old_epoch = events.format_publication_epoch();

    // This is the synchronous props-rebuild failure path: it queues the
    // loss snapshot before its caller flushes notifications.
    events.record_current_format_lost();
    let loss_epoch = events.format_publication_epoch();
    assert_eq!(loss_epoch, old_epoch.wrapping_add(1));

    // A data-path loss already queued against the old format must now be
    // inert, rather than toggling the EnumFormat serial a second time.
    let shared = NodeShared::<SinkDir>::new();
    let target = MainEventTarget::new(&events, shared.alive_token());
    unsafe {
        target.deliver_on_main(MainEvent::FormatLost {
            expected_publication_epoch: old_epoch,
        });
    }
    assert_eq!(events.format_publication_epoch(), loss_epoch);
}

struct DropNodeEventsOwner {
    owner: *mut Option<std::rc::Rc<NodeEvents<SinkDir>>>,
    weak: std::rc::Weak<NodeEvents<SinkDir>>,
    shared: std::sync::Arc<NodeShared<SinkDir>>,
    calls: usize,
    strong_count_after_drop: usize,
}

unsafe extern "C" fn drop_node_events_owner(
    data: *mut c_void,
    _direction: spa_direction,
    _port_id: u32,
    _info: *const spa_port_info,
) {
    let context = unsafe { &mut *data.cast::<DropNodeEventsOwner>() };
    context.calls += 1;
    // Model a listener synchronously destroying the node and dropping State's
    // sole main-thread owner. Deferred delivery must retain its own Rc first.
    context.shared.close();
    drop(unsafe { &mut *context.owner }.take());
    context.strong_count_after_drop = context.weak.strong_count();
}

#[test]
fn deferred_node_event_survives_reentrant_owner_drop() {
    let events = NodeEvents::<SinkDir>::new();
    events.with_port_info(|port| {
        port.fix_pointers();
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
        port.add_param(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
    });

    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new());
    let target = MainEventTarget::new(&events, shared.alive_token());
    let weak = std::rc::Rc::downgrade(&events);
    let epoch = events.format_publication_epoch();
    let mut owner = Some(events);
    let mut context = DropNodeEventsOwner {
        owner: &raw mut owner,
        weak: weak.clone(),
        shared: shared.clone(),
        calls: 0,
        strong_count_after_drop: 0,
    };
    let mut table: spa_node_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_NODE_EVENTS;
    table.port_info = Some(drop_node_events_owner);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let registration_owner = weak.upgrade().expect("test endpoint");
    unsafe {
        registration_owner.with_new_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            |_hooks| {},
        );
    }
    drop(registration_owner);

    unsafe {
        target.deliver_on_main(MainEvent::FormatLost {
            expected_publication_epoch: epoch,
        });
    }

    assert!(owner.is_none());
    assert!(!shared.is_alive());
    assert_eq!(context.calls, 1);
    assert_eq!(
        context.strong_count_after_drop, 1,
        "delivery must own the endpoint while listeners run"
    );
    assert_eq!(
        weak.strong_count(),
        0,
        "delivery releases its temporary owner afterward"
    );
}
