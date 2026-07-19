use super::*;
use crate::sink::SinkDir;

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
        events: &events,
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
        events: &events,
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
        events: &events,
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
    let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
    events.with_port_info(|port| {
        port.fix_pointers();
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
        port.add_param(SPA_PARAM_Buffers, 0);
        port.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
        port.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
    });
    events.advance_format_publication_epoch();
    let failed_epoch = events.format_publication_epoch();
    let shared = NodeShared::new(std::sync::Arc::downgrade(&events));

    // A newer successful format publication retires the delayed loss.
    events.advance_format_publication_epoch();
    shared.main_event(MainEvent::FormatLost {
        expected_publication_epoch: failed_epoch,
    });
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
    shared.main_event(MainEvent::FormatLost {
        expected_publication_epoch: current_epoch,
    });
    assert_eq!(
        published_port_param_flags(&events, SPA_PARAM_Format),
        SPA_PARAM_INFO_WRITE
    );
    assert_eq!(published_port_param_flags(&events, SPA_PARAM_Buffers), 0);
}

#[test]
fn synchronous_format_loss_retires_a_same_epoch_deferred_loss() {
    let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
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
    let shared = NodeShared::new(std::sync::Arc::downgrade(&events));
    shared.main_event(MainEvent::FormatLost {
        expected_publication_epoch: old_epoch,
    });
    assert_eq!(events.format_publication_epoch(), loss_epoch);
}

#[test]
fn rebuild_worker_runs_off_caller_and_survives_a_panicking_job() {
    let mut worker = RebuildWorker::<SinkDir>::start().expect("worker starts");
    let endpoint = worker.endpoint();
    let caller = std::thread::current().id();
    let (started_tx, started_rx) = std::sync::mpsc::channel();
    assert!(matches!(
        endpoint.try_submit(RebuildWork::Test(Box::new(move || {
            started_tx
                .send(std::thread::current().id())
                .expect("test receiver lives");
            panic!("injected worker panic");
        }))),
        WorkSubmission::Submitted
    ));
    let first_thread = started_rx.recv().expect("the first job ran");
    assert_ne!(first_thread, caller, "blocking work must not run inline");

    let (next_tx, next_rx) = std::sync::mpsc::channel();
    assert!(matches!(
        endpoint.try_submit(RebuildWork::Test(Box::new(move || {
            next_tx
                .send(std::thread::current().id())
                .expect("test receiver lives");
        }))),
        WorkSubmission::Submitted
    ));
    assert_eq!(
        next_rx.recv().expect("the worker survived"),
        first_thread,
        "later work stays on the same owned worker"
    );
    worker.stop();
}

#[test]
fn rebuild_worker_shutdown_destroys_queued_ownership_off_caller() {
    struct DropThread(std::sync::mpsc::Sender<std::thread::ThreadId>);
    impl Drop for DropThread {
        fn drop(&mut self) {
            let _ = self.0.send(std::thread::current().id());
        }
    }

    let mut worker = RebuildWorker::<SinkDir>::start().expect("worker starts");
    let caller = std::thread::current().id();
    let (drop_tx, drop_rx) = std::sync::mpsc::channel();
    let probe = DropThread(drop_tx);
    assert!(matches!(
        worker
            .endpoint()
            .try_submit(RebuildWork::Test(Box::new(move || drop(probe)))),
        WorkSubmission::Submitted
    ));
    worker.stop();
    assert_ne!(
        drop_rx.recv().expect("shutdown drained the probe"),
        caller,
        "queued ownership is destroyed before join, on the worker"
    );
}

#[test]
fn rebuild_takeover_waits_for_an_inflight_worker_operation() {
    let mut worker = RebuildWorker::<SinkDir>::start().expect("worker starts");
    let endpoint = worker.endpoint();
    let (entered_tx, entered_rx) = std::sync::mpsc::channel();
    let (release_tx, release_rx) = std::sync::mpsc::channel();
    assert!(matches!(
        endpoint.try_submit(RebuildWork::Test(Box::new(move || {
            entered_tx.send(()).expect("test receiver lives");
            release_rx.recv().expect("the fake open is released");
        }))),
        WorkSubmission::Submitted
    ));
    entered_rx.recv().expect("the fake open is in flight");

    let (idle_tx, idle_rx) = std::sync::mpsc::channel();
    let waiter = std::thread::spawn(move || {
        idle_tx
            .send(endpoint.wait_idle())
            .expect("test receiver lives");
    });
    assert!(
        idle_rx
            .recv_timeout(std::time::Duration::from_millis(20))
            .is_err(),
        "takeover must not cross a taken but unfinished worker command"
    );
    release_tx.send(()).expect("release the fake open");
    assert!(
        idle_rx.recv().expect("takeover completed"),
        "a live worker reaches idle"
    );
    waiter.join().expect("takeover waiter stays sound");
    worker.stop();
}

#[test]
fn rebuild_wait_idle_covers_a_producer_mid_publication() {
    let mut worker = RebuildWorker::<SinkDir>::start().expect("worker starts");
    let endpoint = worker.endpoint();
    let producer_endpoint = endpoint.clone();
    let (claimed_tx, claimed_rx) = std::sync::mpsc::channel();
    let (release_tx, release_rx) = std::sync::mpsc::channel();
    let (ran_tx, ran_rx) = std::sync::mpsc::channel();
    let producer = std::thread::spawn(move || {
        matches!(
            producer_endpoint.try_submit_after_claim(
                RebuildWork::Test(Box::new(move || {
                    ran_tx.send(()).expect("test receiver lives");
                })),
                || {
                    claimed_tx.send(()).expect("test receiver lives");
                    release_rx.recv().expect("publisher is released");
                },
            ),
            WorkSubmission::Submitted
        )
    });
    claimed_rx.recv().expect("producer owns WORK_BUSY");

    let waiter_endpoint = endpoint.clone();
    let (idle_tx, idle_rx) = std::sync::mpsc::channel();
    let waiter = std::thread::spawn(move || {
        idle_tx
            .send(waiter_endpoint.wait_idle())
            .expect("test receiver lives");
    });
    assert!(
        idle_rx
            .recv_timeout(std::time::Duration::from_millis(20))
            .is_err(),
        "BUSY publication is not idle"
    );
    release_tx.send(()).expect("release the publisher");
    assert!(producer.join().expect("producer stays sound"));
    ran_rx.recv().expect("published command ran");
    assert!(idle_rx.recv().expect("waiter completed"));
    waiter.join().expect("waiter stays sound");
    worker.stop();
}

#[test]
fn rebuild_stop_closes_a_concurrently_claimed_submission() {
    struct DropThread(std::sync::mpsc::Sender<std::thread::ThreadId>);
    impl Drop for DropThread {
        fn drop(&mut self) {
            let _ = self.0.send(std::thread::current().id());
        }
    }

    let worker = RebuildWorker::<SinkDir>::start().expect("worker starts");
    let endpoint = worker.endpoint();
    let producer_endpoint = endpoint.clone();
    let (claimed_tx, claimed_rx) = std::sync::mpsc::channel();
    let (release_tx, release_rx) = std::sync::mpsc::channel();
    let (drop_tx, drop_rx) = std::sync::mpsc::channel();
    let producer = std::thread::spawn(move || {
        let probe = DropThread(drop_tx);
        matches!(
            producer_endpoint.try_submit_after_claim(
                RebuildWork::Test(Box::new(move || drop(probe))),
                || {
                    claimed_tx.send(()).expect("test receiver lives");
                    release_rx.recv().expect("publisher is released");
                },
            ),
            WorkSubmission::Submitted
        )
    });
    claimed_rx.recv().expect("producer owns WORK_BUSY");

    let stopper = std::thread::spawn(move || {
        let mut worker = worker;
        worker.stop();
    });
    while !endpoint.stopping.load(std::sync::atomic::Ordering::Acquire) {
        std::thread::yield_now();
    }
    assert_eq!(
        endpoint.state.load(std::sync::atomic::Ordering::Acquire),
        WORK_BUSY
    );
    release_tx.send(()).expect("release the publisher");
    assert!(producer.join().expect("producer stays sound"));
    stopper.join().expect("stopper stays sound");
    assert_eq!(
        endpoint.state.load(std::sync::atomic::Ordering::Acquire),
        WORK_CLOSED
    );
    assert_ne!(
        drop_rx.recv().expect("worker drained ownership"),
        std::thread::current().id(),
        "shutdown destruction stays off the caller"
    );
    assert!(matches!(
        endpoint.try_submit(RebuildWork::Test(Box::new(|| {}))),
        WorkSubmission::Returned(RebuildWork::Test(_))
    ));
}

#[test]
fn unseeded_data_loop_gate_never_falls_back_to_first_process_caller() {
    use std::sync::atomic::{AtomicUsize, Ordering};

    let unseeded = DataThreadGate {
        thread: AtomicUsize::new(0),
        log: crate::spa::Log::test_null(),
    };
    assert!(!check_loop_identity(&unseeded));
    assert_eq!(
        unseeded.thread.load(Ordering::Acquire),
        usize::MAX,
        "an unseeded gate is permanently disabled"
    );

    let current = unsafe { libc::pthread_self() } as usize;
    let seeded = DataThreadGate {
        thread: AtomicUsize::new(current),
        log: crate::spa::Log::test_null(),
    };
    assert!(check_loop_identity(&seeded));
}

#[test]
fn rebuild_work_slot_returns_ownership_when_full_or_stopped() {
    let slot = RebuildWorkSlot::<SinkDir>::new();
    assert!(matches!(
        slot.try_submit(RebuildWork::Test(Box::new(|| {}))),
        WorkSubmission::Submitted
    ));
    let second = RebuildWork::Test(Box::new(|| {}));
    let second = match slot.try_submit(second) {
        WorkSubmission::Submitted => panic!("a full slot must reject the second command"),
        WorkSubmission::Returned(second) => second,
    };
    assert!(matches!(second, RebuildWork::Test(_)));

    let _first = slot.take().expect("drain the first command");
    slot.stopping
        .store(true, std::sync::atomic::Ordering::Release);
    slot.close();
    assert_eq!(
        slot.state.load(std::sync::atomic::Ordering::Acquire),
        WORK_CLOSED
    );
    let stopped = RebuildWork::Test(Box::new(|| {}));
    assert!(
        matches!(
            slot.try_submit(stopped),
            WorkSubmission::Returned(RebuildWork::Test(_))
        ),
        "a stopped endpoint returns ownership"
    );
}

// the oss.fragment normalization contract: 0 stays automatic, everything
// else rounds DOWN to a power of two and clamps into [64, 16384]
#[test]
fn normalize_fragment_rounds_down_and_clamps() {
    assert_eq!(normalize_fragment(0), 0); // automatic
    assert_eq!(normalize_fragment(1), 64); // clamps up to the floor
    assert_eq!(normalize_fragment(63), 64); // rounds to 32, clamps to 64
    assert_eq!(normalize_fragment(64), 64);
    assert_eq!(normalize_fragment(65), 64); // round-down, then in range
    assert_eq!(normalize_fragment(1000), 512); // non-pow2 rounds down
    assert_eq!(normalize_fragment(4096), 4096); // pow2 passes through
    assert_eq!(normalize_fragment(16384), 16384);
    assert_eq!(normalize_fragment(30000), 16384); // clamps to the ceiling
    assert_eq!(normalize_fragment(1 << 31), 16384);
    assert_eq!(normalize_fragment(u32::MAX), 16384);
}

// Latency requests reset on NULL and accept only the opposite direction.
#[test]
fn latency_requests_decode_direction_gated() {
    let dir =
        |d, v: Option<&libspa::pod::Value>| decode_latency_request(d, v).map(|r| r.info.direction);
    assert_eq!(dir(SPA_DIRECTION_INPUT, None), Ok(SPA_DIRECTION_OUTPUT));
    assert_eq!(dir(SPA_DIRECTION_OUTPUT, None), Ok(SPA_DIRECTION_INPUT));

    let info = crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT);
    let value = crate::utils::parse_back(&crate::utils::build_latency_info(&info));
    assert_eq!(
        dir(SPA_DIRECTION_INPUT, Some(&value)),
        Ok(SPA_DIRECTION_OUTPUT)
    );
    // same-direction info and non-latency pods are rejected
    assert_eq!(dir(SPA_DIRECTION_OUTPUT, Some(&value)), Err(-libc::EINVAL));
    assert_eq!(
        dir(SPA_DIRECTION_INPUT, Some(&libspa::pod::Value::Int(1))),
        Err(-libc::EINVAL)
    );
}

// an aligned backing store for the admission tests (every io struct's
// alignment divides 16)
#[repr(align(16))]
struct Aligned([u8; 4096]);

#[test]
fn io_area_admission_null_short_exact_misaligned() {
    let mut area = Aligned([0; 4096]);
    let p = area.0.as_mut_ptr().cast::<c_void>();
    let full = std::mem::size_of::<spa_io_clock>();

    // NULL/0 clears whatever the size says
    assert_eq!(
        io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, std::ptr::null(), 0),
        0
    );
    // exact and oversized areas are admitted
    assert_eq!(io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full), 0);
    assert_eq!(io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full + 8), 0);
    // a non-empty-but-short area is -ENOSPC (the header's "size is too
    // small" errno for set_io/port_set_io)
    assert_eq!(
        io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full - 1),
        -libc::ENOSPC
    );
    // a misaligned one is the generic -EINVAL
    let off = unsafe { p.cast::<u8>().add(1) }.cast::<c_void>();
    assert_eq!(
        io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, off, full),
        -libc::EINVAL
    );
    // ids outside the caller's table are -ENOENT (set_io does not take
    // the port areas and vice versa)
    assert_eq!(
        io_area_ok(&NODE_IO_AREAS, SPA_IO_Buffers, p, full),
        -libc::ENOENT
    );
    assert_eq!(
        io_area_ok(&PORT_IO_AREAS, SPA_IO_Clock, p, full),
        -libc::ENOENT
    );
    // the port table's own areas admit the same policy
    let bsize = std::mem::size_of::<spa_io_buffers>();
    assert_eq!(io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, p, bsize), 0);
    assert_eq!(
        io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, p, bsize - 1),
        -libc::ENOSPC
    );
    // a short AND misaligned area reports the size problem first: the
    // host's remedy (grow the area) subsumes re-placing it
    let off = unsafe { p.cast::<u8>().add(1) }.cast::<c_void>();
    assert_eq!(
        io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, off, bsize - 1),
        -libc::ENOSPC
    );
    assert_eq!(
        io_area_ok(&PORT_IO_AREAS, SPA_IO_RateMatch, std::ptr::null(), 0),
        0
    );
}

// Known Props populate the update; adapter-owned, unknown, and invalid
// values are ignored.
#[test]
fn props_update_parses_known_keys_and_drops_the_rest() {
    use crate::utils::pod_prop;
    use libspa::pod::Value;
    let log = crate::spa::Log::test_null();

    let params = Value::Struct(vec![
        Value::String(crate::keys::OSS_DELAY.into()),
        Value::Int(8),
        Value::String(crate::keys::OSS_FRAGMENT.into()),
        Value::Int(4096),
        Value::String("bogus.key".into()),
        Value::Int(1),
    ]);
    let update = parse_props_update(
        vec![
            pod_prop(SPA_PROP_volume, Value::Float(1.0)), // softvol: adapter's
            pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(250_000)),
            pod_prop(SPA_PROP_params, params),
            pod_prop(0x77777, Value::Int(3)), // unknown key: logged, skipped
        ],
        &log,
    );
    assert_eq!(
        update,
        PropsUpdate {
            latency_offset_ns: Some(250_000),
            oss_delay: Some(8),
            oss_fragment: Some(4096),
        }
    );

    // negative values are ignored, an odd-length struct is ignored whole,
    // and a mistyped latency offset stays None
    let update = parse_props_update(
        vec![
            pod_prop(SPA_PROP_latencyOffsetNsec, Value::Int(250_000)),
            pod_prop(
                SPA_PROP_params,
                Value::Struct(vec![
                    Value::String(crate::keys::OSS_DELAY.into()),
                    Value::Int(-1),
                ]),
            ),
        ],
        &log,
    );
    assert_eq!(update, PropsUpdate::default());
    let update = parse_props_update(
        vec![pod_prop(
            SPA_PROP_params,
            Value::Struct(vec![Value::String(crate::keys::OSS_DELAY.into())]),
        )],
        &log,
    );
    assert_eq!(update, PropsUpdate::default());
}

// Format decoding accepts readback pods and rejects degenerate values.
#[test]
fn decode_format_roundtrips_and_rejects_degenerate_values() {
    let log = crate::spa::Log::test_null();
    let config = PortConfig {
        format: libspa::param::audio::AudioFormat::S16LE,
        rate: 48000,
        channels: 2,
        positions: vec![SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR],
        flags: 0,
        stride: 4,
    };
    // the builder returns bytes; the C parser needs a pod-aligned buffer
    let aligned = |bytes: &[u8]| {
        let mut buf = vec![0u64; bytes.len().div_ceil(8)];
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                buf.as_mut_ptr().cast::<u8>(),
                bytes.len(),
            );
        };
        buf
    };

    let pod = build_port_format_info(&config, SPA_PARAM_Format);
    let buf = aligned(&pod);
    let requested = unsafe { decode_format(buf.as_ptr().cast(), &log) }
        .expect("our own Format pod must decode");
    assert_eq!(requested.raw.format, SPA_AUDIO_FORMAT_S16_LE);
    assert_eq!(requested.raw.rate, 48000);
    assert_eq!(requested.raw.channels, 2);
    assert_eq!(
        &requested.raw.position[..2],
        &[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR]
    );

    // A zero rate is structurally valid but semantically invalid.
    let zero_rate = PortConfig { rate: 0, ..config };
    let pod = build_port_format_info(&zero_rate, SPA_PARAM_Format);
    let buf = aligned(&pod);
    assert_eq!(
        unsafe { decode_format(buf.as_ptr().cast(), &log) }
            .err()
            .expect("rate 0 must be rejected"),
        -libc::EINVAL
    );
}

// the completion mailbox: deposits are visible to the consumer, a fresh
// deposit replaces an unconsumed one (whose payload drops in the
// depositor's thread), take is one-shot, and discard voids the slot
#[test]
fn rebuild_mailbox_delivers_replaces_and_discards() {
    let shared: NodeShared<SinkDir> = NodeShared::new(std::sync::Weak::new());
    assert!(shared.take_swap().is_none());

    shared.deposit(DeviceSwap {
        port_idx: 0,
        generation: 3,
        outcome: SwapOutcome::Aborted,
    });
    shared.deposit(DeviceSwap {
        port_idx: 0,
        generation: 4,
        outcome: SwapOutcome::Failed {
            placeholder: crate::sound::DspWriter::new("/nonexistent/dsp"),
        },
    });
    let swap = shared.take_swap().expect("a deposited swap");
    assert_eq!(swap.generation, 4, "the newer deposit wins");
    assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));
    assert!(shared.take_swap().is_none(), "take is one-shot");

    shared.deposit(DeviceSwap {
        port_idx: 0,
        generation: 5,
        outcome: SwapOutcome::Aborted,
    });
    shared.discard_swap();
    assert!(shared.take_swap().is_none(), "discard voids the slot");
}

// The worker never accesses State: stopped requests abort before opening,
// failed opens preserve the request generation, and cleared nodes receive
// no completion.
#[test]
fn rebuild_task_deposits_and_respects_the_gates() {
    use std::sync::atomic::Ordering;
    let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
    let shared = std::sync::Arc::new(NodeShared::new(std::sync::Arc::downgrade(&events)));
    shared.generation.store(7, Ordering::Release);
    let request = |shared: &std::sync::Arc<NodeShared<SinkDir>>| RebuildRequest {
        port_idx: 0,
        generation: 7,
        config: PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 4,
        },
        path: "/nonexistent/dsp".into(),
        oss_fragment: 0,
        retried: false,
        retire_first: None,
        log: crate::spa::Log::test_null(),
        shared: std::sync::Arc::downgrade(shared),
    };

    // not started: aborted without touching the device
    rebuild_task(request(&shared));
    let swap = shared.take_swap().expect("a deposit even when stopped");
    assert_eq!(swap.generation, 7);
    assert!(matches!(swap.outcome, SwapOutcome::Aborted));

    // started, open fails (no such device): Failed, generation echoed
    shared.started.store(true, Ordering::Release);
    rebuild_task(request(&shared));
    let swap = shared.take_swap().expect("a deposit on failure");
    assert_eq!(swap.generation, 7);
    assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));

    // superseded while queued: abort before opening or publishing a
    // device for the stale generation
    shared.generation.store(8, Ordering::Release);
    rebuild_task(request(&shared));
    let swap = shared.take_swap().expect("a stale task still completes");
    assert_eq!(swap.generation, 7);
    assert!(matches!(swap.outcome, SwapOutcome::Aborted));

    // cleared node (the endpoint is gone): no deposit at all
    drop(events);
    rebuild_task(request(&shared));
    assert!(shared.take_swap().is_none());
}

#[test]
fn rebuild_gate_rechecks_started_and_generation_after_blocking_work() {
    use std::sync::atomic::Ordering;

    let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
    let shared = std::sync::Arc::new(NodeShared::new(std::sync::Arc::downgrade(&events)));
    shared.started.store(true, Ordering::Release);
    shared.generation.store(7, Ordering::Release);
    let worker_shared = shared.clone();
    let (entered_tx, entered_rx) = std::sync::mpsc::channel();
    let (release_tx, release_rx) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        assert!(rebuild_request_is_current(&worker_shared, 7));
        entered_tx.send(()).expect("test receiver lives");
        release_rx.recv().expect("mock open is released");
        rebuild_request_is_current(&worker_shared, 7)
    });
    entered_rx.recv().expect("mock open passed its first gate");

    // Model a data-loop replacement and a concurrent Pause while the
    // blocking open is in progress. Either change must reject its result.
    shared.generation.store(8, Ordering::Release);
    shared.started.store(false, Ordering::Release);
    release_tx.send(()).expect("release the mock open");
    assert!(
        !worker.join().expect("gate worker stays sound"),
        "the post-open gate must reject superseded work"
    );
}

// the unwind guard: a panicking task body still deposits Aborted for
// its generation (queue_task swallows the panic, and without the
// deposit the port's pending claim would be stranded forever); a
// completed task's guard deposits nothing extra
#[test]
fn a_panicking_rebuild_path_still_deposits_aborted() {
    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new(std::sync::Weak::new()));

    let guard = DepositOnUnwind {
        shared: shared.clone(),
        port_idx: 0,
        generation: 9,
        armed: true,
    };
    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
        let _guard = guard; // dropped mid-unwind, like a panicking open
        panic!("injected task panic");
    }));
    assert!(panicked.is_err());
    let swap = shared
        .take_swap()
        .expect("the guard deposited during unwind");
    assert_eq!(swap.generation, 9, "the stale-rebuild fence stays sane");
    assert!(matches!(swap.outcome, SwapOutcome::Aborted));

    // the normal path deposits its outcome exactly once
    let guard = DepositOnUnwind {
        shared: shared.clone(),
        port_idx: 0,
        generation: 10,
        armed: true,
    };
    guard.complete(SwapOutcome::Failed {
        placeholder: crate::sound::DspWriter::new("/nonexistent/dsp"),
    });
    let swap = shared.take_swap().expect("the completed outcome");
    assert_eq!(swap.generation, 10);
    assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));
    assert!(shared.take_swap().is_none(), "no second deposit");
}

// The slot protocol under three-way contention: worker deposits,
// data-loop takes, and main-loop discard/replacement may all overlap.
// The reader never waits, and observed writer generations never regress.
#[test]
fn rebuild_mailbox_is_safe_under_contention() {
    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new(std::sync::Weak::new()));
    let writer = {
        let shared = shared.clone();
        std::thread::spawn(move || {
            for generation in 0..10_000u64 {
                shared.deposit(DeviceSwap {
                    port_idx: 0,
                    generation,
                    outcome: SwapOutcome::Aborted,
                });
            }
        })
    };
    let discarder = {
        let shared = shared.clone();
        std::thread::spawn(move || {
            for _ in 0..10_000 {
                shared.discard_swap();
            }
        })
    };
    let mut last = None;
    loop {
        let done = writer.is_finished() && discarder.is_finished();
        if let Some(swap) = shared.take_swap() {
            if let Some(prev) = last {
                assert!(swap.generation > prev, "a replaced deposit reappeared");
            }
            last = Some(swap.generation);
        } else if done {
            break;
        }
    }
    writer.join().expect("the writer must not panic");
    discarder.join().expect("the discarder must not panic");

    // A post-contention sentinel proves no actor left BUSY behind and the
    // final value still transfers exactly once.
    shared.deposit(DeviceSwap {
        port_idx: 0,
        generation: 10_000,
        outcome: SwapOutcome::Aborted,
    });
    assert_eq!(shared.take_swap().map(|swap| swap.generation), Some(10_000));
    assert!(shared.take_swap().is_none());
}

fn test_port(fd: std::os::raw::c_int) -> Port<SinkDir> {
    Port {
        config: None,
        buffers: vec![],
        io: crate::spa::IoArea::null(),
        rate_match: crate::spa::IoArea::null(),
        dsp: crate::sound::DspWriter::test_on_fd(fd, 8),
        dll: Default::default(),
        setup_period: 0,
        bw_adapt: Default::default(),
        setup_blocksize: 0,
        rebuild_pending: false,
        generation: 0,
        was_matching: false,
        warn_limit: crate::utils::RateLimit::new(),
        pending_xrun: None,
        ext: Default::default(),
    }
}

// a stack fixture: one spa_buffer with one MemPtr data block; the tests
// then break one field at a time
fn fixture(payload: &mut [u8], chunk: *mut spa_chunk) -> (spa_buffer, Box<spa_data>) {
    let mut data: spa_data = unsafe { std::mem::zeroed() };
    data.type_ = SPA_DATA_MemPtr;
    data.maxsize = payload.len() as u32;
    data.data = payload.as_mut_ptr().cast();
    data.chunk = chunk;
    let mut data = Box::new(data);
    let mut buffer: spa_buffer = unsafe { std::mem::zeroed() };
    buffer.n_datas = 1;
    buffer.datas = &mut *data;
    (buffer, data)
}

// the per-cycle buffer gate: exactly one MemPtr block with data, chunk
// and maxsize all valid is admitted; everything else skips (None), never
// faults - buffer_id and the block layout come from the peer
#[test]
fn valid_data_block_admits_only_a_usable_memptr_block() {
    let (r, w) = crate::sound::test_util::pipe_pair(true, true);
    let mut port = test_port(w);
    let log = crate::spa::Log::test_null();
    let mut payload = [0u8; 64];
    let mut chunk: spa_chunk = unsafe { std::mem::zeroed() };

    // Happy path: the descriptor carries the validated pointers by value
    // and the accessors stay inside the block. The chunk says 32 bytes at
    // offset 16, so input_slice views exactly that window; output_slice
    // spans the whole block.
    chunk.offset = 16;
    chunk.size = 32;
    chunk.stride = 8;
    let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
    port.buffers = vec![&mut buffer];
    let mut block = unsafe { valid_data_block(&port, 0, &log) }.expect("a usable MemPtr block");
    assert!(std::ptr::eq(
        block.data_ptr().cast::<u8>(),
        payload.as_ptr()
    ));
    assert_eq!(block.chunk_stride(), 8);
    assert!(std::ptr::eq(
        block.input_slice().as_ptr(),
        payload[16..].as_ptr()
    ));
    assert_eq!(block.input_slice().len(), 32);
    assert_eq!(block.output_slice().len(), payload.len());

    // a peer offset past the block wraps and the size clamps to what
    // remains (the input clamp the sink write path depends on)
    block.publish(60, 8);
    let mut block = unsafe { valid_data_block(&port, 0, &log) }.expect("a usable MemPtr block");
    // publish rewrote the chunk: 60 bytes at offset 0
    assert_eq!(block.input_slice().len(), 60);
    // and re-reading through a chunk pointing past the end stays bounded
    block.output_slice()[0] = 0xaa;
    let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
    chunk.offset = 60;
    chunk.size = 32;
    port.buffers = vec![&mut buffer];
    let block = unsafe { valid_data_block(&port, 0, &log) }.expect("a usable MemPtr block");
    assert_eq!(block.input_slice().len(), 4);
    assert!(std::ptr::eq(
        block.input_slice().as_ptr(),
        payload[60..].as_ptr()
    ));

    // out-of-range buffer_id
    assert!(unsafe { valid_data_block(&port, 1, &log) }.is_none());

    // a null host buffer pointer
    port.buffers = vec![std::ptr::null_mut()];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    // n_datas != 1
    let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
    buffer.n_datas = 2;
    port.buffers = vec![&mut buffer];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    // null datas array
    let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
    buffer.datas = std::ptr::null_mut();
    port.buffers = vec![&mut buffer];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    // null data pointer
    let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
    data.data = std::ptr::null_mut();
    port.buffers = vec![&mut buffer];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    // null chunk
    let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
    data.chunk = std::ptr::null_mut();
    port.buffers = vec![&mut buffer];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    // zero maxsize
    let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
    data.maxsize = 0;
    port.buffers = vec![&mut buffer];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    // a non-MemPtr block
    let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
    data.type_ = SPA_DATA_MemFd;
    port.buffers = vec![&mut buffer];
    assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

    unsafe { libc::close(r) };
}
