use super::*;
use crate::node::sink::SinkDir;
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
// the completion mailbox: deposits are visible to the consumer, a fresh
// deposit replaces an unconsumed one (whose payload drops in the
// depositor's thread), take is one-shot, and discard voids the slot
#[test]
fn rebuild_mailbox_delivers_replaces_and_discards() {
    let shared: NodeShared<SinkDir> = NodeShared::new();
    assert!(!shared.swap_ready());
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
            placeholder: crate::oss::DspWriter::new("/nonexistent/dsp"),
        },
    });
    assert!(shared.swap_ready());
    let swap = shared.take_swap().expect("a deposited swap");
    assert_eq!(swap.generation, 4, "the newer deposit wins");
    assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));
    assert!(!shared.swap_ready());
    assert!(shared.take_swap().is_none(), "take is one-shot");

    shared.deposit(DeviceSwap {
        port_idx: 0,
        generation: 5,
        outcome: SwapOutcome::Aborted,
    });
    shared.discard_swap();
    assert!(!shared.swap_ready());
    assert!(shared.take_swap().is_none(), "discard voids the slot");
}

// The worker never accesses State: stopped requests abort before opening,
// failed opens preserve the request generation, and cleared nodes receive
// no completion.
#[test]
fn rebuild_task_deposits_and_respects_the_gates() {
    use std::sync::atomic::Ordering;
    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new());
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
        fragment_bytes: 0,
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
    shared.close();
    rebuild_task(request(&shared));
    assert!(shared.take_swap().is_none());
}

#[test]
fn rebuild_gate_rechecks_started_and_generation_after_blocking_work() {
    use std::sync::atomic::Ordering;

    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new());
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
    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new());

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
        placeholder: crate::oss::DspWriter::new("/nonexistent/dsp"),
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
    let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new());
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
