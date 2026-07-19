use super::*;

pub(crate) fn normalize_fragment(v: u32) -> u32 {
    if v == 0 {
        0
    } else {
        (1u32 << (31 - v.leading_zeros())).clamp(64, 16384)
    }
}

// The oss.* tunable live re-apply path: store the new loop-owned value on the
// data loop (the prime paths read it there), then rebuild any running port
// from this (main) thread so the next cycle re-primes with the new layout.
// Synchronous contract (see install_device): main-thread entry, blocking
// frame-bounded invokes only.
pub(crate) fn store_and_rebuild<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    store: impl FnOnce(&mut DataState<D>) + Send,
) -> c_int {
    let configs: Option<[Option<PortConfig>; MAX_PORTS]> = data.query(move |data| {
        store(data);
        std::array::from_fn(|i| {
            data.ports[i]
                .dsp
                .is_running()
                .then(|| data.ports[i].config.clone())
                .flatten()
        })
    });
    let Some(configs) = configs else {
        return -libc::EIO;
    };
    for (port_idx, config) in configs.into_iter().enumerate() {
        if let Some(config) = config {
            if install_device(state, data, port_idx, config) != 0 {
                // the host didn't initiate this rebuild; without a re-announce it
                // keeps believing a format is set on a dead port
                emit_format_lost(state);
            }
        }
    }
    0
}

// announce a Props change (so readback stays fresh), then apply it through
// store_and_rebuild; shared by the oss.* prop appliers of both directions
pub(crate) fn apply_props_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    store: impl FnOnce(&mut DataState<D>) + Send,
) -> c_int {
    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Props);
    });
    emit_node_info(state);
    store_and_rebuild(state, data, store)
}

// Release the synchronous rebuild lease after an error. A dead loop cannot
// observe the retained flag, so failure remains best-effort.
pub(super) fn release_rebuild_takeover<D: Direction>(
    data: &DataControl<D>,
    port_idx: usize,
) -> bool {
    data.invoke(move |state| {
        state.rebuild_takeover = false;
        state.ports[port_idx].rebuild_pending = false;
    })
}

// Open and configure on the main thread because device operations may block,
// then swap the device on the data loop. The takeover fence invalidates
// asynchronous rebuilds while the synchronous install is active. On EBUSY,
// retire the current exclusive device and retry; other failures leave the
// port cleared.
pub(super) fn install_device<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
    config: PortConfig,
) -> c_int {
    // Acquire the sole-producer takeover on the data loop before waiting.
    // The generation bump makes both queued and active rebuilds stale;
    // rebuild_takeover makes later cycles skip without consuming or
    // submitting until the final swap releases the lease.
    let Some(deferred) = data.query(move |data| {
        debug_assert!(!data.rebuild_takeover, "synchronous installs serialize");
        data.rebuild_takeover = true;
        let deferred = data.deferred_work.take();
        let port = &mut data.ports[port_idx];
        port.rebuild_pending = true;
        port.generation = port.generation.wrapping_add(1);
        data.shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        deferred
    }) else {
        return -libc::EIO;
    };
    // Any retained RetireAndRetry/device ownership now dies here, never on
    // the data loop.
    drop(deferred);

    // Close a completion that predates the fence, then wait until an active
    // command observes the generation change and finishes. The second drain
    // catches the completion it may have deposited before becoming idle.
    state.shared.discard_swap();
    if !state.rebuild_worker.wait_idle() {
        release_rebuild_takeover(data, port_idx);
        return -libc::EIO;
    }
    state.shared.discard_swap();

    let mut new_dsp = D::Device::new(&state.dsp_path);
    // oss_fragment only mutates from main-thread calls, serialized with us
    let mut res = D::try_open_configure(&mut new_dsp, &config, state.oss_fragment, &state.log);

    if res == -libc::EBUSY {
        let closed = D::Device::new(&state.dsp_path);
        let Some(retired) =
            data.query(move |state| std::mem::replace(&mut state.ports[port_idx].dsp, closed))
        else {
            release_rebuild_takeover(data, port_idx);
            return -libc::EIO;
        };
        drop(retired); // closes the old fd here, off the RT path
        res = D::try_open_configure(&mut new_dsp, &config, state.oss_fragment, &state.log);
    }

    let ok = res == 0;
    let cap_config = config.clone();
    let old_dsp = data.query(move |state| {
        let port = &mut state.ports[port_idx];
        // new_dsp is a closed writer/reader when negotiation failed above
        let old = std::mem::replace(&mut port.dsp, new_dsp);
        port.config = if ok { Some(config) } else { None };
        // Retire any in-flight background rebuild.
        port.generation = port.generation.wrapping_add(1);
        state
            .shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        port.rebuild_pending = false;
        port.was_matching = false; // force a relock when matching resumes
        D::on_device_swapped(state, port_idx);
        state.rebuild_takeover = false;
        old
    });
    let swapped = old_dsp.is_some();
    drop(old_dsp); // ditto

    if !swapped {
        release_rebuild_takeover(data, port_idx);
        return -libc::EIO; // the swap never ran; the port keeps its old state
    }
    if res == 0 {
        publish_ring_quantum_cap(state, &cap_config); // stride is known now
    }
    res
}

// FreeBSD caps every soft ring at CHN_2NDBUFMAXSIZE (131 KiB); at fat strides
// (a 20-channel S32 interface is 80 bytes/frame) the ring holds only ~1.6
// periods at quantum 1024 and both directions glitch structurally - the
// capture side has no room for arrival jitter, the playback side can't hold
// two quanta plus the delay target. Publish node.max-latency once the stride
// is known so the graph never negotiates a quantum the kernel ring can't hold
// four of (pw_impl_node parses the fraction into max_latency, which caps the
// driver quantum). Emitted only when the cap bites below the common
// 2048-frame default in TIME, at a conservative 44.1 kHz reference -
// clock.rate is unknown here and an over-published cap is inert (sound.rs
// advertised_quantum_cap_frames); published once -
// the props dict is append-only, and a stride change without a node rebuild
// is not worth a duplicate entry.
fn publish_ring_quantum_cap<D: Direction>(state: &mut MainState<D>, config: &PortConfig) {
    let stride = config.stride.max(1);
    let rate = config.rate;
    // the shared ring policy (sound.rs); the published fraction is time-based
    // (frames/device rate), so it needs no graph-rate scaling
    let Some(frames) = crate::sound::advertised_quantum_cap_frames(stride, rate) else {
        return;
    };
    if state.ring_cap_published {
        return;
    }
    state.ring_cap_published = true;
    crate::info!(
        state.log,
        "kernel ring ({} bytes) at stride {} holds 4 periods only up to \
    quantum {}; publishing node.max-latency",
        crate::sound::ring_byte_cap(stride, rate),
        stride,
        frames
    );
    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        info.add_prop("node.max-latency", format!("{frames}/{rate}"));
    });
    emit_node_info(state);
}

// Announce a failed background rebuild so the session manager renegotiates
// the cleared format.
fn emit_format_lost<D: Direction>(state: &MainState<D>) {
    state.events.record_current_format_lost();
}

// Asynchronous rebuilds carry owned requests from the data loop to the
// blocking-I/O worker. The worker never accesses State: it returns an owned
// DeviceSwap through NodeShared, and the data loop accepts it only when the
// port generation still matches. Retired devices also move to the worker so
// potentially blocking closes stay off the real-time path.

// The completion mailbox has one slot; multi-port support requires one slot
// per port.
const _: () = assert!(
    MAX_PORTS == 1,
    "NodeShared's completion mailbox assumes a single port"
);

// Shared state for the data loop, rebuild worker, and clear(). Worker guards
// keep it alive independently of State.
pub(crate) struct NodeShared<D: Direction> {
    // A lifetime-safe main-loop event endpoint. State owns the sole strong
    // handle; queued work can test/upgrade this Weak without ever obtaining
    // a State pointer. Once clear drops State, later deliveries are no-ops.
    pub(super) events: std::sync::Weak<NodeEvents<D>>,
    // mirror of State.started, written by send_command on the main thread
    // (Start/Pause/Suspend), read by rebuild_task on the worker: a
    // stop that lands after a task was queued must win, or the task would
    // hand a stopped node an open (possibly exclusive) device
    pub(super) started: std::sync::atomic::AtomicBool,
    // Mirror of the single data-loop port's generation. Worker rebuild
    // work checks it before and after an open so a released/superseded
    // request cannot leave an exclusive stale fd in the completion slot.
    pub(super) generation: std::sync::atomic::AtomicU64,
    // The completion mailbox: a preallocated single-slot cell. The worker
    // deposits (replacing an unconsumed predecessor); the main loop
    // may discard during synchronous changes and teardown, while the
    // data loop consumes at cycle start. The RT side never locks or
    // allocates: take_swap is one CAS plus the in-place move, and when it
    // loses the race against a mid-deposit writer it returns None and polls
    // again next cycle. Only the non-RT writer may spin, and only while
    // the reader is inside its few-instruction move. The value lives in the
    // UnsafeCell and is touched exclusively by whoever holds SLOT_BUSY -
    // the protocol behind the manual Sync impl below.
    pub(super) slot_state: std::sync::atomic::AtomicU8,
    pub(super) slot: std::cell::UnsafeCell<Option<DeviceSwap<D>>>,
}

const SLOT_EMPTY: u8 = 0; // no message; the cell is None
const SLOT_FULL: u8 = 1; // one message; the cell is Some
const SLOT_BUSY: u8 = 2; // one side is moving the value; the cell is theirs

// SAFETY: the slot cell is only read or written by the thread that CASed
// slot_state to SLOT_BUSY (exchange/take_swap below), and the FULL store
// after a deposit is Release, paired with take_swap's Acquire CAS - so the
// message payload is published before the consumer can move it out. The
// remaining fields are atomics or thread-safe Weak handles.
unsafe impl<D: Direction> Sync for NodeShared<D> {}

// Owned event sent from the data loop to the main-loop endpoint.
pub(super) enum MainEvent {
    // Re-announce a format cleared by a failed background rebuild.
    FormatLost { expected_publication_epoch: u64 },
}

impl<D: Direction> NodeShared<D> {
    pub(super) fn new(events: std::sync::Weak<NodeEvents<D>>) -> Self {
        Self {
            events,
            started: std::sync::atomic::AtomicBool::new(false),
            generation: std::sync::atomic::AtomicU64::new(0),
            slot_state: std::sync::atomic::AtomicU8::new(SLOT_EMPTY),
            slot: std::cell::UnsafeCell::new(None),
        }
    }

    // Queued tasks may deliver until clear() drops the event endpoint.
    pub(super) fn is_alive(&self) -> bool {
        self.events.strong_count() != 0
    }

    // Deliver queued messages on the main-loop thread. A dead Weak drops them.
    pub(super) fn main_event(&self, event: MainEvent) {
        let Some(events) = self.events.upgrade() else {
            return;
        };
        match event {
            // SAFETY: this endpoint message contains no State reference and
            // its caller ended the process DataState phase before queueing it.
            MainEvent::FormatLost {
                expected_publication_epoch,
            } => unsafe { events.emit_format_lost_now(expected_publication_epoch) },
        }
    }

    // The non-RT writer side of the slot protocol: acquire SLOT_BUSY from
    // EMPTY or FULL, swap the new value in, publish the resulting state, and
    // hand the predecessor back to the caller to drop off the RT path.
    pub(super) fn exchange(&self, new: Option<DeviceSwap<D>>) -> Option<DeviceSwap<D>> {
        use std::sync::atomic::Ordering;
        loop {
            let cur = self.slot_state.load(Ordering::Relaxed);
            if cur == SLOT_BUSY {
                // Writers are worker/main-loop only, never RT. Yield instead
                // of burning a core if the few-instruction slot owner was
                // preempted while BUSY.
                std::thread::yield_now();
                continue;
            }
            debug_assert!(cur == SLOT_EMPTY || cur == SLOT_FULL);
            if self
                .slot_state
                .compare_exchange_weak(cur, SLOT_BUSY, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
            {
                let full = new.is_some();
                // SAFETY: SLOT_BUSY is held; the cell is exclusively ours
                let prev = unsafe { std::mem::replace(&mut *self.slot.get(), new) };
                self.slot_state
                    .store(if full { SLOT_FULL } else { SLOT_EMPTY }, Ordering::Release);
                return prev;
            }
        }
    }

    // worker: leave the completion for the data loop (replacing an
    // unconsumed predecessor, whose device closes here, off the RT path)
    pub(super) fn deposit(&self, swap: DeviceSwap<D>) {
        let prev = self.exchange(Some(swap));
        drop(prev);
    }

    // Data loop (single consumer): the completion, if one arrived. Never
    // waits: a writer mid-deposit just reads as "nothing yet".
    pub(super) fn take_swap(&self) -> Option<DeviceSwap<D>> {
        use std::sync::atomic::Ordering;
        if self
            .slot_state
            .compare_exchange(SLOT_FULL, SLOT_BUSY, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None; // empty, or a writer holds the slot
        }
        // SAFETY: SLOT_BUSY is held; the cell is exclusively ours
        let swap = unsafe { (*self.slot.get()).take() };
        debug_assert!(swap.is_some(), "SLOT_FULL always covers a Some cell");
        self.slot_state.store(SLOT_EMPTY, Ordering::Release);
        swap
    }

    // main thread (install_device, Suspend, clear): void any undelivered
    // completion; its device closes here, off the RT path
    pub(super) fn discard_swap(&self) {
        let dropped = self.exchange(None);
        drop(dropped);
    }
}

// Rebuild request sent to the worker. It contains everything needed to open
// and configure a device without accessing State.
pub(super) struct RebuildRequest<D: Direction> {
    pub(super) port_idx: usize,
    pub(super) generation: u64,
    pub(super) config: PortConfig,
    pub(super) path: String,
    pub(super) oss_fragment: u32,
    pub(super) retried: bool, // the EBUSY retire round trip already happened
    // RetireAndRetry only: the port's dying fd, closed by the worker under
    // its unwind guard before the retry opens
    pub(super) retire_first: Option<D::Device>,
    pub(super) log: crate::spa::Log,
    // Weak avoids a NodeShared -> mailbox -> retry request -> NodeShared
    // cycle while a RetireAndRetry completion waits for the data loop.
    pub(super) shared: std::sync::Weak<NodeShared<D>>,
}

// Worker result. The data loop applies it only while the port generation
// still matches.
pub(super) struct DeviceSwap<D: Direction> {
    pub(super) port_idx: usize,
    pub(super) generation: u64,
    pub(super) outcome: SwapOutcome<D>,
}

pub(super) enum SwapOutcome<D: Direction> {
    // open+configure succeeded: install and resume
    Installed {
        dsp: D::Device,
        config: PortConfig,
    },
    // the node was stopped when the task ran: drop the pending claim; the
    // next started cycle re-queues if the port still needs a rebuild
    Aborted,
    // open failed with EBUSY and the port's own (dying) fd is the likely
    // blocker on an exclusive device (bitperfect, vchans off): retire it,
    // then re-run the request - the retire needs the data loop, so it is
    // another message round trip
    RetireAndRetry {
        request: RebuildRequest<D>,
        placeholder: D::Device,
    },
    // open/configure failed (even after the retire, for EBUSY): the port
    // loses its format; poll_rebuild clears the config and queues the
    // format-lost re-announce
    Failed {
        placeholder: D::Device,
    },
}

// Owned commands for the per-node blocking-I/O worker. No variant contains
// State or a pointer into it. In particular, retirement transfers device
// ownership all the way to this worker so a Device destructor can never run
// on the data loop.
pub(super) enum RebuildWork<D: Direction> {
    Rebuild(RebuildRequest<D>),
    RetireDevice(D::Device),
    RetireSwap(DeviceSwap<D>),
    #[cfg(test)]
    Test(Box<dyn FnOnce() + Send>),
}

pub(super) enum WorkSubmission<D: Direction> {
    Submitted,
    Returned(RebuildWork<D>),
}

const WORK_EMPTY: u8 = 0;
const WORK_FULL: u8 = 1;
pub(super) const WORK_BUSY: u8 = 2;
pub(super) const WORK_CLOSED: u8 = 3;

// A preallocated, single-producer/single-consumer work slot. MAX_PORTS == 1
// and rebuild_pending permit only one rebuild order at a time. DataState's
// additional deferred_work cell retains the one retirement/retry that can
// collide with an occupied slot. Submission never waits and never allocates.
pub(super) struct RebuildWorkSlot<D: Direction> {
    pub(super) stopping: std::sync::atomic::AtomicBool,
    pub(super) state: std::sync::atomic::AtomicU8,
    value: std::cell::UnsafeCell<Option<RebuildWork<D>>>,
    thread: std::sync::OnceLock<std::thread::Thread>,
    // The worker sets active while holding this mutex before it takes a
    // published command. That closes the otherwise-racy gap between an
    // empty slot and execution for main-thread takeover waits.
    active: std::sync::Mutex<bool>,
    idle: std::sync::Condvar,
}

// SAFETY: the data loop is the sole producer and the worker is the sole
// consumer. Either side may access value only after changing state from
// EMPTY/FULL to BUSY; the Release publication of FULL is paired with the
// worker's Acquire CAS. A failed producer CAS returns its still-owned value.
unsafe impl<D: Direction> Sync for RebuildWorkSlot<D> {}

impl<D: Direction> RebuildWorkSlot<D> {
    pub(super) fn new() -> Self {
        Self {
            stopping: std::sync::atomic::AtomicBool::new(false),
            state: std::sync::atomic::AtomicU8::new(WORK_EMPTY),
            value: std::cell::UnsafeCell::new(None),
            thread: std::sync::OnceLock::new(),
            active: std::sync::Mutex::new(false),
            idle: std::sync::Condvar::new(),
        }
    }

    // Data-loop producer. Ownership is returned on every failure so a
    // device-bearing command cannot be destroyed in this call.
    pub(super) fn try_submit(&self, work: RebuildWork<D>) -> WorkSubmission<D> {
        self.try_submit_after_claim(work, || {})
    }

    // The callback is normally empty and optimizes away. Tests use it to
    // pause a producer after EMPTY->BUSY and deterministically exercise
    // takeover/shutdown against an in-progress publication.
    pub(super) fn try_submit_after_claim(
        &self,
        work: RebuildWork<D>,
        after_claim: impl FnOnce(),
    ) -> WorkSubmission<D> {
        use std::sync::atomic::Ordering;
        if self
            .state
            .compare_exchange(WORK_EMPTY, WORK_BUSY, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return WorkSubmission::Returned(work);
        }
        after_claim();
        // SAFETY: this producer changed EMPTY to BUSY and owns the cell.
        unsafe { *self.value.get() = Some(work) };
        self.state.store(WORK_FULL, Ordering::Release);
        if let Some(thread) = self.thread.get() {
            thread.unpark();
        }
        WorkSubmission::Submitted
    }

    // Worker consumer. BUSY is reported like empty; the publishing producer
    // will unpark us after its short in-place move.
    pub(super) fn take(&self) -> Option<RebuildWork<D>> {
        use std::sync::atomic::Ordering;
        if self
            .state
            .compare_exchange(WORK_FULL, WORK_BUSY, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None;
        }
        // SAFETY: this consumer changed FULL to BUSY and owns the cell.
        let work = unsafe { (*self.value.get()).take() };
        debug_assert!(work.is_some(), "WORK_FULL always covers a Some cell");
        self.state.store(WORK_EMPTY, Ordering::Release);
        work
    }

    pub(super) fn wake(&self) {
        if let Some(thread) = self.thread.get() {
            thread.unpark();
        }
    }

    // Atomically close the EMPTY claim point. A producer that already owns
    // BUSY is allowed to publish; the worker drains it before this loop can
    // win EMPTY->CLOSED. No stale boolean load can reopen the slot afterward.
    pub(super) fn close(&self) {
        use std::sync::atomic::Ordering;
        loop {
            match self.state.compare_exchange(
                WORK_EMPTY,
                WORK_CLOSED,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) | Err(WORK_CLOSED) => return,
                Err(WORK_FULL | WORK_BUSY) => {
                    self.wake();
                    std::thread::yield_now();
                }
                Err(state) => unreachable!("invalid rebuild work state {state}"),
            }
        }
    }

    // Main thread only, after DataState::rebuild_takeover has excluded the
    // ordinary producer. Wait until every command published before the lease
    // has been taken and completely processed.
    pub(super) fn wait_idle(&self) -> bool {
        use std::sync::atomic::Ordering;
        self.wake();
        let mut active = self.active.lock_unpoisoned();
        loop {
            if !*active && self.state.load(Ordering::Acquire) == WORK_EMPTY {
                return true;
            }
            if self.stopping.load(Ordering::Acquire) {
                return false;
            }
            active = self
                .idle
                .wait(active)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
    }
}

// MainState's owned worker handle. Drop is deliberately idempotent: init can
// fail after State is written but before init returns, and drop_in_place must
// not detach a thread parked on an Arc that otherwise has no shutdown owner.
pub(super) struct RebuildWorker<D: Direction> {
    work: std::sync::Arc<RebuildWorkSlot<D>>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl<D: Direction> RebuildWorker<D> {
    pub(super) fn start() -> std::io::Result<Self> {
        let work = std::sync::Arc::new(RebuildWorkSlot::new());
        let worker_work = work.clone();
        let join = std::thread::Builder::new()
            .name(format!("pw-oss-{}-rebuild", D::MEDIA_CLASS))
            .spawn(move || rebuild_worker_loop(worker_work))?;
        // OnceLock cannot already be set: this endpoint was just created.
        let _ = work.thread.set(join.thread().clone());
        // Cover the worker parking before the Thread handle was published.
        work.wake();
        Ok(Self {
            work,
            join: Some(join),
        })
    }

    pub(super) fn endpoint(&self) -> std::sync::Arc<RebuildWorkSlot<D>> {
        self.work.clone()
    }

    pub(super) fn wait_idle(&self) -> bool {
        self.work.wait_idle()
    }

    pub(super) fn stop(&mut self) {
        use std::sync::atomic::Ordering;
        let Some(join) = self.join.take() else {
            return;
        };
        self.work.stopping.store(true, Ordering::Release);
        self.work.wake();
        self.work.close();
        self.work.wake();
        // Per-command panics are contained by the loop. A remaining panic is
        // still joined here so no thread can outlive its node.
        let _ = join.join();
    }
}

impl<D: Direction> Drop for RebuildWorker<D> {
    fn drop(&mut self) {
        self.stop();
    }
}

fn rebuild_worker_loop<D: Direction>(work: std::sync::Arc<RebuildWorkSlot<D>>) {
    use std::sync::atomic::Ordering;
    loop {
        // Set active under the same mutex takeover waiters use, before
        // taking the slot. They can therefore never observe EMPTY/idle in
        // the taken-but-not-yet-executing gap.
        let command = {
            let mut active = work.active.lock_unpoisoned();
            let command = work.take();
            if command.is_some() {
                *active = true;
            }
            command
        };
        if let Some(command) = command {
            if work.stopping.load(Ordering::Acquire) {
                // Device-bearing commands are destroyed here, on the worker,
                // even during shutdown. Rebuild orders are simply cancelled.
                drop(command);
            } else {
                // DepositOnUnwind turns a panicking rebuild into Aborted; the
                // outer catch keeps this worker alive for later commands.
                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run_rebuild_work(command);
                }));
            }
            let mut active = work.active.lock_unpoisoned();
            *active = false;
            work.idle.notify_all();
            continue;
        }
        if work.state.load(Ordering::Acquire) == WORK_CLOSED {
            break;
        }
        if work.stopping.load(Ordering::Acquire) {
            // stop() is waiting to win EMPTY->CLOSED. Yield until a producer
            // publishes or the close becomes visible.
            std::thread::yield_now();
            continue;
        }
        std::thread::park();
    }
}

fn run_rebuild_work<D: Direction>(work: RebuildWork<D>) {
    match work {
        RebuildWork::Rebuild(request) => rebuild_task(request),
        RebuildWork::RetireDevice(device) => drop(device),
        RebuildWork::RetireSwap(swap) => drop(swap),
        #[cfg(test)]
        RebuildWork::Test(test) => test(),
    }
}

// Retry one retained worker command before consuming another completion.
// false tells the cycle to skip: the retained value may own the currently
// retiring device and must remain the next operation observed by the worker.
fn flush_deferred_work<D: Direction>(state: &mut DataState<D>) -> bool {
    let Some(work) = state.deferred_work.take() else {
        return true;
    };
    match state.rebuild_work.try_submit(work) {
        WorkSubmission::Submitted => true,
        WorkSubmission::Returned(work) => {
            state.deferred_work = Some(work);
            false
        }
    }
}

// Submit without ever dropping on failure. The single deferred cell is free
// whenever this is called: poll_rebuild flushes it before taking a completion,
// and queue_rebuild refuses a second order while one is retained.
fn submit_or_defer<D: Direction>(state: &mut DataState<D>, work: RebuildWork<D>) {
    debug_assert!(
        state.deferred_work.is_none(),
        "worker work must preserve its single-producer order"
    );
    if let WorkSubmission::Returned(work) = state.rebuild_work.try_submit(work) {
        state.deferred_work = Some(work);
    }
}

/// Queue an owned worker rebuild order for `port_idx`'s device and mark
/// the port pending (cycles skip until poll_rebuild consumes the
/// completion). Data loop only. Returns whether an order is now in flight;
/// false = no config, or an earlier worker command still needs submission.
/// Callers must not write rebuild_pending themselves.
pub(crate) fn queue_rebuild<D: Direction>(state: &mut DataState<D>, port_idx: usize) -> bool {
    if state.rebuild_takeover {
        return false;
    }
    if !flush_deferred_work(state) {
        return false;
    }
    let port = &state.ports[port_idx];
    let Some(config) = port.config.clone() else {
        return false; // no negotiated format; nothing to rebuild
    };
    let request = RebuildRequest {
        port_idx,
        generation: port.generation,
        config,
        path: state.dsp_path.clone(),
        // loop-owned (the prime paths read it here), so this data-loop read
        // is the serialization-correct snapshot
        oss_fragment: state.oss_fragment,
        retried: false,
        retire_first: None,
        log: state.log.clone(),
        shared: std::sync::Arc::downgrade(&state.shared),
    };
    submit_or_defer(state, RebuildWork::Rebuild(request));
    // The request is either in the worker slot or retained in DataState.
    state.ports[port_idx].rebuild_pending = true;
    true
}

// The unwind guard behind every worker rebuild path: a task that dies
// without depositing strands rebuild_pending forever (nothing but a
// consumed completion clears it while the node runs). Dropped while still
// armed - i.e. during the unwind - it deposits Aborted for the request's
// generation: the next running cycle drops the claim and may re-queue.
pub(super) struct DepositOnUnwind<D: Direction> {
    pub(super) shared: std::sync::Arc<NodeShared<D>>,
    pub(super) port_idx: usize,
    pub(super) generation: u64,
    pub(super) armed: bool,
}

impl<D: Direction> DepositOnUnwind<D> {
    // the normal completion: deposit the computed outcome and disarm
    pub(super) fn complete(mut self, outcome: SwapOutcome<D>) {
        self.armed = false;
        self.shared.deposit(DeviceSwap {
            port_idx: self.port_idx,
            generation: self.generation,
            outcome,
        });
    }
}

impl<D: Direction> Drop for DepositOnUnwind<D> {
    fn drop(&mut self) {
        if self.armed {
            self.shared.deposit(DeviceSwap {
                port_idx: self.port_idx,
                generation: self.generation,
                outcome: SwapOutcome::Aborted,
            });
        }
    }
}

// Runs on the per-node worker with an owned request: opens and configures
// the replacement device off the RT path and deposits the outcome into the
// shared mailbox for poll_rebuild. Atomics synchronize endpoint lifetime,
// started changes, and data-loop generation transitions around the
// potentially blocking open.
pub(super) fn rebuild_request_is_current<D: Direction>(
    shared: &NodeShared<D>,
    generation: u64,
) -> bool {
    use std::sync::atomic::Ordering;
    shared.is_alive()
        && shared.started.load(Ordering::Acquire)
        && shared.generation.load(Ordering::Acquire) == generation
}

pub(super) fn rebuild_task<D: Direction>(mut request: RebuildRequest<D>) {
    let Some(shared) = request.shared.upgrade() else {
        // clear() dropped the rendezvous before this task ran
        return;
    };
    if !shared.is_alive() {
        // clear() ran; nobody is left to consume a deposit (a retire_first
        // payload still closes when `request` drops here, on this thread)
        return;
    }
    // armed from here on: even a panicking open/close below deposits
    let guard = DepositOnUnwind {
        shared,
        port_idx: request.port_idx,
        generation: request.generation,
        armed: true,
    };
    // RetireAndRetry: the dying fd must close before the retry opens (an
    // exclusive device would EBUSY otherwise); under the guard, so a
    // panicking close still unclaims the pending flag
    if let Some(old) = request.retire_first.take() {
        drop(old);
    }
    let outcome = if !rebuild_request_is_current(&guard.shared, request.generation) {
        // A release, replacement, or Suspend/Pause landed after the queue.
        // Do not reopen, but still deliver a completion so the ordinary
        // generation fence can account for the task.
        SwapOutcome::Aborted
    } else {
        let mut dsp = D::Device::new(&request.path);
        let res = D::try_open_configure(
            &mut dsp,
            &request.config,
            request.oss_fragment,
            &request.log,
        );
        if !rebuild_request_is_current(&guard.shared, request.generation) {
            // Clear, Pause, or a concurrent data-loop transition superseded
            // the request during the potentially blocking open/configure.
            // Close the stale fd here, never in the mailbox.
            drop(dsp);
            SwapOutcome::Aborted
        } else if res == 0 {
            SwapOutcome::Installed {
                dsp,
                config: request.config.clone(),
            }
        } else if res == -libc::EBUSY && !request.retried {
            // retire_first is None again here (taken above); poll_rebuild
            // fills it with the dying fd for the retry round trip
            SwapOutcome::RetireAndRetry {
                request: RebuildRequest {
                    retried: true,
                    ..request
                },
                // try_open_configure leaves the device closed on failure;
                // reuse it on the RT side instead of constructing there.
                placeholder: dsp,
            }
        } else {
            crate::warn!(
                request.log,
                "{}: background rebuild failed ({}); the port loses its format",
                request.path,
                res
            );
            // As above, failure leaves a ready closed placeholder.
            SwapOutcome::Failed { placeholder: dsp }
        }
    };
    guard.complete(outcome);
}

// Deliver endpoint-only work after process() has ended its State phase. The
// queued closure carries only NodeShared; no State pointer crosses loops. A
// non-blocking invoke may execute inline on a single-loop host, which is why
// callers must collect the event first and call here only after dropping every
// State reference.
pub(super) fn queue_main_event<D: Direction>(
    main_loop: Option<crate::spa::Loop>,
    shared: std::sync::Arc<NodeShared<D>>,
    log: crate::spa::Log,
    event: MainEvent,
) {
    let Some(main_loop) = main_loop else {
        return;
    };
    // SAFETY: host loops outlive the queued item (queue_task's contract)
    let queued = unsafe {
        crate::utils::queue_task(&main_loop, move || {
            shared.main_event(event);
        })
    };
    if !queued {
        // emission lost: the node stays format-less until the host
        // renegotiates on its own; nothing dangles
        crate::warn!(
            log,
            "can't deliver a deferred node event (main loop unavailable)"
        );
    }
}

// At the start of each data-loop cycle, apply
// a deposited rebuild completion. A matching generation applies it; a stale
// one (superseded by install/release/Suspend) is retired to the worker for
// closing. Returns whether the cycle must skip the port (rebuild still
// in flight, or this cycle consumed a non-install outcome).
pub(crate) fn poll_rebuild<D: Direction>(state: &mut DataState<D>, port_idx: usize) -> bool {
    if state.rebuild_takeover {
        return true;
    }
    if !flush_deferred_work(state) {
        return true;
    }
    let Some(swap) = state.shared.take_swap() else {
        return state.ports[port_idx].rebuild_pending;
    };
    debug_assert_eq!(
        swap.port_idx, port_idx,
        "single mailbox slot: MAX_PORTS == 1"
    );
    if swap.generation != state.ports[port_idx].generation {
        // superseded; the payload may hold an open device - transfer the
        // whole owned message to the blocking-I/O worker.
        submit_or_defer(state, RebuildWork::RetireSwap(swap));
        return state.ports[port_idx].rebuild_pending;
    }
    match swap.outcome {
        SwapOutcome::Installed { dsp, config } => {
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, dsp);
            port.config = Some(config);
            port.generation = port.generation.wrapping_add(1);
            state
                .shared
                .generation
                .store(port.generation, std::sync::atomic::Ordering::Release);
            port.rebuild_pending = false;
            port.was_matching = false; // force a relock when matching resumes
            D::on_device_swapped(state, port_idx);
            crate::info!(
                state.log,
                "{}: background device rebuild applied",
                state.dsp_path
            );
            submit_or_defer(state, RebuildWork::RetireDevice(old));
            false // the cycle continues on the fresh device (prime re-runs)
        }
        SwapOutcome::Aborted => {
            // stopped when the task ran; drop the claim so the next cycle
            // (running again, or it wouldn't poll) can re-queue
            state.ports[port_idx].rebuild_pending = false;
            true
        }
        SwapOutcome::RetireAndRetry {
            mut request,
            placeholder,
        } => {
            // swap the dying fd out behind a closed placeholder so the
            // retry's open can succeed on an exclusive device; it rides the
            // request as retire_first, so close-then-retry runs as one
            // worker command (ordering holds) under the task's unwind guard
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, placeholder);
            request.retire_first = Some(old);
            submit_or_defer(state, RebuildWork::Rebuild(request));
            true
        }
        SwapOutcome::Failed { placeholder } => {
            // mirror install_device's failure shape: closed device, cleared
            // config, and a re-announce so the host renegotiates instead of
            // believing a format is set on a dead port
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, placeholder);
            port.config = None;
            port.generation = port.generation.wrapping_add(1);
            state
                .shared
                .generation
                .store(port.generation, std::sync::atomic::Ordering::Release);
            port.rebuild_pending = false;
            port.was_matching = false;
            D::on_device_swapped(state, port_idx);
            submit_or_defer(state, RebuildWork::RetireDevice(old));
            // process() extracts and queues this only after its &mut DataState
            // phase ends. The endpoint epoch prevents an old loss from
            // overwriting a newer successful format publication.
            state.pending_main_event = Some(MainEvent::FormatLost {
                expected_publication_epoch: state.events.format_publication_epoch(),
            });
            true
        }
    }
}

// spa_node_callbacks leads with `version: u32` (the SPA vtable convention,
// spa/node/node.h); NodeCallbacks::set's prefix read below depends on it
const _: () = assert!(std::mem::offset_of!(spa_node_callbacks, version) == 0);

// A version-checked copy of the host callback table. Hosts must call
// set_callbacks again to publish changes; in-place table mutations are not
// observed.
