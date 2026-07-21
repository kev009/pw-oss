use super::*;

#[cfg(test)]
mod tests;

struct EventInfo {
    node: crate::spa::NodeInfo,
    port: crate::spa::PortInfo,
}

enum NodeNotification {
    Node(Box<crate::spa::NodeInfo>),
    Port(Box<crate::spa::PortInfo>),
    Done(c_int),
    ActivateListeners(std::sync::Arc<crate::spa::ListenerList<spa_node_events>>),
}

struct PendingNodeNotifications {
    queue: std::collections::VecDeque<NodeNotification>,
    dispatching: bool,
}

// Main-loop-owned listener and info state. Callbacks run against owned
// snapshots after mutations finish, so listener reentry cannot alias or
// invalidate the live payload. Cross-loop code receives only the atomic
// publication counter and MainEventTarget, never this listener endpoint.
pub(super) struct NodeEvents<D: Direction> {
    pub(super) hooks: crate::spa::ListenerList<spa_node_events>,
    info: std::sync::Mutex<EventInfo>,
    pending: std::sync::Mutex<PendingNodeNotifications>,
    result_target: crate::spa::LocalListenerTarget<spa_node_events>,
    // Deferred main-loop delivery upgrades this weak self-reference before it
    // invokes listeners. The resulting Rc keeps the endpoint alive if a
    // listener synchronously destroys the node.
    self_weak: std::rc::Weak<NodeEvents<D>>,
    // Changes only when the advertised Format/Buffers state is published.
    // Deferred FormatLost messages carry the value they observed so a newer
    // successful format publication cannot be overwritten by a stale task.
    format_publication: FormatPublication,
    _direction: std::marker::PhantomData<fn() -> D>,
}

#[derive(Clone)]
pub(super) struct FormatPublication(std::sync::Arc<std::sync::atomic::AtomicU64>);

impl FormatPublication {
    fn new() -> Self {
        Self(std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0)))
    }

    pub(super) fn epoch(&self) -> u64 {
        self.0.load(std::sync::atomic::Ordering::Acquire)
    }

    fn advance(&self) {
        self.0.fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    }
}

pub(super) struct MainEventTarget<D: Direction> {
    events: *const NodeEvents<D>,
    alive: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

// SAFETY: the raw pointer is never dereferenced on the sending thread. On the
// main loop, `alive` guarantees that a strong owner remains long enough to
// upgrade NodeEvents::self_weak. Delivery holds that owner through callbacks,
// including callbacks that synchronously clear the node.
unsafe impl<D: Direction> Send for MainEventTarget<D> {}

impl<D: Direction> Clone for MainEventTarget<D> {
    fn clone(&self) -> Self {
        Self {
            events: self.events,
            alive: self.alive.clone(),
        }
    }
}

impl<D: Direction> MainEventTarget<D> {
    pub(super) fn new(
        events: &std::rc::Rc<NodeEvents<D>>,
        alive: std::sync::Arc<std::sync::atomic::AtomicBool>,
    ) -> Self {
        Self {
            events: std::rc::Rc::as_ptr(events),
            alive,
        }
    }

    // SAFETY: callers must run this on the main loop. clear() runs there too,
    // stores false before dropping the State owner, and cannot interleave
    // between the liveness check and the weak upgrade.
    pub(super) unsafe fn deliver_on_main(&self, event: MainEvent) {
        if !self.alive.load(std::sync::atomic::Ordering::Acquire) {
            return;
        }
        let Some(events) = (unsafe { &*self.events }).self_weak.upgrade() else {
            return;
        };
        match event {
            MainEvent::FormatLost {
                expected_publication_epoch,
            } => unsafe { events.emit_format_lost_now(expected_publication_epoch) },
        }
    }
}

pub(super) struct NodeDispatchClaim<'a, D: Direction>(&'a NodeEvents<D>);

impl<D: Direction> Drop for NodeDispatchClaim<'_, D> {
    fn drop(&mut self) {
        self.0.pending.lock_unpoisoned().dispatching = false;
    }
}

impl<D: Direction> NodeEvents<D> {
    pub(super) fn new() -> std::rc::Rc<Self> {
        std::rc::Rc::new_cyclic(|self_weak| Self {
            hooks: crate::spa::ListenerList::new(),
            info: std::sync::Mutex::new(EventInfo {
                node: crate::spa::NodeInfo::new(),
                port: crate::spa::PortInfo::new(),
            }),
            pending: std::sync::Mutex::new(PendingNodeNotifications {
                queue: std::collections::VecDeque::new(),
                dispatching: false,
            }),
            result_target: crate::spa::LocalListenerTarget::new(),
            self_weak: self_weak.clone(),
            format_publication: FormatPublication::new(),
            _direction: std::marker::PhantomData,
        })
    }

    pub(super) fn format_publication(&self) -> FormatPublication {
        self.format_publication.clone()
    }

    pub(super) fn with_info<R>(
        &self,
        apply: impl FnOnce(&mut crate::spa::NodeInfo, &mut crate::spa::PortInfo) -> R,
    ) -> R {
        let mut info = self.info.lock_unpoisoned();
        let EventInfo { node, port } = &mut *info;
        apply(node, port)
    }

    pub(super) fn with_node_info<R>(
        &self,
        apply: impl FnOnce(&mut crate::spa::NodeInfo) -> R,
    ) -> R {
        self.with_info(|node, _port| apply(node))
    }

    pub(super) fn with_port_info<R>(
        &self,
        apply: impl FnOnce(&mut crate::spa::PortInfo) -> R,
    ) -> R {
        self.with_info(|_node, port| apply(port))
    }

    pub(super) fn initial_snapshots(
        &self,
    ) -> (Box<crate::spa::NodeInfo>, Box<crate::spa::PortInfo>) {
        self.with_info(|node, port| {
            let mut node = node.snapshot();
            let mut port = port.snapshot();
            let _ = node.replace_change_mask(crate::spa::SPA_NODE_CHANGE_MASK_ALL as u64);
            let _ = port.replace_change_mask(crate::spa::SPA_PORT_CHANGE_MASK_ALL as u64);
            (node, port)
        })
    }

    pub(super) fn queue_node_info(&self) {
        let snapshot = self.with_node_info(|info| {
            let snapshot = info.snapshot();
            let _ = info.replace_change_mask(0);
            snapshot
        });
        self.pending
            .lock_unpoisoned()
            .queue
            .push_back(NodeNotification::Node(snapshot));
    }

    pub(super) fn queue_port_info(&self) {
        let snapshot = self.with_port_info(|info| {
            let snapshot = info.snapshot();
            let _ = info.replace_change_mask(0);
            snapshot
        });
        self.pending
            .lock_unpoisoned()
            .queue
            .push_back(NodeNotification::Port(snapshot));
    }

    pub(super) fn format_publication_epoch(&self) -> u64 {
        self.format_publication.epoch()
    }

    pub(super) fn advance_format_publication_epoch(&self) {
        self.format_publication.advance();
    }

    // Register immediately on the active list when no older FIFO work exists.
    // Otherwise enqueue an activation barrier before the synchronous initial
    // callbacks: older notifications stay ahead of the new listener, while
    // anything those callbacks queue lands after it.
    pub(super) unsafe fn with_new_listener<R>(
        &self,
        listener: *mut spa_hook,
        events: *const spa_node_events,
        data: *mut c_void,
        initial: impl FnOnce(&crate::spa::ListenerList<spa_node_events>) -> R,
    ) -> R {
        let deferred = {
            let mut pending = self.pending.lock_unpoisoned();
            if pending.dispatching || !pending.queue.is_empty() {
                // Reentrant draining may consume the queued activation while
                // the synchronous initial callback still uses this cohort.
                // Keep a local owner through that callback; listener access
                // itself remains main-loop-only.
                #[expect(clippy::arc_with_non_send_sync)]
                let hooks = std::sync::Arc::new(crate::spa::ListenerList::new());
                pending
                    .queue
                    .push_back(NodeNotification::ActivateListeners(hooks.clone()));
                Some(hooks)
            } else {
                None
            }
        };
        let hooks = deferred.as_deref().unwrap_or(&self.hooks);
        unsafe {
            hooks.with_isolated_listener(listener, events, data, || {
                self.result_target.scoped(hooks, || initial(hooks))
            })
        }
    }

    // SAFETY: no reference into the associated State may be live. Listener
    // code may re-enter any node method and create a new mutable State borrow.
    unsafe fn dispatch(&self, notification: &NodeNotification) {
        match notification {
            NodeNotification::Node(snapshot) => self.hooks.emit(|f, data| {
                if let Some(info) = f.info {
                    // through the C listener vtable (add_listener contract)
                    unsafe { info(data, snapshot.raw()) };
                }
            }),
            NodeNotification::Port(snapshot) => self.hooks.emit(|f, data| {
                if let Some(info) = f.port_info {
                    // through the C listener vtable (add_listener contract)
                    unsafe { info(data, D::DIRECTION, 0, snapshot.raw()) };
                }
            }),
            NodeNotification::Done(seq) => self.hooks.emit(|f, data| {
                if let Some(result) = f.result {
                    unsafe { result(data, *seq, 0, 0, std::ptr::null()) };
                }
            }),
            NodeNotification::ActivateListeners(hooks) => {
                // SAFETY: drain processes barriers between listener
                // traversals; the isolated batch finished its initial
                // callbacks before it was eligible to reach this point.
                unsafe { self.hooks.append_from(hooks) };
            }
        }
    }

    // Claim the endpoint's dispatch turn. A reentrant producer appends to the
    // same FIFO and returns; the outer owner completes its current transaction
    // before draining the nested one.
    pub(super) fn begin_dispatch(&self) -> Option<NodeDispatchClaim<'_, D>> {
        let mut pending = self.pending.lock_unpoisoned();
        if pending.dispatching {
            None
        } else {
            pending.dispatching = true;
            Some(NodeDispatchClaim(self))
        }
    }

    // Called only by the owner returned from begin_dispatch(), after the
    // surrounding State borrow has ended. Pop one notification at a time so
    // the mutex is never held across arbitrary listener code.
    // SAFETY: as dispatch(); callers must end their State phase first.
    pub(super) unsafe fn drain(&self, _claim: &NodeDispatchClaim<'_, D>) {
        loop {
            let notification = {
                let mut pending = self.pending.lock_unpoisoned();
                match pending.queue.pop_front() {
                    Some(notification) => notification,
                    None => return,
                }
            };
            unsafe { self.dispatch(&notification) };
        }
    }

    // SAFETY: no associated State reference may be live. Reentrant flushes
    // only enqueue; the outer drain preserves FIFO transaction ordering.
    pub(super) unsafe fn flush(&self) {
        if let Some(claim) = self.begin_dispatch() {
            unsafe { self.drain(&claim) };
        }
    }

    fn record_format_lost(&self) {
        self.with_port_info(|info| {
            let _ = info.replace_change_mask(0);
            info.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
            info.set_param_flags(SPA_PARAM_Buffers, 0);
            // This serial flip is what audioadapter reacts to: an EnumFormat
            // flags change sets recheck_format and starts renegotiation.
            info.bump_param(SPA_PARAM_EnumFormat);
        });
        self.queue_port_info();
    }

    pub(super) fn record_current_format_lost(&self) {
        self.record_format_lost();
        // Retire duplicate deferred losses before the queued snapshot is
        // flushed and listeners can re-enter.
        self.advance_format_publication_epoch();
    }

    // SAFETY: no State reference may be live during listener dispatch.
    pub(super) unsafe fn emit_format_lost_now(&self, expected_epoch: u64) {
        if self.format_publication_epoch() != expected_epoch {
            return;
        }
        self.record_current_format_lost();
        unsafe { self.flush() };
    }

    // SAFETY: no associated State reference may be live.
    pub(super) unsafe fn emit_done(&self, seq: c_int) {
        self.pending
            .lock_unpoisoned()
            .queue
            .push_back(NodeNotification::Done(seq));
        unsafe { self.flush() };
    }

    // SAFETY: no associated State reference may be live.
    pub(super) unsafe fn emit_result(&self, seq: c_int, result: &spa_result_node_params) {
        self.result_target.with_current(&self.hooks, |hooks| {
            crate::spa::node_emit_result(hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS, result);
        });
    }
}

pub(super) unsafe extern "C" fn add_listener<D: Direction>(
    object: *mut c_void,
    listener: *mut spa_hook,
    events: *const spa_node_events,
    data: *mut c_void,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let node_events = unsafe { main_ref(state).events.clone() };

    let initial = |hooks: &crate::spa::ListenerList<spa_node_events>| {
        // The initial emissions only reach the newly added listener (the list
        // is isolated). One method per traversal, mirroring C's
        // spa_hook_list_call: a listener that removes and frees its hook
        // inside a callback must not be read for the next method.
        let (node_info, port_info) = node_events.initial_snapshots();
        // Hold the endpoint's dispatch turn across the whole initial
        // transaction. Reentrant mutations queue behind both snapshots
        // instead of publishing newer state between them.
        let dispatch_claim = node_events.begin_dispatch();
        hooks.emit(|f, data| {
            if let Some(node_info_fun) = f.info {
                // through the C listener vtable (add_listener contract)
                unsafe { node_info_fun(data, node_info.raw()) };
            }
        });
        hooks.emit(|f, data| {
            if let Some(port_info_fun) = f.port_info {
                // through the C listener vtable (add_listener contract)
                unsafe { port_info_fun(data, D::DIRECTION, 0, port_info.raw()) };
            }
        });
        dispatch_claim
    };
    let dispatch_claim = unsafe { node_events.with_new_listener(listener, events, data, initial) };
    if let Some(claim) = dispatch_claim.as_ref() {
        // SAFETY: the State snapshot borrow ended before isolation, and the
        // scoped helper restored the complete list before nested work drains.
        unsafe { node_events.drain(claim) };
    }

    0
}

// re-emit node_info to every listener (carrying whatever change_mask the caller
// set, e.g. PARAMS), then clear the mask
pub(super) fn emit_node_info<D: Direction>(state: &MainState<D>) {
    let events = state.events.clone();
    events.queue_node_info();
}

// the process latency (user-set latency offset) shifts the node's reported
// latency, so a change re-emits the Props/ProcessLatency node params and the
// port Latency param
pub(crate) fn handle_process_latency<D: Direction>(
    state: &mut MainState<D>,
    info: spa_process_latency_info,
) {
    let ns_changed = state.process_latency.ns != info.ns;
    if state.process_latency.quantum == info.quantum
        && state.process_latency.rate == info.rate
        && !ns_changed
    {
        return;
    }

    state.process_latency = info;

    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        if ns_changed {
            info.bump_param(SPA_PARAM_Props);
        }
        info.bump_param(SPA_PARAM_ProcessLatency);
    });
    emit_node_info(state);

    state.events.with_port_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Latency);
    });
    emit_port_info(state);
}

// re-emit port_info to every listener (carrying whatever change_mask the caller
// set, e.g. RATE/PARAMS), then clear the mask
pub(super) fn emit_port_info<D: Direction>(state: &MainState<D>) {
    let events = state.events.clone();
    events.queue_port_info();
}

pub(super) unsafe extern "C" fn set_callbacks<D: Direction>(
    object: *mut c_void,
    callbacks: *const spa_node_callbacks,
    data: *mut c_void,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null());

    // SAFETY: `callbacks`, when non-null, points at a live table whose
    // version prefix describes its true length, and the host keeps `data`
    // valid while the table is set (the set_callbacks contract)
    let mut new_callbacks = NodeCallbacks::none();
    unsafe { new_callbacks.set(callbacks, data) };

    // on_wake/process call the table from the data loop; store it there.
    // SAFETY: a by-value table copy plus the host data pointer, which stays
    // valid while set (the same contract)
    let new_callbacks = unsafe { crate::spa::SendWrap::new(new_callbacks) };
    let control = unsafe { DataControl::from_raw(state) };
    if !control.invoke(move |state| state.callbacks = new_callbacks.into_inner()) {
        return -libc::EIO;
    }
    0
}

pub(super) unsafe extern "C" fn sync<D: Direction>(object: *mut c_void, seq: c_int) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { main_ref(state).events.clone() };
    // SAFETY: only the independently owned endpoint is borrowed.
    unsafe { events.emit_done(seq) };
    0
}

// emit one filtered enum_params result to every listener (node and port
// enumeration share this shape)
pub(super) unsafe fn emit_param_result<D: Direction>(
    events: &NodeEvents<D>,
    seq: c_int,
    id: u32,
    index: u32,
    param: *mut spa_pod,
) {
    let result = spa_result_node_params {
        id,
        index,
        next: index + 1,
        param,
    };
    unsafe { events.emit_result(seq, &result) };
}

pub(super) unsafe extern "C" fn enum_params<D: Direction>(
    object: *mut c_void,
    seq: c_int,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    // Clone the independently allocated endpoint before enumeration. Each
    // build step below gets a fresh State borrow; dispatch uses only events.
    let events = unsafe { main_ref(state).events.clone() };
    let main = unsafe { main_ptr(state) };

    unsafe {
        crate::spa::enum_params_loop(
            main,
            (start, max),
            filter,
            |state, index| match D::build_node_param(state, id, index) {
                ParamBuild::Built(pod) => crate::spa::ParamStep::Built(pod),
                ParamBuild::Exhausted => crate::spa::ParamStep::Stop(0),
                // unknown param id (ALSA convention)
                ParamBuild::Unknown => crate::spa::ParamStep::Stop(-libc::ENOENT),
            },
            |index, param| emit_param_result(&events, seq, id, index, param),
        )
    }
}
