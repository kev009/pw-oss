use super::*;

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

// Main-loop-owned listener and info state. The Arc gives queued messages a
// lifetime-safe endpoint without exposing State across loops. Info mutation
// is locked only on the main loop; callbacks run after the lock is released,
// against owned snapshots, so listener reentry cannot alias or invalidate the
// live payload. The hook list itself follows SPA's main-loop serialization.
pub(super) struct NodeEvents<D: Direction> {
    pub(super) hooks: crate::spa::ListenerList<spa_node_events>,
    info: std::sync::Mutex<EventInfo>,
    pending: std::sync::Mutex<PendingNodeNotifications>,
    // Changes only when the advertised Format/Buffers state is published.
    // Deferred FormatLost messages carry the value they observed so a newer
    // successful format publication cannot be overwritten by a stale task.
    format_publication_epoch: std::sync::atomic::AtomicU64,
    _direction: std::marker::PhantomData<fn() -> D>,
}

pub(super) struct NodeDispatchClaim<'a, D: Direction>(&'a NodeEvents<D>);

impl<D: Direction> Drop for NodeDispatchClaim<'_, D> {
    fn drop(&mut self) {
        self.0.pending.lock_unpoisoned().dispatching = false;
    }
}

// SAFETY: NodeEvents' safe methods serialize info through the mutex and never
// expose it. Listener-list access is confined to those methods and the SPA
// add-listener entry point, all of which the host calls on the main loop.
// Cross-loop users hold only Weak/Arc handles and queue an owned MainEvent
// back to that loop; they never traverse the list themselves.
unsafe impl<D: Direction> Send for NodeEvents<D> {}
unsafe impl<D: Direction> Sync for NodeEvents<D> {}

impl<D: Direction> NodeEvents<D> {
    pub(super) fn new() -> Self {
        Self {
            hooks: crate::spa::ListenerList::new(),
            info: std::sync::Mutex::new(EventInfo {
                node: crate::spa::NodeInfo::new(),
                port: crate::spa::PortInfo::new(),
            }),
            pending: std::sync::Mutex::new(PendingNodeNotifications {
                queue: std::collections::VecDeque::new(),
                dispatching: false,
            }),
            format_publication_epoch: std::sync::atomic::AtomicU64::new(0),
            _direction: std::marker::PhantomData,
        }
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
        self.format_publication_epoch
            .load(std::sync::atomic::Ordering::Acquire)
    }

    pub(super) fn advance_format_publication_epoch(&self) {
        self.format_publication_epoch
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
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
                // Arc is intentional even though ListenerList itself remains
                // main-loop-only: NodeEvents has cross-loop Arc handles, while
                // this atomic owner keeps a reentrantly drained cohort alive
                // through its synchronous initial callback.
                #[allow(clippy::arc_with_non_send_sync)]
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
        unsafe { hooks.with_isolated_listener(listener, events, data, || initial(hooks)) }
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
        crate::spa::node_emit_result(&self.hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS, result);
    }
}
