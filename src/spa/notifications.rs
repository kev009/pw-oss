use std::cell::RefCell;
use std::collections::VecDeque;

struct Pending<N> {
    queue: VecDeque<N>,
    dispatching: bool,
}

/// A main-loop notification queue that preserves FIFO transaction order when
/// listener callbacks synchronously re-enter their owner.
pub(crate) struct LocalNotificationQueue<N> {
    pending: RefCell<Pending<N>>,
}

pub(crate) struct LocalDispatchGuard<'a, N> {
    queue: &'a LocalNotificationQueue<N>,
}

impl<N> Drop for LocalDispatchGuard<'_, N> {
    fn drop(&mut self) {
        self.queue.pending.borrow_mut().dispatching = false;
    }
}

impl<N> LocalNotificationQueue<N> {
    pub(crate) fn new() -> Self {
        Self {
            pending: RefCell::new(Pending {
                queue: VecDeque::new(),
                dispatching: false,
            }),
        }
    }

    /// Build and enqueue a notification only when an older transaction owns
    /// dispatch or is already waiting. The associated value lets callers keep
    /// a listener cohort beside its activation barrier.
    pub(crate) fn defer_when_busy<R>(&self, build: impl FnOnce() -> (N, R)) -> Option<R> {
        let mut pending = self.pending.borrow_mut();
        if pending.dispatching || !pending.queue.is_empty() {
            let (notification, value) = build();
            pending.queue.push_back(notification);
            Some(value)
        } else {
            None
        }
    }

    pub(crate) fn begin_dispatch(&self) -> Option<LocalDispatchGuard<'_, N>> {
        let mut pending = self.pending.borrow_mut();
        if pending.dispatching {
            None
        } else {
            pending.dispatching = true;
            drop(pending);
            Some(LocalDispatchGuard { queue: self })
        }
    }

    /// Drain without holding a RefCell borrow across callbacks, allowing a
    /// callback to append another complete transaction safely.
    pub(crate) fn drain(&self, _guard: LocalDispatchGuard<'_, N>, mut dispatch: impl FnMut(N)) {
        loop {
            let notification = self.pending.borrow_mut().queue.pop_front();
            match notification {
                Some(notification) => dispatch(notification),
                None => return,
            }
        }
    }

    /// Append the full transaction before dispatch starts so reentrant work
    /// remains behind it.
    pub(crate) fn dispatch_all(
        &self,
        notifications: impl IntoIterator<Item = N>,
        dispatch: impl FnMut(N),
    ) {
        self.pending.borrow_mut().queue.extend(notifications);
        if let Some(guard) = self.begin_dispatch() {
            self.drain(guard, dispatch);
        }
    }
}
