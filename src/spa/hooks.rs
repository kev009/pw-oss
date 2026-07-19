use super::*;

// Iterate with a cursor hook woven into the list (the C spa_hook_list_call
// pattern): the cursor hops past each entry BEFORE its callback runs, so a
// listener may remove any hook - its own, the next one, any other - and the
// plain list unlink fixes the cursor's neighbor pointers like any node's.
// The former grab-next-first walk only survived self-removal.
// The closure receives the hook's spa_callbacks BY VALUE (it is Copy), never
// a reference into the hook: a callback that removes and frees its own hook
// must not leave the closure holding a pointer into freed memory. For the
// same reason callers emit ONE method per traversal (C's spa_hook_list_call
// shape) - a second callback on the same hook after the first freed it would
// still be a use-after-free even with the copied callbacks.
pub(crate) unsafe fn for_each_hook(head: *mut spa_hook_list, mut apply: impl FnMut(spa_callbacks)) {
    struct CursorGuard(*mut spa_list);

    impl Drop for CursorGuard {
        fn drop(&mut self) {
            unsafe {
                (*(*self.0).prev).next = (*self.0).next;
                (*(*self.0).next).prev = (*self.0).prev;
            }
        }
    }

    let list = unsafe { std::ptr::addr_of_mut!((*head).list) };
    // all-zero is the C-struct cursor's valid initial state
    let mut cursor: spa_hook = unsafe { std::mem::zeroed() };
    let cur = std::ptr::addr_of_mut!(cursor.link);

    // insert the cursor at the front
    unsafe {
        (*cur).prev = list;
        (*cur).next = (*list).next;
        (*(*list).next).prev = cur;
        (*list).next = cur;
    }
    // Unlink on both normal return and a Rust-only unwind from `apply`.
    let _cursor_guard = CursorGuard(cur);

    loop {
        let item = unsafe { (*cur).next };
        if item == list {
            break;
        }
        // hop the cursor over the item: unlink, then relink after it
        unsafe {
            (*(*cur).prev).next = (*cur).next;
            (*(*cur).next).prev = (*cur).prev;
            (*cur).prev = item;
            (*cur).next = (*item).next;
            (*(*item).next).prev = cur;
            (*item).next = cur;
        }

        // spa_hook's link is its first field, so the list node IS the hook.
        // Null funcs marks another iteration's cursor woven into this list
        // (a listener callback re-entering an emission path); skip it like
        // C's spa_callback_check does.
        let hook = unsafe { item.cast::<spa_hook>().as_ref() }.expect("broken spa_hook_list");
        if hook.cb.funcs.is_null() {
            continue;
        }
        // copy the callbacks out before the callback can free the hook
        apply(hook.cb);
    }
}

// A listener-events vtable an emission can be routed through: ties the
// events struct to the minimum version we emit to.
// Copy so emissions can hand closures the vtable by value (see emit_events).
pub(crate) trait HookEvents: Copy {
    const VERSION_MIN: u32;
}

impl HookEvents for spa_node_events {
    const VERSION_MIN: u32 = SPA_VERSION_NODE_EVENTS;
}

impl HookEvents for spa_device_events {
    const VERSION_MIN: u32 = SPA_VERSION_DEVICE_EVENTS;
}

// every SPA events vtable leads with `version: u32` (the spa_interface
// convention, spa/utils/hook.h); emit_events' prefix read depends on it
const _: () = assert!(
    std::mem::offset_of!(spa_node_events, version) == 0
        && std::mem::offset_of!(spa_device_events, version) == 0
);

// Emit ONE listener method to every hook in the list (see for_each_hook for
// the one-method-per-traversal contract): each hook's funcs is viewed as the
// typed events vtable and `call` receives a COPY of it with the hook's data
// pointer, so the closure never holds a borrow into the listener-owned
// vtable while it calls out (a callback could otherwise invalidate it).
// The version prefix (offset 0, asserted above) is read alone FIRST: a
// listener built against an older, shorter vtable must be rejected before
// the full E - possibly larger in this build - is copied out of the
// listener's allocation. A too-old listener is skipped - soft, like C's
// spa_callbacks_call version gate - never asserted on: these emissions
// run under extern "C" callers, where a panic aborts the whole daemon.
//
// # Safety
// `head` must point at an initialized hook list whose hooks carry E vtables
// (the matching add_listener contract), valid for the whole traversal.
pub(crate) unsafe fn emit_events<E: HookEvents>(
    head: *mut spa_hook_list,
    mut call: impl FnMut(E, *mut c_void),
) {
    unsafe {
        for_each_hook(head, |cb| {
            // non-null: for_each_hook skips null-funcs entries (cursors)
            assert!(
                !cb.funcs.is_null(),
                "hook funcs are non-null past for_each_hook"
            );
            if version_ok(cb.funcs.cast::<u32>().read(), E::VERSION_MIN) {
                call(cb.funcs.cast::<E>().read(), cb.data);
            }
        });
    }
}

// A typed spa_hook_list. Hooks enter through typed add_listener functions, so
// each funcs pointer names an E vtable. emit_events checks the version prefix
// before reading the full table.
pub(crate) struct ListenerList<E: HookEvents> {
    // Box: the head is intrusive and self-referential (its links point back
    // at it, and every registered hook links to it), so it needs an address
    // that survives moves of the ListenerList value; the heap cell is
    // initialized once in new() and never moves again. UnsafeCell: listener
    // callbacks re-enter through raw pointers while emit() walks the list
    // (add_listener's isolate/join, a hook unlinking itself), so no Rust
    // reference may claim exclusive access to the head across an emission -
    // every access goes through the raw cell pointer.
    list: Box<std::cell::UnsafeCell<spa_hook_list>>,
    _events: std::marker::PhantomData<E>,
}

impl<E: HookEvents> ListenerList<E> {
    pub(crate) fn new() -> Self {
        let list = Box::new(std::cell::UnsafeCell::new(spa_hook_list {
            list: spa_list {
                next: std::ptr::null_mut(),
                prev: std::ptr::null_mut(),
            },
        }));
        // Initialize self-referential links at their final heap address.
        unsafe { spa_hook_list_init(list.get()) };
        Self {
            list,
            _events: std::marker::PhantomData,
        }
    }

    /// Register one listener in isolation while its initial events are emitted.
    ///
    /// The save list stays at a stable address until the closure returns, and
    /// the guard restores the full list before the result leaves this method.
    /// It also restores the list if a Rust-only caller unwinds.
    ///
    /// # Safety
    ///
    /// `listener` must point to writable hook storage, `events` must point to a
    /// valid `E` vtable, and `data` must remain valid for that vtable's calls
    /// for as long as the listener remains linked.
    pub(crate) unsafe fn with_isolated_listener<R>(
        &self,
        listener: *mut spa_hook,
        events: *const E,
        data: *mut c_void,
        initial: impl FnOnce() -> R,
    ) -> R {
        struct JoinGuard {
            list: *mut spa_hook_list,
            save: *mut spa_hook_list,
        }

        impl Drop for JoinGuard {
            fn drop(&mut self) {
                unsafe { spa_hook_list_join(self.list, self.save) };
            }
        }

        let mut save_storage = std::mem::MaybeUninit::<spa_hook_list>::uninit();
        let save = save_storage.as_mut_ptr();
        unsafe {
            spa_hook_list_isolate(self.list.get(), save, listener, events.cast(), data);
        }
        // Keep only a raw pointer across arbitrary listener code: callbacks
        // may unlink hooks in the saved list, so an &mut save would make those
        // raw list mutations violate Rust exclusivity.
        let guard = JoinGuard {
            list: self.list.get(),
            save,
        };
        let result = initial();
        drop(guard);
        // Keep the stack allocation lexical through join without ever
        // creating a Rust reference to its initialized spa_hook_list value.
        let _ = &mut save_storage;
        result
    }

    /// Move every hook from `other` to the tail of this list.
    ///
    /// # Safety
    ///
    /// Neither list may be under traversal, and they must be distinct list
    /// heads. Both lists carry the same `E` vtable type by construction.
    pub(crate) unsafe fn append_from(&self, other: &Self) {
        let list = self.list.get();
        let other = other.list.get();
        assert_ne!(list, other, "a listener list cannot append itself");
        unsafe {
            let list_head = std::ptr::addr_of_mut!((*list).list);
            let other_head = std::ptr::addr_of_mut!((*other).list);
            if (*other_head).next == other_head {
                return;
            }
            // spa_list_insert_list inserts after its first argument. Using
            // the destination tail preserves listener registration order.
            spa_list_insert_list((*list_head).prev, other_head);
            spa_hook_list_init(other);
        }
    }

    // Emit one listener method to every hook (see emit_events for the
    // one-method-per-traversal contract and the version gate). Safe per the
    // construction invariant above; &self on purpose - a callback may
    // re-enter add_listener or unlink hooks mid-walk through raw pointers,
    // mutation an exclusive &mut here would falsely rule out. The closure
    // still owns the unsafe FFI call into each vtable entry it extracts.
    // This proves only the list traversal: an owner whose C callback can
    // re-enter other state must separately end those state borrows before
    // calling emit (the node/device/monitor endpoint dispatch contracts).
    pub(crate) fn emit(&self, call: impl FnMut(E, *mut c_void)) {
        // SAFETY: the head was initialized at its final heap address in
        // new(), every hook carries an E vtable (the construction invariant
        // above), and for_each_hook's woven cursor is built for reentrant
        // list mutation during the walk
        unsafe { emit_events(self.list.get(), call) };
    }
}

pub(crate) fn dev_emit_result(
    hooks: &ListenerList<spa_device_events>,
    seq: c_int,
    res: c_int,
    type_: u32,
    result: &spa_result_device_params,
) {
    hooks.emit(|f, data| {
        if let Some(result_fun) = f.result {
            // one emission through the C listener vtable (the add_listener
            // contract keeps data valid for the call)
            unsafe { result_fun(data, seq, res, type_, result as *const _ as *const c_void) };
        }
    });
}
pub(crate) fn node_emit_result(
    hooks: &ListenerList<spa_node_events>,
    seq: c_int,
    res: c_int,
    type_: u32,
    result: &spa_result_node_params,
) {
    hooks.emit(|f, data| {
        if let Some(result_fun) = f.result {
            // one emission through the C listener vtable (the add_listener
            // contract keeps data valid for the call)
            unsafe { result_fun(data, seq, res, type_, result as *const _ as *const c_void) };
        }
    });
}

#[cfg(test)]
mod tests;

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
