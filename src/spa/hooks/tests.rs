use super::*;

// hand-woven intrusive list: n hooks whose cb.data carries their index
// (boxed nodes on purpose: the links are intrusive, so growth must not
// move them)
#[expect(clippy::vec_box)]
fn hook_list(n: usize) -> (Box<spa_hook_list>, Vec<Box<spa_hook>>) {
    let mut head: Box<spa_hook_list> = Box::new(unsafe { std::mem::zeroed() });
    let list = &raw mut head.list;
    unsafe {
        (*list).next = list;
        (*list).prev = list;
    }
    let mut hooks = Vec::new();
    for i in 0..n {
        let mut h: Box<spa_hook> = Box::new(unsafe { std::mem::zeroed() });
        h.cb.funcs = std::ptr::dangling(); // non-null marks a real hook; never called
        h.cb.data = std::ptr::without_provenance_mut::<c_void>(i);
        let link = &raw mut h.link;
        unsafe {
            // append
            (*link).prev = (*list).prev;
            (*link).next = list;
            (*(*list).prev).next = link;
            (*list).prev = link;
        }
        hooks.push(h);
    }
    (head, hooks)
}

fn unlink(h: &mut spa_hook) {
    let link = &raw mut h.link;
    unsafe {
        (*(*link).prev).next = (*link).next;
        (*(*link).next).prev = (*link).prev;
    }
}

// a callback removing the NEXT hook must not dangle the walk (a
// grab-next-before-calling walk would)
#[test]
fn hook_callback_may_remove_the_next_hook() {
    let (mut head, mut hooks) = hook_list(3);
    let h1 = &raw mut *hooks[1];
    let mut seen = Vec::new();
    unsafe {
        for_each_hook(&mut *head, |cb| {
            seen.push(cb.data.addr());
            if cb.data.addr() == 0 {
                unlink(&mut *h1); // hook 0's callback frees hook 1
            }
        });
    }
    assert_eq!(seen, [0, 2]);
}

#[test]
fn hook_cursor_is_unlinked_during_rust_unwind() {
    let (mut head, _hooks) = hook_list(2);
    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| unsafe {
        for_each_hook(&mut *head, |_cb| panic!("injected traversal panic"));
    }));
    assert!(panicked.is_err());

    let mut seen = Vec::new();
    unsafe { for_each_hook(&mut *head, |cb| seen.push(cb.data.addr())) };
    assert_eq!(seen, [0, 1], "the stack cursor must not remain linked");
}

#[test]
fn isolated_listener_allows_saved_hook_removal_and_unwind() {
    let list = ListenerList::<spa_node_events>::new();
    let mut table: spa_node_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_NODE_EVENTS;
    let mut old_hook: spa_hook = unsafe { std::mem::zeroed() };
    let mut new_hook: spa_hook = unsafe { std::mem::zeroed() };
    let mut unwind_hook: spa_hook = unsafe { std::mem::zeroed() };

    unsafe {
        list.with_isolated_listener(
            &mut old_hook,
            &raw const table,
            std::ptr::without_provenance_mut::<c_void>(1),
            || {},
        );
        list.with_isolated_listener(
            &mut new_hook,
            &raw const table,
            std::ptr::without_provenance_mut::<c_void>(2),
            || spa_hook_remove(&mut old_hook),
        );
    }

    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| unsafe {
        list.with_isolated_listener(
            &mut unwind_hook,
            &raw const table,
            std::ptr::without_provenance_mut::<c_void>(3),
            || panic!("injected initial-listener panic"),
        );
    }));
    assert!(panicked.is_err());

    let mut seen = Vec::new();
    list.emit(|_events, data| seen.push(data.addr()));
    assert_eq!(seen, [2, 3]);
}

// the per-method-traversal contract behind the add_listener emitters: a
// callback that removes its own hook during one traversal is not visited
// by the next one (so freeing the hook mid-callback stays sound)
#[test]
fn self_removal_hides_the_hook_from_later_traversals() {
    let (mut head, mut hooks) = hook_list(2);
    let h0 = &raw mut *hooks[0];
    let mut first = Vec::new();
    let mut second = Vec::new();
    unsafe {
        for_each_hook(&mut *head, |cb| {
            first.push(cb.data.addr());
            if cb.data.addr() == 0 {
                unlink(&mut *h0); // hook 0's callback removes hook 0
            }
        });
        for_each_hook(&mut *head, |cb| second.push(cb.data.addr()));
    }
    assert_eq!(first, [0, 1]);
    assert_eq!(second, [1]);
}

// a callback re-entering an emission path iterates the same list; the
// outer walk's cursor (null funcs) must be invisible to the inner one
#[test]
fn nested_iteration_skips_the_outer_cursor() {
    let (mut head, _hooks) = hook_list(2);
    let head_ptr = &raw mut *head;
    let mut outer = Vec::new();
    let mut inner = Vec::new();
    unsafe {
        for_each_hook(head_ptr, |cb| {
            outer.push(cb.data.addr());
            if cb.data.addr() == 0 {
                for_each_hook(head_ptr, |icb| inner.push(icb.data.addr()));
            }
        });
    }
    assert_eq!(outer, [0, 1]);
    assert_eq!(inner, [0, 1]); // both real hooks, no phantom cursor
}

// the head is boxed precisely so the handle may move while hooks stay
// linked to a stable address: register a hook, move the ListenerList
// value, and the emission must still reach the hook (with the head
// inline, the old address would keep dangling links)
#[test]
fn listener_list_emits_after_the_handle_moves() {
    let mut events: Box<spa_node_events> = Box::new(unsafe { std::mem::zeroed() });
    events.version = SPA_VERSION_NODE_EVENTS; // pass the version gate
    let list: ListenerList<spa_node_events> = ListenerList::new();

    // register a hook the way add_listener does
    let mut hook: Box<spa_hook> = Box::new(unsafe { std::mem::zeroed() });
    unsafe {
        list.with_isolated_listener(
            &mut *hook,
            &raw const *events,
            std::ptr::without_provenance_mut::<c_void>(7),
            || {},
        );
    }

    let moved = list; // move the handle; the boxed head must not move
    let mut seen = Vec::new();
    moved.emit(|_events, data| seen.push(data.addr()));
    assert_eq!(seen, [7]);
}

#[test]
fn listener_target_restores_after_nesting_and_unwind() {
    let fallback = ListenerList::<spa_node_events>::new();
    let outer = ListenerList::<spa_node_events>::new();
    let inner = ListenerList::<spa_node_events>::new();
    let target = LocalListenerTarget::new();
    let current_is = |expected: &ListenerList<spa_node_events>| {
        target.with_current(&fallback, |current| std::ptr::eq(current, expected))
    };

    assert!(current_is(&fallback));
    target.scoped(&outer, || {
        assert!(current_is(&outer));
        target.scoped(&inner, || assert!(current_is(&inner)));
        assert!(current_is(&outer));
    });
    assert!(current_is(&fallback));

    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        target.scoped(&outer, || panic!("injected listener target panic"));
    }));
    assert!(panicked.is_err());
    assert!(current_is(&fallback));
}
