use super::*;

// hand-woven intrusive list: n hooks whose cb.data carries their index
// (boxed nodes on purpose: the links are intrusive, so growth must not
// move them)
#[allow(clippy::vec_box)]
fn hook_list(n: usize) -> (Box<spa_hook_list>, Vec<Box<spa_hook>>) {
    let mut head: Box<spa_hook_list> = Box::new(unsafe { std::mem::zeroed() });
    let list = std::ptr::addr_of_mut!(head.list);
    unsafe {
        (*list).next = list;
        (*list).prev = list;
    }
    let mut hooks = Vec::new();
    for i in 0..n {
        let mut h: Box<spa_hook> = Box::new(unsafe { std::mem::zeroed() });
        h.cb.funcs = std::ptr::dangling(); // non-null marks a real hook; never called
        h.cb.data = i as *mut std::os::raw::c_void;
        let link = std::ptr::addr_of_mut!(h.link);
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
    let link = std::ptr::addr_of_mut!(h.link);
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
    let h1 = std::ptr::addr_of_mut!(*hooks[1]);
    let mut seen = Vec::new();
    unsafe {
        for_each_hook(&mut *head, |cb| {
            seen.push(cb.data as usize);
            if cb.data as usize == 0 {
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
    unsafe { for_each_hook(&mut *head, |cb| seen.push(cb.data as usize)) };
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
    list.emit(|_events, data| seen.push(data as usize));
    assert_eq!(seen, [2, 3]);
}

// the per-method-traversal contract behind the add_listener emitters: a
// callback that removes its own hook during one traversal is not visited
// by the next one (so freeing the hook mid-callback stays sound)
#[test]
fn self_removal_hides_the_hook_from_later_traversals() {
    let (mut head, mut hooks) = hook_list(2);
    let h0 = std::ptr::addr_of_mut!(*hooks[0]);
    let mut first = Vec::new();
    let mut second = Vec::new();
    unsafe {
        for_each_hook(&mut *head, |cb| {
            first.push(cb.data as usize);
            if cb.data as usize == 0 {
                unlink(&mut *h0); // hook 0's callback removes hook 0
            }
        });
        for_each_hook(&mut *head, |cb| second.push(cb.data as usize));
    }
    assert_eq!(first, [0, 1]);
    assert_eq!(second, [1]);
}

// a callback re-entering an emission path iterates the same list; the
// outer walk's cursor (null funcs) must be invisible to the inner one
#[test]
fn nested_iteration_skips_the_outer_cursor() {
    let (mut head, _hooks) = hook_list(2);
    let head_ptr = std::ptr::addr_of_mut!(*head);
    let mut outer = Vec::new();
    let mut inner = Vec::new();
    unsafe {
        for_each_hook(head_ptr, |cb| {
            outer.push(cb.data as usize);
            if cb.data as usize == 0 {
                for_each_hook(head_ptr, |icb| inner.push(icb.data as usize));
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
            7 as *mut std::os::raw::c_void,
            || {},
        );
    }

    let moved = list; // move the handle; the boxed head must not move
    let mut seen = Vec::new();
    moved.emit(|_events, data| seen.push(data as usize));
    assert_eq!(seen, [7]);
}

// Info payloads contain raw self-pointers. A callback snapshot must
// point into its own stable allocation after returning from snapshot(),
// not at the moved temporary or the mutable live info it copied.
#[test]
fn info_snapshots_reweave_their_self_pointers() {
    let mut node = NodeInfo::new();
    node.add_prop("snapshot.key", "snapshot.value");
    node.add_param(SPA_PARAM_Props, SPA_PARAM_INFO_READ);
    node.fix_pointers();
    let node = node.snapshot();
    let node_raw = unsafe { &*node.raw() };
    assert_eq!(node_raw.params, node.params.as_ptr().cast_mut());
    assert_eq!(
        node_raw.props,
        std::ptr::addr_of!(node.props.dict).cast_mut()
    );

    let mut port = PortInfo::new();
    port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
    port.fix_pointers();
    let port = port.snapshot();
    let port_raw = unsafe { &*port.raw() };
    assert_eq!(port_raw.params, port.params.as_ptr().cast_mut());
    assert_eq!(
        port_raw.props,
        std::ptr::addr_of!(port.props.dict).cast_mut()
    );

    let mut device = DeviceInfo::new();
    device.add_prop("snapshot.key", "snapshot.value");
    device.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READ);
    device.fix_pointers();
    let device = device.snapshot();
    let device_raw = unsafe { &*device.raw() };
    assert_eq!(device_raw.params, device.params.as_ptr().cast_mut());
    assert_eq!(
        device_raw.props,
        std::ptr::addr_of!(device.props.dict).cast_mut()
    );
}

// A result callback may mutate the enumerated object. The next build
// step must reacquire State and observe that mutation; retaining &mut S
// across emit would make this exact pattern formally unsound.
#[test]
fn enumeration_reborrows_state_after_reentrant_emit() {
    let mut state = vec![10i32, 20];
    let state_ptr = &raw mut state;
    let mut built = Vec::new();
    let build = |state: &mut Vec<i32>, index: u32| {
        let value = state[index as usize];
        built.push(value);
        ParamStep::Built(crate::utils::serialize_pod(&libspa::pod::Value::Int(value)))
    };
    let emit = |index: u32, _param: *mut spa_pod| {
        if index == 0 {
            // SAFETY: enum_params_loop guarantees its per-step reference
            // ended before emit.
            unsafe { (&mut *state_ptr)[1] = 99 };
        }
    };
    let result = unsafe { enum_params_loop(state_ptr, (0, 2), std::ptr::null(), build, emit) };
    assert_eq!(result, 0);
    assert_eq!(built, [10, 99]);
}
