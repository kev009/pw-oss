use core::ffi::CStr;
use libspa::sys::*;
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_void};

mod notifications;

pub(crate) use notifications::{LocalDispatchGuard, LocalNotificationQueue};

pub(crate) const SPA_DEVICE_CHANGE_MASK_ALL: u32 =
    SPA_DEVICE_CHANGE_MASK_FLAGS | SPA_DEVICE_CHANGE_MASK_PARAMS | SPA_DEVICE_CHANGE_MASK_PROPS;

pub(crate) const SPA_DEVICE_OBJECT_CHANGE_MASK_ALL: u32 =
    SPA_DEVICE_OBJECT_CHANGE_MASK_FLAGS | SPA_DEVICE_OBJECT_CHANGE_MASK_PROPS;

pub(crate) const SPA_NODE_CHANGE_MASK_ALL: u32 =
    SPA_NODE_CHANGE_MASK_FLAGS | SPA_NODE_CHANGE_MASK_PARAMS | SPA_NODE_CHANGE_MASK_PROPS;

pub(crate) const SPA_PORT_CHANGE_MASK_ALL: u32 = SPA_PORT_CHANGE_MASK_FLAGS
    | SPA_PORT_CHANGE_MASK_PARAMS
    | SPA_PORT_CHANGE_MASK_PROPS
    | SPA_PORT_CHANGE_MASK_RATE;

// spa/node/node.h:241; the libspa-sys bindings don't carry the set_param flags
pub(crate) const SPA_NODE_PARAM_FLAG_NEAREST: u32 = 1 << 2;

// The listener-vtable version gate. The SPA_VERSION_*_EVENTS constants are
// currently 0, so a literal `version >= MIN` comparison trips clippy's
// absurd_extreme_comparisons; routing MIN through a runtime parameter keeps
// the check future-proof without module-wide allows.
pub(crate) fn version_ok(version: u32, min: u32) -> bool {
    version >= min
}

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

// A host-shared io area (spa_io_clock/position/buffers/rate_match): a typed
// wrapper over the raw pointer the host hands to set_io/port_set_io. Plain
// data (one pointer), so it marshals through the SendWrap/block_on_loop
// paths unchanged. The single unsafe point is set(); read()/with() lean on
// its contract.
pub(crate) struct IoArea<T> {
    ptr: *mut T,
}

impl<T> IoArea<T> {
    pub(crate) const fn null() -> Self {
        Self {
            ptr: std::ptr::null_mut(),
        }
    }

    /// Point the area at host memory, or clear it with NULL.
    ///
    /// # Safety
    /// The caller has validated `data` against the area's size and alignment
    /// and the host keeps it valid while set (the set_io /
    /// port_set_io contract). The memory is host-shared by design; the
    /// data-loop invoke is what serializes our accesses against the swap.
    pub(crate) unsafe fn set(&mut self, data: *mut std::os::raw::c_void) {
        self.ptr = data.cast();
    }

    pub(crate) fn is_null(&self) -> bool {
        self.ptr.is_null()
    }

    // Run `f` on the live area; None while cleared. &mut self so two live
    // &mut T over one area cannot coexist through safe calls (with &self a
    // nested with() would alias); no call site nests today, this keeps it
    // that way by construction.
    pub(crate) fn with<R>(&mut self, f: impl FnOnce(&mut T) -> R) -> Option<R> {
        // sound per set()'s contract (validity and serialization)
        unsafe { self.ptr.as_mut() }.map(f)
    }

    // read-only view of the live area; None while cleared
    pub(crate) fn with_ref<R>(&self, f: impl FnOnce(&T) -> R) -> Option<R> {
        // sound per set()'s contract (validity and serialization)
        unsafe { self.ptr.as_ref() }.map(f)
    }
}

// Run spa_pod_filter with the output going into `out` through its own builder.
// The source pod must NOT live in `out`: the builder's overflow callback grows
// the Vec by reallocating, which would move the source out from under the
// filter mid-copy. Returns a pointer into `out`, valid until `out` changes.
pub(crate) unsafe fn filter_pod(
    out: &mut Vec<u8>,
    src: *mut spa_pod,
    filter: *const spa_pod,
) -> Option<*mut spa_pod> {
    let builder = libspa::pod::builder::Builder::new(out);
    let mut param: *mut spa_pod = std::ptr::null_mut();
    if unsafe { spa_pod_filter(builder.as_raw_ptr(), &mut param, src, filter) } >= 0 {
        Some(param)
    } else {
        None
    }
}

// one (id, index) step of a param enumeration (enum_params_loop's build closure)
pub(crate) enum ParamStep {
    Built(Vec<u8>), // the serialized pod for this index
    Skip,           // nothing at this index; keep scanning (inactive routes)
    Stop(c_int),    // end the enumeration with this return code
}

/// The shared enum_params frame behind node, port and device param
/// enumeration: walk indices from `start`, build one pod per step, filter it
/// against the host's filter pod and emit up to `max` matches as result
/// events. Each build gets a fresh, short State borrow; that borrow ends
/// before `emit`, so a result listener may safely re-enter and the following
/// index observes any resulting state change.
///
/// # Safety
/// `state` must remain live for the call, and a reentrant listener must not
/// destroy it before enumeration returns. `filter` must be null or point at
/// a valid pod (the spa_pod_filter contract). The emit closure receives a
/// pointer into a buffer valid only for that call.
pub(crate) unsafe fn enum_params_loop<S>(
    state: *mut S,
    (start, max): (u32, u32),
    filter: *const spa_pod,
    mut build: impl FnMut(&mut S, u32) -> ParamStep,
    mut emit: impl FnMut(u32, *mut spa_pod),
) -> c_int {
    assert!(!state.is_null(), "enumerated state must not be null");
    let mut fbuffer = vec![]; // spa_pod_filter output; kept apart from the source pod (see filter_pod)

    let mut index = start;
    let mut count = 0;

    while count < max {
        // Reborrow for one build step only. The reference ends before the
        // listener call below, which may re-enter and mutably borrow S.
        let step = build(
            unsafe { state.as_mut() }.expect("state was checked non-null"),
            index,
        );
        let mut buffer = match step {
            ParamStep::Built(pod) => pod,
            ParamStep::Skip => {
                index += 1;
                continue;
            }
            ParamStep::Stop(res) => return res,
        };

        // the built pod lives in `buffer`, distinct from the filter output
        if let Some(param) =
            unsafe { filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter) }
        {
            emit(index, param);
            count += 1;
        }

        index += 1;
    }

    0
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

pub(crate) unsafe fn for_each_dict_item(dict: &spa_dict, mut apply: impl FnMut(&str, &str)) {
    if dict.n_items == 0 || dict.items.is_null() {
        return;
    }
    // items is non-null (checked above) and valid for n_items per the caller
    for item in unsafe { std::slice::from_raw_parts(dict.items, dict.n_items as usize) } {
        if item.key.is_null() || item.value.is_null() {
            continue; // malformed host dict; skip rather than fault
        }
        // host-supplied strings (null-checked above); don't abort on stray bytes
        let key = unsafe { CStr::from_ptr(item.key) }.to_string_lossy();
        let value = unsafe { CStr::from_ptr(item.value) }.to_string_lossy();
        apply(&key, &value);
    }
}

#[cfg(debug_assertions)]
pub(crate) unsafe fn dump_spa_dict(dict: &spa_dict) {
    unsafe {
        for_each_dict_item(dict, |key, value| {
            eprintln!("dict item: key = {key:?}, value = {value:?}");
        });
    }
}

// The inner enum is module-private on purpose: a crate-visible Ptr variant
// would let safe code smuggle an arbitrary (dangling, non-NUL-terminated)
// pointer into a dict item. Construction goes through the From impls only,
// each of which guarantees a valid NUL-terminated pointee.
pub(crate) struct DictionaryString(DictStr);

enum DictStr {
    CString(CString),
    Ptr(*const c_char),
}

impl From<&str> for DictionaryString {
    fn from(str: &str) -> Self {
        // host/sysctl strings; a stray interior NUL must not abort the daemon
        DictionaryString(DictStr::CString(
            CString::new(str.replace('\0', " ")).expect("NULs replaced"),
        ))
    }
}

impl From<String> for DictionaryString {
    fn from(str: String) -> Self {
        DictionaryString(DictStr::CString(
            CString::new(str.replace('\0', " ")).expect("NULs replaced"),
        ))
    }
}

impl From<&'static CStr> for DictionaryString {
    fn from(c: &'static CStr) -> Self {
        // NUL-terminated and 'static by construction, so the raw pointer
        // stays valid for the dictionary's lifetime
        DictionaryString(DictStr::Ptr(c.as_ptr()))
    }
}

// bindgen renders the SPA key constants as NUL-terminated byte arrays; view
// one as the &'static CStr it is
pub(crate) fn key(k: &'static [u8]) -> &'static CStr {
    CStr::from_bytes_with_nul(k).expect("bindgen keys are NUL-terminated")
}

const MAX_ITEMS: u32 = 1024;

pub(crate) struct Dictionary {
    dict: spa_dict,
    items: Vec<spa_dict_item>,
    strings: Vec<CString>,
}

impl Dictionary {
    pub(crate) fn new() -> Self {
        Self {
            dict: spa_dict {
                flags: 0,
                n_items: 0,
                items: std::ptr::null(),
            },
            items: vec![],
            strings: vec![],
        }
    }

    pub(crate) fn fix_pointers(&mut self) {
        self.dict.items = self.items.as_ptr();
    }

    pub(crate) fn raw(&self) -> *const spa_dict {
        &self.dict as *const spa_dict
    }

    fn raw_mut(&mut self) -> *mut spa_dict {
        &mut self.dict as *mut spa_dict
    }

    pub(crate) fn add_item<K: Into<DictionaryString>, V: Into<DictionaryString>>(
        &mut self,
        key: K,
        value: V,
    ) {
        assert!(self.items.len() < MAX_ITEMS as usize);

        let (key, value): (DictionaryString, DictionaryString) = (key.into(), value.into());
        match (key.0, value.0) {
            (DictStr::CString(key), DictStr::CString(value)) => {
                self.items.push(spa_dict_item {
                    key: key.as_ptr(),
                    value: value.as_ptr(),
                });
                self.strings.push(key);
                self.strings.push(value);
            }
            (DictStr::CString(key), DictStr::Ptr(value)) => {
                self.items.push(spa_dict_item {
                    key: key.as_ptr(),
                    value,
                });
                self.strings.push(key);
            }
            (DictStr::Ptr(key), DictStr::CString(value)) => {
                self.items.push(spa_dict_item {
                    key,
                    value: value.as_ptr(),
                });
                self.strings.push(value);
            }
            (DictStr::Ptr(key), DictStr::Ptr(value)) => {
                self.items.push(spa_dict_item { key, value });
            }
        };

        self.dict.n_items = self.items.len() as u32;
        self.fix_pointers();
    }

    // An owned copy with freshly woven pointers. Event payload snapshots use
    // this instead of copying spa_dict_item verbatim: those raw pointers refer
    // into the source dictionary's CString storage.
    fn snapshot(&self) -> Self {
        let mut result = Self::new();
        for item in &self.items {
            // SAFETY: Dictionary construction guarantees both pointers are
            // live NUL-terminated strings for the dictionary's lifetime.
            let key = unsafe { CStr::from_ptr(item.key) }.to_string_lossy();
            let value = unsafe { CStr::from_ptr(item.value) }.to_string_lossy();
            result.add_item(key.as_ref(), value.as_ref());
        }
        result
    }
}

const MAX_PARAMS: u32 = 16;

pub(crate) struct DeviceInfo {
    info: spa_device_info,
    props: Dictionary,
    params: [spa_param_info; MAX_PARAMS as usize],
}

impl DeviceInfo {
    pub(crate) fn new() -> Self {
        Self {
            info: spa_device_info {
                version: SPA_VERSION_DEVICE_INFO,
                change_mask: 0,
                flags: 0,
                props: std::ptr::null(),
                params: std::ptr::null_mut(),
                n_params: 0,
            },
            props: Dictionary::new(),
            params: [spa_param_info {
                id: 0,
                flags: 0,
                user: 0,
                seq: 0,
                padding: [0, 0, 0, 0],
            }; MAX_PARAMS as usize],
        }
    }

    pub(crate) fn fix_pointers(&mut self) {
        self.info.props = self.props.raw_mut();
        self.info.params = self.params.as_mut_ptr();
    }

    pub(crate) fn raw(&self) -> *const spa_device_info {
        &self.info as *const spa_device_info
    }

    // Stable, self-contained payload for a potentially reentrant device-info
    // callback. Box first, then weave the pointers to the boxed fields.
    pub(crate) fn snapshot(&self) -> Box<Self> {
        let mut result = Box::new(Self {
            info: self.info,
            props: self.props.snapshot(),
            params: self.params,
        });
        result.fix_pointers();
        result
    }

    pub(crate) fn add_prop<K: Into<DictionaryString>, V: Into<DictionaryString>>(
        &mut self,
        key: K,
        value: V,
    ) {
        self.props.add_item(key, value);
        self.info.change_mask |= SPA_DEVICE_CHANGE_MASK_PROPS as u64;
    }

    pub(crate) fn add_param(&mut self, id: u32, flags: u32) {
        assert!(self.info.n_params < MAX_PARAMS);
        self.params[self.info.n_params as usize] = spa_param_info {
            id,
            flags,
            user: 0,
            seq: 0,
            padding: [0, 0, 0, 0],
        };
        self.info.change_mask |= SPA_DEVICE_CHANGE_MASK_PARAMS as u64;
        self.info.n_params += 1;
    }

    // flip a param's serial so consumers re-read it even when the read/write
    // flags didn't change
    pub(crate) fn bump_param(&mut self, id: u32) {
        for p in &mut self.params[0..self.info.n_params as usize] {
            if p.id == id {
                p.flags ^= SPA_PARAM_INFO_SERIAL;
                self.info.change_mask |= SPA_DEVICE_CHANGE_MASK_PARAMS as u64;
                return;
            }
        }
    }

    pub(crate) fn replace_change_mask(&mut self, new_mask: u64) -> u64 {
        let old = self.info.change_mask;
        self.info.change_mask = new_mask;
        old
    }
}

pub(crate) struct NodeInfo {
    info: spa_node_info,
    props: Dictionary,
    params: [spa_param_info; MAX_PARAMS as usize],
}

impl NodeInfo {
    pub(crate) fn new() -> Self {
        Self {
            info: spa_node_info {
                max_input_ports: 0,
                max_output_ports: 0,
                change_mask: 0,
                flags: 0,
                props: std::ptr::null_mut(),
                params: std::ptr::null_mut(),
                n_params: 0,
            },
            props: Dictionary::new(),
            params: [spa_param_info {
                id: 0,
                flags: 0,
                user: 0,
                seq: 0,
                padding: [0, 0, 0, 0],
            }; MAX_PARAMS as usize],
        }
    }

    pub(crate) fn fix_pointers(&mut self) {
        self.info.props = self.props.raw_mut();
        self.info.params = self.params.as_mut_ptr();
    }

    pub(crate) fn raw(&self) -> *const spa_node_info {
        &self.info as *const spa_node_info
    }

    // See PortInfo::snapshot: event callbacks receive an owned payload so
    // reentrant listeners cannot invalidate its backing dictionary/params.
    pub(crate) fn snapshot(&self) -> Box<Self> {
        let mut result = Box::new(Self {
            info: self.info,
            props: self.props.snapshot(),
            params: self.params,
        });
        // Box pins the inline params array and Dictionary header at the
        // addresses installed here; moving the Box afterward is harmless.
        result.fix_pointers();
        result
    }

    pub(crate) fn set_max_input_ports(&mut self, max_ports: u32) {
        self.info.max_input_ports = max_ports;
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64; // does this field count as a flag?
    }

    pub(crate) fn set_max_output_ports(&mut self, max_ports: u32) {
        self.info.max_output_ports = max_ports;
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64; // does this field count as a flag?
    }

    pub(crate) fn set_flags(&mut self, flags: u64) {
        self.info.flags = flags;
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64;
    }

    pub(crate) fn add_prop<K: Into<DictionaryString>, V: Into<DictionaryString>>(
        &mut self,
        key: K,
        value: V,
    ) {
        self.props.add_item(key, value);
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_PROPS as u64;
    }

    pub(crate) fn add_param(&mut self, id: u32, flags: u32) {
        assert!(self.info.n_params < MAX_PARAMS);
        self.params[self.info.n_params as usize] = spa_param_info {
            id,
            flags,
            user: 0,
            seq: 0,
            padding: [0, 0, 0, 0],
        };
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_PARAMS as u64;
        self.info.n_params += 1;
    }

    // flip a param's serial so consumers (the adapter compares flags, not user)
    // re-read it even when the read/write flags didn't change
    pub(crate) fn bump_param(&mut self, id: u32) {
        for p in &mut self.params[0..self.info.n_params as usize] {
            if p.id == id {
                p.flags ^= SPA_PARAM_INFO_SERIAL;
                self.info.change_mask |= SPA_NODE_CHANGE_MASK_PARAMS as u64;
                return;
            }
        }
    }

    pub(crate) fn replace_change_mask(&mut self, new_mask: u64) -> u64 {
        let old = self.info.change_mask;
        self.info.change_mask = new_mask;
        old
    }
}

pub(crate) struct PortInfo {
    info: spa_port_info,
    props: Dictionary,
    params: [spa_param_info; MAX_PARAMS as usize],
}

impl PortInfo {
    pub(crate) fn new() -> Self {
        Self {
            info: spa_port_info {
                change_mask: 0,
                flags: 0,
                rate: spa_fraction { num: 0, denom: 0 },
                props: std::ptr::null_mut(),
                params: std::ptr::null_mut(),
                n_params: 0,
            },
            props: Dictionary::new(),
            params: [spa_param_info {
                id: 0,
                flags: 0,
                user: 0,
                seq: 0,
                padding: [0, 0, 0, 0],
            }; MAX_PARAMS as usize],
        }
    }

    pub(crate) fn fix_pointers(&mut self) {
        self.info.props = self.props.raw_mut();
        self.info.params = self.params.as_mut_ptr();
    }

    pub(crate) fn raw(&self) -> *const spa_port_info {
        &self.info as *const spa_port_info
    }

    // A self-contained callback payload: the scalar info and fixed params
    // copy by value, while the dictionary is rebuilt so none of its pointers
    // refer back into the live PortInfo. Reentrant listeners may then mutate
    // the live state without invalidating the outer callback's payload.
    pub(crate) fn snapshot(&self) -> Box<Self> {
        let mut result = Box::new(Self {
            info: self.info,
            props: self.props.snapshot(),
            params: self.params,
        });
        // As for NodeInfo::snapshot, the Box makes these self-pointers
        // stable across return and callback dispatch.
        result.fix_pointers();
        result
    }

    pub(crate) fn set_flags(&mut self, flags: u64) {
        self.info.flags = flags;
        self.info.change_mask |= SPA_PORT_CHANGE_MASK_FLAGS as u64;
    }

    pub(crate) fn set_rate(&mut self, rate: spa_fraction) {
        self.info.rate = rate;
        self.info.change_mask |= SPA_PORT_CHANGE_MASK_RATE as u64;
    }

    /* currently unused
    pub fn add_prop<K: Into<DictionaryString>, V: Into<DictionaryString>>(&mut self, key: K, value: V) {
      self.props.add_item(key, value);
      self.info.change_mask |= SPA_PORT_CHANGE_MASK_PROPS as u64;
    }*/

    pub(crate) fn add_param(&mut self, id: u32, flags: u32) {
        assert!(self.info.n_params < MAX_PARAMS);
        self.params[self.info.n_params as usize] = spa_param_info {
            id,
            flags,
            user: 0,
            seq: 0,
            padding: [0, 0, 0, 0],
        };
        self.info.change_mask |= SPA_PORT_CHANGE_MASK_PARAMS as u64;
        self.info.n_params += 1;
    }

    // Change an advertised param's read/write flags. The host re-reads a param when
    // its flags change, so flipping Format WRITE<->READWRITE around a format
    // clear/set is what marks the port (re)negotiable.
    pub(crate) fn set_param_flags(&mut self, id: u32, flags: u32) {
        for p in &mut self.params[0..self.info.n_params as usize] {
            if p.id == id {
                p.flags = flags;
                self.info.change_mask |= SPA_PORT_CHANGE_MASK_PARAMS as u64;
                return;
            }
        }
    }

    // flip a param's serial so consumers (the adapter compares flags, not user)
    // re-read it even when the read/write flags didn't change
    pub(crate) fn bump_param(&mut self, id: u32) {
        for p in &mut self.params[0..self.info.n_params as usize] {
            if p.id == id {
                p.flags ^= SPA_PARAM_INFO_SERIAL;
                self.info.change_mask |= SPA_PORT_CHANGE_MASK_PARAMS as u64;
                return;
            }
        }
    }

    pub(crate) fn replace_change_mask(&mut self, new_mask: u64) -> u64 {
        let old = self.info.change_mask;
        self.info.change_mask = new_mask;
        old
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Loop {
    // Keep the host-owned spa_loop as a raw pointer because the host may
    // mutate it. wrap() validates and copies the method slots once; data is
    // read through the raw interface for each call.
    loop_: std::ptr::NonNull<spa_loop>,
    add_source_fn: unsafe extern "C" fn(*mut c_void, *mut spa_source) -> c_int,
    remove_source_fn: unsafe extern "C" fn(*mut c_void, *mut spa_source) -> c_int,
    #[allow(clippy::type_complexity)] // the C slot's signature, verbatim
    invoke_fn: unsafe extern "C" fn(
        *mut c_void,
        spa_invoke_func_t,
        u32,
        *const c_void,
        usize,
        bool,
        *mut c_void,
    ) -> c_int,
}

impl Loop {
    /// # Safety
    /// `loop_` must point at a live, initialized spa_loop from the host's
    /// support array, and the host keeps it (and its methods vtable) valid
    /// for the plugin's lifetime (the spa_support contract).
    pub(crate) unsafe fn wrap(loop_: *mut spa_loop) -> Self {
        let loop_ = std::ptr::NonNull::new(loop_).expect("loop should be initialized");
        // Validate the version and every required slot during initialization.
        let funcs = unsafe { (*loop_.as_ptr()).iface.cb.funcs };
        let methods = std::ptr::NonNull::new(funcs.cast::<spa_loop_methods>().cast_mut())
            .expect("loop methods should be initialized")
            .as_ptr();
        assert!(version_ok(
            unsafe { (*methods).version },
            SPA_VERSION_LOOP_METHODS
        ));
        Self {
            loop_,
            add_source_fn: unsafe { (*methods).add_source }
                .expect("add_source should be initialized"),
            remove_source_fn: unsafe { (*methods).remove_source }
                .expect("remove_source should be initialized"),
            invoke_fn: unsafe { (*methods).invoke }.expect("invoke should be initialized"),
        }
    }

    fn data(&self) -> *mut c_void {
        // per-call raw read (see the struct comment); valid per wrap()'s contract
        unsafe { (*self.loop_.as_ptr()).iface.cb.data }
    }

    /// # Safety
    /// `source` must stay valid (and pinned) until it is removed from the
    /// loop again; the loop stores the pointer.
    pub(crate) unsafe fn add_source(&self, source: *mut spa_source) -> c_int {
        unsafe { (self.add_source_fn)(self.data(), source) }
    }

    /// # Safety
    /// Must be called from the loop thread (or through an invoke); `source`
    /// must be the pointer previously registered with add_source.
    pub(crate) unsafe fn remove_source(&self, source: *mut spa_source) -> c_int {
        unsafe { (self.remove_source_fn)(self.data(), source) }
    }

    /// # Safety
    /// The marshaling contract: `func` must treat `data`/`user_data`
    /// according to `block` (a non-blocking invoke copies `data` and runs
    /// after this call returns, so pointees must outlive the run); see
    /// utils::block_on_loop / utils::queue_task.
    pub(crate) unsafe fn invoke(
        &self,
        func: spa_invoke_func_t,
        seq: u32,
        data: *const c_void,
        size: usize,
        block: bool,
        user_data: *mut c_void,
    ) -> c_int {
        unsafe { (self.invoke_fn)(self.data(), func, seq, data, size, block, user_data) }
    }
}

pub(crate) struct System {
    // Keep the host-owned spa_system as a raw pointer and copy its validated
    // method slots. Safe wrappers pass only scalars and call-scoped references.
    system: std::ptr::NonNull<spa_system>,
    close_fn: unsafe extern "C" fn(*mut c_void, c_int) -> c_int,
    clock_gettime_fn: unsafe extern "C" fn(*mut c_void, c_int, *mut timespec) -> c_int,
    timerfd_create_fn: unsafe extern "C" fn(*mut c_void, c_int, c_int) -> c_int,
    timerfd_read_fn: unsafe extern "C" fn(*mut c_void, c_int, *mut u64) -> c_int,
    timerfd_settime_fn: unsafe extern "C" fn(
        *mut c_void,
        c_int,
        c_int,
        *const itimerspec,
        *mut itimerspec,
    ) -> c_int,
}

impl System {
    /// # Safety
    /// `system` must point at a live, initialized spa_system from the host's
    /// support array, and the host keeps it (and its methods vtable) valid
    /// for the plugin's lifetime (the spa_support contract). That contract
    /// is what makes the safe methods below sound.
    pub(crate) unsafe fn wrap(system: *mut spa_system) -> Self {
        let system = std::ptr::NonNull::new(system).expect("system should be initialized");
        // the whole vtable is validated once here (non-null, version, every
        // slot the methods below call - see Loop::wrap)
        let funcs = unsafe { (*system.as_ptr()).iface.cb.funcs };
        let methods = std::ptr::NonNull::new(funcs.cast::<spa_system_methods>().cast_mut())
            .expect("system methods should be initialized")
            .as_ptr();
        assert!(version_ok(
            unsafe { (*methods).version },
            SPA_VERSION_SYSTEM_METHODS
        ));
        Self {
            system,
            close_fn: unsafe { (*methods).close }.expect("close should be initialized"),
            clock_gettime_fn: unsafe { (*methods).clock_gettime }
                .expect("clock_gettime should be initialized"),
            timerfd_create_fn: unsafe { (*methods).timerfd_create }
                .expect("timerfd_create should be initialized"),
            timerfd_read_fn: unsafe { (*methods).timerfd_read }
                .expect("timerfd_read should be initialized"),
            timerfd_settime_fn: unsafe { (*methods).timerfd_settime }
                .expect("timerfd_settime should be initialized"),
        }
    }

    fn data(&self) -> *mut c_void {
        // per-call raw read (see Loop::data); valid per wrap()'s contract
        unsafe { (*self.system.as_ptr()).iface.cb.data }
    }

    pub(crate) fn close(&self, fd: c_int) -> c_int {
        // sound per wrap()'s contract; fd is an owned scalar
        unsafe { (self.close_fn)(self.data(), fd) }
    }

    pub(crate) fn clock_gettime(&self, clock_id: c_int, value: &mut timespec) -> c_int {
        // sound per wrap()'s contract; `value` is a live &mut for the call
        unsafe { (self.clock_gettime_fn)(self.data(), clock_id, value) }
    }

    pub(crate) fn timerfd_create(&self, clock_id: c_int, flags: c_int) -> c_int {
        // sound per wrap()'s contract; both arguments are owned scalars
        unsafe { (self.timerfd_create_fn)(self.data(), clock_id, flags) }
    }

    pub(crate) fn timerfd_read(&self, fd: c_int, expirations: &mut u64) -> c_int {
        // sound per wrap()'s contract; `expirations` is a live &mut for the call
        unsafe { (self.timerfd_read_fn)(self.data(), fd, expirations) }
    }

    // Callers do not request the previous timer value.
    pub(crate) fn timerfd_settime(&self, fd: c_int, flags: c_int, new_value: &itimerspec) -> c_int {
        // sound per wrap()'s contract; `new_value` is a live shared reference
        unsafe {
            (self.timerfd_settime_fn)(self.data(), fd, flags, new_value, std::ptr::null_mut())
        }
    }
}

#[derive(Clone)]
pub(crate) struct Log {
    // Raw pointers end to end, never long-lived references: the host logger
    // mutates the spa_log pointee (level) and the registered topic
    // (level/has_custom_level) at runtime, so holding a &'static over
    // host-mutated memory would be unsound. The methods vtable gets the
    // same treatment - nothing guarantees the host never touches it, and a
    // per-call raw read costs nothing.
    logger: std::ptr::NonNull<spa_log>,
    methods: std::ptr::NonNull<spa_log_methods>,
    // the module's registered topic (see the lib.rs section entries)
    topic: Option<std::ptr::NonNull<spa_log_topic>>,
}

// The host logger is thread-safe and outlives every node, so cloned handles
// may travel with cross-loop messages.
unsafe impl Send for Log {}

impl Log {
    pub(crate) unsafe fn wrap(
        log: *mut spa_log,
        topic: Option<std::ptr::NonNull<spa_log_topic>>,
    ) -> Self {
        let logger = std::ptr::NonNull::new(log).expect("log should be initialized");
        // the vtable pointer is read once here; the vtable fields are read
        // per call through the raw pointer
        let funcs = unsafe { (*log).iface.cb.funcs };
        let methods = std::ptr::NonNull::new(funcs.cast::<spa_log_methods>().cast_mut())
            .expect("log methods should be initialized");
        // no minimum-version assert: version 0 (predating the logt slot) is
        // accepted - log() gates every logt read on the vtable being v1+,
        // and the v0 `log` method covers the rest
        Self {
            logger,
            methods,
            topic,
        }
    }

    pub(crate) fn log_level(&self) -> spa_log_level {
        use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
        // The host logger rewrites the registered topic's level (and the
        // logger's own level) from its own threads on runtime log-level
        // changes (inherent to the C API). Per-call atomic views over the
        // C-layout fields remove our side's contribution to the race, but
        // the writer side stays plain C stores, so the mixed access
        // formally remains a data race - UB, with no bounded worst case to
        // appeal to. The host writes directly into the structs we handed
        // it, so no plugin-side synchronization can intercept the stores,
        // and every C plugin shares the pattern; the atomics merely narrow
        // the practical exposure (aligned single-word loads, level-check
        // only). The only sound alternative (an FFI shim serializing every
        // read with the logger) would put a call on every log-level check.
        if let Some(topic) = self.topic {
            let topic = topic.as_ptr();
            unsafe {
                let custom = AtomicBool::from_ptr(&raw mut (*topic).has_custom_level);
                if custom.load(Ordering::Relaxed) {
                    return AtomicU32::from_ptr(&raw mut (*topic).level).load(Ordering::Relaxed);
                }
            }
        }
        let logger = self.logger.as_ptr();
        unsafe { AtomicU32::from_ptr(&raw mut (*logger).level).load(Ordering::Relaxed) }
    }

    pub(crate) fn log(&self, level: spa_log_level, file: &str, line: c_int, func: &str, msg: &str) {
        let file = CString::new(file).unwrap(); // ours, no interior NULs
        let func = CString::new(func).unwrap(); // ditto

        // the message can carry host-derived strings; don't abort on an
        // interior NUL
        let msg = CString::new(msg).unwrap_or_else(|_| c"<message contained NUL>".to_owned());
        let topic = self
            .topic
            .map_or(std::ptr::null(), |topic| topic.as_ptr().cast_const());
        let methods = self.methods.as_ptr();
        let data = unsafe { (*self.logger.as_ptr()).iface.cb.data };
        // a v0 vtable has no logt slot at all (the C struct is shorter), so
        // the field may only be read behind the version gate; a missing logt
        // on a v1+ logger falls back the same way
        let logt = if version_ok(unsafe { (*methods).version }, SPA_VERSION_LOG_METHODS) {
            unsafe { (*methods).logt }
        } else {
            None
        };
        unsafe {
            if let Some(logt) = logt {
                logt(
                    data,
                    level,
                    topic,
                    file.as_ptr(),
                    line,
                    func.as_ptr(),
                    c"%s".as_ptr(),
                    msg.as_ptr(),
                );
            } else {
                let log = (*methods).log.expect("log should be initialized");
                log(
                    data,
                    level,
                    file.as_ptr(),
                    line,
                    func.as_ptr(),
                    c"%s".as_ptr(),
                    msg.as_ptr(),
                );
            }
        }
    }
}

// a Log that never emits (level NONE, methods never reached): phase-function
// tests need a Log without a live host logger. Safe because the log! macros
// gate every method call on log_level(), which reads NONE here.
#[cfg(test)]
impl Log {
    pub(crate) fn test_null() -> Self {
        Self {
            logger: std::ptr::NonNull::from(Box::leak(Box::new(unsafe {
                std::mem::zeroed::<spa_log>()
            }))),
            methods: std::ptr::NonNull::from(Box::leak(Box::new(unsafe {
                std::mem::zeroed::<spa_log_methods>()
            }))),
            topic: None,
        }
    }
}

#[macro_export]
macro_rules! log {
    ($log:expr, $log_level:expr, $($arg:tt)*) => {
        if $log.log_level() >= $log_level {
            let file = file!();
            let line = line!();
            let func = ""; // no cheap function-name source; file:line suffices
            $log.log($log_level, file, line as c_int, func, &format!($($arg)*));
        }
    };
}

#[macro_export]
macro_rules! error {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, SPA_LOG_LEVEL_ERROR, $($arg)*)
    };
}

#[macro_export]
macro_rules! warn {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, SPA_LOG_LEVEL_WARN, $($arg)*)
    };
}

#[macro_export]
macro_rules! info {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, SPA_LOG_LEVEL_INFO, $($arg)*)
    };
}

#[macro_export]
macro_rules! debug {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, SPA_LOG_LEVEL_DEBUG, $($arg)*)
    };
}

#[macro_export]
macro_rules! trace {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, SPA_LOG_LEVEL_TRACE, $($arg)*)
    };
}

#[cfg(test)]
mod tests {
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
}
