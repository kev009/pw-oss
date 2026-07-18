use core::ffi::CStr;
use libspa::sys::*;
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_void};

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

pub(crate) unsafe fn for_each_hook(head: *mut spa_hook_list, mut apply: impl FnMut(&spa_hook)) {
    let mut entry = (*head).list.next as *mut spa_hook;
    while (*entry).link != (*head).list {
        // grab next first: a listener may remove (and free) its own hook from
        // inside the callback, which SPA allows. (Removing a *different* hook is
        // still unsafe here; the C helpers use a shared cursor for that.)
        let next = (*entry).link.next as *mut spa_hook;
        apply(entry.as_ref().expect("broken spa_hook_list"));
        entry = next;
    }
}

pub(crate) unsafe fn dev_emit_result(
    hooks: &mut spa_hook_list,
    seq: c_int,
    res: c_int,
    type_: u32,
    result: &spa_result_device_params,
) {
    for_each_hook(hooks, |entry| {
        let f = entry
            .cb
            .funcs
            .cast::<spa_device_events>()
            .as_ref()
            .expect("hook should be initialized");
        assert!(version_ok(f.version, SPA_VERSION_DEVICE_EVENTS));
        if let Some(result_fun) = f.result {
            result_fun(
                entry.cb.data,
                seq,
                res,
                type_,
                result as *const _ as *const c_void,
            );
        }
    });
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
    if spa_pod_filter(builder.as_raw_ptr(), &mut param, src, filter) >= 0 {
        Some(param)
    } else {
        None
    }
}

// sync() replies with an empty result carrying the sequence number
pub(crate) unsafe fn node_emit_done(hooks: &mut spa_hook_list, seq: c_int) {
    for_each_hook(hooks, |entry| {
        let f = entry
            .cb
            .funcs
            .cast::<spa_node_events>()
            .as_ref()
            .expect("hook should be initialized");
        assert!(version_ok(f.version, SPA_VERSION_NODE_EVENTS));
        if let Some(result_fun) = f.result {
            result_fun(entry.cb.data, seq, 0, 0, std::ptr::null());
        }
    });
}

pub(crate) unsafe fn node_emit_result(
    hooks: &mut spa_hook_list,
    seq: c_int,
    res: c_int,
    type_: u32,
    result: &spa_result_node_params,
) {
    for_each_hook(hooks, |entry| {
        let f = entry
            .cb
            .funcs
            .cast::<spa_node_events>()
            .as_ref()
            .expect("hook should be initialized");
        assert!(version_ok(f.version, SPA_VERSION_NODE_EVENTS));
        if let Some(result_fun) = f.result {
            result_fun(
                entry.cb.data,
                seq,
                res,
                type_,
                result as *const _ as *const c_void,
            );
        }
    });
}

pub(crate) unsafe fn for_each_dict_item(dict: &spa_dict, mut apply: impl FnMut(&str, &str)) {
    if dict.n_items == 0 || dict.items.is_null() {
        return;
    }
    for item in std::slice::from_raw_parts(dict.items, dict.n_items as usize) {
        if item.key.is_null() || item.value.is_null() {
            continue; // malformed host dict; skip rather than fault
        }
        // host-supplied strings; don't abort on stray bytes
        let key = CStr::from_ptr(item.key).to_string_lossy();
        let value = CStr::from_ptr(item.value).to_string_lossy();
        apply(&key, &value);
    }
}

#[cfg(debug_assertions)]
pub(crate) unsafe fn dump_spa_dict(dict: &spa_dict) {
    for_each_dict_item(dict, |key, value| {
        eprintln!("dict item: key = {key:?}, value = {value:?}");
    });
}

pub(crate) enum DictionaryString {
    CString(CString),
    Ptr(*const c_char),
}

impl From<&str> for DictionaryString {
    fn from(str: &str) -> Self {
        // host/sysctl strings; a stray interior NUL must not abort the daemon
        DictionaryString::CString(CString::new(str.replace('\0', " ")).expect("NULs replaced"))
    }
}

impl From<String> for DictionaryString {
    fn from(str: String) -> Self {
        DictionaryString::CString(CString::new(str.replace('\0', " ")).expect("NULs replaced"))
    }
}

impl From<*const u8> for DictionaryString {
    fn from(p: *const u8) -> Self {
        DictionaryString::Ptr(p.cast())
    }
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

        match (key.into(), value.into()) {
            (DictionaryString::CString(key), DictionaryString::CString(value)) => {
                self.items.push(spa_dict_item {
                    key: key.as_ptr(),
                    value: value.as_ptr(),
                });
                self.strings.push(key);
                self.strings.push(value);
            }
            (DictionaryString::CString(key), DictionaryString::Ptr(value)) => {
                self.items.push(spa_dict_item {
                    key: key.as_ptr(),
                    value,
                });
                self.strings.push(key);
            }
            (DictionaryString::Ptr(key), DictionaryString::CString(value)) => {
                self.items.push(spa_dict_item {
                    key,
                    value: value.as_ptr(),
                });
                self.strings.push(value);
            }
            (DictionaryString::Ptr(key), DictionaryString::Ptr(value)) => {
                self.items.push(spa_dict_item { key, value });
            }
        };

        self.dict.n_items = self.items.len() as u32;
        self.fix_pointers();
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
        self.info.props = self.props.raw();
        self.info.params = self.params.as_mut_ptr();
    }

    pub(crate) fn raw(&self) -> *const spa_device_info {
        &self.info as *const spa_device_info
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

pub(crate) struct Loop {
    loop_: &'static spa_loop, // not really 'static, but it should outlive our plugin anyway
    methods: &'static spa_loop_methods, // ditto
}

impl Loop {
    pub(crate) unsafe fn wrap(loop_: *mut spa_loop) -> Self {
        let loop_ = loop_
            .cast::<spa_loop>()
            .as_ref()
            .expect("loop should be initialized");
        let methods = loop_
            .iface
            .cb
            .funcs
            .cast::<spa_loop_methods>()
            .as_ref()
            .expect("loop methods should be initialized");
        assert!(version_ok(methods.version, SPA_VERSION_LOOP_METHODS));
        Self { loop_, methods }
    }

    pub(crate) unsafe fn add_source(&self, source: *mut spa_source) -> c_int {
        let spa_loop_add_source = self
            .methods
            .add_source
            .expect("add_source should be initialized");
        spa_loop_add_source(self.loop_.iface.cb.data, source)
    }

    // must be called from the loop thread (or through an invoke)
    pub(crate) unsafe fn remove_source(&self, source: *mut spa_source) -> c_int {
        let spa_loop_remove_source = self
            .methods
            .remove_source
            .expect("remove_source should be initialized");
        spa_loop_remove_source(self.loop_.iface.cb.data, source)
    }

    pub(crate) unsafe fn invoke(
        &self,
        func: spa_invoke_func_t,
        seq: u32,
        data: *const c_void,
        size: usize,
        block: bool,
        user_data: *mut c_void,
    ) -> c_int {
        let spa_loop_invoke = self.methods.invoke.expect("invoke should be initialized");
        spa_loop_invoke(
            self.loop_.iface.cb.data,
            func,
            seq,
            data,
            size,
            block,
            user_data,
        )
    }
}

pub(crate) struct System {
    system: &'static spa_system, // not really 'static, but it should outlive our plugin anyway
    methods: &'static spa_system_methods, // ditto
}

impl System {
    pub(crate) unsafe fn wrap(system: *mut spa_system) -> Self {
        let system = system
            .cast::<spa_system>()
            .as_ref()
            .expect("system should be initialized");
        let methods = system
            .iface
            .cb
            .funcs
            .cast::<spa_system_methods>()
            .as_ref()
            .expect("system methods should be initialized");
        assert!(version_ok(methods.version, SPA_VERSION_SYSTEM_METHODS));
        Self { system, methods }
    }

    pub(crate) unsafe fn close(&self, fd: c_int) -> c_int {
        let spa_system_close = self.methods.close.expect("close should be initialized");
        spa_system_close(self.system.iface.cb.data, fd)
    }

    pub(crate) unsafe fn clock_gettime(&self, clock_id: c_int, value: *mut timespec) -> c_int {
        let spa_system_clock_gettime = self
            .methods
            .clock_gettime
            .expect("clock_gettime should be initialized");
        spa_system_clock_gettime(self.system.iface.cb.data, clock_id, value)
    }

    pub(crate) unsafe fn timerfd_create(&self, clock_id: c_int, flags: c_int) -> c_int {
        let spa_system_timerfd_create = self
            .methods
            .timerfd_create
            .expect("timerfd_create should be assigned");
        spa_system_timerfd_create(self.system.iface.cb.data, clock_id, flags)
    }

    pub(crate) unsafe fn timerfd_read(&self, fd: c_int, expirations: *mut u64) -> c_int {
        let spa_system_timerfd_read = self
            .methods
            .timerfd_read
            .expect("timerfd_read should be initialized");
        spa_system_timerfd_read(self.system.iface.cb.data, fd, expirations)
    }

    pub(crate) unsafe fn timerfd_settime(
        &self,
        fd: c_int,
        flags: c_int,
        new_value: *const itimerspec,
        old_value: *mut itimerspec,
    ) -> c_int {
        let spa_system_timerfd_settime = self
            .methods
            .timerfd_settime
            .expect("timerfd_settime should be initialized");
        spa_system_timerfd_settime(self.system.iface.cb.data, fd, flags, new_value, old_value)
    }
}

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

impl Log {
    pub(crate) unsafe fn wrap(
        log: *mut spa_log,
        topic: Option<std::ptr::NonNull<spa_log_topic>>,
    ) -> Self {
        let logger = std::ptr::NonNull::new(log).expect("log should be initialized");
        // the vtable pointer is read once here; the vtable fields are read
        // per call through the raw pointer
        let funcs = (*log).iface.cb.funcs;
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
