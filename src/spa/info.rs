use super::*;

pub(crate) unsafe fn for_each_dict_item(dict: &spa_dict, mut apply: impl FnMut(&str, &str)) {
    if dict.n_items == 0 || dict.items.is_null() {
        return;
    }
    let len = dict.n_items as usize;
    if !crate::spa::raw_slice_len_ok::<spa_dict_item>(len) {
        return;
    }
    // items is non-null (checked above) and valid for n_items per the caller
    for item in unsafe { std::slice::from_raw_parts(dict.items, len) } {
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
        std::ptr::from_ref(&self.dict)
    }

    fn raw_mut(&mut self) -> *mut spa_dict {
        std::ptr::from_mut(&mut self.dict)
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
        std::ptr::from_ref(&self.info)
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
        std::ptr::from_ref(&self.info)
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

    // SPA has no separate port-limit change bit; consumers read both maxima
    // together with the node flags when FLAGS is set.
    pub(crate) fn set_max_input_ports(&mut self, max_ports: u32) {
        self.info.max_input_ports = max_ports;
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64;
    }

    pub(crate) fn set_max_output_ports(&mut self, max_ports: u32) {
        self.info.max_output_ports = max_ports;
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64;
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
        std::ptr::from_ref(&self.info)
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

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(node_raw.props, (&raw const node.props.dict).cast_mut());

        let mut port = PortInfo::new();
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
        port.fix_pointers();
        let port = port.snapshot();
        let port_raw = unsafe { &*port.raw() };
        assert_eq!(port_raw.params, port.params.as_ptr().cast_mut());
        assert_eq!(port_raw.props, (&raw const port.props.dict).cast_mut());

        let mut device = DeviceInfo::new();
        device.add_prop("snapshot.key", "snapshot.value");
        device.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READ);
        device.fix_pointers();
        let device = device.snapshot();
        let device_raw = unsafe { &*device.raw() };
        assert_eq!(device_raw.params, device.params.as_ptr().cast_mut());
        assert_eq!(device_raw.props, (&raw const device.props.dict).cast_mut());
    }
}
