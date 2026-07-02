use core::ffi::CStr;
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_void};
use libspa::sys::*;

pub const SPA_DEVICE_CHANGE_MASK_ALL: u32 =
  SPA_DEVICE_CHANGE_MASK_FLAGS  |
  SPA_DEVICE_CHANGE_MASK_PARAMS |
  SPA_DEVICE_CHANGE_MASK_PROPS;

pub const SPA_DEVICE_OBJECT_CHANGE_MASK_ALL: u32 =
  SPA_DEVICE_OBJECT_CHANGE_MASK_FLAGS |
  SPA_DEVICE_OBJECT_CHANGE_MASK_PROPS;

pub const SPA_NODE_CHANGE_MASK_ALL: u32 =
  SPA_NODE_CHANGE_MASK_FLAGS  |
  SPA_NODE_CHANGE_MASK_PARAMS |
  SPA_NODE_CHANGE_MASK_PROPS;

pub const SPA_PORT_CHANGE_MASK_ALL: u32 =
  SPA_PORT_CHANGE_MASK_FLAGS  |
  SPA_PORT_CHANGE_MASK_PARAMS |
  SPA_PORT_CHANGE_MASK_PROPS  |
  SPA_PORT_CHANGE_MASK_RATE;

pub unsafe fn for_each_hook(head: *mut spa_hook_list, mut apply: impl FnMut(&spa_hook)) {
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

pub unsafe fn dev_emit_result(hooks: &mut spa_hook_list, seq: c_int, res: c_int, type_: u32, result: &spa_result_device_params) {
  for_each_hook(hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_device_events>().as_ref().expect("hook should be initialized");
    assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);
    if let Some(result_fun) = f.result {
      result_fun(entry.cb.data, seq, res, type_, result as *const _ as *const c_void);
    }
  });
}

// Run spa_pod_filter with the output going into `out` through its own builder.
// The source pod must NOT live in `out`: the builder's overflow callback grows
// the Vec by reallocating, which would move the source out from under the
// filter mid-copy. Returns a pointer into `out`, valid until `out` changes.
pub unsafe fn filter_pod(out: &mut Vec<u8>, src: *mut spa_pod, filter: *const spa_pod) -> Option<*mut spa_pod> {
  let builder = libspa::pod::builder::Builder::new(out);
  let mut param: *mut spa_pod = std::ptr::null_mut();
  if spa_pod_filter(builder.as_raw_ptr(), &mut param, src, filter) >= 0 {
    Some(param)
  } else {
    None
  }
}

// sync() replies with an empty result carrying the sequence number
pub unsafe fn node_emit_done(hooks: &mut spa_hook_list, seq: c_int) {
  for_each_hook(hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_node_events>().as_ref().expect("hook should be initialized");
    assert!(f.version >= SPA_VERSION_NODE_EVENTS);
    if let Some(result_fun) = f.result {
      result_fun(entry.cb.data, seq, 0, 0, std::ptr::null());
    }
  });
}

pub unsafe fn node_emit_result(hooks: &mut spa_hook_list, seq: c_int, res: c_int, type_: u32, result: &spa_result_node_params) {
  for_each_hook(hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_node_events>().as_ref().expect("hook should be initialized");
    assert!(f.version >= SPA_VERSION_NODE_EVENTS);
    if let Some(result_fun) = f.result {
      result_fun(entry.cb.data, seq, res, type_, result as *const _ as *const c_void);
    }
  });
}

pub unsafe fn for_each_dict_item(dict: &spa_dict, mut apply: impl FnMut(&str, &str)) {
  if dict.n_items == 0 || dict.items.is_null() {
    return;
  }
  for item in std::slice::from_raw_parts(dict.items, dict.n_items as usize) {
    if item.key.is_null() || item.value.is_null() {
      continue; // malformed host dict; skip rather than fault
    }
    // host-supplied strings; don't abort on stray bytes
    let key   = CStr::from_ptr(item.key)  .to_string_lossy();
    let value = CStr::from_ptr(item.value).to_string_lossy();
    apply(&key, &value);
  }
}

#[cfg(debug_assertions)]
pub unsafe fn dump_spa_dict(dict: &spa_dict) {
  for_each_dict_item(dict, |key, value| {
    eprintln!("dict item: key = {:?}, value = {:?}", key, value);
  });
}

pub enum DictionaryString {
  CString(CString),
  Ptr(*const c_char)
}

impl From<&str> for DictionaryString {

  fn from(str: &str) -> Self {
    DictionaryString::CString(CString::new(str).unwrap())
  }
}

impl From<String> for DictionaryString {

  fn from(str: String) -> Self {
    DictionaryString::CString(CString::new(str).unwrap())
  }
}

impl From<*const u8> for DictionaryString {

  fn from(p: *const u8) -> Self {
    DictionaryString::Ptr(p.cast())
  }
}

const MAX_ITEMS: u32 = 1024;

pub struct Dictionary {
  dict:    spa_dict,
  items:   Vec<spa_dict_item>,
  strings: Vec<CString>
}

impl Dictionary {

  pub fn new() -> Self {
    Self {
      dict: spa_dict {
        flags:   0,
        n_items: 0,
        items:   std::ptr::null(),
      },
      items:   vec![],
      strings: vec![]
    }
  }

  pub fn fix_pointers(&mut self) {
    self.dict.items = self.items.as_ptr();
  }

  pub unsafe fn raw(&self) -> *const spa_dict {
    &self.dict as *const spa_dict
  }

  unsafe fn raw_mut(&mut self) -> *mut spa_dict {
    &mut self.dict as *mut spa_dict
  }

  /* currently unused
  pub fn len(&self) -> u32 {
    self.items.len() as u32
  }*/

  pub fn add_item<K: Into<DictionaryString>, V: Into<DictionaryString>>(&mut self, key: K, value: V) {

    assert!(self.items.len() < MAX_ITEMS as usize);

    match (key.into(), value.into()) {
      (DictionaryString::CString(key), DictionaryString::CString(value)) => {
        self.items.push(spa_dict_item { key: key.as_ptr(), value: value.as_ptr() });
        self.strings.push(key);
        self.strings.push(value);
      },
      (DictionaryString::CString(key), DictionaryString::Ptr(value)) => {
        self.items.push(spa_dict_item { key: key.as_ptr(), value });
        self.strings.push(key);
      },
      (DictionaryString::Ptr(key), DictionaryString::CString(value)) => {
        self.items.push(spa_dict_item { key, value: value.as_ptr() });
        self.strings.push(value);
      },
      (DictionaryString::Ptr(key), DictionaryString::Ptr(value)) => {
        self.items.push(spa_dict_item { key, value });
      }
    };

    self.dict.n_items = self.items.len() as u32;
    self.fix_pointers();
  }
}

const MAX_PARAMS: u32 = 16;

pub struct DeviceInfo {
  info:    spa_device_info,
  props:   Dictionary,
  params:  [spa_param_info; MAX_PARAMS as usize]
}

impl DeviceInfo {

  pub fn new() -> Self {
    Self {
      info: spa_device_info {
        version:     SPA_VERSION_DEVICE_INFO,
        change_mask: 0,
        flags:       0,
        props:       std::ptr::null(),
        params:      std::ptr::null_mut(),
        n_params:    0
      },
      props:  Dictionary::new(),
      params: [spa_param_info { id: 0, flags: 0, user: 0, seq: 0, padding: [0, 0, 0, 0] }; MAX_PARAMS as usize]
    }
  }

  pub fn fix_pointers(&mut self) {
    self.info.props  = unsafe { self.props.raw() };
    self.info.params = self.params.as_mut_ptr();
  }

  pub unsafe fn raw(&self) -> *const spa_device_info {
    &self.info as *const spa_device_info
  }

  pub fn add_prop<K: Into<DictionaryString>, V: Into<DictionaryString>>(&mut self, key: K, value: V) {
    self.props.add_item(key, value);
    self.info.change_mask |= SPA_DEVICE_CHANGE_MASK_PROPS as u64;
  }

  pub fn add_param(&mut self, id: u32, flags: u32) {
    assert!(self.info.n_params < MAX_PARAMS);
    self.params[self.info.n_params as usize] = spa_param_info {
      id, flags, user: 0, seq: 0, padding: [0, 0, 0, 0]
    };
    self.info.change_mask |= SPA_DEVICE_CHANGE_MASK_PARAMS as u64;
    self.info.n_params += 1;
  }

  // flip a param's serial so consumers re-read it even when the read/write
  // flags didn't change
  pub fn bump_param(&mut self, id: u32) {
    for p in &mut self.params[0..self.info.n_params as usize] {
      if p.id == id {
        p.flags ^= SPA_PARAM_INFO_SERIAL;
        self.info.change_mask |= SPA_DEVICE_CHANGE_MASK_PARAMS as u64;
        return;
      }
    }
  }

  pub fn replace_change_mask(&mut self, new_mask: u64) -> u64 {
    let old = self.info.change_mask;
    self.info.change_mask = new_mask;
    old
  }
}

pub struct NodeInfo {
  info:   spa_node_info,
  props:  Dictionary,
  params: [spa_param_info; MAX_PARAMS as usize],
}

impl NodeInfo {

  pub fn new() -> Self {
    Self {
      info: spa_node_info {
        max_input_ports:  0,
        max_output_ports: 0,
        change_mask:      0,
        flags:            0,
        props:            std::ptr::null_mut(),
        params:           std::ptr::null_mut(),
        n_params:         0
      },
      props:  Dictionary::new(),
      params: [spa_param_info { id: 0, flags: 0, user: 0, seq: 0, padding: [0, 0, 0, 0] }; MAX_PARAMS as usize],
    }
  }

  pub fn fix_pointers(&mut self) {
    self.info.props  = unsafe { self.props.raw_mut() };
    self.info.params = self.params.as_mut_ptr();
  }

  pub unsafe fn raw(&self) -> *const spa_node_info {
    &self.info as *const spa_node_info
  }

  pub fn set_max_input_ports(&mut self, max_ports: u32) {
    self.info.max_input_ports = max_ports;
    self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64; // does this field count as a flag?
  }

  pub fn set_max_output_ports(&mut self, max_ports: u32) {
    self.info.max_output_ports = max_ports;
    self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64; // does this field count as a flag?
  }

  pub fn set_flags(&mut self, flags: u64) {
    self.info.flags = flags;
    self.info.change_mask |= SPA_NODE_CHANGE_MASK_FLAGS as u64;
  }

  pub fn add_prop<K: Into<DictionaryString>, V: Into<DictionaryString>>(&mut self, key: K, value: V) {
    self.props.add_item(key, value);
    self.info.change_mask |= SPA_NODE_CHANGE_MASK_PROPS as u64;
  }

  pub fn add_param(&mut self, id: u32, flags: u32) {
    assert!(self.info.n_params < MAX_PARAMS);
    self.params[self.info.n_params as usize] = spa_param_info {
      id, flags, user: 0, seq: 0, padding: [0, 0, 0, 0]
    };
    self.info.change_mask |= SPA_NODE_CHANGE_MASK_PARAMS as u64;
    self.info.n_params += 1;
  }

  // flip a param's serial so consumers (the adapter compares flags, not user)
  // re-read it even when the read/write flags didn't change
  pub fn bump_param(&mut self, id: u32) {
    for p in &mut self.params[0..self.info.n_params as usize] {
      if p.id == id {
        p.flags ^= SPA_PARAM_INFO_SERIAL;
        self.info.change_mask |= SPA_NODE_CHANGE_MASK_PARAMS as u64;
        return;
      }
    }
  }

  pub fn replace_change_mask(&mut self, new_mask: u64) -> u64 {
    let old = self.info.change_mask;
    self.info.change_mask = new_mask;
    old
  }
}

pub struct PortInfo {
  info:    spa_port_info,
  props:   Dictionary,
  params:  [spa_param_info; MAX_PARAMS as usize]
}

impl PortInfo {

  pub fn new() -> Self {
    Self {
      info: spa_port_info {
        change_mask:      0,
        flags:            0,
        rate:             spa_fraction { num: 0, denom: 0 },
        props:            std::ptr::null_mut(),
        params:           std::ptr::null_mut(),
        n_params:         0
      },
      props:  Dictionary::new(),
      params: [spa_param_info { id: 0, flags: 0, user: 0, seq: 0, padding: [0, 0, 0, 0] }; MAX_PARAMS as usize],
    }
  }

  pub fn fix_pointers(&mut self) {
    self.info.props  = unsafe { self.props.raw_mut() };
    self.info.params = self.params.as_mut_ptr();
  }

  pub unsafe fn raw(&self) -> *const spa_port_info {
    &self.info as *const spa_port_info
  }

  pub fn set_flags(&mut self, flags: u64) {
    self.info.flags = flags;
    self.info.change_mask |= SPA_PORT_CHANGE_MASK_FLAGS as u64;
  }

  pub fn set_rate(&mut self, rate: spa_fraction) {
    self.info.rate = rate;
    self.info.change_mask |= SPA_PORT_CHANGE_MASK_RATE as u64;
  }

  /* currently unused
  pub fn add_prop<K: Into<DictionaryString>, V: Into<DictionaryString>>(&mut self, key: K, value: V) {
    self.props.add_item(key, value);
    self.info.change_mask |= SPA_PORT_CHANGE_MASK_PROPS as u64;
  }*/

  pub fn add_param(&mut self, id: u32, flags: u32) {
    assert!(self.info.n_params < MAX_PARAMS);
    self.params[self.info.n_params as usize] = spa_param_info {
      id, flags, user: 0, seq: 0, padding: [0, 0, 0, 0]
    };
    self.info.change_mask |= SPA_PORT_CHANGE_MASK_PARAMS as u64;
    self.info.n_params += 1;
  }

  // Change an advertised param's read/write flags. The host re-reads a param when
  // its flags change, so flipping Format WRITE<->READWRITE around a format
  // clear/set is what marks the port (re)negotiable.
  pub fn set_param_flags(&mut self, id: u32, flags: u32) {
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
  pub fn bump_param(&mut self, id: u32) {
    for p in &mut self.params[0..self.info.n_params as usize] {
      if p.id == id {
        p.flags ^= SPA_PARAM_INFO_SERIAL;
        self.info.change_mask |= SPA_PORT_CHANGE_MASK_PARAMS as u64;
        return;
      }
    }
  }

  pub fn replace_change_mask(&mut self, new_mask: u64) -> u64 {
    let old = self.info.change_mask;
    self.info.change_mask = new_mask;
    old
  }
}

pub struct Loop {
  loop_:   &'static spa_loop,        // not really 'static, but it should outlive our plugin anyway
  methods: &'static spa_loop_methods // ditto
}

impl Loop {

  pub unsafe fn wrap(loop_: *mut spa_loop) -> Self {
    let loop_   = loop_.cast::<spa_loop>().as_ref()
      .expect("loop should be initialized");
    let methods = loop_.iface.cb.funcs.cast::<spa_loop_methods>().as_ref()
      .expect("loop methods should be initialized");
    assert!(methods.version >= SPA_VERSION_LOOP_METHODS);
    Self { loop_, methods }
  }

  pub unsafe extern "C" fn add_source(&self, source: *mut spa_source) -> c_int {
    let spa_loop_add_source = self.methods.add_source.expect("add_source should be initialized");
    spa_loop_add_source(self.loop_.iface.cb.data, source)
  }

  // must be called from the loop thread (or through an invoke)
  pub unsafe extern "C" fn remove_source(&self, source: *mut spa_source) -> c_int {
    let spa_loop_remove_source = self.methods.remove_source.expect("remove_source should be initialized");
    spa_loop_remove_source(self.loop_.iface.cb.data, source)
  }

  pub unsafe extern "C" fn invoke(&self,
    func: spa_invoke_func_t, seq: u32, data: *const c_void, size: usize, block: bool, user_data: *mut c_void) -> c_int
  {
    let spa_loop_invoke = self.methods.invoke.expect("invoke should be initialized");
    spa_loop_invoke(self.loop_.iface.cb.data, func, seq, data, size, block, user_data)
  }
}

pub struct System {
  system:  &'static spa_system,        // not really 'static, but it should outlive our plugin anyway
  methods: &'static spa_system_methods // ditto
}

impl System {

  pub unsafe fn wrap(system: *mut spa_system) -> Self {
    let system  = system.cast::<spa_system>().as_ref()
      .expect("system should be initialized");
    let methods = system.iface.cb.funcs.cast::<spa_system_methods>().as_ref()
      .expect("system methods should be initialized");
    assert!(methods.version >= SPA_VERSION_SYSTEM_METHODS);
    Self { system, methods }
  }

  pub unsafe extern "C" fn close(&self, fd: c_int) -> c_int {
    let spa_system_close = self.methods.close.expect("close should be initialized");
    spa_system_close(self.system.iface.cb.data, fd)
  }

  pub unsafe extern "C" fn clock_gettime(&self, clock_id: c_int, value: *mut timespec) -> c_int {
    let spa_system_clock_gettime = self.methods.clock_gettime.expect("clock_gettime should be initialized");
    spa_system_clock_gettime(self.system.iface.cb.data, clock_id, value)
  }

  pub unsafe extern "C" fn timerfd_create(&self, clock_id: c_int, flags: c_int) -> c_int {
    let spa_system_timerfd_create = self.methods.timerfd_create.expect("timerfd_create should be assigned");
    spa_system_timerfd_create(self.system.iface.cb.data, clock_id, flags)
  }

  pub unsafe extern "C" fn timerfd_read(&self, fd: c_int, expirations: *mut u64) -> c_int {
    let spa_system_timerfd_read = self.methods.timerfd_read.expect("timerfd_read should be initialized");
    spa_system_timerfd_read(self.system.iface.cb.data, fd, expirations)
  }

  pub unsafe extern "C" fn timerfd_settime(&self,
    fd: c_int, flags: c_int, new_value: *const itimerspec, old_value: *mut itimerspec) -> c_int
  {
    let spa_system_timerfd_settime = self.methods.timerfd_settime.expect("timerfd_settime should be initialized");
    spa_system_timerfd_settime(self.system.iface.cb.data, fd, flags, new_value, old_value)
  }
}

pub struct Log {
  logger:  &'static spa_log,        // not really 'static, but it should outlive our plugin anyway
  methods: &'static spa_log_methods // ditto
}

// registered once with the host logger so PIPEWIRE_DEBUG topic patterns
// (e.g. PIPEWIRE_DEBUG=spa.oss:4) apply; the logger writes the level back
static mut LOG_TOPIC: spa_log_topic = spa_log_topic {
  version:          0,
  topic:            c"spa.oss".as_ptr(),
  level:            SPA_LOG_LEVEL_NONE,
  has_custom_level: false
};

static LOG_TOPIC_INIT: std::sync::Once = std::sync::Once::new();

impl Log {

  pub unsafe fn wrap(log: *mut spa_log) -> Self {
    let logger  = log.cast::<spa_log>().as_ref()
      .expect("log should be initialized");
    let methods = logger.iface.cb.funcs.cast::<spa_log_methods>().as_ref()
      .expect("log methods should be initialized");
    assert!(methods.version >= SPA_VERSION_LOG_METHODS);

    LOG_TOPIC_INIT.call_once(|| {
      if let Some(topic_init) = methods.topic_init {
        topic_init(logger.iface.cb.data, std::ptr::addr_of_mut!(LOG_TOPIC));
      }
    });

    Self { logger, methods }
  }

  pub fn log_level(&self) -> spa_log_level {
    let topic = std::ptr::addr_of!(LOG_TOPIC);
    unsafe {
      // volatile: the host logger rewrites the registered topic's level on
      // runtime log-level changes (inherent to the C API; same as C plugins)
      if std::ptr::read_volatile(std::ptr::addr_of!((*topic).has_custom_level)) {
        std::ptr::read_volatile(std::ptr::addr_of!((*topic).level))
      } else {
        self.logger.level
      }
    }
  }

  pub fn log(&self, level: spa_log_level, file: &str, line: c_int, func: &str, msg: &str) {
    let file = CString::new(file).unwrap(); // ours, no interior NULs
    let func = CString::new(func).unwrap(); // ditto
    // the message can carry host-derived strings; don't abort on an interior NUL
    let msg  = CString::new(msg).unwrap_or_else(|_| c"<message contained NUL>".to_owned());
    unsafe {
      if let Some(logt) = self.methods.logt {
        logt(self.logger.iface.cb.data, level, std::ptr::addr_of!(LOG_TOPIC),
          file.as_ptr(), line, func.as_ptr(), c"%s".as_ptr(), msg.as_ptr());
      } else {
        let log = self.methods.log.expect("log should be initialized");
        log(self.logger.iface.cb.data, level, file.as_ptr(), line, func.as_ptr(), c"%s".as_ptr(), msg.as_ptr());
      }
    }
  }
}

#[macro_export]
macro_rules! log {
  ($log:expr, $log_level:expr, $($arg:tt)*) => {
    if $log.log_level() >= $log_level {
      let file = file!();
      let line = line!();
      let func = ""; //TODO: add something there?
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
