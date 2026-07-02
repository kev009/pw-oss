use std::ffi::CString;
use libc::sysctlbyname;
use nix::errno::Errno;

pub enum SysctlName {
  CString(CString)
}

impl From<&str> for SysctlName {

  fn from(str: &str) -> Self {
    SysctlName::CString(CString::new(str).unwrap())
  }
}

impl From<String> for SysctlName {

  fn from(str: String) -> Self {
    SysctlName::CString(CString::new(str).unwrap())
  }
}

pub struct SysctlReader {
  scratch_buffer: Vec<u8>
}

impl SysctlReader {

  pub fn new() -> Self {
    Self {
      scratch_buffer: Vec::with_capacity(32)
    }
  }

  pub fn read_string<T: Into<SysctlName>>(&mut self, name: T, max_len: usize) -> Result<String, Errno> {

    let SysctlName::CString(name) = name.into();

    let mut len = 0;
    if unsafe { sysctlbyname(name.as_ptr(), std::ptr::null_mut(), &mut len, std::ptr::null(), 0) } == -1 {
      return Err(Errno::last())
    }

    if len > max_len {
      return Err(Errno::ENOMEM);
    }

    if len == 0 {
      return Ok("".to_string());
    }

    self.scratch_buffer.resize(len, 0);
    if unsafe { sysctlbyname(name.as_ptr(), self.scratch_buffer.as_mut_ptr().cast(), &mut len, std::ptr::null(), 0) } == -1 {
      return Err(Errno::last());
    }

    Ok(String::from_utf8_lossy(&self.scratch_buffer[0..len]).to_string())
  }
}

use std::os::fd::AsRawFd;
use std::os::fd::RawFd;
use uds::UnixSeqpacketConn;

pub struct DevdSocket {
  socket: UnixSeqpacketConn,
  buffer: Vec<u8>
}

impl DevdSocket {

  pub fn open() -> Result<Self, std::io::Error> {
    let socket = UnixSeqpacketConn::connect("/var/run/devd.seqpacket.pipe")?;
    let buffer = [0; 8192 /* DEVCTL_MAXBUF */].to_vec();
    Ok(Self {
      socket,
      buffer
    })
  }

  pub fn fd(&self) -> RawFd {
    self.socket.as_raw_fd()
  }

  pub fn read_event(&mut self, mut apply: impl FnMut(&str)) {
    if let Ok(len) = self.socket.recv(&mut self.buffer) {
      assert!(len <= self.buffer.len());
      apply(std::str::from_utf8(&self.buffer[..len]).unwrap());
    }
  }
}

pub unsafe fn build_enum_format_info(b: &mut libspa::pod::builder::Builder, mono: bool) -> Result<(), rustix::io::Errno> {

  use libspa::sys::*;

  let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut outer, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat)?;

  b.add_prop(SPA_FORMAT_mediaType, 0)?;
  b.add_id(libspa::utils::Id(SPA_MEDIA_TYPE_audio))?;

  b.add_prop(SPA_FORMAT_mediaSubtype, 0)?;
  b.add_id(libspa::utils::Id(SPA_MEDIA_SUBTYPE_raw))?;

  b.add_prop(SPA_FORMAT_AUDIO_format, 0)?;
  b.push_choice(&mut inner, SPA_CHOICE_Enum, 0)?;
  for fmt in [
    SPA_AUDIO_FORMAT_S32,
    SPA_AUDIO_FORMAT_S32_OE,
    SPA_AUDIO_FORMAT_S16,
    SPA_AUDIO_FORMAT_S16_OE
  ] {
    b.add_id(libspa::utils::Id(fmt))?;
  }
  b.pop(inner.assume_init_mut());

  b.add_prop(SPA_FORMAT_AUDIO_rate, 0)?;
  b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
  b.add_int( 48000)?;
  b.add_int(     1)?;
  b.add_int(192000)?;
  b.pop(inner.assume_init_mut());

  if !mono {
    b.add_prop(SPA_FORMAT_AUDIO_channels, 0)?;
    b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
    b.add_int(2)?;
    b.add_int(1)?;
    b.add_int(SPA_AUDIO_MAX_CHANNELS as i32)?;
    b.pop(inner.assume_init_mut());

    b.add_prop(SPA_FORMAT_AUDIO_position, 0)?;
    b.add_array(std::mem::size_of_val(&SPA_AUDIO_CHANNEL_FL) as u32, SPA_TYPE_Id, 2,
      [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR].as_ptr().cast())?;
  } else {
    b.add_prop(SPA_FORMAT_AUDIO_channels, 0)?;
    b.add_int(1)?;
  }

  b.pop(outer.assume_init_mut());

  Ok(())
}

pub unsafe fn build_buffers_info(b: &mut libspa::pod::builder::Builder, stride: u32) -> Result<(), rustix::io::Errno> {

  use libspa::sys::*;

  // The point here is dataType = MemPtr: process() maps the buffer memory
  // directly, so a MemFd/DmaBuf block would be unusable. Sizes are permissive;
  // the graph's quantum drives the real per-buffer size.
  let default = 1024 * stride;
  let max     = 16384 * stride;

  let mut obj    = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut choice = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut obj, SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers)?;

  b.add_prop(SPA_PARAM_BUFFERS_buffers, 0)?;
  b.push_choice(&mut choice, SPA_CHOICE_Range, 0)?;
  b.add_int(2)?;  b.add_int(1)?;  b.add_int(32)?;   // default, min, max
  b.pop(choice.assume_init_mut());

  b.add_prop(SPA_PARAM_BUFFERS_blocks, 0)?;
  b.add_int(1)?;

  b.add_prop(SPA_PARAM_BUFFERS_size, 0)?;
  b.push_choice(&mut choice, SPA_CHOICE_Range, 0)?;
  b.add_int(default as i32)?;  b.add_int(stride as i32)?;  b.add_int(max as i32)?;
  b.pop(choice.assume_init_mut());

  b.add_prop(SPA_PARAM_BUFFERS_stride, 0)?;
  b.add_int(stride as i32)?;

  b.add_prop(SPA_PARAM_BUFFERS_align, 0)?;
  b.add_int(16)?;

  b.add_prop(SPA_PARAM_BUFFERS_dataType, 0)?;
  b.add_int(1i32 << SPA_DATA_MemPtr)?;

  b.pop(obj.assume_init_mut());

  Ok(())
}

// run `f` on the data loop and wait for it; serializes main-thread
// reconfiguration against process()/on_timeout() (runs inline when already on
// the loop thread)
pub unsafe fn block_on_loop<T, F: FnOnce(&mut T)>(loop_: &crate::spa::Loop, target: *mut T, f: F) {

  struct Ctx<T, F> {
    target: *mut T,
    f:      Option<F>
  }

  unsafe extern "C" fn trampoline<T, F: FnOnce(&mut T)>(
    _loop:     *mut libspa::sys::spa_loop,
    _async:    bool,
    _seq:      u32,
    _data:     *const std::os::raw::c_void,
    _size:     usize,
    user_data: *mut std::os::raw::c_void
  ) -> std::os::raw::c_int
  {
    let ctx = user_data.cast::<Ctx<T, F>>().as_mut()
      .expect("user_data is not supposed to be null");
    let f = ctx.f.take()
      .expect("the invoked function only runs once");
    f(ctx.target.as_mut().expect("target is not supposed to be null"));
    0
  }

  // blocking, so `ctx` outlives the call
  let mut ctx = Ctx { target, f: Some(f) };
  let err = loop_.invoke(Some(trampoline::<T, F>), 0, std::ptr::null(), 0, true,
    &mut ctx as *mut _ as *mut std::os::raw::c_void);
  assert!(err >= 0);
}

pub fn latency_info_default(direction: libspa::sys::spa_direction) -> libspa::sys::spa_latency_info {
  libspa::sys::spa_latency_info {
    direction,
    min_quantum: 0.0,
    max_quantum: 0.0,
    min_rate:    0,
    max_rate:    0,
    min_ns:      0,
    max_ns:      0
  }
}

// spa_latency_parse is static inline C, so reimplemented here
pub unsafe fn parse_latency_info(param: *const libspa::sys::spa_pod) -> Option<libspa::sys::spa_latency_info> {

  use libspa::sys::*;
  use libspa::pod::{Value, Object, Pod};
  use libspa::pod::deserialize::PodDeserializer;

  match PodDeserializer::deserialize_any_from(Pod::from_raw(param).as_bytes()) {
    Ok((_, Value::Object(Object { type_, properties, .. }))) if type_ == SPA_TYPE_OBJECT_ParamLatency => {
      let mut info = latency_info_default(SPA_DIRECTION_INPUT);
      for p in properties {
        #[allow(non_upper_case_globals)]
        match (p.key, p.value) {
          (SPA_PARAM_LATENCY_direction,  Value::Id(v))    => info.direction   = v.0 & 1,
          (SPA_PARAM_LATENCY_minQuantum, Value::Float(v)) => info.min_quantum = v,
          (SPA_PARAM_LATENCY_maxQuantum, Value::Float(v)) => info.max_quantum = v,
          (SPA_PARAM_LATENCY_minRate,    Value::Int(v))   => info.min_rate    = v,
          (SPA_PARAM_LATENCY_maxRate,    Value::Int(v))   => info.max_rate    = v,
          (SPA_PARAM_LATENCY_minNs,      Value::Long(v))  => info.min_ns      = v,
          (SPA_PARAM_LATENCY_maxNs,      Value::Long(v))  => info.max_ns      = v,
          _ => ()
        }
      }
      Some(info)
    },
    _ => None
  }
}

// spa_latency_build is static inline C, so reimplemented here
pub unsafe fn build_latency_info(b: &mut libspa::pod::builder::Builder, info: &libspa::sys::spa_latency_info) -> Result<(), rustix::io::Errno> {

  use libspa::sys::*;

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut frame, SPA_TYPE_OBJECT_ParamLatency, SPA_PARAM_Latency)?;

  b.add_prop(SPA_PARAM_LATENCY_direction, 0)?;
  b.add_id(libspa::utils::Id(info.direction))?;

  b.add_prop(SPA_PARAM_LATENCY_minQuantum, 0)?;
  b.add_float(info.min_quantum)?;
  b.add_prop(SPA_PARAM_LATENCY_maxQuantum, 0)?;
  b.add_float(info.max_quantum)?;

  b.add_prop(SPA_PARAM_LATENCY_minRate, 0)?;
  b.add_int(info.min_rate)?;
  b.add_prop(SPA_PARAM_LATENCY_maxRate, 0)?;
  b.add_int(info.max_rate)?;

  b.add_prop(SPA_PARAM_LATENCY_minNs, 0)?;
  b.add_long(info.min_ns)?;
  b.add_prop(SPA_PARAM_LATENCY_maxNs, 0)?;
  b.add_long(info.max_ns)?;

  b.pop(frame.assume_init_mut());

  Ok(())
}

pub fn now_ns(system: &crate::spa::System) -> u64 {
  let mut now = libspa::sys::timespec { tv_sec: 0, tv_nsec: 0 };
  let err = unsafe { system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
  assert!(err != -1);
  (now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64
}

#[cfg(debug_assertions)]
pub fn now_ns_libc() -> u64 {
  let mut now = libc::timespec { tv_sec: 0, tv_nsec: 0 };
  let err = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
  assert!(err != -1);
  (now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64
}

pub fn spa_command_to_str(body: &libspa::sys::spa_pod_object_body) -> &'static str {
  use libspa::sys::*;
  #[allow(non_upper_case_globals)]
  match (body.type_, body.id) {
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start)      => "SPA_NODE_COMMAND_Start",
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend)    => "SPA_NODE_COMMAND_Suspend",
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause)      => "SPA_NODE_COMMAND_Pause",
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin) => "SPA_NODE_COMMAND_ParamBegin",
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamEnd)   => "SPA_NODE_COMMAND_ParamEnd",
    (SPA_TYPE_COMMAND_Node, _)                           => "SPA_NODE_COMMAND_???",
    _ => "???"
  }
}
