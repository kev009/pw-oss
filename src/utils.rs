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

  pub fn read_u32<T: Into<SysctlName>>(&mut self, name: T) -> Result<u32, Errno> {
    let SysctlName::CString(name) = name.into();
    let mut value: u32 = 0;
    let mut len = std::mem::size_of::<u32>();
    if unsafe { sysctlbyname(name.as_ptr(), (&mut value as *mut u32).cast(), &mut len, std::ptr::null(), 0) } == -1 {
      return Err(Errno::last());
    }
    Ok(value)
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
      // devd events should be ASCII, but don't abort on a stray byte
      apply(&String::from_utf8_lossy(&self.buffer[..len]));
    }
  }
}

// sys/dev/sound/pcm/matrix.h interleave order; note 5.1/7.1 put FC/LF after
// the rears, unlike WAV/ALSA
pub fn channel_positions(channels: u32) -> Option<&'static [u32]> {
  use libspa::sys::*;
  static C1: [u32; 1] = [SPA_AUDIO_CHANNEL_MONO];
  static C2: [u32; 2] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR];
  static C4: [u32; 4] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                         SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR];
  static C6: [u32; 6] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                         SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR,
                         SPA_AUDIO_CHANNEL_FC, SPA_AUDIO_CHANNEL_LFE];
  static C8: [u32; 8] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                         SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR,
                         SPA_AUDIO_CHANNEL_FC, SPA_AUDIO_CHANNEL_LFE,
                         SPA_AUDIO_CHANNEL_SL, SPA_AUDIO_CHANNEL_SR];
  match channels {
    1 => Some(&C1),
    2 => Some(&C2),
    4 => Some(&C4),
    6 => Some(&C6),
    8 => Some(&C8),
    _ => None
  }
}

// One EnumFormat result per channel count the device grants, with the kernel's
// interleave order as the position array. Returns false when `index` is past
// the last result.
pub unsafe fn build_enum_format_info(b: &mut libspa::pod::builder::Builder, caps: &crate::sound::DspCaps, index: u32) -> Result<bool, rustix::io::Errno> {

  use libspa::sys::*;

  // formats supported by both us and the device, best first
  let all = [
    (crate::sound::AFMT_S32_LE, SPA_AUDIO_FORMAT_S32_LE),
    (crate::sound::AFMT_S32_BE, SPA_AUDIO_FORMAT_S32_BE),
    (crate::sound::AFMT_S16_LE, SPA_AUDIO_FORMAT_S16_LE),
    (crate::sound::AFMT_S16_BE, SPA_AUDIO_FORMAT_S16_BE)
  ];
  let mut formats = all.iter().filter(|(m, _)| caps.formats & m != 0).map(|(_, f)| *f).collect::<Vec<_>>();
  if formats.is_empty() {
    // the device only does formats we don't (e.g. S24); the kernel converts
    formats = all.iter().map(|(_, f)| *f).collect();
  }

  // counts with a defined kernel interleave order, within the granted range;
  // stereo first: the host takes the first result as the default format
  let mut counts = [2u32, 4, 6, 8, 1].iter().copied()
    .filter(|c| *c >= caps.min_channels && *c <= caps.max_channels)
    .collect::<Vec<_>>();
  // always offer the full native width too (e.g. 10-channel USB mixers);
  // non-standard counts go out as AUX channels
  if !counts.contains(&caps.max_channels) {
    counts.push(caps.max_channels);
  }

  let Some(&channels) = counts.get(index as usize) else {
    return Ok(false);
  };

  let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut outer, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat)?;

  b.add_prop(SPA_FORMAT_mediaType, 0)?;
  b.add_id(libspa::utils::Id(SPA_MEDIA_TYPE_audio))?;

  b.add_prop(SPA_FORMAT_mediaSubtype, 0)?;
  b.add_id(libspa::utils::Id(SPA_MEDIA_SUBTYPE_raw))?;

  b.add_prop(SPA_FORMAT_AUDIO_format, 0)?;
  if formats.len() == 1 {
    b.add_id(libspa::utils::Id(formats[0]))?;
  } else {
    b.push_choice(&mut inner, SPA_CHOICE_Enum, 0)?;
    for fmt in &formats {
      b.add_id(libspa::utils::Id(*fmt))?;
    }
    b.pop(inner.assume_init_mut());
  }

  b.add_prop(SPA_FORMAT_AUDIO_rate, 0)?;
  if caps.min_rate == caps.max_rate {
    b.add_int(caps.min_rate as i32)?;
  } else {
    b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
    b.add_int(48000.clamp(caps.min_rate as i32, caps.max_rate as i32))?;
    b.add_int(caps.min_rate as i32)?;
    b.add_int(caps.max_rate as i32)?;
    b.pop(inner.assume_init_mut());
  }

  b.add_prop(SPA_FORMAT_AUDIO_channels, 0)?;
  b.add_int(channels as i32)?;

  let aux_positions;
  let positions: &[u32] = match channel_positions(channels) {
    Some(positions) => positions,
    None => {
      aux_positions = (0..channels).map(|i| SPA_AUDIO_CHANNEL_AUX0 + i).collect::<Vec<u32>>();
      &aux_positions
    }
  };

  b.add_prop(SPA_FORMAT_AUDIO_position, 0)?;
  b.add_array(std::mem::size_of::<u32>() as u32, SPA_TYPE_Id,
    positions.len() as u32, positions.as_ptr().cast())?;

  b.pop(outer.assume_init_mut());

  Ok(true)
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

// Run `f` on the data loop and wait for it; serializes main-thread
// reconfiguration against process()/on_timeout() (runs inline when already on
// the loop thread). The closure and target cross a thread boundary; callers
// only capture raw pointers and plain data. Returns false when the invoke
// failed or the closure panicked - the closure then may not have run.
pub unsafe fn block_on_loop<T, F: FnOnce(&mut T)>(loop_: &crate::spa::Loop, target: *mut T, f: F) -> bool {

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
    let target = ctx.target;
    // a panic must not unwind into the C loop (that aborts the daemon)
    let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
      f(target.as_mut().expect("target is not supposed to be null"));
    }));
    if ok.is_err() { -libc::ECANCELED } else { 0 }
  }

  // blocking, so `ctx` outlives the call
  let mut ctx = Ctx { target, f: Some(f) };
  let err = loop_.invoke(Some(trampoline::<T, F>), 0, std::ptr::null(), 0, true,
    &mut ctx as *mut _ as *mut std::os::raw::c_void);
  err >= 0
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

pub fn process_latency_default() -> libspa::sys::spa_process_latency_info {
  libspa::sys::spa_process_latency_info { quantum: 0.0, rate: 0, ns: 0 }
}

// spa_process_latency_parse is static inline C, so reimplemented here
pub unsafe fn parse_process_latency_info(param: *const libspa::sys::spa_pod) -> Option<libspa::sys::spa_process_latency_info> {

  use libspa::sys::*;
  use libspa::pod::{Value, Object, Pod};
  use libspa::pod::deserialize::PodDeserializer;

  match PodDeserializer::deserialize_any_from(Pod::from_raw(param).as_bytes()) {
    Ok((_, Value::Object(Object { type_, properties, .. }))) if type_ == SPA_TYPE_OBJECT_ParamProcessLatency => {
      let mut info = process_latency_default();
      for p in properties {
        #[allow(non_upper_case_globals)]
        match (p.key, p.value) {
          (SPA_PARAM_PROCESS_LATENCY_quantum, Value::Float(v)) => info.quantum = v,
          (SPA_PARAM_PROCESS_LATENCY_rate,    Value::Int(v))   => info.rate    = v,
          (SPA_PARAM_PROCESS_LATENCY_ns,      Value::Long(v))  => info.ns      = v,
          _ => ()
        }
      }
      Some(info)
    },
    _ => None
  }
}

// spa_process_latency_build is static inline C, so reimplemented here
pub unsafe fn build_process_latency_info(b: &mut libspa::pod::builder::Builder, info: &libspa::sys::spa_process_latency_info) -> Result<(), rustix::io::Errno> {

  use libspa::sys::*;

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut frame, SPA_TYPE_OBJECT_ParamProcessLatency, SPA_PARAM_ProcessLatency)?;

  b.add_prop(SPA_PARAM_PROCESS_LATENCY_quantum, 0)?;
  b.add_float(info.quantum)?;
  b.add_prop(SPA_PARAM_PROCESS_LATENCY_rate, 0)?;
  b.add_int(info.rate)?;
  b.add_prop(SPA_PARAM_PROCESS_LATENCY_ns, 0)?;
  b.add_long(info.ns)?;

  b.pop(frame.assume_init_mut());

  Ok(())
}

// spa_process_latency_info_add is static inline C, so reimplemented here
pub fn process_latency_info_add(process: &libspa::sys::spa_process_latency_info, info: &mut libspa::sys::spa_latency_info) {
  info.min_quantum += process.quantum;
  info.max_quantum += process.quantum;
  info.min_rate    += process.rate;
  info.max_rate    += process.rate;
  info.min_ns      += process.ns;
  info.max_ns      += process.ns;
}

pub unsafe fn build_latency_offset_prop_info(b: &mut libspa::pod::builder::Builder) -> Result<(), rustix::io::Errno> {

  use libspa::sys::*;

  let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut outer, SPA_TYPE_OBJECT_PropInfo, SPA_PARAM_PropInfo)?;

  b.add_prop(SPA_PROP_INFO_id, 0)?;
  b.add_id(libspa::utils::Id(SPA_PROP_latencyOffsetNsec))?;

  b.add_prop(SPA_PROP_INFO_description, 0)?;
  b.add_string("Latency offset (ns)")?;

  b.add_prop(SPA_PROP_INFO_type, 0)?;
  b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
  b.add_long(0)?;
  b.add_long(0)?;
  b.add_long(2 * SPA_NSEC_PER_SEC as i64)?;
  b.pop(inner.assume_init_mut());

  b.pop(outer.assume_init_mut());

  Ok(())
}

pub unsafe fn build_latency_offset_props(b: &mut libspa::pod::builder::Builder, ns: i64, oss_delay: Option<u32>) -> Result<(), rustix::io::Errno> {

  use libspa::sys::*;

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut frame, SPA_TYPE_OBJECT_Props, SPA_PARAM_Props)?;

  b.add_prop(SPA_PROP_latencyOffsetNsec, 0)?;
  b.add_long(ns)?;

  // custom key/value props ride in the params struct
  if let Some(delay) = oss_delay {
    let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
    b.add_prop(SPA_PROP_params, 0)?;
    b.push_struct(&mut inner)?;
    b.add_string("oss.delay")?;
    b.add_int(delay as i32)?;
    b.pop(inner.assume_init_mut());
  }

  b.pop(frame.assume_init_mut());

  Ok(())
}

// identify our device clock (spa_io_clock.name) so consumers can tell whether
// two nodes tick from the same hardware
pub unsafe fn set_clock_name(clock: *mut libspa::sys::spa_io_clock, name: &std::ffi::CStr) {
  if clock.is_null() {
    return;
  }
  let bytes = name.to_bytes_with_nul();
  let n = bytes.len().min(63);
  std::ptr::copy_nonoverlapping(bytes.as_ptr().cast(), (*clock).name.as_mut_ptr(), n);
  (*clock).name[63] = 0;
}

// does the driver's clock in `position` carry our clock name? (then we tick
// from the same device and rate matching is pointless - ALSA does the same
// clock-name comparison)
pub unsafe fn same_clock(position: *const libspa::sys::spa_io_position, name: &std::ffi::CStr) -> bool {
  if position.is_null() {
    return false;
  }
  let theirs = &(*position).clock.name;
  let ours   = name.to_bytes();
  if ours.is_empty() || ours.len() >= theirs.len() || theirs[0] == 0 {
    return false;
  }
  for (i, &b) in ours.iter().enumerate() {
    if theirs[i] as u8 != b {
      return false;
    }
  }
  theirs[ours.len()] == 0
}

// one graph cycle expressed in device bytes; the device rate can differ from
// the graph rate (the adapter's resampler makes up the difference)
pub fn device_period_bytes(target_duration: u64, device_rate: u32, graph_rate: u32, stride: u32) -> u32 {
  if graph_rate == 0 {
    return 0;
  }
  // saturate: a corrupt duration must not wrap (or panic in debug builds)
  (target_duration.saturating_mul(device_rate as u64) / graph_rate as u64)
    .saturating_mul(stride as u64)
    .min(u32::MAX as u64) as u32
}

// Fire-and-forget: queue `f` to run once on the given loop (from any thread;
// non-blocking, RT-safe on the caller side). The closure is boxed and freed
// after it runs. Returns false when it could not even be queued.
pub unsafe fn invoke_on_loop<T, F: FnOnce(&mut T)>(loop_: &crate::spa::Loop, target: *mut T, f: F) -> bool {

  struct Ctx<T, F> {
    target: *mut T,
    f:      F
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
    let ctx = Box::from_raw(user_data.cast::<Ctx<T, F>>());
    let target = ctx.target;
    let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
      (ctx.f)(target.as_mut().expect("target is not supposed to be null"));
    }));
    if ok.is_err() { -libc::ECANCELED } else { 0 }
  }

  let ctx = Box::into_raw(Box::new(Ctx { target, f }));
  let err = loop_.invoke(Some(trampoline::<T, F>), 0, std::ptr::null(), 0, false,
    ctx as *mut std::os::raw::c_void);
  if err < 0 {
    drop(Box::from_raw(ctx));
    return false;
  }
  true
}

// at most one message a second from a per-cycle warn site, with a count of
// what went unsaid (ALSA uses spa_ratelimit for the same purpose)
pub struct RateLimit {
  last:       u64,
  suppressed: u32
}

impl RateLimit {

  pub const fn new() -> Self {
    Self { last: 0, suppressed: 0 }
  }

  // Some(previously suppressed count) when the caller may log now
  pub fn check(&mut self, now: u64) -> Option<u32> {
    if now.saturating_sub(self.last) >= 1_000_000_000 {
      self.last = now;
      Some(std::mem::take(&mut self.suppressed))
    } else {
      self.suppressed += 1;
      None
    }
  }
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
