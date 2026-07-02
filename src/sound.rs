use std::collections::BTreeMap;
use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_long, c_uint, c_ulong, c_void};
use libc::{size_t, ssize_t};
use nix::errno::Errno;

pub const AFMT_S16_LE: u32 = 0x00000010;
pub const AFMT_S16_BE: u32 = 0x00000020;
pub const AFMT_S32_LE: u32 = 0x00001000;
pub const AFMT_S32_BE: u32 = 0x00002000;

const SNDCTL_DSP_SPEED:       c_ulong = nix::request_code_readwrite!(b'P',  2, std::mem::size_of::<c_int>());
const SNDCTL_DSP_SETFMT:      c_ulong = nix::request_code_readwrite!(b'P',  5, std::mem::size_of::<c_int>());
const SNDCTL_DSP_CHANNELS:    c_ulong = nix::request_code_readwrite!(b'P',  6, std::mem::size_of::<c_int>());
const SNDCTL_DSP_SETFRAGMENT: c_ulong = nix::request_code_readwrite!(b'P', 10, std::mem::size_of::<c_int>());
const SNDCTL_DSP_GETFMTS:     c_ulong = nix::request_code_read!     (b'P', 11, std::mem::size_of::<c_int>());
const SNDCTL_DSP_GETOSPACE:   c_ulong = nix::request_code_read!     (b'P', 12, std::mem::size_of::<audio_buf_info>());
const SNDCTL_DSP_GETISPACE:   c_ulong = nix::request_code_read!     (b'P', 13, std::mem::size_of::<audio_buf_info>());
const SNDCTL_DSP_SETTRIGGER:  c_ulong = nix::request_code_write!    (b'P', 16, std::mem::size_of::<c_int>());
//const SNDCTL_DSP_GETPLAYVOL:  c_ulong = nix::request_code_read!     (b'P', 24, std::mem::size_of::<c_int>());
//const SNDCTL_DSP_SETPLAYVOL:  c_ulong = nix::request_code_readwrite!(b'P', 24, std::mem::size_of::<c_int>());
const SNDCTL_DSP_GETODELAY:   c_ulong = nix::request_code_read!     (b'P', 23, std::mem::size_of::<c_int>());
const SNDCTL_DSP_GETERROR:    c_ulong = nix::request_code_read!     (b'P', 25, std::mem::size_of::<audio_errinfo>());
const SNDCTL_DSP_HALT:        c_ulong = nix::request_code_none!     (b'P',  0); // aka SNDCTL_DSP_RESET
const SNDCTL_ENGINEINFO:      c_ulong = nix::request_code_readwrite!(b'X', 12, std::mem::size_of::<oss_audioinfo>());

// sys/soundcard.h; the ioctl encodes the size, so a layout mismatch fails
// cleanly instead of corrupting memory
#[repr(C)]
struct oss_audioinfo {
  dev:              c_int,
  name:             [c_char; 64],
  busy:             c_int,
  pid:              c_int,
  caps:             c_int,
  iformats:         c_int,
  oformats:         c_int,
  magic:            c_int,
  cmd:              [c_char; 64],
  card_number:      c_int,
  port_number:      c_int,
  mixer_dev:        c_int,
  legacy_device:    c_int,
  enabled:          c_int,
  flags:            c_int,
  min_rate:         c_int,
  max_rate:         c_int,
  min_channels:     c_int,
  max_channels:     c_int,
  binding:          c_int,
  rate_source:      c_int,
  handle:           [c_char; 32],
  nrates:           c_uint,
  rates:            [c_uint; 20],
  song_name:        [c_char; 64],
  label:            [c_char; 16],
  latency:          c_int,
  devnode:          [c_char; 32],
  next_play_engine: c_int,
  next_rec_engine:  c_int,
  filler:           [c_int; 184]
}

// sys/dev/sound/pcm/matrix.h: SETCHANNELS requests are clamped to this
const SND_CHN_MAX: c_int = 8;

// currently unused
#[allow(dead_code)]
const PCM_ENABLE_INPUT:  c_int = 0x00000001;
const PCM_ENABLE_OUTPUT: c_int = 0x00000002;

// sys/dev/sound/pcm/channel.h
const CHN_2NDBUFMAXSIZE: usize = 131072;

#[repr(C)]
struct audio_buf_info {
  fragments:  c_int,
  fragstotal: c_int,
  fragsize:   c_int,
  bytes:      c_int
}

#[repr(C)]
struct audio_errinfo {
  play_underruns:  c_int,
  rec_overruns:    c_int,
  play_ptradjust:  c_uint,
  rec_ptradjust:   c_uint,
  play_errorcount: c_int,
  rec_errorcount:  c_int,
  play_lasterror:  c_int,
  rec_lasterror:   c_int,
  play_errorparm:  c_long,
  rec_errorparm:   c_long,
  filler:          [c_int; 16]
}

#[derive(Debug, PartialEq)]
enum DspState {
  Closed,
  Setup,
  Running
}

#[derive(Debug, Clone, Copy)]
pub struct DspCaps {
  pub formats:        u32, // AFMT_* mask
  pub min_channels:   u32,
  pub max_channels:   u32,
  pub min_rate:       u32,
  pub max_rate:       u32,
  pub preferred_rate: Option<u32> // the parent's vchan mix rate, when known
}

impl DspCaps {

  // used when the device can't be probed (e.g. busy); conservative
  pub fn fallback() -> Self {
    Self {
      formats:      AFMT_S16_LE | AFMT_S16_BE | AFMT_S32_LE | AFMT_S32_BE,
      min_channels: 1,
      max_channels: 2,
      min_rate:       8000,
      max_rate:       192000,
      preferred_rate: None
    }
  }
}

// Ask the device what it actually supports. Two sources, merged:
// - empirical SETCHANNELS/SPEED probes at the extremes (OSS grants the nearest
//   supported value) - but the kernel clamps channel requests to SND_CHN_MAX
//   and bitperfect devices reject unsupported values instead of snapping;
// - SNDCTL_AUDIOINFO, which reports the real hardware limits (dsp.c
//   aggregates chn_getcaps over the device), covering both gaps above.
// Uses a transient open; the caller falls back if the device is busy.
pub fn probe_caps(path: &str, play: bool) -> Option<DspCaps> {

  let cpath = CString::new(path).ok()?;
  let mode  = if play { libc::O_WRONLY } else { libc::O_RDONLY };
  let fd    = unsafe { libc::open(cpath.as_ptr(), mode | libc::O_NONBLOCK) };
  if fd == -1 {
    return None;
  }

  let mut formats: c_int = 0;
  let formats_ok = unsafe { libc::ioctl(fd, SNDCTL_DSP_GETFMTS, &mut formats) } != -1;

  // ENGINEINFO with dev == -1 resolves the channel bound to THIS fd, so the
  // limits are per-direction (AUDIOINFO blends play and rec across the
  // device). Note: kernels before the 15.x sound rewrite report a vchan's
  // fixed rate here instead of the feeder range; harmless, since these
  // values are only consulted when the empirical probe fails.
  let (ai_min_ch, ai_max_ch, ai_min_rate, ai_max_rate, ai_caps) = unsafe {
    let mut ai = std::mem::MaybeUninit::<oss_audioinfo>::zeroed();
    (*ai.as_mut_ptr()).dev = -1; // this fd's channel
    if libc::ioctl(fd, SNDCTL_ENGINEINFO, ai.as_mut_ptr()) == -1 {
      (0, 0, 0, 0, 0)
    } else {
      let ai = ai.assume_init();
      (ai.min_channels, ai.max_channels, ai.min_rate, ai.max_rate, ai.caps)
    }
  };
  const PCM_CAP_VIRTUAL: c_int = 0x0004_0000;

  let probe = |req: c_ulong, val: c_int| -> c_int {
    let mut v = val;
    if unsafe { libc::ioctl(fd, req, &mut v) } == -1 { -1 } else { v }
  };

  // a failed probe (bitperfect device) defers to the audioinfo limits
  let pick = |probed: c_int, ai_val: c_int| if probed >= 1 { probed } else { ai_val };

  // On a vchan the feeder converts and SETCHANNELS clamps at SND_CHN_MAX, so
  // advertising the engine's wider native count would only fail at configure
  // time. On a DIRECT channel (bitperfect / vchans off) the grant snaps to a
  // native format and wider counts are genuinely negotiable, so the engine
  // width extends the probe there (e.g. 10-channel USB mixers).
  let direct = ai_caps != 0 && ai_caps & PCM_CAP_VIRTUAL == 0;
  let min_channels = pick(probe(SNDCTL_DSP_CHANNELS, 1), ai_min_ch);
  let max_channels = {
    let probed = pick(probe(SNDCTL_DSP_CHANNELS, SND_CHN_MAX), ai_max_ch);
    if direct { probed.max(ai_max_ch) } else { probed }
  };
  let min_rate     = pick(probe(SNDCTL_DSP_SPEED, 8000), ai_min_rate);
  let max_rate     = pick(probe(SNDCTL_DSP_SPEED, 192000), ai_max_rate);

  unsafe { libc::close(fd) };

  if !formats_ok || min_channels < 1 || max_channels < min_channels || min_rate < 1 || max_rate < min_rate {
    return None;
  }

  // On a vchan the parent hardware mixes at dev.pcm.N.{play,rec}.vchanrate;
  // preferring it avoids a second in-kernel resample on non-48k parents.
  // ENODEV/EINVAL (direct channel, vchans off) just means no preference.
  let preferred_rate = path.trim_start_matches("/dev/dsp").parse::<u32>().ok()
    .and_then(|unit| {
      let dir = if play { "play" } else { "rec" };
      crate::utils::SysctlReader::new()
        .read_u32(format!("dev.pcm.{}.{}.vchanrate", unit, dir)).ok()
    })
    .filter(|r| (min_rate as u32..=max_rate as u32).contains(r));

  Some(DspCaps {
    formats:        formats as u32,
    min_channels:   min_channels as u32,
    max_channels:   max_channels as u32,
    min_rate:       min_rate as u32,
    max_rate:       max_rate as u32,
    preferred_rate
  })
}

// hw.snd.feeder_rate_round: the kernel snaps a requested rate within this of
// the hardware clock to the exact hardware rate (channel.c chn_setparam);
// it's a runtime sysctl (0..500), so read it, falling back to the default
const FEEDER_RATE_ROUND_DEFAULT: u32 = 25;

fn feeder_rate_round() -> u32 {
  crate::utils::SysctlReader::new()
    .read_u32("hw.snd.feeder_rate_round")
    .unwrap_or(FEEDER_RATE_ROUND_DEFAULT)
    .min(500)
}

// OSS grants the nearest supported value instead of failing, so a grant that
// differs from the request beyond `tolerance` is a rejection here
fn set_value(fd: c_int, req: c_ulong, value: u32, tolerance: u32) -> Result<(), Errno> {
  let mut v = value as c_int;
  if unsafe { libc::ioctl(fd, req, &mut v) } == -1 {
    return Err(Errno::last());
  }
  if (v as i64 - value as i64).unsigned_abs() > tolerance as u64 {
    return Err(Errno::EINVAL);
  }
  Ok(())
}

fn ospace_in_bytes(fd: c_int) -> c_int {
  let mut info = std::mem::MaybeUninit::<audio_buf_info>::uninit();
  unsafe {
    if libc::ioctl(fd, SNDCTL_DSP_GETOSPACE, info.as_mut_ptr()) == -1 {
      return 0; // e.g. the device was unplugged mid-stream
    }
    info.assume_init().bytes
  }
}

fn set_fragment(fd: c_int, n_frags: u16, frag_size_selector: u16) {
  let mut s = ((n_frags as u32) << 16) | frag_size_selector as u32;
  // best-effort: the caller reads the real grant back via GETOSPACE
  let _ = unsafe { libc::ioctl(fd, SNDCTL_DSP_SETFRAGMENT, &mut s) };
  // FreeBSD can grant a smaller layout than requested. The caller reads the real
  // size from GETOSPACE, so don't assert the request was honored.
}

fn set_trigger(fd: c_int, mask: c_int) -> bool {
  let mut m = mask;
  unsafe { libc::ioctl(fd, SNDCTL_DSP_SETTRIGGER, &mut m) != -1 }
}

fn odelay(fd: c_int) -> c_int {
  let mut delay: c_int = -1;
  if unsafe { libc::ioctl(fd, SNDCTL_DSP_GETODELAY, &mut delay) } == -1 {
    return 0; // e.g. the device was unplugged mid-stream
  }
  delay
}

// The fragment size the driver actually granted, which need not match the
// SETFRAGMENT request: some drivers (e.g. snd_hdspe) force a fixed period.
// GETBLKSIZE returns EINVAL here, so read GETOSPACE's fragsize field.
fn blocksize(fd: c_int) -> c_int {
  let mut info = std::mem::MaybeUninit::<audio_buf_info>::uninit();
  let err = unsafe { libc::ioctl(fd, SNDCTL_DSP_GETOSPACE, info.as_mut_ptr()) };
  if err != -1 { unsafe { info.assume_init().fragsize } } else { 0 }
}

fn get_error(fd: c_int) -> audio_errinfo {
  let mut info = std::mem::MaybeUninit::<audio_errinfo>::zeroed();
  unsafe {
    if libc::ioctl(fd, SNDCTL_DSP_GETERROR, info.as_mut_ptr()) == -1 {
      return std::mem::zeroed(); // e.g. the device was unplugged mid-stream
    }
    info.assume_init()
  }
}

pub struct Dsp {
  path:  CString,
  fd:    c_int,
  state: DspState,
  needs_trigger: bool // trigger-suspended: NOTRIGGER must be cleared on restart
}

impl Dsp {

  pub fn new(path: &str) -> Self {
    Self { path: CString::new(path).unwrap(), fd: -1, state: DspState::Closed, needs_trigger: false }
  }

  pub fn is_closed(&self) -> bool {
    self.state == DspState::Closed
  }

  pub fn is_running(&self) -> bool {
    self.state == DspState::Running
  }

  pub fn open(&mut self) -> Result<(), Errno> {
    assert_eq!(self.state, DspState::Closed);

    // O_RDONLY, not O_RDWR: on devices with asymmetric play/rec channel
    // counts (e.g. RODECaster) the kernel won't take per-direction counts on
    // one fd (shkhln/pw-oss#3)
    let fd = unsafe { libc::open(self.path.as_ptr(), libc::O_RDONLY) };
    if fd == -1 {
      return Err(Errno::last());
    }

    self.fd    = fd;
    self.state = DspState::Setup;

    Ok(())
  }

  pub fn close(&mut self) {
    assert_ne!(self.state, DspState::Closed);
    unsafe { libc::close(self.fd) };
    self.fd    = -1;
    self.state = DspState::Closed;
    self.needs_trigger = false;
  }

  pub fn configure(&mut self, format: u32, channels: u32, rate: u32) -> Result<(), Errno> {
    assert_eq!(self.state, DspState::Setup);
    set_value(self.fd, SNDCTL_DSP_SETFMT,   format,   0)?;
    set_value(self.fd, SNDCTL_DSP_CHANNELS, channels, 0)?;
    set_value(self.fd, SNDCTL_DSP_SPEED,    rate,     feeder_rate_round())
  }

  // Size the capture ring into small fragments: the kernel's poll trigger is
  // one fragment (chn_polltrigger, lw = blksz), and the hw.snd.latency default
  // can exceed a small graph period, which would make every poll come up empty.
  pub fn set_small_fragments(&mut self) {
    assert_eq!(self.state, DspState::Setup);
    set_fragment(self.fd, 64, 10); // 64 x 1 KiB
  }

  // GETOSPACE requires a write channel, so the shared helper reads 0 on a
  // capture fd; GETISPACE's fragsize is the capture-side equivalent
  pub fn blocksize(&self) -> u32 {
    let mut info = std::mem::MaybeUninit::<audio_buf_info>::uninit();
    if unsafe { libc::ioctl(self.fd, SNDCTL_DSP_GETISPACE, info.as_mut_ptr()) } == -1 {
      return 0;
    }
    unsafe { info.assume_init().fragsize.max(0) as u32 }
  }

  // Stop the channel but keep the fd: SETTRIGGER(0) aborts, resets the ring
  // and clears TRIGGERED, so the next prime retunes and poll() force-starts
  // the channel again (chn_poll ignores NOTRIGGER). false = driver refused;
  // the caller falls back to closing.
  pub fn suspend(&mut self) -> bool {
    if self.state != DspState::Running {
      return true; // nothing runs; already primable
    }
    if !set_trigger(self.fd, 0) {
      return false;
    }
    self.state = DspState::Setup;
    self.needs_trigger = true;
    true
  }

  pub unsafe fn read(&mut self, buf: *mut c_void, count: size_t) -> ssize_t {
    if self.state == DspState::Setup {
      self.state = DspState::Running;
    }
    assert_eq!(self.state, DspState::Running);
    libc::read(self.fd, buf, count)
  }

  pub fn ready_for_reading(&mut self, timeout_ms: usize) -> bool {

    if self.state == DspState::Setup {
      self.state = DspState::Running;
    }

    assert_eq!(self.state, DspState::Running);

    // poll(2), not select(2): FD_SET writes out of bounds past FD_SETSIZE
    // (1024) fds, which a busy daemon can reach; poll also triggers the
    // capture channel just like select/read do (dsp_poll -> chn_poll)
    let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLIN, revents: 0 };
    let n = unsafe { libc::poll(&mut pfd, 1, timeout_ms as i32) };
    // poll force-starts a trigger-suspended channel but leaves NOTRIGGER
    // set, which would keep the channel from ever auto-restarting; clear it
    if self.needs_trigger {
      self.needs_trigger = false;
      let _ = set_trigger(self.fd, PCM_ENABLE_INPUT);
    }
    n > 0 && (pfd.revents & libc::POLLIN) != 0
  }

  pub fn ispace_in_bytes(&mut self) -> c_int {
    assert_eq!(self.state, DspState::Running);
    let mut info = std::mem::MaybeUninit::<audio_buf_info>::uninit();
    let err = unsafe { libc::ioctl(self.fd, SNDCTL_DSP_GETISPACE, info.as_mut_ptr()) };
    if err != -1 {
      unsafe { info.assume_init().bytes }
    } else {
      0
    }
  }

  pub fn overruns(&self) -> u32 {
    assert_eq!(self.state, DspState::Running);
    get_error(self.fd).rec_overruns.max(0) as u32
  }
}

impl Drop for Dsp {

  fn drop(&mut self) {
    if !self.is_closed() {
      self.close();
    }
  }
}

pub struct DspWriter {
  pub path: String,
  fd:      c_int,
  state:   DspState,
  needs_trigger: bool, // trigger-suspended: writes buffer until armed
  #[cfg(debug_assertions)]
  prev_ns: u64
}

static ZEROES: [u8; CHN_2NDBUFMAXSIZE] = [0u8; CHN_2NDBUFMAXSIZE];

impl DspWriter {

  pub fn new(path: &str) -> Self {
    Self {
      path:    path.to_string(),
      fd:      -1,
      state:   DspState::Closed,
      needs_trigger: false,
      #[cfg(debug_assertions)]
      prev_ns: 0
    }
  }

  pub fn is_closed(&self) -> bool {
    self.state == DspState::Closed
  }

  pub fn is_running(&self) -> bool {
    self.state == DspState::Running
  }

  pub fn open(&mut self) -> Result<(), Errno> {
    assert_eq!(self.state, DspState::Closed);
    let path = CString::new(self.path.clone()).unwrap();
    let fd   = unsafe { libc::open(path.as_ptr(), libc::O_WRONLY | libc::O_NONBLOCK) };
    if fd == -1 {
      return Err(Errno::last());
    }
    self.fd    = fd;
    self.state = DspState::Setup;
    Ok(())
  }

  pub fn close(&mut self) {
    assert_ne!(self.state, DspState::Closed);
    // discard the queued buffer so close() doesn't block draining it
    unsafe { libc::ioctl(self.fd, SNDCTL_DSP_HALT); }
    unsafe { libc::close(self.fd) };
    self.fd    = -1;
    self.state = DspState::Closed;
    self.needs_trigger = false;
  }

  // Stop the channel but keep the fd: SETTRIGGER(0) aborts, resets the ring
  // and sets NOTRIGGER (writes only buffer until armed), and clears
  // TRIGGERED so the next prime's SETFRAGMENT is legal again; write() and
  // write_zeroes() arm the channel once the prefill is buffered. false =
  // driver refused; the caller falls back to closing.
  pub fn suspend(&mut self) -> bool {
    if self.state != DspState::Running {
      return true; // nothing runs; already primable
    }
    if !set_trigger(self.fd, 0) {
      return false;
    }
    self.state = DspState::Setup;
    self.needs_trigger = true;
    true
  }

  // start a trigger-suspended channel with whatever is buffered
  fn arm(&mut self) {
    if self.needs_trigger {
      self.needs_trigger = false;
      if !set_trigger(self.fd, PCM_ENABLE_OUTPUT) {
        eprintln!("{}: SETTRIGGER(OUTPUT) failed after a trigger suspend", self.path);
      }
    }
  }

  pub fn configure(&mut self, format: u32, channels: u32, rate: u32) -> Result<(), Errno> {
    assert_eq!(self.state, DspState::Setup);
    set_value(self.fd, SNDCTL_DSP_SETFMT,   format,   0)?;
    set_value(self.fd, SNDCTL_DSP_CHANNELS, channels, 0)?;
    set_value(self.fd, SNDCTL_DSP_SPEED,    rate,     feeder_rate_round())
  }

  /// Request a `len`-byte output buffer and return the size the device granted.
  /// FreeBSD clamps the fragment count, so the grant can be much smaller than
  /// requested; size the target delay to the return value, not `len`.
  pub fn set_buffer_size(&mut self, len: u32) -> u32 {
    assert_eq!(self.state, DspState::Setup);
    // the fragment count field is 16 bits; an extreme oss.delay x quantum
    // request must clamp, not truncate
    set_fragment(self.fd, len.div_ceil(1024).min(u16::MAX as u32) as u16, 10);
    // nothing's written yet, so GETOSPACE reports the granted buffer size
    let granted = ospace_in_bytes(self.fd);
    if granted > 0 { granted as u32 } else { len }
  }

  pub unsafe fn write(&mut self, buf: *const c_void, count: u32) -> ssize_t {
    let n = self.write_buffered(buf, count);
    // a trigger-suspended channel starts once real data is buffered
    self.arm();
    n
  }

  unsafe fn write_buffered(&mut self, buf: *const c_void, count: u32) -> ssize_t {
    if self.state == DspState::Setup {
      self.state = DspState::Running;
    }
    assert_eq!(self.state, DspState::Running);

    #[cfg(debug_assertions)]
    let space = ospace_in_bytes(self.fd) as usize;
    #[cfg(debug_assertions)]
    let delay = odelay(self.fd);

    let nbytes = libc::write(self.fd, buf, count as size_t);

    #[cfg(debug_assertions)]
    {
      let now         = crate::utils::now_ns_libc();
      let space_after = ospace_in_bytes(self.fd) as usize;
      let delay_after = odelay(self.fd);
      eprintln!("{}: {:9} @ {}, count = {:5}, ospace = {:5} -> {:5}, odelay = {:5} -> {:5}",
        self.path, now - self.prev_ns, now, count, space, space_after, delay, delay_after);
      self.prev_ns = now;
    }

    nbytes
  }

  pub fn write_zeroes(&mut self, mut count: u32) {
    // even a zero-length prime must leave the writer Running: callers assume
    // the space/underrun ioctls are usable after priming
    if self.state == DspState::Setup {
      self.state = DspState::Running;
    }
    // chunk from ZEROES (`count` can exceed its len). The fd is O_NONBLOCK, so a
    // short write or EAGAIN is normal; prime best-effort rather than asserting and
    // panicking out of the `extern "C"` callback (which aborts the process).
    while count > 0 {
      let chunk  = count.min(ZEROES.len() as u32);
      let nbytes = unsafe { self.write_buffered(ZEROES.as_ptr().cast(), chunk) };
      if nbytes < 0 {
        let errno = Errno::last();
        if errno != Errno::EAGAIN { // EAGAIN is just a full buffer; surface anything else
          eprintln!("{}: write_zeroes: {}", self.path, errno);
        }
        break;
      }
      if nbytes == 0 {
        break;
      }
      count -= nbytes as u32;
    }
  }

  pub fn odelay(&self) -> u32 {
    assert_eq!(self.state, DspState::Running);
    odelay(self.fd).max(0) as u32
  }

  /// The fragment size the driver actually granted (may differ from what
  /// SETFRAGMENT asked for; some drivers force a fixed period).
  pub fn blocksize(&self) -> u32 {
    blocksize(self.fd).max(0) as u32
  }

  pub fn underruns(&self) -> u32 {
    assert_eq!(self.state, DspState::Running);
    get_error(self.fd).play_underruns.max(0) as u32
  }

}

impl Drop for DspWriter {

  fn drop(&mut self) {
    if !self.is_closed() {
      self.close();
    }
  }
}

use std::fs::read_to_string;

pub fn read_sndstat() -> Result<Vec<u32>, Errno> {
  let mut result = vec![];
  match read_to_string("/dev/sndstat") {
    Ok(str) =>
      for line in str.lines() {
        if line.starts_with("pcm") {
          if let Some(separator_index) = line.find(':') {
            if let Ok(index) = line[3..separator_index].parse::<u32>() {
              result.push(index);
            }
          }
        }
      },
    Err(err) => {
      return Err(Errno::from_raw(err.raw_os_error().unwrap_or(libc::EINVAL)));
    }
  }
  Ok(result)
}

#[derive(Debug)]
pub struct PcmDevice {
  pub index:    u32,
  pub desc:     String,
  pub location: String,
  pub play:     bool,
  pub rec:      bool
}

pub fn read_pcm_device_description(sysctl: &mut crate::utils::SysctlReader, index: u32) -> Option<String> {

  let parent = sysctl.read_string(format!("dev.pcm.{}.%parent", index), 1024).ok()?; // the device can detach mid-enumeration
  if let Some(str) = parent.strip_prefix("uaudio") {
    if let Ok(idx) = str.parse::<u32>() {
      if let Ok(desc) = sysctl.read_string(format!("dev.uaudio.{}.%desc", idx), 1024) {
        // let's get rid of ", class %d/%d, rev %x.%02x/%x.%02x, addr %d" suffix
        static RE: std::sync::OnceLock<regex::Regex> = std::sync::OnceLock::new();
        let re = RE.get_or_init(|| regex::Regex::new(r"^(.*?), class \d+/\d+, rev [^\s]+, addr \d$").unwrap());
        if let Some(groups) = re.captures(&desc) {
          if let Some(str) = groups.get(1) {
            return Some(str.as_str().to_string());
          }
        } else {
          return Some(desc);
        }
      }
    }
  }

  sysctl.read_string(format!("dev.pcm.{}.%desc", index), 1024).ok()
}

pub fn group_pcm_devices_by_parent(indexes: &[u32]) -> BTreeMap<String, Vec<u32>> {
  let mut sysctl = crate::utils::SysctlReader::new();
  let mut indexes_by_parent: BTreeMap<String, Vec<u32>> = BTreeMap::new();
  for index in indexes {
    if let Ok(parent) = sysctl.read_string(format!("dev.pcm.{}.%parent", index), 1024) {
      let values = indexes_by_parent.entry(parent).or_default();
      values.push(*index);
    }
  }
  indexes_by_parent
}

pub fn list_pcm_devices(indexes: &[u32]) -> Vec<PcmDevice> {

  let mut result = Vec::with_capacity(indexes.len());
  let mut sysctl = crate::utils::SysctlReader::new();

  for index in indexes {
    if let Some(desc) = read_pcm_device_description(&mut sysctl, *index) {
      if let Ok(location) = sysctl.read_string(format!("dev.pcm.{}.%location", index), 1024) {
        // dev.pcm.N.mode reports direction support from the channel counts
        // (1 = mixer, 2 = play, 4 = rec); the vchanformat sysctls previously
        // used here return ENODEV with vchans disabled - i.e. bitperfect
        // devices vanished. Fall back to vchanformat for pre-13.1 kernels.
        let (play, rec) = match sysctl.read_u32(format!("dev.pcm.{}.mode", index)) {
          Ok(mode) => (mode & 2 != 0, mode & 4 != 0),
          Err(_) => (
            sysctl.read_string(format!("dev.pcm.{}.play.vchanformat", index), 1024).is_ok(),
            sysctl.read_string(format!("dev.pcm.{}.rec.vchanformat",  index), 1024).is_ok()
          )
        };
        result.push(PcmDevice { index: *index, desc, location, play, rec });
      }
    }
  }

  result
}
