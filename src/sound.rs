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
const SNDCTL_DSP_LOW_WATER:   c_ulong = nix::request_code_write!    (b'P', 34, std::mem::size_of::<c_int>());
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

#[derive(Debug, Clone, PartialEq)]
pub struct DspCaps {
  pub formats:        u32, // AFMT_* mask
  pub min_channels:   u32,
  pub max_channels:   u32,
  pub min_rate:       u32,
  pub max_rate:       u32,
  pub preferred_rate: Option<u32>, // the parent's vchan mix rate, when known
  pub rates:          Vec<u32>, // discrete native rates (exclusive devices); empty = the range
  pub convertless:    bool // bitperfect: no feeder, only native values negotiate
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
      preferred_rate: None,
      rates:          vec![],
      convertless:    false
    }
  }

  // Lenient admission check for a host-requested format: rejects only clear
  // violations of the advertised caps (staleness is handled by the caller's
  // configure backstop). Rates within the feeder snap window pass.
  pub fn admits(&self, oss_format: u32, channels: u32, rate: u32) -> bool {
    // format only matters where no feeder converts (bitperfect): a
    // non-native SETFMT there snaps and fails the strict grant check -
    // after the EBUSY retire already killed the working fd
    if self.convertless && self.formats & oss_format == 0 {
      return false;
    }
    if channels < self.min_channels || channels > self.max_channels {
      return false;
    }
    if !self.rates.is_empty() {
      return self.rates.contains(&rate);
    }
    let slack = feeder_rate_round();
    rate.saturating_add(slack) >= self.min_rate && rate <= self.max_rate.saturating_add(slack)
  }
}

// Ask the device what it actually supports. Two sources, merged:
// - empirical SETCHANNELS/SPEED probes at the extremes (OSS grants the nearest
//   supported value) - but the kernel clamps channel requests to SND_CHN_MAX
//   and bitperfect devices reject unsupported values instead of snapping;
// - SNDCTL_AUDIOINFO, which reports the real hardware limits (dsp.c
//   aggregates chn_getcaps over the device), covering both gaps above.
// Uses a transient open; the caller falls back if the device is busy.
#[repr(C)]
struct SndstiocNvArg {
  nbytes: usize,
  buf:    *mut c_void
}

const SNDSTIOC_REFRESH_DEVS: c_ulong = nix::request_code_none!(b'D', 100);
const SNDSTIOC_GET_DEVS:     c_ulong = nix::request_code_readwrite!(b'D', 101, std::mem::size_of::<SndstiocNvArg>());

// native per-direction device info from the sndstat(4) nvlist interface -
// no dsp open, so an exclusive device's only channel stays unclaimed
pub struct SndstatDspInfo {
  pub formats:    u32,
  pub min_rate:   u32,
  pub max_rate:   u32,
  pub min_chn:    u32,
  pub max_chn:    u32,
  pub exclusive:  Option<bool>, // vchans off for this direction; None = can't tell
  pub vchan_rate: u32, // the parent's mix rate while vchans are on
  pub bitperfect: bool
}

// one packed snapshot of every sound device from sndstat(4)
fn sndstat_snapshot() -> Option<crate::nv::NvList> {

  let fd = unsafe { libc::open(c"/dev/sndstat".as_ptr(), libc::O_RDONLY | libc::O_CLOEXEC) };
  if fd == -1 {
    return None;
  }
  struct FdGuard(c_int);
  impl Drop for FdGuard {
    fn drop(&mut self) { unsafe { libc::close(self.0) }; }
  }
  let _guard = FdGuard(fd);

  // best-effort; GET still returns the last snapshot if refresh fails
  let _ = unsafe { libc::ioctl(fd, SNDSTIOC_REFRESH_DEVS) };

  // two-call protocol (size, then fill); the snapshot is per-open cdevpriv,
  // so it can't change between the calls (a too-small buffer would come back
  // as nbytes = 0 and unpack cleanly to None)
  let mut buf: Vec<u8> = Vec::new();
  for _ in 0..2 {
    let mut arg = SndstiocNvArg { nbytes: buf.len(), buf: buf.as_mut_ptr().cast() };
    if buf.is_empty() {
      arg.buf = std::ptr::null_mut();
    }
    if unsafe { libc::ioctl(fd, SNDSTIOC_GET_DEVS, &mut arg) } == -1 {
      return None;
    }
    if !buf.is_empty() && arg.nbytes <= buf.len() {
      buf.truncate(arg.nbytes);
      break;
    }
    buf = vec![0; arg.nbytes];
  }

  crate::nv::NvList::unpack(&buf)
}

// pcm unit numbers plus per-direction channel presence; user-registered
// devices (virtual_oss) are excluded explicitly instead of by name prefix
fn sndstat_pcm_devices() -> Option<Vec<(u32, bool, bool)>> {
  let nvl = sndstat_snapshot()?;
  let mut out = vec![];
  for dev in nvl.root().nvlist_array(c"dsps") {
    if dev.boolean(c"from_user").unwrap_or(false) {
      continue;
    }
    let Some(unit) = dev.string(c"nameunit")
      .and_then(|nu| nu.strip_prefix("pcm"))
      .and_then(|u| u.parse::<u32>().ok()) else { continue };
    out.push((unit,
      dev.number(c"pchan").unwrap_or(0) > 0,
      dev.number(c"rchan").unwrap_or(0) > 0));
  }
  Some(out)
}

pub fn sndstat_dsp_info(devnode: &str, play: bool) -> Option<SndstatDspInfo> {

  let nvl  = sndstat_snapshot()?;
  let root = nvl.root();
  for dev in root.nvlist_array(c"dsps") {
    // a user-registered device (virtual_oss) may carry any devnode string;
    // don't let it shadow a kernel one
    if dev.boolean(c"from_user").unwrap_or(false) || dev.string(c"devnode") != Some(devnode) {
      continue;
    }
    // absent for a direction with no channels
    let info = dev.nvlist(if play { c"info_play" } else { c"info_rec" })?;
    let num  = |r: crate::nv::NvRef, k: &std::ffi::CStr| r.number(k).unwrap_or(0) as u32;

    let (mut exclusive, mut vchan_rate, mut bitperfect) = (None, 0, false);
    if let Some(p) = dev.nvlist(c"provider_info") {
      // pvchan/rvchan is a NUMBER of LIVE vchans on 14.x (an idle device
      // reads 0 with vchans enabled!) and a BOOL enabled flag on 15.0+
      // (sndstat.c 0c0bb4c1401c). Only the bool and a positive count are
      // unambiguous; a zero count means "can't tell, probe".
      let key = if play { c"pvchan" } else { c"rvchan" };
      exclusive = match (p.boolean(key), p.number(key)) {
        (Some(enabled), _)       => Some(!enabled),
        (None, Some(n)) if n > 0 => Some(false),
        _                        => None
      };
      vchan_rate = num(p, if play { c"pvchanrate" } else { c"rvchanrate" });
      bitperfect = p.boolean(c"bitperfect").unwrap_or(false);
    }

    return Some(SndstatDspInfo {
      formats:    num(info, c"formats"),
      min_rate:   num(info, c"min_rate"),
      max_rate:   num(info, c"max_rate"),
      min_chn:    num(info, c"min_chn"),
      max_chn:    num(info, c"max_chn"),
      exclusive,
      vchan_rate,
      bitperfect
    });
  }
  None
}

fn caps_from_sndstat(nv: &SndstatDspInfo, rates: Vec<u32>) -> DspCaps {
  DspCaps {
    formats:        nv.formats,
    min_channels:   nv.min_chn.max(1),
    max_channels:   nv.max_chn.max(nv.min_chn).max(1),
    min_rate:       nv.min_rate.max(1),
    max_rate:       nv.max_rate.max(nv.min_rate).max(1),
    preferred_rate: None, // the native values are the preference
    rates,
    convertless:    nv.bitperfect
  }
}

// The native rate SET of an exclusive device, from a brief ENGINEINFO-only
// open. Bitperfect rates aren't a dense range: the kernel snaps the DMA to
// the nearest native rate but SNDCTL_DSP_SPEED echoes the REQUEST back for
// playback (feeder_chain keeps c->speed = target), so an in-range non-native
// rate would negotiate fine and play pitch-shifted with no diagnostics.
fn native_rates(path: &str, play: bool) -> Vec<u32> {
  let Ok(cpath) = CString::new(path) else { return vec![] };
  let mode = if play { libc::O_WRONLY } else { libc::O_RDONLY };
  let fd = unsafe { libc::open(cpath.as_ptr(), mode | libc::O_NONBLOCK) };
  if fd == -1 {
    return vec![]; // busy: the caller keeps the min..max range
  }
  let mut rates = vec![];
  unsafe {
    let mut ai = std::mem::MaybeUninit::<oss_audioinfo>::zeroed();
    (*ai.as_mut_ptr()).dev = -1; // this fd's channel
    if libc::ioctl(fd, SNDCTL_ENGINEINFO, ai.as_mut_ptr()) != -1 {
      let ai = ai.assume_init();
      for i in 0..ai.nrates.min(20) as usize {
        rates.push(ai.rates[i]);
      }
    }
    libc::close(fd);
  }
  rates.retain(|r| *r > 0);
  rates.sort_unstable();
  rates.dedup();
  rates
}

pub fn probe_caps(path: &str, play: bool) -> Option<DspCaps> {

  let native = sndstat_dsp_info(path.trim_start_matches("/dev/"), play);

  // An exclusive channel (bitperfect or vchans off) negotiates the native
  // values verbatim, and a probe open would briefly claim the only channel;
  // build the caps from sndstat without opening at all.
  if let Some(nv) = &native {
    if nv.bitperfect || nv.exclusive == Some(true) {
      let mut rates = native_rates(path, play);
      // native_rates opens the device briefly; if that failed (busy) on a
      // bitperfect device, fall back to the native EXTREMES - they are
      // themselves native, while a dense min..max range would admit
      // pitch-shifting non-native rates (playback echoes the request back)
      if nv.bitperfect && rates.is_empty() && nv.min_rate != nv.max_rate {
        rates = vec![nv.min_rate.max(1), nv.max_rate.max(nv.min_rate).max(1)];
      }
      return Some(caps_from_sndstat(nv, rates));
    }
  }


  let cpath = CString::new(path).ok()?;
  let mode  = if play { libc::O_WRONLY } else { libc::O_RDONLY };
  let fd    = unsafe { libc::open(cpath.as_ptr(), mode | libc::O_NONBLOCK) };
  if fd == -1 {
    // busy or transiently gone: the native info still beats the caller's
    // conservative stereo fallback
    return native.as_ref().map(|nv| caps_from_sndstat(nv, vec![]));
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

  // On a vchan the parent hardware mixes at its vchanrate (from the sndstat
  // nvlist); preferring it avoids a second in-kernel resample on non-48k
  // parents. Zero/absent (direct channel) just means no preference.
  let preferred_rate = native.as_ref()
    .map(|nv| nv.vchan_rate)
    .filter(|r| *r != 0 && (min_rate as u32..=max_rate as u32).contains(r));

  Some(DspCaps {
    formats:        formats as u32,
    min_channels:   min_channels as u32,
    max_channels:   max_channels as u32,
    min_rate:       min_rate as u32,
    max_rate:       max_rate as u32,
    preferred_rate,
    rates:          vec![], // the feeder converts; the range really is dense
    convertless:    false
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
  pub hw_quantum_ns: u64, // the hardware drain quantum (sndstat); 0 = fragment-accurate
  fd:    c_int,
  state: DspState,
  needs_trigger: bool // trigger-suspended: NOTRIGGER must be cleared on restart
}

impl Dsp {

  pub fn new(path: &str) -> Self {
    Self {
      path: CString::new(path).unwrap(),
      hw_quantum_ns: drain_quantum_ns(path, false),
      fd: -1,
      state: DspState::Closed,
      needs_trigger: false
    }
  }

  pub fn is_closed(&self) -> bool {
    self.state == DspState::Closed
  }

  // on direct opens the hardware blocksize is per-session state; call after
  // configure so the snapshot reflects THIS session (see drain_quantum_ns)
  pub fn refresh_hw_quantum(&mut self) {
    if let Ok(path) = self.path.to_str() {
      self.hw_quantum_ns = drain_quantum_ns(path, false);
    }
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

  // Size the capture ring into small fragments and make poll byte-accurate.
  // Small fragments set the DMA delivery granularity (the servo's measurement
  // quantization); the hw.snd.latency default can exceed a small graph
  // period. The low-water mark then decouples the poll trigger from the
  // GRANTED fragment size (chn_polltrigger fires at lw, which SETFRAGMENT
  // resets to blksz (channel.c:1980) - so the order here matters, and the
  // mark survives a trigger suspend since chn_resetbuf doesn't touch it).
  // `fragment` is the normalized oss.fragment override (0 = the 1 KiB
  // default); either way the ring keeps a 64 KiB byte budget.
  pub fn set_small_fragments(&mut self, fragment: u32, ring: u32) {
    if self.state != DspState::Setup {
      return; // triggered channels can't retune; the next re-prime will
    }
    let ring = ring.clamp(65536, CHN_2NDBUFMAXSIZE as u32);
    if fragment == 0 {
      set_fragment(self.fd, (ring >> 10).min(u16::MAX as u32) as u16, 10); // 1 KiB fragments
    } else {
      // fragment is a power of two in [64, 16384] (node.rs
      // normalize_fragment), so the selector stays inside the kernel's
      // RANGE(fragln, 4, 16) (dsp.c:1251) and the count never drops under
      // the kernel minimum of 2 (dsp.c:1256)
      let count = (ring >> fragment.trailing_zeros()).max(2u32);
      set_fragment(self.fd, count.min(u16::MAX as u32) as u16, fragment.trailing_zeros() as u16);
    }
    let mut lw: c_int = 1;
    // best-effort: without it, poll readiness is merely fragment-coarse
    let _ = unsafe { libc::ioctl(self.fd, SNDCTL_DSP_LOW_WATER, &mut lw) };
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

// frame bytes for a sound4 AFMT value (width by encoding bit, channels from
// the AFMT_CHANNEL field, sound.h:344); approximate widths are fine - the
// quantum this feeds is a floor, and overstating errs toward more margin
fn afmt_frame_bytes(format: u32) -> u32 {
  let width: u32 = if format & (AFMT_S32_LE | AFMT_S32_BE | 0x0000c000 | 0x30000000) != 0 {
    4 // S32/U32/F32
  } else if format & 0x000f0000 != 0 { // AFMT_S24/U24
    3
  } else if format & (AFMT_S16_LE | AFMT_S16_BE | 0x00000180) != 0 {
    2
  } else {
    1
  };
  let channels = ((format & 0x07f00000) >> 20).max(1); // AFMT_CHANNEL (sound.h:344)
  width * channels
}

// The device's real drain quantum, as TIME: the hardware buffer blocksize of
// the primary (non-virtual) channel for the direction, from the sndstat
// channel info. The soft fragsize GETOSPACE reports can understate it badly
// on drivers that ignore SETFRAGMENT and pull fixed transfers (uaudio:
// buffer_ms of audio per completion) - and for vchan children the parent's
// hardware cadence governs the mix pull the same way. Time-domain so a later
// rate renegotiation converts cleanly; drivers with rate-proportional blocks
// (fixed frame counts) read slightly large or small across rates, which only
// shifts a floor. 0 = unknown, use the soft fragsize alone.
pub fn drain_quantum_ns(devnode: &str, play: bool) -> u64 {
  let devnode = devnode.trim_start_matches("/dev/"); // sndstat devnodes are bare
  let want_dir = if play { 0x00020000u64 } else { 0x00010000 }; // PCM_CAP_OUTPUT/INPUT
  let mut quantum: u64 = 0;
  let Some(nvl) = sndstat_snapshot() else { return 0 };
  for dev in nvl.root().nvlist_array(c"dsps") {
    if dev.boolean(c"from_user").unwrap_or(false) || dev.string(c"devnode") != Some(devnode) {
      continue;
    }
    let Some(p) = dev.nvlist(c"provider_info") else { return 0 };
    for chan in p.nvlist_array(c"channel_info") {
      let caps = chan.number(c"caps").unwrap_or(0);
      if caps & 0x00040000 != 0 || caps & want_dir == 0 { // PCM_CAP_VIRTUAL
        continue;
      }
      let blksz  = chan.number(c"hwbuf_blksz").unwrap_or(0);
      let rate   = chan.number(c"hwbuf_rate").unwrap_or(0);
      let stride = afmt_frame_bytes(chan.number(c"hwbuf_format").unwrap_or(0) as u32) as u64;
      if blksz > 0 && rate > 0 {
        quantum = quantum.max(blksz.saturating_mul(1_000_000_000) / (rate * stride));
      }
    }
    break;
  }
  quantum
}

pub struct DspWriter {
  pub path: String,
  pub hw_quantum_ns: u64, // the hardware drain quantum (sndstat); 0 = fragment-accurate
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
      hw_quantum_ns: drain_quantum_ns(path, true), // main thread; nodes are built there
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
  /// `fragment` is the normalized oss.fragment override (0 = 1 KiB default).
  pub fn set_buffer_size(&mut self, len: u32, fragment: u32) -> u32 {
    assert_eq!(self.state, DspState::Setup);
    if fragment == 0 {
      // the fragment count field is 16 bits; an extreme oss.delay x quantum
      // request must clamp, not truncate
      set_fragment(self.fd, len.div_ceil(1024).min(u16::MAX as u32) as u16, 10);
    } else {
      // fragment is a power of two in [64, 16384] (node.rs
      // normalize_fragment), keeping the selector inside the kernel's
      // RANGE(fragln, 4, 16) (dsp.c:1251); the count clamp mirrors the
      // kernel's own bounds (min 2, total <= CHN_2NDBUFMAXSIZE, dsp.c:1256-1259)
      let count = len.div_ceil(fragment).clamp(2, CHN_2NDBUFMAXSIZE as u32 / fragment);
      set_fragment(self.fd, count as u16, fragment.trailing_zeros() as u16);
    }
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


pub fn read_sndstat() -> Result<Vec<u32>, Errno> {
  // sndstat's nvlist interface; the plugin assumes FreeBSD 14.4+
  sndstat_pcm_devices()
    .map(|devs| devs.into_iter().map(|(unit, _, _)| unit).collect())
    .ok_or(Errno::ENXIO)
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
  // Direction support from the nvlist channel counts (vchans on or off);
  // dev.pcm.N.mode (1 = mixer, 2 = play, 4 = rec) only covers a transient
  // nvlist failure.
  let chans = sndstat_pcm_devices();

  for index in indexes {
    if let Some(desc) = read_pcm_device_description(&mut sysctl, *index) {
      if let Ok(location) = sysctl.read_string(format!("dev.pcm.{}.%location", index), 1024) {
        let from_nv = chans.as_ref().and_then(|c|
          c.iter().find(|(unit, _, _)| unit == index).map(|&(_, play, rec)| (play, rec)));
        let (play, rec) = match from_nv {
          Some(dirs) => dirs,
          None => match sysctl.read_u32(format!("dev.pcm.{}.mode", index)) {
            Ok(mode) => (mode & 2 != 0, mode & 4 != 0),
            Err(_)   => (false, false)
          }
        };
        result.push(PcmDevice { index: *index, desc, location, play, rec });
      }
    }
  }

  result
}

#[cfg(test)]
mod tests {
  #[test]
  fn drain_quantum_probe() {
    for unit in [0u32, 1, 6] {
      let node = format!("/dev/dsp{}", unit); // the production string shape
      println!("{}: play {} ns, rec {} ns", node,
        super::drain_quantum_ns(&node, true), super::drain_quantum_ns(&node, false));
    }
  }
}
