use std::os::raw::c_int;

use libspa::sys::*;

use crate::node::{Direction, ParamBuild, State, MAX_PORTS};

// several State fields are per-port in disguise (rate_match, active_buffers,
// the single PortInfo); fix those before raising this
const _: () = assert!(MAX_PORTS == 1);
const EMPTY_CYCLE: isize = -1; // no data queued this cycle (scheduling jitter)

pub(crate) enum SourceDir {}

// direction-specific State fields (State.ext)
#[derive(Default)]
pub(crate) struct SourceExt {
  pub active_buffers: usize
}

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SourcePortExt {
  pub primed: bool
}

#[derive(Debug, Clone)]
pub struct PortConfig {
  pub format:    libspa::param::audio::AudioFormat,
  pub rate:      u32,
  pub channels:  u32,
  pub positions: Vec<u32>, // the negotiated channel positions, replayed in the Format readback
  pub flags:     u32,
  pub stride:    u32
}

impl PortConfig {

  fn oss_format(&self) -> u32 {
    match self.format {
      libspa::param::audio::AudioFormat::S32LE => crate::sound::AFMT_S32_LE,
      libspa::param::audio::AudioFormat::S32BE => crate::sound::AFMT_S32_BE,
      libspa::param::audio::AudioFormat::S16LE => crate::sound::AFMT_S16_LE,
      libspa::param::audio::AudioFormat::S16BE => crate::sound::AFMT_S16_BE,
      _ => unreachable!() // rejected at negotiation
    }
  }
}

impl crate::node::ConfigOps for PortConfig {
  fn oss_format(&self) -> u32 {
    PortConfig::oss_format(self)
  }
  fn rate(&self) -> u32        { self.rate }
  fn channels(&self) -> u32    { self.channels }
  fn stride(&self) -> u32      { self.stride }
  fn format_raw(&self) -> u32  { self.format.0 }
  fn flags(&self) -> u32       { self.flags }
  fn positions(&self) -> &[u32] { &self.positions }
}

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time). The pre-read fill level
// here and process()'s post-drain accounting see the same signal: we drain
// the ring every cycle, so what's queued is one period's accumulation.
unsafe fn timeout_servo(state: &mut State<SourceDir>, nsec: u64, rate: u32) -> (f64, i64) {
  let mut corr:  f64 = 1.0;
  let mut delay: i64 = 0;
  for port in &mut state.ports {
    let Some(cfg) = port.config.as_ref() else { continue };
    let stride      = cfg.stride.max(1);
    let device_rate = cfg.rate.max(1);
    if !port.dsp.is_running() || !port.ext.primed || port.setup_period == 0 || port.resetup_pending {
      continue;
    }

    let queued = port.dsp.ispace_in_bytes().max(0) as u32;
    // device frames scale to the graph rate; the resampler queue is already
    // graph-side (matching the sink's publication)
    let resamp = if state.rate_match.is_null() { 0 } else { (*state.rate_match).delay as i64 };
    delay = (queued / stride) as i64 * rate as i64 / device_rate as i64 + resamp;

    // capture error is inverted vs the sink: a slow device queues less than a
    // period; clamp so wakeup jitter can't wind up the integrator
    let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
    let err = (port.setup_period as f64 - queued as f64).clamp(-max_err, max_err);
    corr = port.dll.update(err);
    port.bw_adapt.update(&mut port.dll, err, stride, port.setup_blocksize,
      nsec, port.setup_period, device_rate * stride);

    // a diverged servo must not wedge the graph clock
    if !(0.5..=2.0).contains(&corr) {
      crate::warn!(state.log, "capture DLL diverged (corr {}); relocking", corr);
      port.dll.init();
      port.bw_adapt.reset();
      corr = 1.0;
    }

    #[cfg(debug_assertions)]
    eprintln!("capture: corr = {}, queued = {}", corr, queued);
  }
  (corr, delay)
}

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(dsp: &mut crate::sound::Dsp, config: &PortConfig, fragment: u32, log: &crate::spa::Log) -> c_int {
  // a busy or vanished device must fail negotiation, not abort
  if let Err(err) = dsp.open() {
    crate::warn!(log, "dsp open: {}", err);
    return -(err as c_int);
  }
  // ditto for a device that won't take the format exactly
  if let Err(err) = dsp.configure(config.oss_format(), config.channels, config.rate) {
    crate::warn!(log, "device rejected {:?}: {}", config, err);
    dsp.close();
    return -(err as c_int);
  }
  dsp.set_small_fragments(fragment, 65536); // normalized oss.fragment (0 = 1 KiB default)
  0
}

unsafe fn process_ports(state: &mut State<SourceDir>) -> c_int {

  let mut result = SPA_STATUS_OK as i32;
  let state_ptr: *mut State<SourceDir> = state;

  for (port_idx, port) in state.ports.iter_mut().enumerate() {

    if port.config.is_none() {
      continue;
    }

    if port.buffers.is_empty() || port.io.is_null() {
      continue; // not (fully) negotiated yet
    }

    if port.resetup_pending {
      continue; // the main thread is rebuilding the device
    }

    if port.dsp.is_closed() {
      // Suspend closed the device but the host restarted without a fresh
      // format; rebuild off-loop instead of tripping the dsp state asserts
      port.resetup_pending = state.main_loop.as_ref().is_some_and(|main_loop|
        crate::utils::invoke_on_loop(main_loop, state_ptr, move |state| crate::node::resetup_task(state, port_idx)));
      continue;
    }

    if (*port.io).status == SPA_STATUS_HAVE_DATA as i32 {
      // a pending buffer the peer hasn't consumed yet: report HAVE_DATA, or
      // the adapter treats the cycle as empty (alsa-pcm-source.c does this)
      result |= SPA_STATUS_HAVE_DATA as i32;
      continue;
    }
    if (*port.io).status != SPA_STATUS_OK as i32 && (*port.io).status != SPA_STATUS_NEED_DATA as i32 {
      continue;
    }

    let buffer_id = if (*port.io).buffer_id == -1i32 as u32 {
      // hand out the next never-used buffer; the host returns ids after that
      let idx = state.ext.active_buffers;
      state.ext.active_buffers += 1;
      idx as u32
    } else {
      (*port.io).buffer_id
    };

    // buffer_id (or our fallback index) and n_datas come from outside. Validate
    // them instead of asserting; a panic here aborts the process across extern "C".
    let buffer = match port.buffers.get(buffer_id as usize).copied().and_then(|b| b.as_ref()) {
      Some(b) if b.n_datas == 1 => b, // we fill the block directly, so need exactly one
      _ => {
        crate::warn!(state.log, "unusable buffer (id {}); skipping", buffer_id);
        continue;
      }
    };

    // we read straight into the block, so require a MemPtr with data, chunk and
    // maxsize all valid. as_ref() (not offset(0)) handles a null datas pointer.
    let data_0 = match buffer.datas.as_ref() {
      Some(d) if d.type_ == SPA_DATA_MemPtr && !d.data.is_null() && !d.chunk.is_null() && d.maxsize > 0 => d,
      _ => {
        crate::warn!(state.log, "buffer data is not a usable MemPtr block; skipping");
        continue;
      }
    };

    let stride = port.config.as_ref().unwrap().stride.max(1);
    let rate   = port.config.as_ref().unwrap().rate;
    let matching = state.following && !crate::utils::same_clock(state.position, &state.clock_name);

    let mut corr: f64 = 1.0; // DLL rate correction for the follower rate match

    // one period in device bytes (0 while position is absent)
    let mut period_in_bytes = 0u32;
    if !state.position.is_null() {
      let driver_clock = (*state.position).clock;
      if driver_clock.target_rate.denom > 0 {
        period_in_bytes = crate::utils::device_period_bytes(
          driver_clock.target_duration, rate, driver_clock.target_rate.denom, stride);
      }
    }

    // a period change re-tunes the servo; capture needs no reopen (its ring
    // isn't SETFRAGMENT-sized), but the DLL gain and target change - ALSA
    // compensates the error by the threshold delta, we relock fast instead
    if port.ext.primed && port.setup_period != 0 && period_in_bytes != 0 && period_in_bytes != port.setup_period {
      port.setup_period = period_in_bytes;
      port.dll.init();
      port.bw_adapt.reset();
    }

    let freewheel = !state.position.is_null() &&
      (*state.position).clock.flags & SPA_IO_CLOCK_FLAG_FREEWHEEL != 0;

    let nbytes = if freewheel && period_in_bytes > 0 {
      // freewheeling: hand out silence without touching the device (ALSA
      // skips its reads); the ring overflows meanwhile and the overrun
      // recovery re-primes when realtime resumes
      let len = period_in_bytes.min(data_0.maxsize);
      std::ptr::write_bytes(data_0.data.cast::<u8>(), 0, len as usize);
      len as isize
    } else if !port.ext.primed && period_in_bytes > 0 {
      // Capture analogue of the sink's zero priming: trigger the device,
      // discard any backlog so the fill level starts out known, and hand the
      // graph one period of silence while the ring fills. Don't wait for real
      // data: an empty first cycle reads as a missed deadline to the graph.
      // Re-apply the fragment layout while the channel is in setup (legal
      // after a trigger suspend too, so live oss.fragment changes reach a
      // suspended source). The capture fragment is capped at the period:
      // queued readings move in fragment steps, and a fragment far above
      // the period makes the servo target unreachable - the error pegs at
      // the clamp and the integrator ramps. The ring scales with the
      // period so large quanta keep some overrun slack.
      if !port.dsp.is_running() {
        let m    = period_in_bytes.max(1024);
        let cap  = 1u32 << (31 - m.leading_zeros());
        let frag = if state.oss_fragment == 0 { 1024 } else { state.oss_fragment.min(cap) };
        port.dsp.set_small_fragments(frag, period_in_bytes.saturating_mul(4));
      }
      if port.dsp.ready_for_reading(0) {
        let mut backlog = port.dsp.ispace_in_bytes().max(0) as u32;
        while backlog > 0 {
          let chunk = backlog.min(data_0.maxsize);
          let n = port.dsp.read(data_0.data, chunk as usize);
          if n <= 0 {
            break;
          }
          backlog -= n as u32;
        }
      }
      port.ext.primed      = true;
      port.setup_period    = period_in_bytes;
      port.setup_blocksize = port.dsp.blocksize();
      port.dll.init();
      port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle

      let len = period_in_bytes.min(data_0.maxsize);
      std::ptr::write_bytes(data_0.data.cast::<u8>(), 0, len as usize);
      len as isize
    } else if !port.dsp.is_running() {
      // un-primed and no usable position yet (the prime branch needs a
      // period): the device is still in setup, where the space ioctls assert
      EMPTY_CYCLE
    } else {
      // Gate on the queued byte count, not poll: the kernel's poll trigger
      // is one full fragment, which can exceed a small graph period - every
      // read (and the servo error) would then be biased by a fragment. The
      // priming pass already triggered the channel; GETISPACE doesn't need
      // the trigger.
      let queued = port.dsp.ispace_in_bytes().max(0) as u32;
      if queued == 0 { crate::source::EMPTY_CYCLE } else {

      // when driving, the servo runs in on_timeout where the clock is
      // published; here the DLL only serves rate matching as a follower on a
      // foreign clock (a same-device follower has nothing to correct)
      if matching && period_in_bytes > 0 && port.setup_period != 0 {
        let now = crate::utils::now_ns(&state.data_system);
        if !port.was_matching {
          // matching just engaged; relock rather than apply stale state
          port.dll.init();
          port.bw_adapt.reset();
        }
        // capture error is inverted vs the sink: a slow device queues less
        let err_raw = period_in_bytes as f64 - queued as f64;
        if err_raw.abs() > port.setup_period as f64 {
          // fill snap (see the sink): a level error past one period would
          // wind the integrator against the +/-1% clamp; the bounded read
          // above drains genuine backlog, so just relock here
          port.dll.init();
          port.bw_adapt.reset();
        } else {
          let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
          let err = err_raw.clamp(-max_err, max_err);
          corr = port.dll.update(err);
          port.bw_adapt.update(&mut port.dll, err, stride, port.setup_blocksize,
            now, port.setup_period, rate * stride);
        }

        #[cfg(debug_assertions)]
        eprintln!("capture: corr = {}, err = {}", corr, err_raw);
      }

      // Bounded read: one period, plus only the backlog beyond two periods
      // (genuine catch-up). Draining everything each cycle turns consumer
      // backpressure into a permanent extra period of latency (an oversized
      // chunk holds io.status HAVE_DATA, we skip the device next cycle, it
      // queues 2 periods, repeat) and pollutes the servo error.
      let want = if port.setup_period != 0 {
        // catch-up beyond 1.5 periods; the servo handles the rest without a
        // pegged error (a 2-period threshold stranded a full period that
        // only the 1% actuator could drain)
        port.setup_period.saturating_add(
          queued.saturating_sub(port.setup_period.saturating_mul(3) / 2))
      } else {
        queued
      };
      let ispace = want.min(queued).min(data_0.maxsize);
      #[cfg(debug_assertions)]
      crate::trace!(state.log, "ispace: {}", ispace);
      port.dsp.read(data_0.data, ispace as usize)
      }
    };

    // Rate-match only as a follower on a foreign clock: when driving, the
    // timer steering applies the correction, and a same-device follower ticks
    // from our clock so there is nothing to match (ALSA gates on the clock
    // name the same way).
    port.was_matching = matching;
    // an empty cycle didn't run the servo; keep the previous correction
    if nbytes >= 0 && !state.rate_match.is_null() {
      if matching {
        (*state.rate_match).flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
        (*state.rate_match).rate   = (1.0 / corr).clamp(0.99, 1.01);
      } else {
        (*state.rate_match).flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
        (*state.rate_match).rate   = 1.0;
      }
    }

    // Report overruns to the host (pw-top's xrun counter); the length isn't
    // known, so pass 0 delay. The freewheel branch never triggers the device
    // (it may still be in setup), and while freewheeling the ring overruns by
    // design - the exit path re-primes, so don't flood the counter meanwhile.
    let overrun_count = if port.dsp.is_running() && !freewheel { port.dsp.overruns() } else { 0 };
    if overrun_count > 0 {
      let now = crate::utils::now_ns(&state.data_system);
      if let Some(suppressed) = port.warn_limit.check(now) {
        crate::warn!(state.log, "OSS reported {:3} overruns @ {} (+{} warnings suppressed)", overrun_count, now, suppressed);
      }
      let node_callbacks = state.callbacks.funcs.cast::<spa_node_callbacks>().as_ref();
      if let Some(xrun_fun) = node_callbacks.and_then(|c| c.xrun) {
        xrun_fun(state.callbacks.data, now / 1000, 0, std::ptr::null_mut());
      }

      // recover like the sink's underrun path: re-enter priming next cycle,
      // which drains the backlog and relocks the DLL - otherwise the
      // un-drained backlog becomes permanent capture latency while the
      // integrator winds up against an error the reads can't remove
      port.ext.primed = false;
      port.bw_adapt.reset();
      port.dll.init();
    }

    if nbytes != -1 {
      #[cfg(debug_assertions)]
      if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
        crate::trace!(state.log, "nbytes: {}", nbytes);
        spa_debug_mem(0, data_0.data, 16.min(nbytes) as usize);
      }

      (*data_0.chunk).offset = 0;
      (*data_0.chunk).size   = nbytes as u32;
      (*data_0.chunk).stride = port.config.as_ref().unwrap().stride as i32;
      (*data_0.chunk).flags  = 0;

      (*port.io).buffer_id   = buffer_id;
      (*port.io).status      = SPA_STATUS_HAVE_DATA as i32;

      result |= SPA_STATUS_HAVE_DATA as i32;
    } else {
      (*port.io).buffer_id   = buffer_id; // -1i32 as u32;
      (*port.io).status      = SPA_STATUS_OK as i32;
    }
  }

  result
}

impl Direction for SourceDir {

  const DIRECTION: spa_direction = SPA_DIRECTION_OUTPUT;
  const PLAYBACK: bool = false;
  const MEDIA_CLASS: &'static str = "Audio/Source";
  // a capture driver signals HAVE_DATA (alsa-pcm.c capture_ready); the
  // NEED_DATA form is for playback drivers
  const READY_STATUS: i32 = SPA_STATUS_HAVE_DATA as i32;
  const CMD_WARN_PREFIX: &'static str = "oss-source: ";

  type Device  = crate::sound::Dsp;
  type Config  = PortConfig;
  type Ext     = SourceExt;
  type PortExt = SourcePortExt;

  fn info_item(_ext: &mut SourceExt, _key: &str, _value: &str) {}

  fn ext_ready(_ext: &mut SourceExt) {}

  unsafe fn build_node_param(state: &mut State<SourceDir>, b: &mut libspa::pod::builder::Builder, id: u32, index: u32) -> ParamBuild {
    #[allow(non_upper_case_globals)]
    match (id, index) {
      (SPA_PARAM_PropInfo, 0)       => crate::utils::build_latency_offset_prop_info(b).unwrap(),
      (SPA_PARAM_PropInfo, 1)       => crate::utils::build_params_prop_info(b, crate::keys::OSS_FRAGMENT,
        "OSS fragment size (bytes, power of two, 0 = automatic)", state.oss_fragment, 16384).unwrap(),
      (SPA_PARAM_PropInfo, _)       => return ParamBuild::Exhausted,
      (SPA_PARAM_Props, 0)          => crate::utils::build_latency_offset_props(b, state.process_latency.ns,
        &[(crate::keys::OSS_FRAGMENT, state.oss_fragment)]).unwrap(),
      (SPA_PARAM_Props, _)          => return ParamBuild::Exhausted,
      (SPA_PARAM_ProcessLatency, 0) => crate::utils::build_process_latency_info(b, &state.process_latency).unwrap(),
      (SPA_PARAM_ProcessLatency, _) => return ParamBuild::Exhausted,
      _ => return ParamBuild::Unknown
    };
    ParamBuild::Built
  }

  // a NULL Props pod resets the props to their defaults and re-applies them
  unsafe fn reset_props(state: &mut State<SourceDir>) -> c_int {
    let res = crate::node::store_and_rebuild(state, |state| {
      state.oss_fragment = state.oss_fragment_default;
    });
    if res != 0 {
      return res;
    }
    crate::node::handle_process_latency(state, crate::utils::process_latency_default());
    0
  }

  unsafe fn set_props_params(state: &mut State<SourceDir>, value: &libspa::pod::Value) -> c_int {
    use libspa::pod::Value;
    match value {
      Value::Struct(values) if values.len() % 2 == 0 => {
        for kv in values.chunks(2) {
          match (&kv[0], &kv[1]) {
            // pw-cli set-param <object-id> Props '{ "params": ["oss.fragment", 4096]}'
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_FRAGMENT && *x >= 0 => {
              // stored normalized, so the Props readback reports the
              // effective (rounded/clamped) value, not the raw request
              let new_fragment = crate::node::normalize_fragment(*x as u32);
              if new_fragment != state.oss_fragment {
                // unchanged echoes must not rebuild a running device
                let res = crate::node::apply_props_param(state, move |state| state.oss_fragment = new_fragment);
                if res != 0 {
                  return res;
                }
              }
            },
            _ => ()
          }
        }
      }
      _ => ()
    }
    0
  }

  unsafe fn parse_config(state: &mut State<SourceDir>, raw: &spa_audio_info_raw) -> Result<PortConfig, c_int> {
    let format = libspa::param::audio::AudioFormat(raw.format);

    // only formats from our EnumFormat are expected; reject the rest
    let (oss_format, bytes_per_sample) = match format {
      libspa::param::audio::AudioFormat::S32LE => (crate::sound::AFMT_S32_LE, 4),
      libspa::param::audio::AudioFormat::S32BE => (crate::sound::AFMT_S32_BE, 4),
      libspa::param::audio::AudioFormat::S16LE => (crate::sound::AFMT_S16_LE, 2),
      libspa::param::audio::AudioFormat::S16BE => (crate::sound::AFMT_S16_BE, 2),
      _ => {
        crate::warn!(state.log, "rejecting unsupported format {:?}", format);
        return Err(-libc::ENOTSUP);
      }
    };

    let config = PortConfig {
      format,
      rate:      raw.rate,
      channels:  raw.channels,
      positions: raw.position[..raw.channels as usize].to_vec(),
      flags:     raw.flags,
      stride:    bytes_per_sample * raw.channels // bytes per interleaved frame
    };

    crate::debug!(state.log, "reconfiguring with {:?}", config);

    let _ = oss_format;
    Ok(config)
  }

  fn try_open_configure(dsp: &mut crate::sound::Dsp, config: &PortConfig, fragment: u32, log: &crate::spa::Log) -> c_int {
    try_open_configure(dsp, config, fragment, log)
  }

  unsafe fn on_device_swapped(state: &mut State<SourceDir>, port_idx: usize) {
    let port = &mut state.ports[port_idx];
    port.dll.init(); // fresh device, fresh servo
    port.ext.primed = false;
    state.ext.active_buffers = 0;
  }

  unsafe fn on_buffers_swapped(state: &mut State<SourceDir>) {
    state.ext.active_buffers = 0;
  }

  unsafe fn on_start_loop(state: &mut State<SourceDir>) {
    // the device kept capturing across a Pause; re-prime so the first
    // cycles deliver fresh audio at a known fill, not the paused backlog
    for port in &mut state.ports {
      port.ext.primed = false;
      port.dll.init();
      port.bw_adapt.reset();
    }
  }

  unsafe fn on_suspend_loop(state: &mut State<SourceDir>) {
    for port in &mut state.ports {
      port.ext.primed = false; // resume re-primes for a known fill
    }
  }

  unsafe fn on_role_flip(_state: &mut State<SourceDir>) {}

  // data loop only
  unsafe fn update_timers(state: &mut State<SourceDir>) {

    #[cfg(debug_assertions)]
    crate::trace!(state.log, "update_timers");

    let mut now = timespec { tv_sec: 0, tv_nsec: 0 };
    let err = state.data_system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now);
    assert!(err >= 0);

    state.next_time = (now.tv_sec * SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64;

    if state.started && !state.following && !state.position.is_null() {
      #[cfg(debug_assertions)]
      crate::trace!(state.log, "next time {}", state.next_time);
      crate::node::set_timeout(state, state.next_time);
    } else {
      #[cfg(debug_assertions)]
      crate::trace!(state.log, "next time {}", 0);
      crate::node::set_timeout(state, 0);
    }
  }

  unsafe fn debug_cycle(_state: &State<SourceDir>, _now: u64, _nsec: u64) {}

  unsafe fn timeout_servo(state: &mut State<SourceDir>, nsec: u64, rate: u32) -> (f64, i64) {
    timeout_servo(state, nsec, rate)
  }

  unsafe fn process_ports(state: &mut State<SourceDir>) -> c_int {
    process_ports(state)
  }
}

const OSS_SOURCE_FACTORY_INFO: spa_dict = spa_dict {
  flags:   0,
  n_items: 0,
  items:   std::ptr::null()
};

pub const OSS_SOURCE_FACTORY: spa_handle_factory = spa_handle_factory {
  version:             SPA_VERSION_HANDLE_FACTORY,
  name:                c"freebsd-oss.source".as_ptr(),
  info:                &OSS_SOURCE_FACTORY_INFO,
  get_size:            Some(crate::node::get_size::<SourceDir>),
  init:                Some(crate::node::init::<SourceDir>),
  enum_interface_info: Some(crate::node::enum_interface_info)
};
