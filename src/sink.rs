use std::os::raw::c_int;

use libspa::sys::*;

use crate::node::{Direction, ParamBuild, State, MAX_PORTS};

// several State fields are per-port in disguise (rate_match, the single
// PortInfo, on_timeout's last-port-wins clock delay); fix those before
// raising this
const _: () = assert!(MAX_PORTS == 1);

pub(crate) enum SinkDir {}

// direction-specific State fields (State.ext)
pub(crate) struct SinkExt {
  pub cur_timestamp: u64,  // method invocation timestamp for `process`
  pub old_timestamp: u64,
  pub oss_delay:     u32, // additional delay in 1/8ths of period
  pub oss_delay_default: u32 // init-time value, restored by a NULL Props reset
}

impl Default for SinkExt {
  fn default() -> Self {
    Self {
      cur_timestamp: 0,
      old_timestamp: 0,
      // default fill target: 10/8 of a period
      oss_delay:         10,
      oss_delay_default: 10
    }
  }
}

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SinkPortExt {
  pub xrun_timestamp: u64, // the moment we noticed an underrun (which is a bit later than the start of it)
  pub target_delay:   u32, // OSS buffer fill target in bytes, clamped to the granted buffer
  pub period_mismatch: u32 // consecutive cycles at a different period (debounce)
}

#[derive(Debug, Clone)]
pub struct PortConfig {
  pub format:    libspa::param::audio::AudioFormat,
  pub rate:      u32,
  pub channels:  u32,
  pub positions: Vec<u32>, // the negotiated channel positions, replayed in the Format readback
  pub flags:     u32
}

impl PortConfig {

  fn bytes_per_sample(&self) -> u32 {
    match self.format {
      libspa::param::audio::AudioFormat::S32LE => 4,
      libspa::param::audio::AudioFormat::S32BE => 4,
      libspa::param::audio::AudioFormat::S16LE => 2,
      libspa::param::audio::AudioFormat::S16BE => 2,
      _ => unreachable!()
    }
  }

  fn stride(&self) -> u32 {
    self.bytes_per_sample() * self.channels
  }

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
  fn stride(&self) -> u32      { PortConfig::stride(self) }
  fn format_raw(&self) -> u32  { self.format.0 }
  fn flags(&self) -> u32       { self.flags }
  fn positions(&self) -> &[u32] { &self.positions }
}

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time). One FreeBSD difference:
// GETODELAY reports the soft buffer only - the kernel pre-fills the hardware
// buffer at trigger and never counts it - so the absolute delay is
// understated by bufhard; the servo only needs cycle-to-cycle consistency
// and is unaffected.
unsafe fn timeout_servo(state: &mut State<SinkDir>, nsec: u64, rate: u32) -> (f64, i64) {
  let mut corr:  f64 = 1.0;
  let mut delay: i64 = 0;
  for port in &mut state.ports {
    let Some(cfg) = port.config.as_ref() else { continue };
    let stride      = cfg.stride().max(1);
    let device_rate = cfg.rate.max(1);
    if !port.dsp.is_running() || port.setup_period == 0 || port.resetup_pending {
      continue;
    }

    let odelay = port.dsp.odelay();
    // device frames scale to the graph rate; the resampler queue is already
    // graph-side (audioconvert reports it unscaled, like ALSA adds it)
    let resamp = if state.rate_match.is_null() { 0 } else { (*state.rate_match).delay as i64 };
    delay = (odelay as i64 / stride as i64) * rate as i64 / device_rate as i64 + resamp;

    if port.ext.xrun_timestamp != 0 {
      continue; // recovering; process() is discarding buffers, hold the servo
    }

    // clamp the error so a wakeup-jitter spike can't wind up the integrator
    // against an actuator that moves slowly (ALSA clamps to max_error too)
    let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
    let err = (odelay as f64 - port.ext.target_delay as f64).clamp(-max_err, max_err);
    corr = port.dll.update(err);
    port.bw_adapt.update(&mut port.dll, err, stride, port.setup_blocksize,
      nsec, port.setup_period, device_rate * stride);

    // a diverged servo must not wedge the graph clock
    if !(0.5..=2.0).contains(&corr) {
      crate::warn!(state.log, "{}: DLL diverged (corr {}); relocking", port.dsp.path, corr);
      port.dll.init();
      port.bw_adapt.reset();
      corr = 1.0;
    }

    #[cfg(debug_assertions)]
    eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, odelay as f64 - port.ext.target_delay as f64);
  }
  (corr, delay)
}

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(dsp: &mut crate::sound::DspWriter, config: &PortConfig, log: &crate::spa::Log) -> c_int {
  // a busy or vanished device must fail negotiation, not abort
  if let Err(err) = dsp.open() {
    crate::warn!(log, "{}: open: {}", dsp.path, err);
    return -(err as c_int);
  }
  // ditto for a device that won't take the format exactly
  if let Err(err) = dsp.configure(config.oss_format(), config.channels, config.rate) {
    crate::warn!(log, "{}: device rejected {:?}: {}", dsp.path, config, err);
    dsp.close();
    return -(err as c_int);
  }
  // on direct opens the hardware blocksize is per-session state; re-read it
  // now that THIS configuration is in effect (vchan/uaudio values are stable)
  dsp.hw_quantum_ns = crate::sound::drain_quantum_ns(&dsp.path, true);
  0
}

unsafe fn process_ports(state: &mut State<SinkDir>) -> c_int {

  state.ext.old_timestamp = state.ext.cur_timestamp;
  state.ext.cur_timestamp = crate::utils::now_ns(&state.data_system);

  // Freewheeling: the graph runs faster than realtime, so consume the input
  // without touching the device. The io NEED_DATA + return HAVE_DATA pair
  // looks odd for a sink but matches alsa-pcm-sink.c:788-791; it is what
  // keeps the freewheel pump running.
  if (*state.position).clock.flags & SPA_IO_CLOCK_FLAG_FREEWHEEL != 0 {
    for port in &mut state.ports {
      if !port.io.is_null() {
        (*port.io).status = SPA_STATUS_NEED_DATA as i32;
      }
    }
    return SPA_STATUS_HAVE_DATA as i32;
  }

  let mut result = SPA_STATUS_OK as i32;
  let state_ptr: *mut State<SinkDir> = state;

  for (port_idx, port) in state.ports.iter_mut().enumerate() {

    if port.config.is_none() {
      continue;
    }

    let port_config = port.config.as_ref().unwrap();

    if port.buffers.is_empty() || port.io.is_null() {
      continue; // not (fully) negotiated yet
    }

    if port.resetup_pending {
      // the main thread is rebuilding the device; drop cycles until it lands
      (*port.io).status = SPA_STATUS_NEED_DATA as i32;
      result |= SPA_STATUS_NEED_DATA as i32;
      continue;
    }

    if port.dsp.is_closed() {
      // Suspend closed the device but the host restarted without a fresh
      // format; rebuild off-loop instead of tripping the dsp state asserts
      port.resetup_pending = state.main_loop.as_ref().is_some_and(|main_loop|
        crate::utils::invoke_on_loop(main_loop, state_ptr, move |state| crate::node::resetup_task(state, port_idx)));
      (*port.io).status = SPA_STATUS_NEED_DATA as i32;
      result |= SPA_STATUS_NEED_DATA as i32;
      continue;
    }

    if (*port.io).status != SPA_STATUS_HAVE_DATA as i32 {
      // no input this cycle (e.g. draining after stop); the clock (incl. the
      // draining delay) is published from on_timeout now, so just ask for data
      (*port.io).status = SPA_STATUS_NEED_DATA as i32;
      result |= SPA_STATUS_NEED_DATA as i32; // in the return too: the host prefetches only on this bit
      continue;
    }

    // buffer_id, n_datas and the data type all come from the peer. Validate them
    // instead of asserting; a panic here aborts the process across extern "C".
    let buffer_id = (*port.io).buffer_id;
    let buffer = match port.buffers.get(buffer_id as usize).copied().and_then(|b| b.as_ref()) {
      Some(b) if b.n_datas == 1 => b, // we map the block directly, so need exactly one
      _ => {
        crate::warn!(state.log, "{}: unusable buffer (id {}); skipping", port.dsp.path, buffer_id);
        (*port.io).status = SPA_STATUS_NEED_DATA as i32;
        result |= SPA_STATUS_NEED_DATA as i32; // return status, not just io, so the host refills
        continue;
      }
    };

    // the code below maps data, derefs chunk and divides by maxsize, so require a
    // MemPtr block with all three valid. as_ref() (not offset(0)) handles a null
    // datas pointer without UB.
    let data_0 = match buffer.datas.as_ref() {
      Some(d) if d.type_ == SPA_DATA_MemPtr && !d.data.is_null() && !d.chunk.is_null() && d.maxsize > 0 => d,
      _ => {
        crate::warn!(state.log, "{}: buffer data is not a usable MemPtr block; skipping", port.dsp.path);
        (*port.io).status = SPA_STATUS_NEED_DATA as i32;
        result |= SPA_STATUS_NEED_DATA as i32; // return status, not just io, so the host refills
        continue;
      }
    };

    // chunk non-null and maxsize > 0 guaranteed above
    let offset = (*data_0.chunk).offset % data_0.maxsize;
    let size   = (*data_0.chunk).size.min(data_0.maxsize - offset);

    debug_assert_eq!((*data_0.chunk).stride, port_config.stride() as i32);

    #[cfg(debug_assertions)]
    if (*state.position).clock.flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER != 0 {
      crate::warn!(state.log, "{}: SPA_IO_CLOCK_FLAG_XRUN_RECOVER @ {}", port.dsp.path, state.ext.cur_timestamp);
    }

    #[cfg(debug_assertions)]
    if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
      crate::trace!(state.log, "offset: {}, chunk size: {}", offset, size);
      spa_debug_mem(0, data_0.data.offset(offset as isize), 16.min(size) as usize);
    }

    let driver_clock = (*state.position).clock;
    let matching = state.following && !crate::utils::same_clock(state.position, &state.clock_name);

    // the resampler can legitimately hand us a few frames over a quantum; warn
    // rather than debug_assert!, which would abort the process (panic across the
    // extern "C" boundary). The write path below caps and drops the excess.
    #[cfg(debug_assertions)]
    if size > driver_clock.target_duration as u32 * port_config.stride() {
      crate::warn!(state.log, "{}: chunk size {} exceeds one quantum {}",
        port.dsp.path, size, driver_clock.target_duration as u32 * port_config.stride());
    }

    // one graph cycle in device bytes (see utils::device_period_bytes)
    let period_in_bytes = crate::utils::device_period_bytes(
      driver_clock.target_duration, port_config.rate, driver_clock.target_rate.denom, port_config.stride());

    // A quantum or graph-rate change needs a different buffer layout, and a
    // triggered OSS channel can't be retuned (SETFRAGMENT and the params
    // return EINVAL once running, unlike ALSA's in-place rethreshold). The
    // rebuild happens on the main thread - device opens and closes can sleep,
    // which must stay off the shared data loop - while cycles are dropped.
    if port.dsp.is_running() && port.setup_period != 0 && period_in_bytes != 0 && period_in_bytes != port.setup_period {
      // debounce: a single-cycle flip usually means a renegotiation is in
      // flight (which re-primes anyway); rebuilding on it costs an audible
      // gap per storm event. Write at the old size for one cycle instead.
      port.ext.period_mismatch += 1;
      if port.ext.period_mismatch >= 2 {
        crate::info!(state.log, "{}: period {} -> {} bytes; re-setting up", port.dsp.path, port.setup_period, period_in_bytes);
        port.resetup_pending = state.main_loop.as_ref().is_some_and(|main_loop|
          crate::utils::invoke_on_loop(main_loop, state_ptr, move |state| crate::node::resetup_task(state, port_idx)));
        if port.resetup_pending {
          port.was_matching = false; // the gap invalidates matching history
          (*port.io).status = SPA_STATUS_NEED_DATA as i32;
          result |= SPA_STATUS_NEED_DATA as i32;
          continue;
        }
        // no main loop (unusual host): keep running at the stale size; the
        // write path drops or underruns but nothing stalls or aborts
      }
    } else {
      port.ext.period_mismatch = 0;
    }

    if !port.dsp.is_running() {

      #[cfg(debug_assertions)]
      {
        fn prio_type(type_: libc::c_ushort) -> &'static str {
          match type_ {
            libc::RTP_PRIO_REALTIME => "realtime",
            libc::RTP_PRIO_NORMAL   => "normal",
            libc::RTP_PRIO_IDLE     => "idle",
            _ => unreachable!()
          }
        }

        fn gettid() -> i32 {
          let mut tid = 0;
          if unsafe { libc::thr_self(&mut tid) } != -1 {
            assert!(tid <= i32::MAX as i64);
            tid as i32
          } else {
            0
          }
        }

        let mut rtp = libc::rtprio { type_: 0, prio:  0 };

        let pid = libc::getpid();
        if libc::rtprio(libc::RTP_LOOKUP, pid, &mut rtp) != -1 {
          crate::warn!(state.log, "process priority ({:5}): type = {}, prio = {}", pid, prio_type(rtp.type_), rtp.prio);
        }

        let tid = gettid();
        if libc::rtprio_thread(libc::RTP_LOOKUP, tid, &mut rtp) != -1 {
          crate::warn!(state.log, "thread priority ({:6}): type = {}, prio = {}", tid, prio_type(rtp.type_), rtp.prio);
        }
      }

      let desired_delay = (period_in_bytes / 8).saturating_mul(state.ext.oss_delay);

      // Size the fill to the granted buffer and the device's real fragment.
      // oss_fragment (0 = automatic 1 KiB) only mutates on this loop, so the
      // read is race-free; no ioctls beyond what the prime always issued
      // The measurement/drain quantum is the granted fragment - unless the
      // device's hardware cadence is coarser (drivers that ignore
      // SETFRAGMENT and pull fixed transfers; vchan parents), which the
      // soft fragsize can't see and sndstat can. Floor, headroom and the
      // servo noise model key on the larger - and the buffer REQUEST must
      // include it, or a device that honors the request grants no room for
      // the ceiling above the floor.
      let chunk = ((port.dsp.hw_quantum_ns as u128)
        .saturating_mul(port_config.rate as u128)
        .saturating_mul(port_config.stride() as u128) / 1_000_000_000)
        .min(u32::MAX as u128) as u32;
      let frag_est = if state.oss_fragment == 0 { 1024 } else { state.oss_fragment };
      let request = period_in_bytes.saturating_mul(2).saturating_add(desired_delay)
        .max(period_in_bytes
          .saturating_add((period_in_bytes / 4).max(frag_est.max(chunk)))
          .saturating_add(period_in_bytes) // write_max at rate_match.size <= period
          .saturating_add(frag_est.max(chunk)));
      let granted   = port.dsp.set_buffer_size(request, state.oss_fragment);
      let blocksize = port.dsp.blocksize().max(chunk);

      // saturating arithmetic: blocksize/rate_match.size are device-provided and
      // an overflow here would abort the data loop.
      let mut delay_capped = false;
      port.ext.target_delay = if granted >= period_in_bytes.saturating_mul(2) {
        // Calibrated: period/8 per oss.delay step. The floor keeps a jitter
        // margin - a quarter period, or one device fragment when the
        // fragment dwarfs the quantum - so a small oss.delay (or a tiny
        // quantum) can't starve the wakeup fill. The ceiling always leaves
        // room above target for the largest expected write (a quantum, or
        // the resampler's size if larger) plus one fragment of servo
        // wander: the OSS write is non-blocking, so a write that doesn't
        // fit short-writes and DROPS the tail. A driver that grants many
        // small fragments in a large buffer (uaudio) must
        // not be fill-targeted near-full - that both adds 100+ ms of
        // latency and leaves one fragment of headroom, dropping a chunk on
        // every normal servo excursion. (uaudio drains buffer_ms-sized
        // transfers, folded into blocksize above.) On a genuinely small grant
        // (snd_hdspe forces both the fragment and the total) the ceiling
        // lands just under near-full, which is the best a two-quanta
        // buffer can do.
        let rate_match_bytes = if state.rate_match.is_null() { 0 } else { (*state.rate_match).size.saturating_mul(port_config.stride()) };
        let write_max = period_in_bytes.max(rate_match_bytes);
        let floor = period_in_bytes.saturating_add((period_in_bytes / 4).max(blocksize));
        let ceil  = granted.saturating_sub(write_max.saturating_add(blocksize)).max(period_in_bytes);
        delay_capped = desired_delay.max(floor) > ceil;
        desired_delay.max(floor).min(ceil).max(period_in_bytes)
      } else {
        granted / 2 // buffer too small for two quanta; best-effort, will drop (warned below)
      };

      port.setup_period    = period_in_bytes;
      port.setup_blocksize = blocksize; // the effective quantum, incl. hw chunk
      port.dll.init();
      port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle

      crate::warn!(state.log, "{}: granted {}, blocksize {}, period {}, target delay {}",
        port.dsp.path, granted, blocksize, period_in_bytes, port.ext.target_delay);
      if delay_capped {
        crate::info!(state.log, "{}: the oss.delay target is capped by the granted buffer ({})",
          port.dsp.path, granted);
      }
      if granted < period_in_bytes.saturating_mul(2) {
        crate::warn!(state.log, "{}: granted OSS buffer ({}) is smaller than two quanta ({}); \
          audio will glitch. Lower the PipeWire quantum; we set the fragment size \
          explicitly, so hw.snd.latency has no effect",
          port.dsp.path, granted, period_in_bytes * 2);
      }

      port.dsp.write_zeroes(port.ext.target_delay);
    } else {
      let underrun_count = port.dsp.underruns();
      // The vchan mixer counts a momentarily-short child as an xrun and pads
      // it with silence (feeder_mixer.c); with the fill still healthy that's
      // accounting noise, not a dropout - only a genuinely low fill at
      // wakeup is a real underrun worth recovery and reporting. "Low" is a
      // period, capped by the healthy sawtooth floor (target minus one
      // fragment): with a fragment wider than the period the fill routinely
      // dips under one fragment while perfectly locked, and gating on the
      // fragment size would fire recovery on every accounting tick there.
      if underrun_count > 0 {
        // (cached blocksize: the channel can't be retuned while triggered,
        // and the gate must not cost ioctls on healthy cycles)
        let low = period_in_bytes
          .min(port.ext.target_delay.saturating_sub(port.setup_blocksize))
          .max(period_in_bytes / 4);
        // A late cycle finds a legitimately lower fill (the device kept
        // draining), so the threshold tracks the expected healthy fill at
        // THIS moment; the floor keeps a true empty ring (a real underrun
        // reads 0 until we write) detectable at any lateness.
        let elapsed = state.ext.cur_timestamp.saturating_sub(driver_clock.nsec);
        let drained = ((elapsed as u128)
          .saturating_mul(port_config.rate as u128)
          .saturating_mul(port_config.stride().max(1) as u128) / 1_000_000_000) as u32;
        let wander = (period_in_bytes / 4).max(port.setup_blocksize);
        let low = low
          .min(port.ext.target_delay.saturating_sub(drained).saturating_sub(wander))
          .max(period_in_bytes / 16);
        let odelay_now = port.dsp.odelay();
        if odelay_now < low {
          if let Some(suppressed) = port.warn_limit.check(state.ext.cur_timestamp) {
            crate::warn!(state.log, "{}: OSS reported {:3} underruns @ {} (+{} warnings suppressed)",
              port.dsp.path, underrun_count, state.ext.cur_timestamp, suppressed);
          }
          if port.ext.xrun_timestamp == 0 {
            // snapshot the DRIVER clock, not wall time: the recovery
            // condition compares against driver_clock.nsec (idealized cycle
            // start, which lags wall time by any lateness); a wall snapshot
            // deferred recovery by the lateness, discarding a buffer per
            // late cycle
            port.ext.xrun_timestamp = driver_clock.nsec.max(1);

            // report the EVENT to the host (pw-top's xrun counter) once,
            // not per held cycle; the length isn't known at detection, so
            // 0 delay
            let node_callbacks = state.callbacks.funcs.cast::<spa_node_callbacks>().as_ref();
            if let Some(xrun_fun) = node_callbacks.and_then(|c| c.xrun) {
              xrun_fun(state.callbacks.data, state.ext.cur_timestamp / 1000, 0, std::ptr::null_mut());
            }
          }
        } else {
          // suppressed counts stay diagnosable: a marginal system that
          // ticks the counter while self-healing shows up at debug level
          crate::debug!(state.log, "{}: {} underrun counts ignored (fill {} >= {})",
            port.dsp.path, underrun_count, odelay_now, low);
        }
      }
    }

    let mut corr: f64 = 1.0; // DLL rate correction, published as clock.rate_diff below
    let nbytes = if port.ext.xrun_timestamp != 0 {

      // Recover on the first data cycle past the event (ALSA does the same:
      // snap the fill, resume immediately). Waiting for a particular process
      // cadence discards real buffers per failed attempt, and a follower
      // under a corr-steered driver may never hit a fixed window at all.
      if driver_clock.nsec > port.ext.xrun_timestamp && driver_clock.flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER == 0 {
        port.ext.xrun_timestamp = 0;

        port.dll.init();
        port.bw_adapt.reset();

        // buffer's already sized; re-prime only up to target, accounting for what's
        // still queued (a full target_delay would push odelay past the buffer)
        let odelay = port.dsp.odelay();
        let refill = port.ext.target_delay.saturating_sub(odelay);

        #[cfg(debug_assertions)]
        crate::warn!(state.log, "{}: re-priming with {} zeroes (odelay {})", port.dsp.path, refill, odelay);

        port.dsp.write_zeroes(refill);
        // write `size`, not `period_in_bytes`: only `size` bytes at `offset` are owned
        port.dsp.write(data_0.data.offset(offset as isize), size)
      } else {
        #[cfg(debug_assertions)]
        crate::warn!(state.log, "{}: skipping buffer @ {}", port.dsp.path, driver_clock.nsec);

        size as isize
      }
    } else {
      // when driving, the servo runs in on_timeout where the clock is
      // published; here the DLL only serves rate matching as a follower on a
      // foreign clock - a same-device follower has nothing to correct, and
      // updating anyway would wind the integrator (ALSA gates the same way)
      let mut skip_write = false;
      if matching && port.setup_period != 0 && port.ext.period_mismatch == 0 {
        let stride   = port_config.stride().max(1);
        let cfg_rate = port_config.rate;
        if !port.was_matching {
          // matching just engaged; relock rather than apply stale state
          port.dll.init();
          port.bw_adapt.reset();
        }
        let odelay  = port.dsp.odelay();
        let err_raw = odelay as f64 - port.ext.target_delay as f64;
        if err_raw.abs() > port.setup_period as f64 {
          // Fill snap (ALSA's max_resync): a level error past one period is
          // beyond what the +/-1% actuator removes promptly and would wind the
          // integrator against the clamp. Correct the level directly -
          // refill on underfill, drain a cycle on overfill - and relock.
          port.dll.init();
          port.bw_adapt.reset();
          if err_raw < 0.0 {
            port.dsp.write_zeroes(port.ext.target_delay.saturating_sub(odelay));
          } else {
            skip_write = true;
          }
        } else {
          let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
          let err = err_raw.clamp(-max_err, max_err);
          corr = port.dll.update(err);
          port.bw_adapt.update(&mut port.dll, err, stride, port.setup_blocksize,
            state.ext.cur_timestamp, port.setup_period, cfg_rate * stride);
        }

        #[cfg(debug_assertions)]
        eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, err_raw);
      }

      if state.following && !matching && port.setup_period != 0 && port.ext.period_mismatch == 0 {
        // same-device follower: no rate to match, but the level can still
        // drift on missed cycles; correct it directly
        let odelay  = port.dsp.odelay();
        let err_raw = odelay as f64 - port.ext.target_delay as f64;
        if err_raw < -(port.setup_period as f64) {
          port.dsp.write_zeroes(port.ext.target_delay.saturating_sub(odelay));
        } else if err_raw > port.setup_period as f64 {
          skip_write = true;
        }
      }

      if skip_write {
        size as isize // consumed; the device drains toward target meanwhile
      } else {
        port.dsp.write(data_0.data.offset(offset as isize), size)
      }
    };

    // Rate-match only as a follower on a foreign clock: when driving, the
    // timer steering applies the correction, and a same-device follower ticks
    // from our clock so there is nothing to match (ALSA gates on the clock
    // name the same way).
    port.was_matching = matching;
    if !state.rate_match.is_null() {
      if matching {
        (*state.rate_match).flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
        (*state.rate_match).rate   = corr.clamp(0.99, 1.01);
      } else {
        (*state.rate_match).flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
        (*state.rate_match).rate   = 1.0;
      }
    }

    if nbytes < size as isize {
      if let Some(suppressed) = port.warn_limit.check(state.ext.cur_timestamp) {
        crate::warn!(state.log, "{}: dropped {} bytes (+{} warnings suppressed)",
          port.dsp.path, if nbytes > 0 { size - nbytes as u32 } else { size }, suppressed);
      }
    }

    (*port.io).status = SPA_STATUS_NEED_DATA as i32;

    // a sink has no output, so the return bit is NEED_DATA ("can accept input
    // next cycle"), matching the port io status, not HAVE_DATA.
    result |= SPA_STATUS_NEED_DATA as i32;
  }

  result
}

impl Direction for SinkDir {

  const DIRECTION: spa_direction = SPA_DIRECTION_INPUT;
  const PLAYBACK: bool = true;
  const MEDIA_CLASS: &'static str = "Audio/Sink";
  const READY_STATUS: i32 = SPA_STATUS_NEED_DATA as i32;
  const CMD_WARN_PREFIX: &'static str = "";

  type Device  = crate::sound::DspWriter;
  type Config  = PortConfig;
  type Ext     = SinkExt;
  type PortExt = SinkPortExt;

  fn info_item(ext: &mut SinkExt, key: &str, value: &str) {
    if key == crate::keys::OSS_DELAY {
      // per-device default, e.g. from a wireplumber node rule
      if let Ok(v) = value.parse::<u32>() {
        ext.oss_delay = v.min(1024);
      }
    }
  }

  fn ext_ready(ext: &mut SinkExt) {
    ext.oss_delay_default = ext.oss_delay;
  }

  unsafe fn build_node_param(state: &mut State<SinkDir>, b: &mut libspa::pod::builder::Builder, id: u32, index: u32) -> ParamBuild {
    #[allow(non_upper_case_globals)]
    match (id, index) {
      (SPA_PARAM_PropInfo, 0)       => crate::utils::build_latency_offset_prop_info(b).unwrap(),
      (SPA_PARAM_PropInfo, 1)       => crate::utils::build_params_prop_info(b, crate::keys::OSS_DELAY,
        "OSS buffer fill target (1/8ths of a period)", state.ext.oss_delay, 1024).unwrap(),
      (SPA_PARAM_PropInfo, 2)       => crate::utils::build_params_prop_info(b, crate::keys::OSS_FRAGMENT,
        "OSS fragment size (bytes, power of two, 0 = automatic)", state.oss_fragment, 16384).unwrap(),
      (SPA_PARAM_PropInfo, _)       => return ParamBuild::Exhausted,
      (SPA_PARAM_Props, 0)          => crate::utils::build_latency_offset_props(b, state.process_latency.ns,
        &[(crate::keys::OSS_DELAY, state.ext.oss_delay), (crate::keys::OSS_FRAGMENT, state.oss_fragment)]).unwrap(),
      (SPA_PARAM_Props, _)          => return ParamBuild::Exhausted,
      (SPA_PARAM_ProcessLatency, 0) => crate::utils::build_process_latency_info(b, &state.process_latency).unwrap(),
      (SPA_PARAM_ProcessLatency, _) => return ParamBuild::Exhausted,
      _ => return ParamBuild::Unknown
    };
    ParamBuild::Built
  }

  // a NULL Props pod resets the props to their defaults and re-applies them
  unsafe fn reset_props(state: &mut State<SinkDir>) -> c_int {
    let res = crate::node::store_and_rebuild(state, |state| {
      state.ext.oss_delay = state.ext.oss_delay_default; // read by process()
      state.oss_fragment  = state.oss_fragment_default;  // ditto (the prime path)
    });
    if res != 0 {
      return res;
    }
    crate::node::handle_process_latency(state, crate::utils::process_latency_default());
    0
  }

  unsafe fn set_props_params(state: &mut State<SinkDir>, value: &libspa::pod::Value) -> c_int {
    use libspa::pod::Value;
    match value {
      Value::Struct(values) if values.len() % 2 == 0 => {
        for kv in values.chunks(2) {
          match (&kv[0], &kv[1]) {
            // pw-cli set-param <object-id> Props '{ "params": ["oss.delay", 8]}'
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_DELAY && *x >= 0 => {
              // cap it: period/8 * oss_delay runs in the RT path and must not overflow
              let new_delay = (*x as u32).min(1024);
              if new_delay != state.ext.oss_delay {
                // unchanged echoes must not rebuild a running device
                let res = crate::node::apply_props_param(state, move |state| state.ext.oss_delay = new_delay);
                if res != 0 {
                  return res;
                }
              }
            },
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_FRAGMENT && *x >= 0 => {
              // stored normalized, so the Props readback reports the
              // effective (rounded/clamped) value, not the raw request
              let new_fragment = crate::node::normalize_fragment(*x as u32);
              if new_fragment != state.oss_fragment {
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

  unsafe fn parse_config(state: &mut State<SinkDir>, raw: &spa_audio_info_raw) -> Result<PortConfig, c_int> {
    let format    = libspa::param::audio::AudioFormat(raw.format);

    let config = PortConfig {
      format,
      rate:      raw.rate,
      channels:  raw.channels,
      positions: raw.position[..raw.channels as usize].to_vec(),
      flags:     raw.flags
    };

    crate::debug!(state.log, "reconfiguring with {:?}", config);

    // only formats from our EnumFormat are expected; reject the rest
    let oss_format = match config.format {
      libspa::param::audio::AudioFormat::S32LE => crate::sound::AFMT_S32_LE,
      libspa::param::audio::AudioFormat::S32BE => crate::sound::AFMT_S32_BE,
      libspa::param::audio::AudioFormat::S16LE => crate::sound::AFMT_S16_LE,
      libspa::param::audio::AudioFormat::S16BE => crate::sound::AFMT_S16_BE,
      _ => {
        crate::warn!(state.log, "rejecting unsupported format {:?}", config.format);
        return Err(-libc::ENOTSUP);
      }
    };

    let _ = oss_format;
    Ok(config)
  }

  fn try_open_configure(dsp: &mut crate::sound::DspWriter, config: &PortConfig, _fragment: u32, log: &crate::spa::Log) -> c_int {
    // the sink's SETFRAGMENT happens at prime time (process_ports), where
    // the graph period the layout depends on is known
    try_open_configure(dsp, config, log)
  }

  unsafe fn on_device_swapped(state: &mut State<SinkDir>, port_idx: usize) {
    state.ports[port_idx].ext.xrun_timestamp = 0;
  }

  unsafe fn on_buffers_swapped(_state: &mut State<SinkDir>) {}

  unsafe fn on_start_loop(state: &mut State<SinkDir>) {
    for port in &mut state.ports {
      port.ext.xrun_timestamp = 0;
    }
    state.ext.cur_timestamp = 0;
    state.ext.old_timestamp = 0;
  }

  unsafe fn on_suspend_loop(_state: &mut State<SinkDir>) {}

  unsafe fn on_role_flip(state: &mut State<SinkDir>) {
    // a role flip shifts the servo's measurement phase, not the fill:
    // relock the DLL instead of holding playback like an underrun (the
    // fill snap in the write path corrects any real level error)
    for port in &mut state.ports {
      port.dll.init();
      port.bw_adapt.reset();
      port.was_matching = false;
    }
  }

  // data loop only
  unsafe fn update_timers(state: &mut State<SinkDir>) {

    #[cfg(debug_assertions)]
    crate::trace!(state.log, "update_timers");

    if state.started && !state.following && !state.position.is_null() {
      state.next_time = crate::utils::now_ns(&state.data_system);
      #[cfg(debug_assertions)]
      crate::trace!(state.log, "next time {}", state.next_time);
      crate::node::set_timeout(state, state.next_time);
    } else {
      #[cfg(debug_assertions)]
      crate::trace!(state.log, "next time {}", 0);
      crate::node::set_timeout(state, 0);
    }
  }

  unsafe fn debug_cycle(_state: &State<SinkDir>, _now: u64, _nsec: u64) {
    #[cfg(debug_assertions)]
    eprintln!("cycle: {}, delay: {} ms @ {}", (*_state.position).clock.cycle, _now.saturating_sub(_nsec) as f64 / 1000000.0, _now);
  }

  unsafe fn timeout_servo(state: &mut State<SinkDir>, nsec: u64, rate: u32) -> (f64, i64) {
    timeout_servo(state, nsec, rate)
  }

  unsafe fn process_ports(state: &mut State<SinkDir>) -> c_int {
    process_ports(state)
  }
}

const OSS_SINK_FACTORY_INFO: spa_dict = spa_dict {
  flags:   0,
  n_items: 0,
  items:   std::ptr::null()
};

pub const OSS_SINK_FACTORY: spa_handle_factory = spa_handle_factory {
  version:             SPA_VERSION_HANDLE_FACTORY,
  name:                c"freebsd-oss.sink".as_ptr(),
  info:                &OSS_SINK_FACTORY_INFO,
  get_size:            Some(crate::node::get_size::<SinkDir>),
  init:                Some(crate::node::init::<SinkDir>),
  enum_interface_info: Some(crate::node::enum_interface_info)
};
