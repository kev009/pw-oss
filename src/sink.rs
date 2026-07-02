use std::mem::MaybeUninit;
use std::os::raw::{c_char, c_int, c_void};

use libspa::sys::*;

const MAX_PORTS: usize = 1;

#[repr(C)]
struct State {
  handle:        spa_handle,
  node:          spa_node,
  node_info:     crate::spa::NodeInfo,
  port_info:     crate::spa::PortInfo,
  data_loop:     crate::spa::Loop,
  data_system:   crate::spa::System,
  log:           crate::spa::Log,
  clock:         *mut spa_io_clock,
  position:      *mut spa_io_position,
  rate_match:    *mut spa_io_rate_match,
  clock_name:    std::ffi::CString, // stamped into spa_io_clock.name
  timer_source:  spa_source,
  next_time:     u64,
  hooks:         spa_hook_list,
  callbacks:     spa_callbacks,
  ports:         [Port; MAX_PORTS],
  caps:          crate::sound::DspCaps,
  latency:       [spa_latency_info; 2], // indexed by direction; written by the host, replayed on read
  process_latency: spa_process_latency_info,
  started:       bool,
  following:     bool,
  cur_timestamp: u64,  // method invocation timestamp for `process`
  old_timestamp: u64,
  oss_delay:     u32, // additional delay in 1/8ths of period
  oss_delay_default: u32 // init-time value, restored by a NULL Props reset
}

impl State {

  fn node_is_follower(&self) -> bool {
    !self.clock.is_null() && !self.position.is_null() && unsafe { (*self.position).clock.id != (*self.clock).id }
  }
}

struct Port {
  config:         Option<PortConfig>,
  buffers:        Vec<*mut spa_buffer>,
  io:             *mut spa_io_buffers,
  dsp:            crate::sound::DspWriter,
  xrun_timestamp: u64, // the moment we noticed an underrun (which is a bit later than the start of it)
  dll:            crate::dll::SpaDLL,
  target_delay:   u32, // OSS buffer fill target in bytes, clamped to the granted buffer
  setup_period:   u32, // device bytes per graph cycle the stream was set up for
  bw_fast_until:  u64  // while nonzero, the DLL runs at BW_MAX for a fast lock
}

#[derive(Debug)]
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

unsafe extern "C" fn add_listener(object: *mut c_void, listener: *mut spa_hook, events: *const spa_node_events, data: *mut c_void) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  let mut save = MaybeUninit::<spa_hook_list>::uninit();
  spa_hook_list_isolate(&mut state.hooks, save.as_mut_ptr(), listener, events.cast(), data);

  // note that this only iterates over the newly added listener
  crate::spa::for_each_hook(&mut state.hooks, |entry| {

    let f = entry.cb.funcs.cast::<spa_node_events>().as_ref()
      .expect("we just assigned events to this very hook by calling spa_hook_list_isolate");

    assert!(f.version >= SPA_VERSION_NODE_EVENTS);

    if let Some(node_info_fun) = f.info {
      let old_mask = state.node_info.replace_change_mask(crate::spa::SPA_NODE_CHANGE_MASK_ALL as u64);
      node_info_fun(entry.cb.data, state.node_info.raw());
      let _ = state.node_info.replace_change_mask(old_mask);
    }

    if let Some(port_info_fun) = f.port_info {
      let old_mask = state.port_info.replace_change_mask(crate::spa::SPA_PORT_CHANGE_MASK_ALL as u64);
      port_info_fun(entry.cb.data, SPA_DIRECTION_INPUT, 0, state.port_info.raw());
      let _ = state.port_info.replace_change_mask(old_mask);
    }
  });

  spa_hook_list_join(&mut state.hooks, save.assume_init_mut());

  0
}

// re-emit node_info to every listener (carrying whatever change_mask the caller
// set, e.g. PARAMS), then clear the mask
unsafe fn emit_node_info(state: &mut State) {
  crate::spa::for_each_hook(&mut state.hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_node_events>().as_ref()
      .expect("hook should be initialized");
    if f.version >= SPA_VERSION_NODE_EVENTS {
      if let Some(node_info_fun) = f.info {
        node_info_fun(entry.cb.data, state.node_info.raw());
      }
    }
  });
  let _ = state.node_info.replace_change_mask(0);
}

// the process latency (user-set latency offset) shifts the node's reported
// latency, so a change re-emits the Props/ProcessLatency node params and the
// port Latency param
unsafe fn handle_process_latency(state: &mut State, info: spa_process_latency_info) {

  let ns_changed = state.process_latency.ns != info.ns;
  if state.process_latency.quantum == info.quantum &&
     state.process_latency.rate    == info.rate && !ns_changed {
    return;
  }

  state.process_latency = info;

  let _ = state.node_info.replace_change_mask(0);
  if ns_changed {
    state.node_info.bump_param(SPA_PARAM_Props);
  }
  state.node_info.bump_param(SPA_PARAM_ProcessLatency);
  emit_node_info(state);

  let _ = state.port_info.replace_change_mask(0);
  state.port_info.bump_param(SPA_PARAM_Latency);
  emit_port_info(state);
}

// re-emit port_info to every listener (carrying whatever change_mask the caller
// set, e.g. RATE/PARAMS), then clear the mask
unsafe fn emit_port_info(state: &mut State) {
  crate::spa::for_each_hook(&mut state.hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_node_events>().as_ref()
      .expect("hook should be initialized");
    if f.version >= SPA_VERSION_NODE_EVENTS {
      if let Some(port_info_fun) = f.port_info {
        port_info_fun(entry.cb.data, SPA_DIRECTION_INPUT, 0, state.port_info.raw());
      }
    }
  });
  let _ = state.port_info.replace_change_mask(0);
}

unsafe extern "C" fn set_callbacks(object: *mut c_void, callbacks: *const spa_node_callbacks, data: *mut c_void) -> c_int {
  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");
  state.callbacks.funcs = callbacks as *const c_void;
  state.callbacks.data  = data;
  0
}

unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {
  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");
  crate::spa::node_emit_done(&mut state.hooks, seq);
  0
}

unsafe fn build_oss_delay_prop_info(b: &mut libspa::pod::builder::Builder, current: u32) -> Result<(), rustix::io::Errno> {

  let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut outer, SPA_TYPE_OBJECT_PropInfo, SPA_PARAM_PropInfo)?;

  b.add_prop(SPA_PROP_INFO_name, 0)?;
  b.add_string("oss.delay")?;

  b.add_prop(SPA_PROP_INFO_description, 0)?;
  b.add_string("OSS buffer fill target (1/8ths of a period)")?;

  b.add_prop(SPA_PROP_INFO_type, 0)?;
  b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
  b.add_int(current as i32)?;
  b.add_int(0)?;
  b.add_int(1024)?;
  b.pop(inner.assume_init_mut());

  b.add_prop(SPA_PROP_INFO_params, 0)?;
  b.add_bool(true)?; // settable through the Props params struct

  b.pop(outer.assume_init_mut());

  Ok(())
}

unsafe extern "C" fn enum_params(object: *mut c_void, seq: c_int, id: u32, start: u32, max: u32, filter: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  if max == 0 {
    return 0;
  }

  let mut buffer  = vec![];
  let mut fbuffer = vec![]; // spa_pod_filter output; kept apart from the source pod (see spa::filter_pod)

  let mut index = start;
  let mut count = 0;

  while count < max {

    use libspa::pod::builder::Builder;

    let mut builder = Builder::new(&mut buffer);

    #[allow(non_upper_case_globals)]
    match (id, index) {
      (SPA_PARAM_PropInfo, 0)       => crate::utils::build_latency_offset_prop_info(&mut builder).unwrap(),
      (SPA_PARAM_PropInfo, 1)       => build_oss_delay_prop_info(&mut builder, state.oss_delay).unwrap(),
      (SPA_PARAM_PropInfo, _)       => return 0,
      (SPA_PARAM_Props, 0)          => crate::utils::build_latency_offset_props(&mut builder, state.process_latency.ns, Some(state.oss_delay)).unwrap(),
      (SPA_PARAM_Props, _)          => return 0,
      (SPA_PARAM_ProcessLatency, 0) => crate::utils::build_process_latency_info(&mut builder, &state.process_latency).unwrap(),
      (SPA_PARAM_ProcessLatency, _) => return 0,
      _ => return -libc::EINVAL
    };

    drop(builder); // its borrow of `buffer` must end before we take the source pointer

    let mut result = spa_result_node_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if let Some(param) = crate::spa::filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter) {
      result.param = param;
      crate::spa::node_emit_result(&mut state.hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS, &result);
      count += 1;
    }

    index += 1;
  }

  0
}

unsafe extern "C" fn set_param(object: *mut c_void, id: u32, _flags: u32, param: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  use libspa::pod::{Value, Object, Pod};
  use libspa::pod::deserialize::PodDeserializer;

  #[allow(non_upper_case_globals)]
  match id {
    SPA_PARAM_Props => {
      if param.is_null() {
        // a NULL pod resets the props to their defaults
        state.oss_delay = state.oss_delay_default;
        handle_process_latency(state, crate::utils::process_latency_default());
        return 0;
      }
      match PodDeserializer::deserialize_any_from(Pod::from_raw(param).as_bytes()) {
        Ok((_, Value::Object(Object { type_, properties, .. }))) if type_ == SPA_TYPE_OBJECT_Props => {
          for property in properties {
            match property.key {
              // there is no way adapter is actually supposed to pass all those properties (or parameters?) to us,
              // it's probably a bug
              SPA_PROP_volume         => (), // fuck it
              SPA_PROP_mute           => (), // ditto
              SPA_PROP_channelVolumes => (), // ditto
              SPA_PROP_channelMap     => (), // ditto
              SPA_PROP_monitorMute    => (), // ditto
              SPA_PROP_monitorVolumes => (), // ditto
              SPA_PROP_softMute       => (), // ditto
              SPA_PROP_softVolumes    => (), // ditto
              SPA_PROP_latencyOffsetNsec => {
                if let Value::Long(ns) = property.value {
                  let mut info = state.process_latency;
                  info.ns = ns;
                  handle_process_latency(state, info);
                }
              },
              SPA_PROP_params         => {
                match property.value {
                  Value::Struct(values) if values.len() % 2 == 0 => {
                    for kv in values.chunks(2) {
                      match (&kv[0], &kv[1]) {
                        // pw-cli set-param <object-id> Props '{ "params": ["oss.delay", 8]}'
                        (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_DELAY && *x >= 0 => {
                          // cap it: period/8 * oss_delay runs in the RT path and must not overflow
                          state.oss_delay = (*x as u32).min(1024);
                          // announce the new value so Props readback stays fresh
                          let _ = state.node_info.replace_change_mask(0);
                          state.node_info.bump_param(SPA_PARAM_Props);
                          emit_node_info(state);

                          // apply immediately: reopen the device so the next
                          // cycle re-sizes the buffer and re-primes with the
                          // new target (a brief gap, like a format change)
                          let state_ptr: *mut State = state;
                          crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, |state| {
                            for port in &mut state.ports {
                              let Some(cfg) = port.config.as_ref() else { continue };
                              if !port.dsp.is_running() {
                                continue; // not streaming; picked up at start
                              }
                              port.dsp.close();
                              if port.dsp.open().is_err() ||
                                 port.dsp.configure(cfg.oss_format(), cfg.channels, cfg.rate).is_err() {
                                crate::warn!(state.log, "{}: reopen for oss.delay failed", port.dsp.path);
                                if !port.dsp.is_closed() {
                                  port.dsp.close();
                                }
                                port.config = None;
                                continue;
                              }
                              port.xrun_timestamp = 0;
                            }
                          });
                        },
                        _ => ()
                      }
                    }
                  }
                  _ => ()
                }
              },
              key => {
                crate::debug!(state.log, "ignoring unknown prop {}", key);
              }
            }
          }
        },
        _ => return -libc::EINVAL
      }
      0
    },
    SPA_PARAM_ProcessLatency => {
      if param.is_null() {
        handle_process_latency(state, crate::utils::process_latency_default());
        return 0;
      }
      match crate::utils::parse_process_latency_info(param) {
        Some(info) => { handle_process_latency(state, info); 0 },
        None       => -libc::EINVAL
      }
    },
    id => {
      crate::warn!(state.log, "set_param: unknown param {}", id);
      -libc::ENOENT
    }
  }
}

// ALSA adapts the DLL bandwidth continuously from the error variance
// (alsa-pcm.c, BW_PERIOD); we approximate with two stages: a fast lock at
// BW_MAX after (re)start, then the low steady-state bandwidth
const DLL_FAST_NSEC: u64 = 3 * SPA_NSEC_PER_SEC as u64;

fn maybe_relax_dll(port: &mut Port, device_rate: u32, stride: u32, now: u64) {
  if port.bw_fast_until != 0 && now >= port.bw_fast_until {
    port.bw_fast_until = 0;
    port.dll.set_bw(crate::dll::SPA_DLL_BW_MIN, port.setup_period, device_rate * stride);
  }
}

unsafe extern "C" fn on_timeout(source: *mut spa_source) {

  let state = (*source).data.cast::<State>().as_mut()
    .expect("(*source).data is not supposed to be null");

  #[cfg(debug_assertions)]
  crate::trace!(state.log, "on_timeout");

  let mut expirations = 0;
  if state.data_system.timerfd_read(state.timer_source.fd, &mut expirations) < 0 {
    // disarmed (Pause/Suspend) in this same wakeup; nothing to read
    return;
  }

  // stopped between the timer firing and this callback; don't signal ready()
  // into a node being reconfigured, and don't re-arm
  if !state.started || state.following {
    return;
  }

  if state.position.is_null() || state.clock.is_null() {
    return; // ios cleared while the timer was armed; skip the cycle
  }

  let now = crate::utils::now_ns(&state.data_system);

  // resync after a long stall instead of replaying a burst of stale cycles
  // (ALSA snaps when more than a second behind)
  if now.saturating_sub(state.next_time) > SPA_NSEC_PER_SEC as u64 {
    crate::warn!(state.log, "timer stalled ({} ns behind); resyncing", now - state.next_time);
    state.next_time = now;
  }

  let nsec = state.next_time;

  #[cfg(debug_assertions)]
  eprintln!("cycle: {}, delay: {} ms @ {}", (*state.position).clock.cycle, now.saturating_sub(nsec) as f64 / 1000000.0, now);

  let duration = (*state.position).clock.target_duration;
  let rate     = (*state.position).clock.target_rate.denom;
  if duration == 0 || rate == 0 {
    set_timeout(state, nsec + SPA_NSEC_PER_SEC as u64 / 100); // malformed position; idle-tick
    return;
  }

  // Run the servo before the clock is published so every field below belongs
  // to this cycle (the shape of ALSA's update_time). One FreeBSD difference:
  // GETODELAY reports the soft buffer only - the kernel pre-fills the hardware
  // buffer at trigger and never counts it - so the absolute delay is
  // understated by bufhard; the servo only needs cycle-to-cycle consistency
  // and is unaffected.
  let mut corr:  f64 = 1.0;
  let mut delay: i64 = 0;
  for port in &mut state.ports {
    let Some(cfg) = port.config.as_ref() else { continue };
    let stride      = cfg.stride().max(1);
    let device_rate = cfg.rate.max(1);
    if !port.dsp.is_running() || port.setup_period == 0 {
      continue;
    }

    let odelay = port.dsp.odelay();
    // device frames scaled to the graph rate, plus the resampler's queue
    let resamp = if state.rate_match.is_null() { 0 } else { (*state.rate_match).delay as i64 };
    delay = (odelay as i64 / stride as i64 + resamp) * rate as i64 / device_rate as i64;

    if port.xrun_timestamp != 0 {
      continue; // recovering; process() is discarding buffers, hold the servo
    }

    maybe_relax_dll(port, device_rate, stride, nsec);

    // clamp the error so a wakeup-jitter spike can't wind up the integrator
    // against an actuator that moves slowly (ALSA clamps to max_error too)
    let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
    let err = (odelay as f64 - port.target_delay as f64).clamp(-max_err, max_err);
    corr = port.dll.update(err);

    // a diverged servo must not wedge the graph clock
    if !(0.5..=2.0).contains(&corr) {
      crate::warn!(state.log, "{}: DLL diverged (corr {}); relocking", port.dsp.path, corr);
      port.dll.init();
      port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, port.setup_period, device_rate * stride);
      port.bw_fast_until = nsec + DLL_FAST_NSEC;
      corr = 1.0;
    }

    #[cfg(debug_assertions)]
    eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, odelay as f64 - port.target_delay as f64);
  }

  // steer the timer by the correction so the published clock genuinely follows
  // the device (ALSA warps next_time the same way); this also closes the loop
  // in passthrough setups where no resampler consumes a rate_match
  state.next_time = nsec + (duration as f64 * SPA_NSEC_PER_SEC as f64 / (rate as f64 * corr)) as u64;

  (*state.clock).nsec      = nsec;
  (*state.clock).rate      = (*state.clock).target_rate;
  (*state.clock).position += (*state.clock).duration;
  (*state.clock).duration  = duration;
  (*state.clock).delay     = delay;
  (*state.clock).rate_diff = corr;
  (*state.clock).next_nsec = state.next_time;

  let Some(node_callbacks) = state.callbacks.funcs.cast::<spa_node_callbacks>().as_ref() else {
    set_timeout(state, state.next_time);
    return; // no callbacks (yet, or cleared); keep the clock ticking
  };
  if let Some(ready_fun) = node_callbacks.ready {
    let err = ready_fun(state.callbacks.data, SPA_STATUS_NEED_DATA as i32);
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "ready -> {}", err);
    #[cfg(not(debug_assertions))]
    let _ = err;
  }

  set_timeout(state, state.next_time);
}

unsafe fn set_timeout(state: &mut State, next_time: u64) {

  #[cfg(debug_assertions)]
  crate::trace!(state.log, "set_timeout {}", next_time);

  let timerspec = itimerspec {
    it_value: timespec {
      tv_sec:  (next_time / SPA_NSEC_PER_SEC as u64) as i64,
      tv_nsec: (next_time % SPA_NSEC_PER_SEC as u64) as i64
    },
    it_interval: timespec { tv_sec: 0, tv_nsec: 0 }
  };

  state.data_system.timerfd_settime(state.timer_source.fd, SPA_FD_TIMER_ABSTIME as i32, &timerspec, std::ptr::null_mut());
}

// data loop only
unsafe fn update_timers(state: &mut State) {

  #[cfg(debug_assertions)]
  crate::trace!(state.log, "update_timers");

  if state.started && !state.following {
    state.next_time = crate::utils::now_ns(&state.data_system);
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "next time {}", state.next_time);
    set_timeout(state, state.next_time);
  } else {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "next time {}", 0);
    set_timeout(state, 0);
  }
}

unsafe extern "C" fn set_io(object: *mut c_void, id: u32, data: *mut c_void, size: usize) -> c_int {

  let state: *mut State = object.cast();
  assert!(!state.is_null());

  #[allow(non_upper_case_globals)]
  let min_size = match id {
    SPA_IO_Clock    => std::mem::size_of::<spa_io_clock>(),
    SPA_IO_Position => std::mem::size_of::<spa_io_position>(),
    _ => return -libc::ENOENT
  };
  // NULL/0 clears the area; only a non-empty-but-short one is an error
  if !data.is_null() && size < min_size {
    return -libc::EINVAL;
  }

  // clock/position are read on the data loop; apply the change there
  crate::utils::block_on_loop(&(*state).data_loop, state, |state| {

    #[allow(non_upper_case_globals)]
    match id {
      SPA_IO_Clock    => {
        state.clock = data.cast(); // null clears
        // identify our clock so same-device followers can skip rate matching
        crate::utils::set_clock_name(state.clock, &state.clock_name);
      },
      SPA_IO_Position => state.position = data.cast(), // null clears
      _ => () // filtered above
    };

    if state.started {
      let following = state.node_is_follower();
      if state.following != following {
        state.following = following;
        update_timers(state);

        // there are some weird PipeWire xruns on clock changes that are messing up our OSS buffer delay,
        // we'll just preemptively treat them as OSS underruns for now
        for port in &mut state.ports {
          port.xrun_timestamp = crate::utils::now_ns(&state.data_system);
          #[cfg(debug_assertions)]
          crate::warn!(state.log, "{}: clock change @ {}", port.dsp.path, port.xrun_timestamp);
        }
      }
    }
  });

  0
}

unsafe extern "C" fn send_command(object: *mut c_void, command: *const spa_command) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  assert!(!command.is_null());
  let body = (*command).body.body;

  crate::debug!(state.log, "received command: {}", crate::utils::spa_command_to_str(&body));

  #[allow(non_upper_case_globals)]
  match (body.type_, body.id) {
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start) => {
      if state.ports.iter().any(|p| p.config.is_none() || p.buffers.is_empty()) {
        return -libc::EIO; // not negotiated yet (ALSA rejects this too)
      }
      let state: *mut State = state;
      crate::utils::block_on_loop(&(*state).data_loop, state, |state| {

        for port in &mut state.ports {
          port.xrun_timestamp = 0;
        }

        // sane clock delay/rate_diff until process() publishes measured values
        if !state.clock.is_null() {
          (*state.clock).delay     = 0;
          (*state.clock).rate_diff = 1.0;
        }

        state.started   = true;
        state.following = state.node_is_follower();

        state.cur_timestamp = 0;
        state.old_timestamp = 0;

        update_timers(state);
      });
      0
    },
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => {
      let state: *mut State = state;
      crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
        state.started = false;
        update_timers(state);
      });
      0
    },
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => {
      let state: *mut State = state;
      crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
        for port in &mut state.ports {
          if !port.dsp.is_closed() {
            port.dsp.close();
          }
        }
        state.started = false;
        update_timers(state);
      });
      0
    },
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin | SPA_NODE_COMMAND_ParamEnd) => 0, // we don't care
    (cmd_type, cmd_id) => {
      crate::warn!(state.log, "unknown command: {}, {}", cmd_type, cmd_id);
      -libc::ENOTSUP
    }
  }
}

unsafe extern "C" fn add_port(_object: *mut c_void, _direction: spa_direction, _port_id: u32, _props: *const spa_dict) -> c_int {
  -libc::ENOTSUP // the ports are static
}

unsafe extern "C" fn remove_port(_object: *mut c_void, _direction: spa_direction, _port_id: u32) -> c_int {
  -libc::ENOTSUP // the ports are static
}

//TODO: SPA_PARAM_PORT_CONFIG_MODE_none vs SPA_PARAM_PORT_CONFIG_MODE_passthrough vs SPA_PARAM_PORT_CONFIG_MODE_convert
/*unsafe fn build_port_config_info(builder: &mut libspa::pod::builder::Builder, config: &PortConfig, id: u32) -> Result<(), Errno> {

  let mut frame = MaybeUninit::<spa_pod_frame>::uninit();

  builder.push_object(&mut frame, SPA_TYPE_OBJECT_ParamPortConfig, SPA_PARAM_PortConfig)?;

  builder.add_prop(SPA_PARAM_PORT_CONFIG_direction, 0)?;
  builder.add_id(libspa::utils::Id(SPA_DIRECTION_INPUT))?;

  builder.add_prop(SPA_PARAM_PORT_CONFIG_mode, 0)?;
  builder.add_id(libspa::utils::Id(SPA_PARAM_PORT_CONFIG_MODE_none))?;

  builder.add_prop(SPA_PARAM_PORT_CONFIG_monitor, 0)?;
  builder.add_bool(false)?;

  builder.add_prop(SPA_PARAM_PORT_CONFIG_control, 0)?;
  builder.add_bool(false)?;

  builder.add_prop(SPA_PARAM_PORT_CONFIG_format, 0)?;
  build_port_format_info(builder, config, id);

  builder.pop(frame.assume_init_mut());

  Ok(())
}*/

// replays the negotiated format exactly, for port_enum_params(Format)
unsafe fn build_port_format_info(builder: &mut libspa::pod::builder::Builder, config: &PortConfig, id: u32) {

  let mut position = [0u32; 64];
  for (slot, &p) in position.iter_mut().zip(config.positions.iter()) {
    *slot = p;
  }

  let raw = spa_audio_info_raw {
    format:   config.format.0,
    flags:    config.flags,
    rate:     config.rate,
    channels: config.channels,
    position
  };

  spa_format_audio_raw_build(builder.as_raw_ptr(), id, &raw);
}

unsafe extern "C" fn port_enum_params(
  object:    *mut c_void,
  seq:       c_int,
  direction: spa_direction,
  port_id:   u32,
  id:        u32,
  start:     u32,
  max:       u32,
  filter:    *const spa_pod
) -> c_int
{
  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  if direction != SPA_DIRECTION_INPUT || (port_id as usize) >= MAX_PORTS {
    return -libc::EINVAL;
  }
  if max == 0 {
    return 0;
  }

  let mut buffer  = vec![];
  let mut fbuffer = vec![]; // spa_pod_filter output; kept apart from the source pod (see spa::filter_pod)

  let mut index = start;
  let mut count = 0;

  while count < max {

    use libspa::pod::builder::Builder;

    let mut builder = Builder::new(&mut buffer);

    #[allow(non_upper_case_globals)]
    match (id, index) {
      (SPA_PARAM_EnumFormat, i) => {
        if !crate::utils::build_enum_format_info(&mut builder, &state.caps, i).unwrap() {
          return 0;
        }
      },
      (SPA_PARAM_Format, 0) => {
        match state.ports[port_id as usize].config.as_ref() {
          Some(cfg) => build_port_format_info(&mut builder, cfg, SPA_PARAM_Format),
          None      => return -libc::ENOENT // no format negotiated yet
        }
      },
      (SPA_PARAM_Format, _)     => return 0,
      (SPA_PARAM_Buffers, 0) => {
        match state.ports[port_id as usize].config.as_ref() {
          Some(cfg) => crate::utils::build_buffers_info(&mut builder, cfg.stride()).unwrap(),
          None      => return -libc::ENOENT // format not negotiated yet
        }
      },
      (SPA_PARAM_Buffers, _)    => return 0,
      (SPA_PARAM_Latency, 0 | 1) => {
        let mut info = state.latency[index as usize];
        // the process latency shifts what we report upstream
        if info.direction == SPA_DIRECTION_INPUT {
          crate::utils::process_latency_info_add(&state.process_latency, &mut info);
        }
        crate::utils::build_latency_info(&mut builder, &info).unwrap()
      },
      (SPA_PARAM_Latency, _)     => return 0,
      _ => return -libc::EINVAL
    };

    drop(builder); // its borrow of `buffer` must end before we take the source pointer

    let mut result = spa_result_node_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if let Some(param) = crate::spa::filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter) {
      result.param = param;
      crate::spa::node_emit_result(&mut state.hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS, &result);
      count += 1;
    }

    index += 1;
  }

  0
}

unsafe extern "C" fn port_set_param(object: *mut c_void, direction: spa_direction, port_id: u32, id: u32, _flags: u32, param: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  if direction != SPA_DIRECTION_INPUT || (port_id as usize) >= MAX_PORTS {
    return -libc::EINVAL;
  }

  #[allow(non_upper_case_globals)]
  match id {
    SPA_PARAM_Format => {
      let mut res: c_int = 0;
      if !param.is_null() {
        use libspa::param::format::{MediaType, MediaSubtype};
        use libspa::param::format_utils::parse_format;

        match parse_format(libspa::pod::Pod::from_raw(param)) {
          Ok((MediaType::Audio, MediaSubtype::Raw)) => {
            let mut raw = MaybeUninit::<spa_audio_info_raw>::uninit();
            if spa_format_audio_raw_parse(param, raw.as_mut_ptr()) < 0 {
              crate::warn!(state.log, "spa_format_audio_raw_parse failed");
              return -libc::EINVAL;
            }

            let raw = raw.assume_init();

            //TODO: check whether format is supported by OSS

            // reject bad values rather than assert (an FFI panic aborts pipewire);
            // flags are stored but unused, OSS writes interleaved frames
            if raw.rate == 0 || raw.channels == 0 || raw.channels > SPA_AUDIO_MAX_CHANNELS {
              crate::warn!(state.log, "rejecting format: rate={} channels={}", raw.rate, raw.channels);
              return -libc::EINVAL;
            }

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
                return -libc::ENOTSUP;
              }
            };

            // the host renegotiates on a live node; swap the device and
            // config on the data loop
            let port_idx = port_id as usize;
            let state_ptr: *mut State = state;
            let res_ref = &mut res;
            crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {

              let port = &mut state.ports[port_idx];

              if !port.dsp.is_closed() {
                port.dsp.close();
              }

              // a busy or vanished device must fail negotiation, not abort
              if let Err(err) = port.dsp.open() {
                crate::warn!(state.log, "{}: open: {}", port.dsp.path, err);
                port.config = None;
                *res_ref = -(err as c_int);
                return;
              }

              // ditto for a device that won't take the format exactly
              if let Err(err) = port.dsp.configure(oss_format, config.channels, config.rate) {
                crate::warn!(state.log, "{}: device rejected {:?}: {}", port.dsp.path, config, err);
                port.dsp.close();
                port.config = None;
                *res_ref = -(err as c_int);
                return;
              }

              port.config = Some(config);
            });
          },
          Ok((t, st)) => {
            crate::warn!(state.log, "unknown media type combination: {:?}, {:?}", t, st);
            return -libc::ENOENT;
          },
          Err(err) => {
            crate::warn!(state.log, "parse_format failed: {}", err);
            return -libc::EINVAL
          }
        };
      } else {
        // releasing the format: close the device and drop the buffers (the
        // Suspend path may have closed the dsp already, hence the guard); all
        // three are read by process(), so do it from the data loop
        let port_idx = port_id as usize;
        let state_ptr: *mut State = state;
        crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
          let port = &mut state.ports[port_idx];
          if !port.dsp.is_closed() {
            port.dsp.close();
          }
          port.buffers.clear();
          port.config = None;
        });
      }

      // update the port rate and flip Format/Buffers flags to reflect whether a
      // format is negotiated, then re-emit so the host re-reads them (PipeWire
      // ALSA sink pattern)
      let _ = state.port_info.replace_change_mask(0);
      if let Some(cfg) = state.ports[port_id as usize].config.as_ref() {
        state.port_info.set_rate(spa_fraction { num: 1, denom: cfg.rate });
        state.port_info.set_param_flags(SPA_PARAM_Format,  SPA_PARAM_INFO_READWRITE);
        state.port_info.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
      } else {
        state.port_info.set_param_flags(SPA_PARAM_Format,  SPA_PARAM_INFO_WRITE);
        state.port_info.set_param_flags(SPA_PARAM_Buffers, 0);
      }
      // emit even on failure: the flags derive from the (now cleared) config
      emit_port_info(state);

      res
    },
    SPA_PARAM_Latency => {
      // the host writes the reverse-direction (here: downstream) latency;
      // store it and re-emit so it propagates through the graph
      let other = direction ^ 1;
      let info = if param.is_null() {
        crate::utils::latency_info_default(other)
      } else {
        match crate::utils::parse_latency_info(param) {
          Some(info) if info.direction == other => info,
          _ => return -libc::EINVAL
        }
      };
      state.latency[info.direction as usize] = info;

      let _ = state.port_info.replace_change_mask(0);
      state.port_info.bump_param(SPA_PARAM_Latency);
      emit_port_info(state);

      0
    },
    SPA_PARAM_Tag     => 0,
    id => {
      crate::warn!(state.log, "port_set_param: unknown param {}", id);
      -libc::ENOENT
    }
  }
}

unsafe extern "C" fn process(object: *mut c_void) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  // a cycle that was already signaled when we paused can still land here; drop
  // it instead of assert!()ing, which aborts the daemon across extern "C"
  if !state.started || state.position.is_null() {
    return SPA_STATUS_OK as i32;
  }

  state.old_timestamp = state.cur_timestamp;
  state.cur_timestamp = crate::utils::now_ns(&state.data_system);

  let mut result = SPA_STATUS_OK as i32;

  for port in &mut state.ports {

    if port.config.is_none() {
      continue;
    }

    let port_config = port.config.as_ref().unwrap();

    if port.buffers.is_empty() || port.io.is_null() {
      continue; // not (fully) negotiated yet
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
      crate::warn!(state.log, "{}: SPA_IO_CLOCK_FLAG_XRUN_RECOVER @ {}", port.dsp.path, state.cur_timestamp);
    }

    #[cfg(debug_assertions)]
    if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
      crate::trace!(state.log, "offset: {}, chunk size: {}", offset, size);
      spa_debug_mem(0, data_0.data.offset(offset as isize), 16.min(size) as usize);
    }

    let driver_clock = (*state.position).clock;

    // the resampler can legitimately hand us a few frames over a quantum; warn
    // rather than debug_assert!, which would abort the process (panic across the
    // extern "C" boundary). The write path below caps and drops the excess.
    #[cfg(debug_assertions)]
    if size > driver_clock.target_duration as u32 * port_config.stride() {
      crate::warn!(state.log, "{}: chunk size {} exceeds one quantum {}",
        port.dsp.path, size, driver_clock.target_duration as u32 * port_config.stride());
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

      let period_in_bytes = driver_clock.target_duration as u32 * port_config.stride();
      let desired_delay   = period_in_bytes / 8 * state.oss_delay;

      // Size the fill to the granted buffer and the device's real fragment. We
      // write about one quantum per cycle, so if the forced fragment is much
      // smaller than the quantum (snd_hdspe forces 256 frames) the ring drains
      // between writes and underruns; keep it near-full then, otherwise
      // half-full. "Near-full" still leaves headroom for a write that runs a few
      // frames over a quantum (the resampler's output size varies), or the OSS
      // write short-writes and drops those frames every cycle.
      let granted   = port.dsp.set_buffer_size(period_in_bytes * 2 + desired_delay);
      let blocksize = port.dsp.blocksize();
      // saturating arithmetic: blocksize/rate_match.size are device-provided and
      // an overflow here would abort the data loop.
      port.target_delay = if granted >= 2 * period_in_bytes {
        if blocksize > 0 && blocksize.saturating_mul(4) < period_in_bytes {
          // near-full, leaving headroom for the largest expected write (a quantum,
          // or the resampler's size if larger) plus one fragment. A chunk over
          // even that drops the excess: on a buffer this small no target both
          // avoids underruns and fits an arbitrary write.
          let rate_match_bytes = if state.rate_match.is_null() { 0 } else { (*state.rate_match).size.saturating_mul(port_config.stride()) };
          let write_max = period_in_bytes.max(rate_match_bytes);
          granted.saturating_sub(write_max.saturating_add(blocksize))
        } else {
          desired_delay.max(granted / 2).clamp(period_in_bytes, granted - period_in_bytes)
        }
      } else {
        granted / 2 // buffer too small for two quanta; best-effort, will drop (warned below)
      };

      port.setup_period  = period_in_bytes;
      port.bw_fast_until = state.cur_timestamp + DLL_FAST_NSEC;
      port.dll.init();
      port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, period_in_bytes, driver_clock.target_rate.denom * port_config.stride());

      crate::warn!(state.log, "{}: granted {}, blocksize {}, period {}, target delay {}",
        port.dsp.path, granted, blocksize, period_in_bytes, port.target_delay);
      if granted < 2 * period_in_bytes {
        crate::warn!(state.log, "{}: granted OSS buffer ({}) is smaller than two quanta ({}); \
          audio will glitch. Lower the PipeWire quantum; we set the fragment size \
          explicitly, so hw.snd.latency has no effect",
          port.dsp.path, granted, period_in_bytes * 2);
      }

      port.dsp.write_zeroes(port.target_delay);
    } else {
      let underrun_count = port.dsp.underruns();
      if underrun_count > 0 {
        crate::warn!(state.log, "{}: OSS reported {:3} underruns @ {}", port.dsp.path, underrun_count, state.cur_timestamp);
        if port.xrun_timestamp == 0 {
          port.xrun_timestamp = state.cur_timestamp;
        }

        // report it to the host (pw-top's xrun counter); the length isn't
        // known at detection time, so pass 0 delay
        let node_callbacks = state.callbacks.funcs.cast::<spa_node_callbacks>().as_ref();
        if let Some(xrun_fun) = node_callbacks.and_then(|c| c.xrun) {
          xrun_fun(state.callbacks.data, state.cur_timestamp / 1000, 0, std::ptr::null_mut());
        }
      }
    }

    let mut corr: f64 = 1.0; // DLL rate correction, published as clock.rate_diff below
    let nbytes = if port.xrun_timestamp != 0 {

      let period = driver_clock.target_duration * SPA_NSEC_PER_SEC as u64 / driver_clock.target_rate.denom as u64;
      let diff   = state.cur_timestamp - state.old_timestamp;

      // not sure if that does anything of value
      /*if !state.clock.is_null() {
        (*state.clock).xrun += diff;
      }*/

      // we are going to wait for the appropriate conditions to continue normal playback
      if driver_clock.nsec > port.xrun_timestamp && driver_clock.flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER == 0 &&
        diff >= period && diff < period + 1_000_000 /* ? */
      {
        port.xrun_timestamp = 0;

        let period_in_bytes = driver_clock.target_duration as u32 * port_config.stride();

        port.bw_fast_until = state.cur_timestamp + DLL_FAST_NSEC;
        port.dll.init();
        port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, period_in_bytes, driver_clock.target_rate.denom * port_config.stride());

        // buffer's already sized; re-prime only up to target, accounting for what's
        // still queued (a full target_delay would push odelay past the buffer)
        let odelay = port.dsp.odelay();
        let refill = port.target_delay.saturating_sub(odelay);

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
      // published; here the DLL only serves rate matching as a follower
      if state.following && port.setup_period != 0 {
        let stride = port_config.stride().max(1);
        maybe_relax_dll(port, port_config.rate, stride, state.cur_timestamp);
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = (port.dsp.odelay() as f64 - port.target_delay as f64).clamp(-max_err, max_err);
        corr = port.dll.update(err);

        #[cfg(debug_assertions)]
        eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, err);
      }

      port.dsp.write(data_0.data.offset(offset as isize), size)
    };

    // Rate-match only as a follower on a foreign clock: when driving, the
    // timer steering applies the correction, and a same-device follower ticks
    // from our clock so there is nothing to match (ALSA gates on the clock
    // name the same way).
    if !state.rate_match.is_null() {
      let matching = state.following && !crate::utils::same_clock(state.position, &state.clock_name);
      if matching {
        (*state.rate_match).flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
        (*state.rate_match).rate   = corr.clamp(0.99, 1.01);
      } else {
        (*state.rate_match).flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
        (*state.rate_match).rate   = 1.0;
      }
    }

    if nbytes < size as isize {
      crate::warn!(state.log, "{}: dropped {} bytes", port.dsp.path, if nbytes > 0 { size - nbytes as u32 } else { size });
    }

    (*port.io).status = SPA_STATUS_NEED_DATA as i32;

    // a sink has no output, so the return bit is NEED_DATA ("can accept input
    // next cycle"), matching the port io status, not HAVE_DATA.
    result |= SPA_STATUS_NEED_DATA as i32;
  }

  result
}

unsafe extern "C" fn port_use_buffers(object: *mut c_void, direction: spa_direction, port_id: u32, flags: u32, buffers: *mut *mut spa_buffer, n_buffers: u32) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  if direction != SPA_DIRECTION_INPUT || (port_id as usize) >= MAX_PORTS {
    return -libc::EINVAL;
  }
  let _ = flags;

  let new_buffers = if !buffers.is_null() && n_buffers > 0 {
    std::slice::from_raw_parts(buffers, n_buffers as usize).to_vec()
  } else {
    vec![]
  };

  // process() walks this vec on the data loop; swap it there
  let port_idx = port_id as usize;
  let state_ptr: *mut State = state;
  crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
    state.ports[port_idx].buffers = new_buffers;
  });

  0
}

unsafe extern "C" fn port_set_io(object: *mut c_void, direction: spa_direction, port_id: u32, id: u32, data: *mut c_void, _size: usize) -> c_int {

  if direction != SPA_DIRECTION_INPUT || (port_id as usize) >= MAX_PORTS {
    return -libc::EINVAL;
  }

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  #[allow(non_upper_case_globals)]
  match id {
    SPA_IO_Buffers => {
      state.ports[port_id as usize].io = data.cast();
      0
    },
    // you'd think that would be a node parameter instead
    SPA_IO_RateMatch => {
      // ACTIVE is managed per cycle in process(): only set while matching
      state.rate_match = data as *mut spa_io_rate_match;
      0
    },
    _ => -libc::ENOENT
  }
}

unsafe extern "C" fn port_reuse_buffer(_object: *mut c_void, _port_id: u32, _buffer_id: u32) -> c_int {
  -libc::ENOTSUP // buffers are recycled through io.buffer_id
}

const NODE_IMPL: spa_node_methods = spa_node_methods {
  version:           SPA_VERSION_NODE_METHODS,
  add_listener:      Some(add_listener),
  set_callbacks:     Some(set_callbacks),
  sync:              Some(sync),
  enum_params:       Some(enum_params),
  set_param:         Some(set_param),
  set_io:            Some(set_io),
  send_command:      Some(send_command),
  add_port:          Some(add_port),
  remove_port:       Some(remove_port),
  port_enum_params:  Some(port_enum_params),
  port_set_param:    Some(port_set_param),
  port_use_buffers:  Some(port_use_buffers),
  port_set_io:       Some(port_set_io),
  port_reuse_buffer: Some(port_reuse_buffer),
  process:           Some(process),
};

unsafe extern "C" fn get_interface(handle: *mut spa_handle, type_: *const c_char, interface: *mut *mut c_void) -> c_int {
  let state = handle.cast::<State>().as_mut()
    .expect("handle is not supposed to be null");
  assert!(!interface.is_null());
  if spa_streq(type_, SPA_TYPE_INTERFACE_Node.as_ptr().cast()) {
    *interface = &mut state.node as *mut _ as *mut c_void;
  } else {
    return -libc::ENOENT;
  }
  0
}

unsafe extern "C" fn clear(handle: *mut spa_handle) -> c_int {
  let state: *mut State = handle.cast();
  assert!(!state.is_null());

  // the data loop still holds the timer source; detach it there before the
  // state is freed, then close the timerfd
  crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
    state.data_loop.remove_source(&mut state.timer_source);
  });
  (*state).data_system.close((*state).timer_source.fd);

  std::ptr::drop_in_place(state);
  0
}

unsafe extern "C" fn get_size(_factory: *const spa_handle_factory, _params: *const spa_dict) -> usize {
  std::mem::size_of::<State>()
}

unsafe extern "C" fn init(
  _factory:  *const spa_handle_factory,
  handle:    *mut   spa_handle,
  info:      *const spa_dict,
  support:   *const spa_support,
  n_support: u32
) -> c_int
{
  let log = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast()) as *mut spa_log;
  let log = crate::spa::Log::wrap(log);

  let data_loop   = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_DataLoop  .as_ptr().cast()) as *mut spa_loop;
  let data_system = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_DataSystem.as_ptr().cast()) as *mut spa_system;

  if data_loop.is_null() || data_system.is_null() {
    return -libc::EINVAL;
  }

  let data_loop   = crate::spa::Loop  ::wrap(data_loop);
  let data_system = crate::spa::System::wrap(data_system);

  let timer_fd = data_system.timerfd_create(libc::CLOCK_MONOTONIC, (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as i32);
  assert!(timer_fd >= 0);

  let mut dsp_path  = None;
  let mut oss_delay = 10u32; // default fill target: 10/8 of a period

  if let Some(info) = info.as_ref() {
    #[cfg(debug_assertions)]
    crate::spa::dump_spa_dict(info);

    //TODO: would be better with an iterator
    crate::spa::for_each_dict_item(info, |key, value| {
      if key == crate::keys::OSS_DSP_PATH {
        dsp_path = Some(value.to_string());
      } else if key == crate::keys::OSS_DELAY {
        // per-device default, e.g. from a wireplumber node rule
        if let Ok(v) = value.parse::<u32>() {
          oss_delay = v.min(1024);
        }
      }
    });
  }

  let Some(dsp_path) = dsp_path else {
    crate::error!(log, "{} missing from the node properties", crate::keys::OSS_DSP_PATH);
    return -libc::EINVAL;
  };

  let caps = crate::sound::probe_caps(&dsp_path, true).unwrap_or_else(|| {
    crate::warn!(log, "{}: can't probe device caps; using fallback", dsp_path);
    crate::sound::DspCaps::fallback()
  });
  crate::debug!(log, "{}: {:?}", dsp_path, caps);

  let state = handle.cast::<State>().as_mut()
    .expect("handle is not supposed to be null");

  std::ptr::write(state, State {

    handle: spa_handle {
      version:       SPA_VERSION_HANDLE,
      get_interface: Some(get_interface),
      clear:         Some(clear)
    },

    node: spa_node {
      iface: spa_interface {
        type_:   SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
        version: SPA_VERSION_NODE,
        cb: spa_callbacks {
          funcs: &NODE_IMPL as *const _ as *const c_void,
          data:  state as *mut _ as *mut c_void
        }
      }
    },

    node_info: crate::spa::NodeInfo::new(),
    port_info: crate::spa::PortInfo::new(),

    data_loop,
    data_system,
    log,

    clock:      std::ptr::null_mut(),
    position:   std::ptr::null_mut(),
    rate_match: std::ptr::null_mut(),
    clock_name: std::ffi::CString::new(format!("freebsd-oss.{}", dsp_path.trim_start_matches("/dev/"))).unwrap_or_default(),

    timer_source: spa_source {
      loop_: std::ptr::null_mut(),
      func:  Some(on_timeout),
      data:  state as *mut _ as *mut c_void,
      fd:    timer_fd,
      mask:  SPA_IO_IN,
      rmask: 0,
      priv_: std::ptr::null_mut()
    },

    next_time: 0,

    hooks: spa_hook_list {
      list: spa_list {
        next: std::ptr::null_mut(),
        prev: std::ptr::null_mut()
      }
    },

    callbacks: spa_callbacks {
      funcs: std::ptr::null(),
      data:  std::ptr::null_mut()
    },

    ports: [
      Port {
        config:         None,
        buffers:        vec![],
        io:             std::ptr::null_mut(),
        dsp:            crate::sound::DspWriter::new(&dsp_path),
        xrun_timestamp: 0,
        dll:            std::default::Default::default(),
        target_delay:   0,
        setup_period:   0,
        bw_fast_until:  0
      };
      MAX_PORTS
    ],

    caps,

    latency: [
      crate::utils::latency_info_default(SPA_DIRECTION_INPUT),
      crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT)
    ],

    process_latency: crate::utils::process_latency_default(),

    started:   false,
    following: false,

    cur_timestamp: 0,
    old_timestamp: 0,

    oss_delay,
    oss_delay_default: oss_delay
  });

  state.node_info.fix_pointers();

  state.node_info.set_max_input_ports(1);
  state.node_info.set_flags(SPA_NODE_FLAG_RT as u64); // ?

  state.node_info.add_prop(SPA_KEY_MEDIA_CLASS.as_ptr(), "Audio/Sink");
  state.node_info.add_prop(SPA_KEY_NODE_DRIVER.as_ptr(), "true");

  //state.node_info.add_param(SPA_PARAM_IO,             SPA_PARAM_INFO_READ);
  //state.node_info.add_param(SPA_PARAM_EnumFormat,     SPA_PARAM_INFO_READ);
  //state.node_info.add_param(SPA_PARAM_EnumPortConfig, SPA_PARAM_INFO_READ);
  //state.node_info.add_param(SPA_PARAM_PortConfig,     SPA_PARAM_INFO_READ);
  state.node_info.add_param(SPA_PARAM_PropInfo,       SPA_PARAM_INFO_READ);
  state.node_info.add_param(SPA_PARAM_Props,          SPA_PARAM_INFO_READWRITE);
  state.node_info.add_param(SPA_PARAM_ProcessLatency, SPA_PARAM_INFO_READWRITE);

  state.port_info.fix_pointers();

  state.port_info.set_flags((SPA_PORT_FLAG_PHYSICAL | SPA_PORT_FLAG_TERMINAL) as u64);
  state.port_info.set_rate(spa_fraction { num: 1, denom: 48000 }); // ?

  // advertise the format as writable so the host (re)negotiates it; Buffers is
  // unreadable until a format is set (it needs the stride). Flags flip in
  // port_set_param.
  state.port_info.add_param(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
  state.port_info.add_param(SPA_PARAM_Format,     SPA_PARAM_INFO_WRITE);
  state.port_info.add_param(SPA_PARAM_Buffers,    0);
  state.port_info.add_param(SPA_PARAM_Latency,    SPA_PARAM_INFO_READWRITE);

  //state.port_info.add_param(SPA_PARAM_PortConfig, SPA_PARAM_INFO_READWRITE);
  //state.port_info.add_param(SPA_PARAM_IO,         SPA_PARAM_INFO_READ);
  //state.port_info.add_param(SPA_PARAM_Buffers,    SPA_PARAM_INFO_WRITE); // ?

  spa_hook_list_init(&mut state.hooks);

  let err = state.data_loop.add_source(&mut state.timer_source);
  assert!(err >= 0);

  0
}

const INTERFACE_INFO: [spa_interface_info; 1] = [
  spa_interface_info {
    type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast()
  }
];

unsafe extern "C" fn enum_interface_info(_factory: *const spa_handle_factory, info: *mut *const spa_interface_info, index: *mut u32) -> c_int {
  assert!(!info .is_null());
  assert!(!index.is_null());
  match *index {
    0 => { *info = &INTERFACE_INFO[0]; *index += 1; 1 }
    _ => 0
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
  get_size:            Some(get_size),
  init:                Some(init),
  enum_interface_info: Some(enum_interface_info)
};
