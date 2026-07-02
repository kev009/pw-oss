use std::mem::MaybeUninit;
use std::os::raw::{c_char, c_int, c_void};

use libspa::sys::*;

const MAX_PORTS: usize = 1;
// several State fields are per-port in disguise (rate_match, active_buffers,
// the single PortInfo); fix those before raising this
const _: () = assert!(MAX_PORTS == 1);
const EMPTY_CYCLE: isize = -1; // no data queued this cycle (scheduling jitter)

#[repr(C)]
struct State {
  handle:         spa_handle,
  node:           spa_node,
  node_info:      crate::spa::NodeInfo,
  port_info:      crate::spa::PortInfo,
  data_loop:      crate::spa::Loop,
  data_system:    crate::spa::System,
  log:            crate::spa::Log,
  clock:          *mut spa_io_clock,
  position:       *mut spa_io_position,
  rate_match:     *mut spa_io_rate_match,
  clock_name:     std::ffi::CString, // stamped into spa_io_clock.name
  main_loop:      Option<crate::spa::Loop>, // for deferring device rebuilds off the data loop
  dsp_path:       String,
  timer_source:   spa_source,
  next_time:      u64,
  hooks:          spa_hook_list,
  callbacks:      spa_callbacks,
  ports:          [Port; MAX_PORTS],
  caps:           crate::sound::DspCaps,
  caps_fallback: bool, // init-time probe failed (busy device); re-probe lazily
  loop_thread:   std::sync::atomic::AtomicUsize, // thread process()/on_timeout run on (0 = unseen)
  latency:        [spa_latency_info; 2], // indexed by direction; written by the host, replayed on read
  process_latency: spa_process_latency_info,
  started:        bool,
  clearing:       bool, // teardown in progress; queued tasks must no-op
  following:      bool,
  active_buffers: usize
}

impl State {

  fn node_is_follower(&self) -> bool {
    !self.clock.is_null() && !self.position.is_null() && unsafe { (*self.position).clock.id != (*self.clock).id }
  }
}

struct Port {
  config:        Option<PortConfig>,
  buffers:       Vec<*mut spa_buffer>,
  io:            *mut spa_io_buffers,
  dsp:           crate::sound::Dsp,
  dll:           crate::dll::SpaDLL,
  primed:        bool,
  setup_period:  u32, // device bytes per graph cycle the servo was tuned for
  bw_fast_until: u64, // while nonzero, the DLL runs at BW_MAX for a fast lock
  resetup_pending: bool, // a main-thread device rebuild is queued; skip cycles
  was_matching:  bool, // rate matching active last cycle (relock on transition)
  warn_limit:    crate::utils::RateLimit
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
      port_info_fun(entry.cb.data, SPA_DIRECTION_OUTPUT, 0, state.port_info.raw());
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
// set, e.g. PARAMS), then clear the mask
unsafe fn emit_port_info(state: &mut State) {
  crate::spa::for_each_hook(&mut state.hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_node_events>().as_ref()
      .expect("hook should be initialized");
    if f.version >= SPA_VERSION_NODE_EVENTS {
      if let Some(port_info_fun) = f.port_info {
        port_info_fun(entry.cb.data, SPA_DIRECTION_OUTPUT, 0, state.port_info.raw());
      }
    }
  });
  let _ = state.port_info.replace_change_mask(0);
}

unsafe extern "C" fn set_callbacks(object: *mut c_void, callbacks: *const spa_node_callbacks, data: *mut c_void) -> c_int {
  let state: *mut State = object.cast();
  assert!(!state.is_null());
  // read by on_timeout/process on the data loop
  if !crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
    state.callbacks.funcs = callbacks as *const c_void;
    state.callbacks.data  = data;
  }) {
    return -libc::EIO;
  }
  0
}

unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {
  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");
  crate::spa::node_emit_done(&mut state.hooks, seq);
  0
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
      (SPA_PARAM_PropInfo, _)       => return 0,
      (SPA_PARAM_Props, 0)          => crate::utils::build_latency_offset_props(&mut builder, state.process_latency.ns, None).unwrap(),
      (SPA_PARAM_Props, _)          => return 0,
      (SPA_PARAM_ProcessLatency, 0) => crate::utils::build_process_latency_info(&mut builder, &state.process_latency).unwrap(),
      (SPA_PARAM_ProcessLatency, _) => return 0,
      _ => return -libc::ENOENT // unknown param id (ALSA convention)
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
              SPA_PROP_params         => (), // ditto
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

// two-stage DLL bandwidth (see sink.rs: a simplification of ALSA's
// variance-driven adaptation): fast lock after (re)start, then steady state
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

  if !check_loop_identity(state) {
    return; // poisoned: leave the timer disarmed
  }

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

  let duration = (*state.position).clock.target_duration;
  let rate     = (*state.position).clock.target_rate.denom;
  if duration == 0 || rate == 0 {
    // malformed position: idle-tick, and advance next_time so the deadline
    // isn't stale when the position recovers
    state.next_time = nsec + SPA_NSEC_PER_SEC as u64 / 100;
    set_timeout(state, state.next_time);
    return;
  }

  // Run the servo before the clock is published so every field below belongs
  // to this cycle (the shape of ALSA's update_time). The pre-read fill level
  // here and process()'s post-drain accounting see the same signal: we drain
  // the ring every cycle, so what's queued is one period's accumulation.
  let mut corr:  f64 = 1.0;
  let mut delay: i64 = 0;
  for port in &mut state.ports {
    let Some(cfg) = port.config.as_ref() else { continue };
    let stride      = cfg.stride.max(1);
    let device_rate = cfg.rate.max(1);
    if !port.dsp.is_running() || !port.primed || port.setup_period == 0 || port.resetup_pending {
      continue;
    }

    let queued = port.dsp.ispace_in_bytes().max(0) as u32;
    // device frames scale to the graph rate; the resampler queue is already
    // graph-side (matching the sink's publication)
    let resamp = if state.rate_match.is_null() { 0 } else { (*state.rate_match).delay as i64 };
    delay = (queued / stride) as i64 * rate as i64 / device_rate as i64 + resamp;

    maybe_relax_dll(port, device_rate, stride, nsec);

    // capture error is inverted vs the sink: a slow device queues less than a
    // period; clamp so wakeup jitter can't wind up the integrator
    let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
    let err = (port.setup_period as f64 - queued as f64).clamp(-max_err, max_err);
    corr = port.dll.update(err);

    // a diverged servo must not wedge the graph clock
    if !(0.5..=2.0).contains(&corr) {
      crate::warn!(state.log, "capture DLL diverged (corr {}); relocking", corr);
      port.dll.init();
      port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, port.setup_period, device_rate * stride);
      port.bw_fast_until = nsec + DLL_FAST_NSEC;
      corr = 1.0;
    }

    #[cfg(debug_assertions)]
    eprintln!("capture: corr = {}, queued = {}", corr, queued);
  }

  // steer the timer by the correction so the published clock genuinely follows
  // the device (ALSA warps next_time the same way)
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
    // a capture driver signals HAVE_DATA (alsa-pcm.c capture_ready); the
    // NEED_DATA form is for playback drivers
    let err = ready_fun(state.callbacks.data, SPA_STATUS_HAVE_DATA as i32);
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

  let mut now = timespec { tv_sec: 0, tv_nsec: 0 };
  let err = state.data_system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now);
  assert!(err >= 0);

  state.next_time = (now.tv_sec * SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64;

  if state.started && !state.following && !state.position.is_null() {
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
  let applied = crate::utils::block_on_loop(&(*state).data_loop, state, |state| {

    let was_armed = !state.clock.is_null() && !state.position.is_null();

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
      let armed     = !state.clock.is_null() && !state.position.is_null();
      let following = state.node_is_follower();
      let flipped   = state.following != following;
      if flipped {
        state.following = following;
      }
      // rearm/park only on a real transition (io presence or role); resetting
      // the timer phase on every re-point causes cycle bunching
      if flipped || was_armed != armed {
        update_timers(state);
      }
    }
  });
  if !applied {
    return -libc::EIO;
  }

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
      if !crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
        // sane clock delay/rate_diff until process() publishes measured values
        if !state.clock.is_null() {
          (*state.clock).delay     = 0;
          (*state.clock).rate_diff = 1.0;
        }
        // the device kept capturing across a Pause; re-prime so the first
        // cycles deliver fresh audio at a known fill, not the paused backlog
        for port in &mut state.ports {
          port.primed        = false;
          port.bw_fast_until = 0;
          port.dll.init();
        }
        state.started   = true;
        state.following = state.node_is_follower();
        update_timers(state);
      }) {
        return -libc::EIO;
      }
      0
    },
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => {
      let state: *mut State = state;
      if !crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
        state.started = false;
        update_timers(state);
      }) {
        return -libc::EIO;
      }
      0
    },
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => {
      // swap the devices out on the loop and close them here: HALT+close can
      // sleep (chn_abort), which would stall every node on the data loop
      let mut retired: [Option<crate::sound::Dsp>; MAX_PORTS] = std::default::Default::default();
      // pre-built here: constructing them in the closure would allocate on
      // the RT loop
      let mut replacements: [Option<crate::sound::Dsp>; MAX_PORTS] =
        std::array::from_fn(|_| Some(crate::sound::Dsp::new(&state.dsp_path)));
      {
        let retired_ref = &mut retired;
        let replacements_ref = &mut replacements;
        let state: *mut State = state;
        if !crate::utils::block_on_loop(&(*state).data_loop, state, move |state| {
          for (i, port) in state.ports.iter_mut().enumerate() {
            if !port.dsp.is_closed() {
              let closed = replacements_ref[i].take().expect("replacement is pre-built");
              retired_ref[i] = Some(std::mem::replace(&mut port.dsp, closed));
            }
          }
          state.started = false;
          update_timers(state);
        }) {
          return -libc::EIO;
        }
      }
      for old in retired.iter_mut() {
        if let Some(dsp) = old.as_mut() {
          if !dsp.is_closed() {
            dsp.close(); // off the RT path
          }
        }
      }
      0
    },
    (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin | SPA_NODE_COMMAND_ParamEnd) => 0, // we don't care
    (cmd_type, cmd_id) => {
      crate::warn!(state.log, "oss-source: unknown command: {}, {}", cmd_type, cmd_id);
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
  builder.add_id(libspa::utils::Id(SPA_DIRECTION_OUTPUT))?;

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

  if direction != SPA_DIRECTION_OUTPUT || (port_id as usize) >= MAX_PORTS {
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
        if state.caps_fallback {
          // the init-time probe hit a busy device and baked in fallback
          // caps; retry now (main thread, transient open)
          if let Some(caps) = crate::sound::probe_caps(&state.dsp_path.clone(), false) {
            crate::info!(state.log, "re-probed caps: {:?}", caps);
            state.caps = caps;
            state.caps_fallback = false;
          }
        }
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
      (SPA_PARAM_Format, _) => return 0,
      (SPA_PARAM_Buffers, 0) => {
        match state.ports[port_id as usize].config.as_ref() {
          Some(cfg) => crate::utils::build_buffers_info(&mut builder, cfg.stride).unwrap(),
          None      => return -libc::ENOENT // format not negotiated yet
        }
      },
      (SPA_PARAM_Buffers, _) => return 0,
      (SPA_PARAM_Latency, 0 | 1) => {
        let mut info = state.latency[index as usize];
        // the process latency shifts what we report downstream
        if info.direction == SPA_DIRECTION_OUTPUT {
          crate::utils::process_latency_info_add(&state.process_latency, &mut info);
        }
        crate::utils::build_latency_info(&mut builder, &info).unwrap()
      },
      (SPA_PARAM_Latency, _)     => return 0,
      _ => return -libc::ENOENT // unknown param id (ALSA convention)
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

  if direction != SPA_DIRECTION_OUTPUT || (port_id as usize) >= MAX_PORTS {
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
            // flags are accepted but ignored
            if raw.rate == 0 || raw.channels == 0 || raw.channels > SPA_AUDIO_MAX_CHANNELS {
              crate::warn!(state.log, "rejecting format: rate={} channels={}", raw.rate, raw.channels);
              return -libc::EINVAL;
            }

            let format = libspa::param::audio::AudioFormat(raw.format);

            // only formats from our EnumFormat are expected; reject the rest
            let (oss_format, bytes_per_sample) = match format {
              libspa::param::audio::AudioFormat::S32LE => (crate::sound::AFMT_S32_LE, 4),
              libspa::param::audio::AudioFormat::S32BE => (crate::sound::AFMT_S32_BE, 4),
              libspa::param::audio::AudioFormat::S16LE => (crate::sound::AFMT_S16_LE, 2),
              libspa::param::audio::AudioFormat::S16BE => (crate::sound::AFMT_S16_BE, 2),
              _ => {
                crate::warn!(state.log, "rejecting unsupported format {:?}", format);
                return -libc::ENOTSUP;
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
            res = install_device(state, port_id as usize, config);
            if res == -libc::EINVAL || res == -libc::ENOTSUP {
              // the device rejected caps-derived values: the snapshot may be
              // stale (vchans/bitperfect toggled at runtime); re-probe and
              // re-announce EnumFormat so the host renegotiates from reality
              if let Some(caps) = crate::sound::probe_caps(&state.dsp_path.clone(), false) {
                crate::info!(state.log, "re-probed caps after rejection: {:?}", caps);
                state.caps = caps;
                state.caps_fallback = false;
                state.port_info.bump_param(SPA_PARAM_EnumFormat);
              }
            }
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
        if !crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
          let port = &mut state.ports[port_idx];
          if !port.dsp.is_closed() {
            port.dsp.close();
          }
          port.buffers.clear();
          port.config = None;
        }) {
          return -libc::EIO; // the loop still holds the buffers; freeing them would dangle
        }
      }

      // update the port rate and flip Format/Buffers flags to reflect whether a
      // format is negotiated, then re-emit so the host re-reads them (PipeWire
      // ALSA source pattern)
      let _ = state.port_info.replace_change_mask(0);
      if let Some(cfg) = state.ports[port_id as usize].config.as_ref() {
        state.port_info.set_rate(spa_fraction { num: 1, denom: cfg.rate });
        state.port_info.set_param_flags(SPA_PARAM_Format,  SPA_PARAM_INFO_READWRITE);
        state.port_info.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
      } else {
        state.port_info.set_param_flags(SPA_PARAM_Format,  SPA_PARAM_INFO_WRITE);
        state.port_info.set_param_flags(SPA_PARAM_Buffers, 0);
      }
      emit_port_info(state);

      res
    },
    SPA_PARAM_Latency => {
      // the host writes the reverse-direction (here: upstream) latency;
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

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(dsp: &mut crate::sound::Dsp, config: &PortConfig, log: &crate::spa::Log) -> c_int {
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
  dsp.set_small_fragments();
  0
}

// see the sink's install_device: main-thread open/configure, loop-side swap,
// EBUSY falls back to retiring the old device first (exclusive devices)
unsafe fn install_device(state: &mut State, port_idx: usize, config: PortConfig) -> c_int {

  let mut new_dsp = crate::sound::Dsp::new(&state.dsp_path);
  let mut res = try_open_configure(&mut new_dsp, &config, &state.log);

  if res == -libc::EBUSY {
    let mut retired = None;
    {
      let retired_ref = &mut retired;
      let closed = crate::sound::Dsp::new(&state.dsp_path);
      let state_ptr: *mut State = state;
      if !crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
        *retired_ref = Some(std::mem::replace(&mut state.ports[port_idx].dsp, closed));
        // a cycle landing in this window must skip, not queue a rebuild of
        // the device we are about to install (cleared by the final swap)
        state.ports[port_idx].resetup_pending = true;
      }) {
        return -libc::EIO;
      }
    }
    drop(retired); // closes the old fd here, off the RT path
    res = try_open_configure(&mut new_dsp, &config, &state.log);
  }

  let ok = res == 0;
  let mut old_dsp = None;
  let swapped = {
    let old_ref = &mut old_dsp;
    let state_ptr: *mut State = state;
    crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
      let port = &mut state.ports[port_idx];
      // new_dsp is a closed reader when negotiation failed above
      *old_ref = Some(std::mem::replace(&mut port.dsp, new_dsp));
      port.config = if ok { Some(config) } else { None };
      port.dll.init(); // fresh device, fresh servo
      port.primed          = false;
      port.resetup_pending = false;
      port.was_matching    = false; // force a relock when matching resumes
      state.active_buffers = 0;
    })
  };
  drop(old_dsp); // ditto

  if !swapped {
    return -libc::EIO; // the swap never ran; the port keeps its old state
  }
  res
}

// A device rebuild the HOST didn't initiate just failed and cleared the
// config: flip the param flags and re-emit port info so the session manager
// renegotiates, instead of stranding a silently dead node (port_set_param
// does the same for host-initiated failures).
unsafe fn emit_format_lost(state: &mut State) {
  let _ = state.port_info.replace_change_mask(0);
  state.port_info.set_param_flags(SPA_PARAM_Format,  SPA_PARAM_INFO_WRITE);
  state.port_info.set_param_flags(SPA_PARAM_Buffers, 0);
  emit_port_info(state);
}

// runs on the main thread (queued from the data loop via invoke_on_loop)
unsafe fn resetup_task(state: &mut State, port_idx: usize) {
  if state.clearing {
    return; // teardown is flushing us out; don't touch the device
  }
  // consume-or-bail: an intervening install_device (renegotiation) already
  // cleared the flag, making this task stale
  let mut still_pending = false;
  {
    let pending_ref = &mut still_pending;
    let state_ptr: *mut State = state;
    crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
      *pending_ref = state.ports[port_idx].resetup_pending;
    });
  }
  if !still_pending {
    return;
  }
  // config only mutates from main-thread calls, which are serialized with us
  match state.ports[port_idx].config.clone() {
    Some(config) => {
      if install_device(state, port_idx, config) != 0 {
        emit_format_lost(state);
      }
    },
    None => {
      let state_ptr: *mut State = state;
      crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
        state.ports[port_idx].resetup_pending = false;
      });
    }
  }
}

// see the sink's check_loop_identity: detect a divergent multi-data-loop
// assignment and refuse to process rather than corrupt loop-owned state
unsafe fn check_loop_identity(state: &mut State) -> bool {
  use std::sync::atomic::Ordering;
  let tid = libc::pthread_self() as usize;
  let seen = match state.loop_thread.compare_exchange(0, tid, Ordering::Relaxed, Ordering::Relaxed) {
    Ok(_) => tid,
    Err(seen) => seen
  };
  if seen == tid {
    return true;
  }
  if seen != usize::MAX && state.loop_thread.swap(usize::MAX, Ordering::Relaxed) != usize::MAX {
    crate::warn!(state.log, "process() and our data loop run on different threads \
      (multi-data-loop config?); pin node.loop.name for this node. Disabling processing.");
  }
  false
}

unsafe extern "C" fn process(object: *mut c_void) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  // mirror the sink: a paused or position-less cycle is dropped, not asserted
  if !state.started || state.position.is_null() {
    return SPA_STATUS_OK as i32;
  }

  if !check_loop_identity(state) {
    return SPA_STATUS_OK as i32;
  }

  let mut result = SPA_STATUS_OK as i32;
  let state_ptr: *mut State = state;

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
        crate::utils::invoke_on_loop(main_loop, state_ptr, move |state| resetup_task(state, port_idx)));
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
      let idx = state.active_buffers;
      state.active_buffers += 1;
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
    if port.primed && port.setup_period != 0 && period_in_bytes != 0 && period_in_bytes != port.setup_period {
      port.setup_period  = period_in_bytes;
      port.bw_fast_until = crate::utils::now_ns(&state.data_system) + DLL_FAST_NSEC;
      port.dll.init();
      port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, period_in_bytes, rate * stride);
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
    } else if !port.primed && period_in_bytes > 0 {
      // Capture analogue of the sink's zero priming: trigger the device,
      // discard any backlog so the fill level starts out known, and hand the
      // graph one period of silence while the ring fills. Don't wait for real
      // data: an empty first cycle reads as a missed deadline to the graph.
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
      port.primed        = true;
      port.setup_period  = period_in_bytes;
      port.bw_fast_until = crate::utils::now_ns(&state.data_system) + DLL_FAST_NSEC;
      port.dll.init();
      port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, period_in_bytes, rate * stride);

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
          port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, port.setup_period, rate * stride);
          port.bw_fast_until = now + DLL_FAST_NSEC;
        }
        maybe_relax_dll(port, rate, stride, now);
        // capture error is inverted vs the sink: a slow device queues less
        let err_raw = period_in_bytes as f64 - queued as f64;
        if err_raw.abs() > port.setup_period as f64 {
          // fill snap (see the sink): a level error past one period would
          // wind the integrator against the +/-1% clamp; the bounded read
          // above drains genuine backlog, so just relock here
          port.dll.init();
          port.dll.set_bw(crate::dll::SPA_DLL_BW_MAX, port.setup_period, rate * stride);
          port.bw_fast_until = now + DLL_FAST_NSEC;
        } else {
          let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
          corr = port.dll.update(err_raw.clamp(-max_err, max_err));
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
        port.setup_period.saturating_add(queued.saturating_sub(port.setup_period.saturating_mul(2)))
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
      port.primed        = false;
      port.bw_fast_until = 0;
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

unsafe extern "C" fn port_use_buffers(object: *mut c_void, direction: spa_direction, port_id: u32, flags: u32, buffers: *mut *mut spa_buffer, n_buffers: u32) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  if direction != SPA_DIRECTION_OUTPUT || (port_id as usize) >= MAX_PORTS {
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
  if !crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
    state.ports[port_idx].buffers = new_buffers;
    state.active_buffers = 0;
  }) {
    return -libc::EIO; // keeping stale host buffer pointers would be a UAF
  }

  0
}

unsafe extern "C" fn port_set_io(object: *mut c_void, direction: spa_direction, port_id: u32, id: u32, data: *mut c_void, _size: usize) -> c_int {

  if direction != SPA_DIRECTION_OUTPUT || (port_id as usize) >= MAX_PORTS {
    return -libc::EINVAL;
  }

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  #[allow(non_upper_case_globals)]
  match id {
    SPA_IO_Buffers | SPA_IO_RateMatch => (),
    _ => return -libc::ENOENT
  }

  // these pointers are dereferenced by process() on the data loop
  let state: *mut State = state;
  let applied = crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
    #[allow(non_upper_case_globals)]
    match id {
      SPA_IO_Buffers   => state.ports[port_id as usize].io = data.cast(), // null clears
      // you'd think RateMatch would be a node parameter instead; ACTIVE is
      // managed per cycle in process(), only set while matching
      SPA_IO_RateMatch => state.rate_match = data as *mut spa_io_rate_match,
      _ => ()
    }
  });
  if !applied {
    return -libc::EIO;
  }

  0
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

  // A queued resetup_task holds this state pointer; a blocking self-invoke
  // on the main loop flushes all pending queue items (in submission order)
  // before we free anything, and `clearing` makes the flushed tasks no-op.
  (*state).clearing = true;
  if let Some(main_loop) = (*state).main_loop.as_ref() {
    crate::utils::block_on_loop(main_loop, state, |_| {});
  }

  // the data loop still holds the timer source; detach it there before the
  // state is freed, then close the timerfd
  if !crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
    state.data_loop.remove_source(&mut state.timer_source);
  }) {
    // freeing the state now would leave the loop a dangling source; a clean
    // abort beats a use-after-free on the next timer tick
    eprintln!("freebsd-oss: can't detach the timer source; aborting");
    std::process::abort();
  }
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
  let main_loop   = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop      .as_ptr().cast()) as *mut spa_loop;

  if data_loop.is_null() || data_system.is_null() {
    return -libc::EINVAL;
  }

  let data_loop   = crate::spa::Loop  ::wrap(data_loop);
  let data_system = crate::spa::System::wrap(data_system);

  let timer_fd = data_system.timerfd_create(libc::CLOCK_MONOTONIC, (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as i32);
  if timer_fd < 0 {
    return timer_fd; // fd exhaustion fails node creation, not the daemon
  }

  let mut dsp_path = None;

  if let Some(info) = info.as_ref() {
    #[cfg(debug_assertions)]
    crate::spa::dump_spa_dict(info);

    //TODO: would be better with an iterator
    crate::spa::for_each_dict_item(info, |key, value| {
      if key == crate::keys::OSS_DSP_PATH {
        dsp_path = Some(value.to_string());
      }
    });
  }

  let Some(dsp_path) = dsp_path else {
    data_system.close(timer_fd);
    crate::error!(log, "{} missing from the node properties", crate::keys::OSS_DSP_PATH);
    return -libc::EINVAL;
  };

  let mut caps_fallback = false;
  let caps = crate::sound::probe_caps(&dsp_path, false).unwrap_or_else(|| {
    crate::warn!(log, "{}: can't probe device caps; using fallback", dsp_path);
    caps_fallback = true;
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
    main_loop:  if main_loop.is_null() { None } else { Some(crate::spa::Loop::wrap(main_loop)) },
    dsp_path:   dsp_path.clone(),

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

    ports: [Port { config: None, buffers: vec![], io: std::ptr::null_mut(), dsp: crate::sound::Dsp::new(&dsp_path), dll: std::default::Default::default(), primed: false, setup_period: 0, bw_fast_until: 0, resetup_pending: false, was_matching: false, warn_limit: crate::utils::RateLimit::new() }; MAX_PORTS],

    caps,
    caps_fallback,
    loop_thread: std::sync::atomic::AtomicUsize::new(0),

    latency: [
      crate::utils::latency_info_default(SPA_DIRECTION_INPUT),
      crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT)
    ],

    process_latency: crate::utils::process_latency_default(),

    started:    false,
    clearing:   false,
    following: false,

    active_buffers: 0
  });

  state.node_info.fix_pointers();

  state.node_info.set_max_output_ports(1);
  state.node_info.set_flags(SPA_NODE_FLAG_RT as u64);

  state.node_info.add_prop(SPA_KEY_MEDIA_CLASS.as_ptr(), "Audio/Source");
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

  spa_hook_list_init(&mut state.hooks);

  let err = state.data_loop.add_source(&mut state.timer_source);
  if err < 0 {
    state.data_system.close(state.timer_source.fd);
    return err;
  }

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

const OSS_SOURCE_FACTORY_INFO: spa_dict = spa_dict {
  flags:   0,
  n_items: 0,
  items:   std::ptr::null()
};

pub const OSS_SOURCE_FACTORY: spa_handle_factory = spa_handle_factory {
  version:             SPA_VERSION_HANDLE_FACTORY,
  name:                c"freebsd-oss.source".as_ptr(),
  info:                &OSS_SOURCE_FACTORY_INFO,
  get_size:            Some(get_size),
  init:                Some(init),
  enum_interface_info: Some(enum_interface_info)
};
