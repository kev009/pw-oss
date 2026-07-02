use std::mem::MaybeUninit;
use std::os::raw::{c_char, c_int, c_void};

use libspa::sys::*;

const MAX_PORTS: usize = 1;

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
  timer_source:   spa_source,
  next_time:      u64,
  hooks:          spa_hook_list,
  callbacks:      spa_callbacks,
  ports:          [Port; MAX_PORTS],
  caps:           crate::sound::DspCaps,
  latency:        [spa_latency_info; 2], // indexed by direction; written by the host, replayed on read
  process_latency: spa_process_latency_info,
  started:        bool,
  following:      bool,
  active_buffers: usize
}

impl State {

  fn node_is_follower(&self) -> bool {
    !self.clock.is_null() && !self.position.is_null() && unsafe { (*self.position).clock.id != (*self.clock).id }
  }
}

struct Port {
  config:  Option<PortConfig>,
  buffers: Vec<*mut spa_buffer>,
  io:      *mut spa_io_buffers,
  dsp:     crate::sound::Dsp,
  dll:     crate::dll::SpaDLL,
  primed:  bool
}

#[derive(Debug)]
pub struct PortConfig {
  #[allow(dead_code)] // only read by Debug until the Format readback lands
  pub format:    libspa::param::audio::AudioFormat,
  pub rate:      u32,
  pub channels:  u32,
  // pub positions: Vec<u32>, // currently unused; consumer is commented out below
  pub stride:    u32
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
  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");
  state.callbacks.funcs = callbacks as *const c_void;
  state.callbacks.data  = data;
  0
}

#[allow(unused_variables)]
unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {
  unimplemented!()
}

unsafe extern "C" fn enum_params(object: *mut c_void, seq: c_int, id: u32, start: u32, max: u32, filter: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  assert_ne!(max, 0);

  let mut buffer = vec![];

  let mut index = start;
  let mut count = 0;

  while count < max {

    use libspa::pod::builder::Builder;

    let mut builder = Builder::new(&mut buffer);

    #[allow(non_upper_case_globals)]
    match (id, index) {
      (SPA_PARAM_PropInfo, 0)       => crate::utils::build_latency_offset_prop_info(&mut builder).unwrap(),
      (SPA_PARAM_PropInfo, _)       => return 0,
      (SPA_PARAM_Props, 0)          => crate::utils::build_latency_offset_props(&mut builder, state.process_latency.ns).unwrap(),
      (SPA_PARAM_Props, _)          => return 0,
      (SPA_PARAM_ProcessLatency, 0) => crate::utils::build_process_latency_info(&mut builder, &state.process_latency).unwrap(),
      (SPA_PARAM_ProcessLatency, _) => return 0,
      _ => return -libc::EINVAL
    };

    let mut result = spa_result_node_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if spa_pod_filter(builder.as_raw_ptr(), &mut result.param, buffer.as_mut_ptr() as *mut spa_pod, filter) >= 0 {
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
      assert!(!param.is_null());
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
              _ => unimplemented!()
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
    _ => unimplemented!()
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

  let nsec = state.next_time;

  assert!(!state.position.is_null());

  let duration = (*state.position).clock.target_duration;
  let rate     = (*state.position).clock.target_rate.denom;

  crate::trace!(state.log, "duration = {}, rate = {}", duration, rate);

  state.next_time = nsec + duration * SPA_NSEC_PER_SEC as u64 / rate as u64;

  assert!(!state.clock.is_null());

  (*state.clock).nsec      = nsec;
  (*state.clock).rate      = (*state.clock).target_rate;
  (*state.clock).position += (*state.clock).duration;
  (*state.clock).duration  = duration;
  // .delay and .rate_diff are published from process(), where the queued input
  // and the DLL correction are known; keep last cycle's values here (set at Start).
  (*state.clock).next_nsec = state.next_time;

  let node_callbacks = state.callbacks.funcs.cast::<spa_node_callbacks>().as_ref()
    .expect("callbacks should be initialized");
  assert!(node_callbacks.version >= SPA_VERSION_NODE_CALLBACKS);
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

  let mut now = timespec { tv_sec: 0, tv_nsec: 0 };
  let err = state.data_system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now);
  assert!(err >= 0);

  state.next_time = (now.tv_sec * SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64;

  if state.started && !state.following {
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

  // clock/position are read on the data loop; apply the change there
  crate::utils::block_on_loop(&(*state).data_loop, state, |state| {

    #[allow(non_upper_case_globals)]
    match id {
      SPA_IO_Clock    => {
        assert_eq!(size, std::mem::size_of::<spa_io_clock>());
        state.clock = data.cast();
      },
      SPA_IO_Position => {
        assert_eq!(size, std::mem::size_of::<spa_io_position>());
        state.position = data.cast();
      },
      _ => unimplemented!()
    };

    if state.started {
      let following = state.node_is_follower();
      if state.following != following {
        state.following = following;
        update_timers(state);
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
      let state: *mut State = state;
      crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
        // sane clock delay/rate_diff until process() publishes measured values
        if !state.clock.is_null() {
          (*state.clock).delay     = 0;
          (*state.clock).rate_diff = 1.0;
        }
        state.started   = true;
        state.following = state.node_is_follower();
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
      crate::warn!(state.log, "oss-source: unknown command: {}, {}", cmd_type, cmd_id);
      -libc::ENOTSUP
    }
  }
}

#[allow(unused_variables)]
unsafe extern "C" fn add_port(object: *mut c_void, direction: spa_direction, port_id: u32, props: *const spa_dict) -> c_int {
  unimplemented!()
}

#[allow(unused_variables)]
unsafe extern "C" fn remove_port(object: *mut c_void, direction: spa_direction, port_id: u32) -> c_int {
  unimplemented!()
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

/*unsafe fn build_port_format_info(builder: &mut libspa::pod::builder::Builder, config: &PortConfig, id: u32) {

  assert!(config.positions.len() <= 64);

  let mut position = [0u32; 64];
  for i in 0..config.positions.len() {
    position[i] = config.positions[i];
  }

  let mut raw = spa_audio_info_raw {
    format:   config.format.as_raw(),
    flags:    0,
    rate:     config.rate,
    channels: config.channels,
    position
  };

  spa_format_audio_raw_build(builder.as_raw_ptr(), id, &mut raw);
}*/

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

  assert_eq!(direction, SPA_DIRECTION_OUTPUT);
  assert!((port_id as usize) < MAX_PORTS);
  assert_ne!(max, 0);

  let mut buffer = vec![];

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
      (SPA_PARAM_Buffers, _)    => return -libc::ENOENT,
      (SPA_PARAM_Latency, 0 | 1) => {
        let mut info = state.latency[index as usize];
        // the process latency shifts what we report downstream
        if info.direction == SPA_DIRECTION_OUTPUT {
          crate::utils::process_latency_info_add(&state.process_latency, &mut info);
        }
        crate::utils::build_latency_info(&mut builder, &info).unwrap()
      },
      (SPA_PARAM_Latency, _)     => return 0,
      _ => return -libc::EINVAL
    };

    let mut result = spa_result_node_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if spa_pod_filter(builder.as_raw_ptr(), &mut result.param, buffer.as_mut_ptr() as *mut spa_pod, filter) >= 0 {
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

  assert_eq!(direction, SPA_DIRECTION_OUTPUT);
  assert!((port_id as usize) < MAX_PORTS);
  //assert_eq!(flags, 0);

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
              rate:     raw.rate,
              channels: raw.channels,
              stride:   bytes_per_sample * raw.channels // bytes per interleaved frame
            };

            crate::debug!(state.log, "reconfiguring with {:?}", config);

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
                crate::warn!(state.log, "dsp open: {}", err);
                port.config = None;
                *res_ref = -(err as c_int);
                return;
              }

              // ditto for a device that won't take the format exactly
              if let Err(err) = port.dsp.configure(oss_format, config.channels, config.rate) {
                crate::warn!(state.log, "device rejected {:?}: {}", config, err);
                port.dsp.close();
                port.config = None;
                *res_ref = -(err as c_int);
                return;
              }

              port.config = Some(config);
              port.dll.init(); // fresh device, fresh servo; set_bw happens on the first cycle
              port.primed = false;
              state.active_buffers = 0;
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
        // config is read by process() on the data loop, so clear it from there
        let port_idx = port_id as usize;
        let state_ptr: *mut State = state;
        crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
          state.ports[port_idx].config = None;
        });
      }

      //TODO: emit port info

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
    _ => unimplemented!()
  }
}

unsafe extern "C" fn process(object: *mut c_void) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  if !state.started {
    return SPA_STATUS_OK as i32;
  }

  let mut result = SPA_STATUS_OK as i32;

  for port in &mut state.ports {

    if port.config.is_none() {
      continue;
    }

    assert!(!port.buffers.is_empty());
    assert!(!port.io.is_null());

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

    let mut corr: f64 = 1.0; // DLL rate correction, published below
    let mut queued_frames: i64 = 0;

    // one period in device bytes (0 while position is absent)
    let mut period_in_bytes = 0u32;
    if !state.position.is_null() {
      let driver_clock = (*state.position).clock;
      if driver_clock.target_rate.denom > 0 {
        period_in_bytes = (driver_clock.target_duration * rate as u64
          / driver_clock.target_rate.denom as u64) as u32 * stride;
      }
    }

    let nbytes = if !port.primed && period_in_bytes > 0 {
      // Capture analogue of the sink's zero priming: trigger the device,
      // discard any backlog so the fill level starts out known, and hand the
      // graph one period of silence while the ring fills. Don't wait for real
      // data: an empty first cycle reads as a missed deadline to the graph.
      if port.dsp.ready_for_reading(1) {
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
      port.primed = true;
      port.dll.init(); // servo starts fresh on the next cycle

      let len = period_in_bytes.min(data_0.maxsize);
      std::ptr::write_bytes(data_0.data.cast::<u8>(), 0, len as usize);
      len as isize
    } else if port.dsp.ready_for_reading(1) {
      let queued = port.dsp.ispace_in_bytes().max(0) as u32;
      queued_frames = (queued / stride) as i64;

      // We drain the ring every cycle, so the pre-read level is what the
      // device captured in one period; its deviation from one period is the
      // clock error. Note the sign: a slow device queues less than a period
      // and must push corr below 1.0, the inverse of the sink's error.
      if period_in_bytes > 0 {
        if port.dll.bw() == 0.0 {
          port.dll.init();
          port.dll.set_bw(crate::dll::SPA_DLL_BW_MIN, period_in_bytes, rate * stride);
        }
        let err = (period_in_bytes as i64 - queued as i64) as f64;
        corr = port.dll.update(err);

        #[cfg(debug_assertions)]
        eprintln!("capture: corr = {}, err = {}", corr, err);
      }

      // the device can report more queued input than the buffer holds; cap it
      let ispace = queued.min(data_0.maxsize);
      #[cfg(debug_assertions)]
      crate::trace!(state.log, "ispace: {}", ispace);
      port.dsp.read(data_0.data, ispace as usize)
    } else {
      -1
    };

    // publish device latency (queued frames) and the rate correction when
    // driving; hand the correction to the resampler when following. Capture
    // uses the inverse rate (matching ALSA's update_time).
    if !state.following && !state.clock.is_null() {
      (*state.clock).delay     = queued_frames;
      (*state.clock).rate_diff = corr;
    }
    if !state.rate_match.is_null() {
      (*state.rate_match).rate = (1.0 / corr).clamp(0.99, 1.01);
    }

    // the dsp is running after ready_for_reading; report overruns to the host
    // (pw-top's xrun counter); the length isn't known, so pass 0 delay
    let overrun_count = port.dsp.overruns();
    if overrun_count > 0 {
      let now = crate::utils::now_ns(&state.data_system);
      crate::warn!(state.log, "OSS reported {:3} overruns @ {}", overrun_count, now);
      let node_callbacks = state.callbacks.funcs.cast::<spa_node_callbacks>().as_ref()
        .expect("callbacks should be initialized");
      assert!(node_callbacks.version >= SPA_VERSION_NODE_CALLBACKS);
      if let Some(xrun_fun) = node_callbacks.xrun {
        xrun_fun(state.callbacks.data, now / 1000, 0, std::ptr::null_mut());
      }
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

  assert_eq!(direction, SPA_DIRECTION_OUTPUT);
  assert!((port_id as usize) < MAX_PORTS);
  assert_eq!(flags, 0);

  let new_buffers = if !buffers.is_null() {
    assert!(n_buffers > 0);
    std::slice::from_raw_parts(buffers, n_buffers as usize).to_vec()
  } else {
    vec![]
  };

  // process() walks this vec on the data loop; swap it there
  let port_idx = port_id as usize;
  let state_ptr: *mut State = state;
  crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
    state.ports[port_idx].buffers = new_buffers;
    state.active_buffers = 0;
  });

  0
}

unsafe extern "C" fn port_set_io(object: *mut c_void, direction: spa_direction, port_id: u32, id: u32, data: *mut c_void, _size: usize) -> c_int {

  assert_eq!(direction, SPA_DIRECTION_OUTPUT);
  assert!((port_id as usize) < MAX_PORTS);

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  #[allow(non_upper_case_globals)]
  match id {
    SPA_IO_Buffers => {
      crate::debug!(state.log, "SPA_IO_Buffers: port_id={}", port_id);
      if !data.is_null() {
        state.ports[port_id as usize].io = data.cast();
      } else {
        state.ports[port_id as usize].io = std::ptr::null_mut();
      }
      0
    },
    SPA_IO_RateMatch => {
      let rate_match = data as *mut spa_io_rate_match;
      if !rate_match.is_null() {
        assert_eq!(MAX_PORTS, 1); // the code assumes a single port
        (*rate_match).flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
      }
      state.rate_match = rate_match;
      0
    },
    _ => unimplemented!()
  }
}

#[allow(unused_variables)]
unsafe extern "C" fn port_reuse_buffer(object: *mut c_void, port_id: u32, buffer_id: u32) -> c_int {
  unimplemented!()
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
    unimplemented!()
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

  let dsp_path = dsp_path.unwrap();

  let caps = crate::sound::probe_caps(&dsp_path, false).unwrap_or_else(|| {
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

    ports: [Port { config: None, buffers: vec![], io: std::ptr::null_mut(), dsp: crate::sound::Dsp::new(&dsp_path), dll: std::default::Default::default(), primed: false }; MAX_PORTS],

    caps,

    latency: [
      crate::utils::latency_info_default(SPA_DIRECTION_INPUT),
      crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT)
    ],

    process_latency: crate::utils::process_latency_default(),

    started:   false,
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

  //state.port_info.add_param(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
  //state.port_info.add_param(SPA_PARAM_Format,     SPA_PARAM_INFO_READWRITE);
  //state.port_info.add_param(SPA_PARAM_PortConfig, SPA_PARAM_INFO_READWRITE);
  //state.port_info.add_param(SPA_PARAM_IO,         SPA_PARAM_INFO_READ);
  //state.port_info.add_param(SPA_PARAM_Buffers,    SPA_PARAM_INFO_WRITE); // ?
  state.port_info.add_param(SPA_PARAM_Latency,    SPA_PARAM_INFO_READWRITE);

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
