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
  timer_source:  spa_source,
  next_time:     u64,
  hooks:         spa_hook_list,
  callbacks:     spa_callbacks,
  ports:         [Port; MAX_PORTS],
  started:       bool,
  following:     bool,
  cur_timestamp: u64,  // method invocation timestamp for `process`
  old_timestamp: u64,
  oss_delay:     u32 // additional delay in 1/8ths of period
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
  target_delay:   u32  // OSS buffer fill target in bytes, clamped to the granted buffer
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

#[allow(unused_variables)]
unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {
  unimplemented!()
}

/*unsafe extern "C" fn enum_params(object: *mut c_void, seq: c_int, id: u32, start: u32, max: u32, filter: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  assert_ne!(max, 0);

  let mut buffer = vec![];

  let mut index = start;
  let mut count = 0;

  while count < max {

    use libspa::pod::builder::Builder;
    use libspa::pod::builder::builder_add;

    let mut builder = Builder::new(&mut buffer);

    #[allow(non_upper_case_globals)]
    match (id, index) {
      //TODO: ?
      _ => unimplemented!()
    };

    let mut result = spa_result_node_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if spa_pod_filter(builder.as_raw_ptr(), &mut result.param, buffer.as_mut_ptr() as *mut spa_pod, filter) >= 0 {
      crate::spa::node_emit_result(&mut state.hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS, &result);
      count += 1;
    }

    index += 1;
  }

  0
}*/

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
              SPA_PROP_params         => {
                match property.value {
                  Value::Struct(values) if values.len() % 2 == 0 => {
                    for kv in values.chunks(2) {
                      match (&kv[0], &kv[1]) {
                        // pw-cli set-param <object-id> Props '{ "params": ["oss.delay", 8]}'
                        (Value::String(s), Value::Int(x)) if s == "oss.delay" && *x >= 0 => {
                          // cap it: period/8 * oss_delay runs in the RT path and must not overflow
                          state.oss_delay = (*x as u32).min(1024);
                        },
                        _ => ()
                      }
                    }
                  }
                  _ => ()
                }
              },
              _ => unimplemented!()
            }
          }
        },
        _ => return -libc::EINVAL
      }
      0
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

  #[cfg(debug_assertions)]
  {
    let now   = crate::utils::now_ns(&state.data_system);
    let delay = now - nsec;
    eprintln!("cycle: {}, delay: {} ms @ {}", (*state.position).clock.cycle, delay as f64 / 1000000.0, now);
  }

  let duration = (*state.position).clock.target_duration;
  let rate     = (*state.position).clock.target_rate.denom;

  state.next_time = nsec + duration * SPA_NSEC_PER_SEC as u64 / rate as u64;

  assert!(!state.clock.is_null());

  (*state.clock).nsec      = nsec;
  (*state.clock).rate      = (*state.clock).target_rate; // ?
  (*state.clock).position += (*state.clock).duration;
  (*state.clock).duration  = duration;
  // .delay and .rate_diff are published from process(), where odelay() and the
  // DLL correction are known; keep last cycle's values here (set at Start).
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

  let mut raw = spa_audio_info_raw {
    format:   config.format.0,
    flags:    config.flags,
    rate:     config.rate,
    channels: config.channels,
    position
  };

  spa_format_audio_raw_build(builder.as_raw_ptr(), id, &mut raw);
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

  assert_eq!(direction, SPA_DIRECTION_INPUT);
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
      (SPA_PARAM_EnumFormat, 0) => crate::utils::build_enum_format_info(&mut builder, false).unwrap(),
      (SPA_PARAM_EnumFormat, _) => return 0,
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

  assert_eq!(direction, SPA_DIRECTION_INPUT);
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

              port.dsp.set_format(oss_format);
              port.dsp.set_channels(config.channels);
              port.dsp.set_rate(config.rate);

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
    SPA_PARAM_Latency => 0,
    SPA_PARAM_Tag     => 0,
    _ => unimplemented!()
  }
}

unsafe extern "C" fn process(object: *mut c_void) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  // a cycle that was already signaled when we paused can still land here; drop
  // it instead of assert!()ing, which aborts the daemon across extern "C"
  if !state.started {
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

    assert!(!port.buffers.is_empty());
    assert!(!port.io.is_null());

    if (*port.io).status != SPA_STATUS_HAVE_DATA as i32 {
      // no input this cycle (e.g. draining after stop): keep clock.delay ticking
      // down (when driving) so the graph's drain completes, and ask for data.
      if !state.following && !state.clock.is_null() && port.dsp.is_running() {
        (*state.clock).delay     = port.dsp.odelay() as i64 / port_config.stride() as i64;
        (*state.clock).rate_diff = 1.0;
      }
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

      port.dll.init();
      port.dll.set_bw(crate::dll::SPA_DLL_BW_MIN, period_in_bytes, driver_clock.target_rate.denom * port_config.stride());

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
        //TODO: spa_node_call_xrun?
        crate::warn!(state.log, "{}: OSS reported {:3} underruns @ {}", port.dsp.path, underrun_count, state.cur_timestamp);
        if port.xrun_timestamp == 0 {
          port.xrun_timestamp = state.cur_timestamp;
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

        port.dll.init();
        port.dll.set_bw(crate::dll::SPA_DLL_BW_MIN, period_in_bytes, driver_clock.target_rate.denom * port_config.stride());

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
      // Always run the DLL so we can publish rate_diff even with no resampler.
      // err is in bytes (queued - target), matching set_bw's units.
      let err = (port.dsp.odelay() as isize - port.target_delay as isize) as f64;
      corr = port.dll.update(err);

      #[cfg(debug_assertions)]
      eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, err);

      // the resampler (when present) pulls its input at this corrected rate
      if !state.rate_match.is_null() {
        (*state.rate_match).rate = corr.clamp(0.99, 1.01);
      }

      port.dsp.write(data_0.data.offset(offset as isize), size)
    };

    // publish device latency (queued frames + resampler delay) and the rate
    // correction, but only when driving: a follower's clock is unread.
    if !state.following && !state.clock.is_null() {
      let stride = port_config.stride() as i64;
      let resamp = if state.rate_match.is_null() { 0 } else { (*state.rate_match).delay as i64 };
      (*state.clock).delay     = port.dsp.odelay() as i64 / stride + resamp;
      (*state.clock).rate_diff = corr;
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

  assert_eq!(direction, SPA_DIRECTION_INPUT);
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
  });

  0
}

unsafe extern "C" fn port_set_io(object: *mut c_void, direction: spa_direction, port_id: u32, id: u32, data: *mut c_void, _size: usize) -> c_int {

  assert_eq!(direction, SPA_DIRECTION_INPUT);
  assert!((port_id as usize) < MAX_PORTS);

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
  enum_params:       None, // Some(enum_params),
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

    ports: [
      Port {
        config:         None,
        buffers:        vec![],
        io:             std::ptr::null_mut(),
        dsp:            crate::sound::DspWriter::new(&dsp_path),
        xrun_timestamp: 0,
        dll:            std::default::Default::default(),
        target_delay:   0
      };
      MAX_PORTS
    ],

    started:   false,
    following: false,

    cur_timestamp: 0,
    old_timestamp: 0,

    oss_delay: 10 // eh, whatever
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
  //state.node_info.add_param(SPA_PARAM_Props,          SPA_PARAM_INFO_READWRITE);
  //state.node_info.add_param(SPA_PARAM_PropInfo,       SPA_PARAM_INFO_READ);

  state.port_info.fix_pointers();

  state.port_info.set_flags((SPA_PORT_FLAG_PHYSICAL | SPA_PORT_FLAG_TERMINAL) as u64);
  state.port_info.set_rate(spa_fraction { num: 1, denom: 48000 }); // ?

  // advertise the format as writable so the host (re)negotiates it; Buffers is
  // unreadable until a format is set (it needs the stride). Flags flip in
  // port_set_param.
  state.port_info.add_param(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
  state.port_info.add_param(SPA_PARAM_Format,     SPA_PARAM_INFO_WRITE);
  state.port_info.add_param(SPA_PARAM_Buffers,    0);

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
