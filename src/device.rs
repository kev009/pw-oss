use std::mem::MaybeUninit;
use std::os::raw::{c_char, c_int, c_uint, c_void};
use libspa::sys::*;

// One hardware route per (pcm device, direction) that has a usable mixer
// control. The shadow fields mirror the kernel mixer state; the poll timer
// and set_param keep them in sync so re-emissions never report placeholders.
struct RouteState {
  node_id: u32,    // our node object id (index * 2 + rec)
  rec:     bool,
  name:    String, // stable, never localized: WirePlumber's persistence key
  description: String,
  mixer:   usize,  // index into State::mixers
  control: c_uint, // mixer device selected by mixer::{output,input}_control
  levels:  (u32, u32), // shadow OSS levels, 0-100 each
  mute:    bool,
  save:    bool    // echoed back in the Route pod, never interpreted
}

struct MixerHandle {
  mixer:   crate::mixer::Mixer,
  counter: c_int // modify_counter baseline for external-change detection
}

#[repr(C)]
struct State {
  handle:      spa_handle,
  device:      spa_device,
  dev_info:    crate::spa::DeviceInfo,
  hooks:       spa_hook_list,
  pcm_devices: Vec<crate::sound::PcmDevice>,
  description: String,
  profile:     u32, // 0 = off, 1 = default
  profile_save: bool, // echoed back in the Profile pod
  routes:      Vec<RouteState>,
  mixers:      Vec<MixerHandle>,
  main_loop:   Option<crate::spa::Loop>,   // for the mixer poll timer
  system:      Option<crate::spa::System>, // ditto
  timer_source: spa_source,
  timer_added: bool,
  log:         crate::spa::Log
}

// OSS levels are a 0-100 slider scale, so map them through the cubic curve
// like ALSA devices without a dB scale (acp channel_map.c); a 1:1 linear map
// would make the volume keys feel wrong at the bottom of the range.
fn linear_to_oss(v: f32) -> u32 {
  if v.is_nan() || v <= 0.0 { // hostile pods included
    return 0;
  }
  (v.min(1.0).cbrt() * 100.0).round() as u32
}

// report the quantized readback, never the request, so the session manager
// converges on values the hardware can actually hold
fn oss_to_linear(l: u32) -> f32 {
  let x = l.min(100) as f32 / 100.0;
  x * x * x
}

// the mixer is stereo everywhere (STEREODEVS is the devmask, mixer.c:1094),
// so routes carry fixed FL/FR maps whatever width the node negotiates
const ROUTE_CHANNELS: u32 = 2;
const ROUTE_MAP: [u32; ROUTE_CHANNELS as usize] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR];

// emit (or, with present = false, retract) the node object for every pcm device
unsafe fn emit_objects(f: &spa_device_events, data: *mut c_void, pcm_devices: &[crate::sound::PcmDevice],
                       routes: &[RouteState], description: &str, present: bool) {

  let Some(obj_info_fun) = f.object_info else {
    return;
  };

  for device in pcm_devices {
    for (rec, enabled) in [(false, device.play), (true, device.rec)] {

      if !enabled {
        continue;
      }

      let id = device.index * 2 + rec as u32;

      if !present {
        obj_info_fun(data, id, std::ptr::null());
        continue;
      }

      let mut dict = crate::spa::Dictionary::new();

      dict.add_item(SPA_KEY_NODE_NAME.as_ptr(), format!("pcm{}.{}", device.index, if rec { "rec" } else { "play" }));

      if device.desc == description && !device.location.is_empty() {
        dict.add_item(SPA_KEY_NODE_DESCRIPTION.as_ptr(), format!("{} @ {}", device.desc, device.location));
      } else {
        dict.add_item(SPA_KEY_NODE_DESCRIPTION.as_ptr(), device.desc.as_str());
      }

      dict.add_item(crate::keys::OSS_DSP_PATH, format!("/dev/dsp{}", device.index));

      // Only nodes with a hardware route get linked to it; the rest (no
      // mixer, or no usable control - the bitperfect-purist case included)
      // keep the session manager's node softvol as their only volume.
      if routes.iter().any(|r| r.node_id == id) {
        dict.add_item("card.profile.device", format!("{}", id));
        dict.add_item("device.routes", "1");
      }

      let obj_info = spa_device_object_info {
        version:      SPA_VERSION_DEVICE_OBJECT_INFO,
        type_:        SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
        factory_name: if rec { c"freebsd-oss.source".as_ptr() } else { c"freebsd-oss.sink".as_ptr() },
        change_mask:  crate::spa::SPA_DEVICE_OBJECT_CHANGE_MASK_ALL as u64,
        flags:        0,
        props:        dict.raw()
      };

      obj_info_fun(data, id, &obj_info);
    }
  }
}

// re-emit dev_info to every listener (carrying whatever change_mask the caller
// set, e.g. PARAMS), then clear the mask
unsafe fn emit_device_info(state: &mut State) {
  crate::spa::for_each_hook(&mut state.hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_device_events>().as_ref()
      .expect("hook should be initialized");
    assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);
    if let Some(info_fun) = f.info {
      info_fun(entry.cb.data, state.dev_info.raw());
    }
  });
  let _ = state.dev_info.replace_change_mask(0);
}

unsafe fn build_profile_info(b: &mut libspa::pod::builder::Builder, id: u32, index: u32,
                             state: &State, current: bool) -> Result<(), rustix::io::Errno> {

  let (name, description, priority) = if index == 0 {
    ("off", "Off", 0)
  } else {
    ("default", "Default", 100)
  };

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut frame, SPA_TYPE_OBJECT_ParamProfile, id)?;

  b.add_prop(SPA_PARAM_PROFILE_index, 0)?;
  b.add_int(index as i32)?;
  b.add_prop(SPA_PARAM_PROFILE_name, 0)?;
  b.add_string(name)?;
  b.add_prop(SPA_PARAM_PROFILE_description, 0)?;
  b.add_string(description)?;
  b.add_prop(SPA_PARAM_PROFILE_priority, 0)?;
  b.add_int(priority)?;
  b.add_prop(SPA_PARAM_PROFILE_available, 0)?;
  b.add_id(libspa::utils::Id(SPA_PARAM_AVAILABILITY_yes))?;

  // The classes struct is what WirePlumber's select-routes walks to map
  // nodes to this profile; without it no route is ever applied. Every node
  // is listed, routed or not (pod shape: alsa-acp-device.c:326-384).
  let mut capture:  Vec<i32> = vec![];
  let mut playback: Vec<i32> = vec![];
  for device in &state.pcm_devices {
    if device.play { playback.push((device.index * 2)     as i32); }
    if device.rec  { capture .push((device.index * 2 + 1) as i32); }
  }

  let classes: [(&str, &Vec<i32>); 2] = [("Audio/Source", &capture), ("Audio/Sink", &playback)];
  let n_classes = if index == 0 { 0 } else { classes.iter().filter(|(_, ids)| !ids.is_empty()).count() };

  b.add_prop(SPA_PARAM_PROFILE_classes, 0)?;
  b.push_struct(&mut inner)?;
  b.add_int(n_classes as i32)?;
  if index != 0 {
    for (class, ids) in classes {
      if ids.is_empty() {
        continue;
      }
      let mut cls = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
      b.push_struct(&mut cls)?;
      b.add_string(class)?;
      b.add_int(ids.len() as i32)?;
      b.add_string("card.profile.devices")?;
      b.add_array(std::mem::size_of::<i32>() as u32, SPA_TYPE_Int, ids.len() as u32, ids.as_ptr().cast())?;
      b.pop(cls.assume_init_mut());
    }
  }
  b.pop(inner.assume_init_mut());

  if current {
    b.add_prop(SPA_PARAM_PROFILE_save, 0)?;
    b.add_bool(state.profile_save)?;
  }

  b.pop(frame.assume_init_mut());

  Ok(())
}

// EnumRoute (full = false) carries the static description only; Route
// (full = true) adds device/profile/save and the volume props object
// (pod shape: alsa-acp-device.c build_route)
unsafe fn build_route_info(b: &mut libspa::pod::builder::Builder, id: u32,
                           state: &State, pos: usize, full: bool) -> Result<(), rustix::io::Errno> {

  let route = &state.routes[pos];

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut frame, SPA_TYPE_OBJECT_ParamRoute, id)?;

  b.add_prop(SPA_PARAM_ROUTE_index, 0)?;
  b.add_int(pos as i32)?;
  // note: PLAYBACK maps to OUTPUT here (the route points out of the graph)
  b.add_prop(SPA_PARAM_ROUTE_direction, 0)?;
  b.add_id(libspa::utils::Id(if route.rec { SPA_DIRECTION_INPUT } else { SPA_DIRECTION_OUTPUT }))?;
  b.add_prop(SPA_PARAM_ROUTE_name, 0)?;
  b.add_string(&route.name)?;
  b.add_prop(SPA_PARAM_ROUTE_description, 0)?;
  b.add_string(&route.description)?;
  b.add_prop(SPA_PARAM_ROUTE_priority, 0)?;
  b.add_int(100)?;
  b.add_prop(SPA_PARAM_ROUTE_available, 0)?;
  b.add_id(libspa::utils::Id(SPA_PARAM_AVAILABILITY_yes))?;

  let profiles = [1i32];
  b.add_prop(SPA_PARAM_ROUTE_profiles, 0)?;
  b.add_array(std::mem::size_of::<i32>() as u32, SPA_TYPE_Int, profiles.len() as u32, profiles.as_ptr().cast())?;

  let devices = [route.node_id as i32];
  b.add_prop(SPA_PARAM_ROUTE_devices, 0)?;
  b.add_array(std::mem::size_of::<i32>() as u32, SPA_TYPE_Int, devices.len() as u32, devices.as_ptr().cast())?;

  if full {
    b.add_prop(SPA_PARAM_ROUTE_device, 0)?;
    b.add_int(route.node_id as i32)?;

    b.add_prop(SPA_PARAM_ROUTE_props, 0)?;
    b.push_object(&mut inner, SPA_TYPE_OBJECT_Props, id)?;

    b.add_prop(SPA_PROP_mute, SPA_POD_PROP_FLAG_HARDWARE)?;
    b.add_bool(route.mute)?;

    let volumes = [oss_to_linear(route.levels.0), oss_to_linear(route.levels.1)];
    b.add_prop(SPA_PROP_channelVolumes, SPA_POD_PROP_FLAG_HARDWARE)?;
    b.add_array(std::mem::size_of::<f32>() as u32, SPA_TYPE_Float, ROUTE_CHANNELS, volumes.as_ptr().cast())?;

    b.add_prop(SPA_PROP_volumeBase, SPA_POD_PROP_FLAG_READONLY)?;
    b.add_float(1.0)?;
    b.add_prop(SPA_PROP_volumeStep, SPA_POD_PROP_FLAG_READONLY)?;
    b.add_float(1.0 / 101.0)?;

    b.add_prop(SPA_PROP_channelMap, 0)?;
    b.add_array(std::mem::size_of::<u32>() as u32, SPA_TYPE_Id, ROUTE_CHANNELS, ROUTE_MAP.as_ptr().cast())?;

    // the hardware does the attenuation; the node's software volume must
    // stay at unity or the signal is attenuated twice
    let soft = [1.0f32; ROUTE_CHANNELS as usize];
    b.add_prop(SPA_PROP_softVolumes, 0)?;
    b.add_array(std::mem::size_of::<f32>() as u32, SPA_TYPE_Float, ROUTE_CHANNELS, soft.as_ptr().cast())?;

    b.pop(inner.assume_init_mut());

    b.add_prop(SPA_PARAM_ROUTE_profile, 0)?;
    b.add_int(state.profile as i32)?;
    b.add_prop(SPA_PARAM_ROUTE_save, 0)?;
    b.add_bool(route.save)?;
  }

  b.pop(frame.assume_init_mut());

  Ok(())
}

unsafe fn build_object_config(b: &mut libspa::pod::builder::Builder, node_id: u32,
                              volume: Option<(u32, u32)>, mute: Option<bool>) -> Result<(), rustix::io::Errno> {

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
  let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

  b.push_object(&mut frame, SPA_TYPE_EVENT_Device, SPA_DEVICE_EVENT_ObjectConfig)?;

  b.add_prop(SPA_EVENT_DEVICE_Object, 0)?;
  b.add_int(node_id as i32)?;

  b.add_prop(SPA_EVENT_DEVICE_Props, 0)?;
  b.push_object(&mut inner, SPA_TYPE_OBJECT_Props, SPA_EVENT_DEVICE_Props)?;

  if let Some((left, right)) = volume {
    let volumes = [oss_to_linear(left), oss_to_linear(right)];
    b.add_prop(SPA_PROP_channelVolumes, 0)?;
    b.add_array(std::mem::size_of::<f32>() as u32, SPA_TYPE_Float, ROUTE_CHANNELS, volumes.as_ptr().cast())?;
    b.add_prop(SPA_PROP_channelMap, 0)?;
    b.add_array(std::mem::size_of::<u32>() as u32, SPA_TYPE_Id, ROUTE_CHANNELS, ROUTE_MAP.as_ptr().cast())?;
    let soft = [1.0f32; ROUTE_CHANNELS as usize];
    b.add_prop(SPA_PROP_softVolumes, 0)?;
    b.add_array(std::mem::size_of::<f32>() as u32, SPA_TYPE_Float, ROUTE_CHANNELS, soft.as_ptr().cast())?;
  }

  if let Some(mute) = mute {
    b.add_prop(SPA_PROP_mute, 0)?;
    b.add_bool(mute)?;
    b.add_prop(SPA_PROP_softMute, 0)?;
    b.add_bool(mute)?;
  }

  b.pop(inner.assume_init_mut());
  b.pop(frame.assume_init_mut());

  Ok(())
}

// Tell the session manager to push the new hardware state into the child
// node's Props (channelVolumes/softVolumes or mute/softMute), keeping
// audioconvert at unity - the anti-double-attenuation mechanism
// (pod shape: alsa-acp-device.c:1015-1084).
unsafe fn emit_object_config(state: &mut State, pos: usize, volume: bool) {

  let route = &state.routes[pos];
  let (node_id, levels, mute) = (route.node_id, route.levels, route.mute);

  let mut buffer = vec![];
  let mut builder = libspa::pod::builder::Builder::new(&mut buffer);
  let built = if volume {
    build_object_config(&mut builder, node_id, Some(levels), None)
  } else {
    build_object_config(&mut builder, node_id, None, Some(mute))
  };
  drop(builder);
  if built.is_err() {
    return;
  }

  crate::spa::for_each_hook(&mut state.hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_device_events>().as_ref()
      .expect("hook should be initialized");
    assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);
    if let Some(event_fun) = f.event {
      event_fun(entry.cb.data, buffer.as_ptr() as *const spa_event);
    }
  });
}

// announce a Route change: flip the serial so consumers re-read the param
unsafe fn announce_route_change(state: &mut State) {
  let _ = state.dev_info.replace_change_mask(0);
  state.dev_info.bump_param(SPA_PARAM_Route);
  emit_device_info(state);
}

// The ~1 Hz external-change poll: on a modify_counter tick, value-diff the
// levels and mute against the shadow and re-emit only on a real change. The
// counter is only a hint (it misses RECSRC changes and writes-to-muted); the
// value diff is what prevents spurious re-emissions either way.
unsafe fn poll_mixers(state: &mut State) {

  let mut changed: Vec<(usize, bool, bool)> = vec![]; // (route, volume, mute)

  for mi in 0..state.mixers.len() {

    let Some(counter) = state.mixers[mi].mixer.modify_counter() else {
      continue; // the device may be mid-detach; the node teardown handles it
    };
    // Diff by VALUE every tick, not only when the counter moved: the kernel
    // doesn't bump it for writes to a muted control (mixer.c early-returns
    // into level_muted), and an external write landing inside our own
    // write-then-refresh window is swallowed by the baseline. The counter is
    // still tracked for log/debug value.
    state.mixers[mi].counter = counter;

    for pos in 0..state.routes.len() {
      if state.routes[pos].mixer != mi {
        continue;
      }
      let control = state.routes[pos].control;
      let mut vol_changed  = false;
      let mut mute_changed = false;
      if let Some(levels) = state.mixers[mi].mixer.level(control) {
        if levels != state.routes[pos].levels {
          state.routes[pos].levels = levels;
          vol_changed = true;
        }
      }
      if let Some(mute) = state.mixers[mi].mixer.muted(control) {
        if mute != state.routes[pos].mute {
          state.routes[pos].mute = mute;
          mute_changed = true;
        }
      }
      if vol_changed || mute_changed {
        crate::info!(state.log, "route {} changed externally: levels {:?}, mute {}",
          state.routes[pos].name, state.routes[pos].levels, state.routes[pos].mute);
        changed.push((pos, vol_changed, mute_changed));
      }
    }
  }

  if changed.is_empty() {
    return;
  }

  announce_route_change(state);

  for (pos, vol_changed, mute_changed) in changed {
    if vol_changed {
      emit_object_config(state, pos, true);
    }
    if mute_changed {
      emit_object_config(state, pos, false);
    }
  }
}

unsafe extern "C" fn on_mixer_timeout(source: *mut spa_source) {

  let state = (*source).data.cast::<State>().as_mut()
    .expect("(*source).data is not supposed to be null");

  // drain the periodic timerfd or the level-triggered source spins the loop
  let Some(system) = &state.system else {
    return; // the source is only registered when the system interface exists
  };
  let mut expirations = 0;
  if system.timerfd_read(state.timer_source.fd, &mut expirations) < 0 {
    return;
  }

  poll_mixers(state);
}

unsafe extern "C" fn add_listener(object: *mut c_void, listener: *mut spa_hook, events: *const spa_device_events, data: *mut c_void) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  let mut save = MaybeUninit::<spa_hook_list>::uninit();
  spa_hook_list_isolate(&mut state.hooks, save.as_mut_ptr(), listener, events.cast(), data);

  crate::spa::for_each_hook(&mut state.hooks, |entry| {

    let f = entry.cb.funcs.cast::<spa_device_events>().as_ref()
      .expect("we just assigned events to this very hook by calling spa_hook_list_isolate");

    assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);

    if let Some(dev_info_fun) = f.info {
      let old_mask = state.dev_info.replace_change_mask(crate::spa::SPA_DEVICE_CHANGE_MASK_ALL as u64);
      dev_info_fun(entry.cb.data, state.dev_info.raw());
      let _ = state.dev_info.replace_change_mask(old_mask);
    }

    emit_objects(f, entry.cb.data, &state.pcm_devices, &state.routes, &state.description, state.profile != 0);
  });

  spa_hook_list_join(&mut state.hooks, save.assume_init_mut());
  0
}

unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  crate::spa::for_each_hook(&mut state.hooks, |entry| {
    let f = entry.cb.funcs.cast::<spa_device_events>().as_ref().expect("hook should be initialized");
    assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);
    if let Some(result_fun) = f.result {
      result_fun(entry.cb.data, seq, 0, 0, std::ptr::null());
    }
  });

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
      (SPA_PARAM_EnumProfile, 0 | 1) => build_profile_info(&mut builder, SPA_PARAM_EnumProfile, index, state, false).unwrap(),
      (SPA_PARAM_EnumProfile, _)     => return 0,
      (SPA_PARAM_Profile, 0)         => build_profile_info(&mut builder, SPA_PARAM_Profile, state.profile, state, true).unwrap(),
      (SPA_PARAM_Profile, _)         => return 0,
      (SPA_PARAM_EnumRoute, i) if (i as usize) < state.routes.len() =>
        build_route_info(&mut builder, SPA_PARAM_EnumRoute, state, i as usize, false).unwrap(),
      (SPA_PARAM_EnumRoute, _)       => return 0,
      // no Route pods while Off is active: there is nothing routed
      (SPA_PARAM_Route, i) if state.profile != 0 && (i as usize) < state.routes.len() =>
        build_route_info(&mut builder, SPA_PARAM_Route, state, i as usize, true).unwrap(),
      (SPA_PARAM_Route, _)           => return 0,
      _ => return -libc::ENOENT
    };

    drop(builder); // its borrow of `buffer` must end before we take the source pointer

    let mut result = spa_result_device_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if let Some(param) = crate::spa::filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter) {
      result.param = param;
      crate::spa::dev_emit_result(&mut state.hooks, seq, 0, SPA_RESULT_TYPE_DEVICE_PARAMS, &result);
      count += 1;
    }

    index += 1;
  }

  0
}

// apply a Route props object to the hardware and the shadow; unknown props
// are ignored (WirePlumber sends softVolumes and friends along)
unsafe fn apply_route_props(state: &mut State, pos: usize, props: libspa::pod::Object,
                            vol_changed: &mut bool, mute_changed: &mut bool) {

  use libspa::pod::{Value, ValueArray};

  let mi      = state.routes[pos].mixer;
  let control = state.routes[pos].control;

  for p in props.properties {
    #[allow(non_upper_case_globals)]
    match (p.key, p.value) {
      (SPA_PROP_mute, Value::Bool(mute)) => {
        if mute != state.routes[pos].mute && state.mixers[mi].mixer.set_muted(control, mute) {
          state.routes[pos].mute = mute;
          *mute_changed = true;
        }
      },
      (SPA_PROP_channelVolumes, Value::ValueArray(ValueArray::Float(v))) if !v.is_empty() => {
        // any width is accepted: mixer channel i reads v[i % n], so a mono
        // request fans out and a wider-than-stereo one folds down
        #[cfg(debug_assertions)]
        crate::debug!(state.log, "route {} channelVolumes {:?}", state.routes[pos].name, v);
        let levels = (linear_to_oss(v[0]), linear_to_oss(v[1 % v.len()]));
        if levels != state.routes[pos].levels && state.mixers[mi].mixer.set_level(control, levels.0, levels.1) {
          state.routes[pos].levels = levels;
          *vol_changed = true;
        }
      },
      _ => ()
    }
  }
}

unsafe extern "C" fn set_param(object: *mut c_void, id: u32, _flags: u32, param: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  use libspa::pod::{Value, Pod};
  use libspa::pod::deserialize::PodDeserializer;

  #[allow(non_upper_case_globals)]
  match id {
    SPA_PARAM_Profile => {

      let mut index = None;
      let mut name  = None;
      let mut save  = false;
      if param.is_null() {
        index = Some(1); // a NULL pod resets to the boot default (on)
      } else {
        match PodDeserializer::deserialize_any_from(Pod::from_raw(param).as_bytes()) {
        Ok((_, Value::Object(o))) if o.type_ == SPA_TYPE_OBJECT_ParamProfile => {
          for p in o.properties {
            #[allow(non_upper_case_globals)]
            match (p.key, p.value) {
              (SPA_PARAM_PROFILE_index, Value::Int(v)) if (0..=1).contains(&v) => index = Some(v as u32),
              (SPA_PARAM_PROFILE_name,  Value::String(v)) => name = Some(v),
              (SPA_PARAM_PROFILE_save,  Value::Bool(v))   => save = v,
              _ => ()
            }
          }
        },
        _ => return -libc::EINVAL
        }
      }

      // session managers may address profiles by name instead of index
      if index.is_none() {
        index = match name.as_deref() {
          Some("off")     => Some(0),
          Some("default") => Some(1),
          _               => None
        };
      }

      let Some(index) = index else {
        return -libc::EINVAL;
      };

      state.profile_save = save;

      if state.profile != index {
        state.profile = index;
        crate::info!(state.log, "profile -> {}", if index == 0 { "off" } else { "default" });

        // add or remove the nodes, then re-announce the params tied to the
        // active profile (Route pods appear/vanish with it)
        crate::spa::for_each_hook(&mut state.hooks, |entry| {
          let f = entry.cb.funcs.cast::<spa_device_events>().as_ref()
            .expect("hook should be initialized");
          assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);
          emit_objects(f, entry.cb.data, &state.pcm_devices, &state.routes, &state.description, index != 0);
        });

        let _ = state.dev_info.replace_change_mask(0);
        state.dev_info.bump_param(SPA_PARAM_Profile);
        state.dev_info.bump_param(SPA_PARAM_EnumRoute);
        state.dev_info.bump_param(SPA_PARAM_Route);
        emit_device_info(state);
      } else {
        // the save flag is part of the Profile readback; keep it fresh
        let _ = state.dev_info.replace_change_mask(0);
        state.dev_info.bump_param(SPA_PARAM_Profile);
        emit_device_info(state);
      }

      0
    },
    SPA_PARAM_Route => {

      if param.is_null() {
        return -libc::EINVAL;
      }

      let object = match PodDeserializer::deserialize_any_from(Pod::from_raw(param).as_bytes()) {
        Ok((_, Value::Object(o))) if o.type_ == SPA_TYPE_OBJECT_ParamRoute => o,
        _ => return -libc::EINVAL
      };

      let mut index  = None;
      let mut name   = None;
      let mut device = None;
      let mut save   = false;
      let mut props  = None;

      for p in object.properties {
        #[allow(non_upper_case_globals)]
        match (p.key, p.value) {
          (SPA_PARAM_ROUTE_index,  Value::Int(v)) if v >= 0 => index  = Some(v as usize),
          (SPA_PARAM_ROUTE_name,   Value::String(v))        => name   = Some(v),
          (SPA_PARAM_ROUTE_device, Value::Int(v)) if v >= 0 => device = Some(v as u32),
          (SPA_PARAM_ROUTE_save,   Value::Bool(v))          => save   = v,
          (SPA_PARAM_ROUTE_props,  Value::Object(o)) if o.type_ == SPA_TYPE_OBJECT_Props => props = Some(o),
          _ => ()
        }
      }

      let Some(device) = device else {
        return -libc::EINVAL;
      };

      // resolve by index, else by name (restores may come back either way)
      let pos = index.filter(|i| *i < state.routes.len())
        .or_else(|| name.as_deref().and_then(|n| state.routes.iter().position(|r| r.name == n)));
      let Some(pos) = pos else {
        return -libc::EINVAL;
      };

      if state.routes[pos].node_id != device {
        return -libc::EINVAL;
      }

      state.routes[pos].save = save;

      // a port-switch message carries no props and must not touch the volume
      let mut vol_changed  = false;
      let mut mute_changed = false;
      if let Some(props) = props {
        apply_route_props(state, pos, props, &mut vol_changed, &mut mute_changed);
      }

      // refresh the counter baseline in the same open as our own writes so
      // the poll doesn't echo them back as an external change
      let mi = state.routes[pos].mixer;
      if let Some(counter) = state.mixers[mi].mixer.modify_counter() {
        state.mixers[mi].counter = counter;
      }

      announce_route_change(state);

      if vol_changed {
        emit_object_config(state, pos, true);
      }
      if mute_changed {
        emit_object_config(state, pos, false);
      }

      0
    },
    _ => -libc::ENOENT // unknown param id (ALSA convention)
  }
}

const DEVICE_IMPL: spa_device_methods = spa_device_methods {
  version:           SPA_VERSION_DEVICE_METHODS,
  add_listener:      Some(add_listener),
  sync:              Some(sync),
  enum_params:       Some(enum_params),
  set_param:         Some(set_param),
};

unsafe extern "C" fn get_interface(handle: *mut spa_handle, type_: *const c_char, interface: *mut *mut c_void) -> c_int {
  let state = handle.cast::<State>().as_mut()
    .expect("handle is not supposed to be null");
  assert!(!interface.is_null());
  if spa_streq(type_, SPA_TYPE_INTERFACE_Device.as_ptr().cast()) {
    *interface = &mut state.device as *mut _ as *mut c_void;
  } else {
    return -libc::ENOENT;
  }
  0
}

unsafe extern "C" fn clear(handle: *mut spa_handle) -> c_int {
  let state = handle.cast::<State>().as_mut()
    .expect("handle is not supposed to be null");
  // clear runs on the main loop's thread, so detach the poll source directly
  if state.timer_added {
    if let Some(main_loop) = &state.main_loop {
      main_loop.remove_source(&mut state.timer_source);
    }
    if let Some(system) = &state.system {
      system.close(state.timer_source.fd);
    }
    state.timer_source.fd = -1;
    state.timer_added = false;
  }
  std::ptr::drop_in_place(state);
  0
}

unsafe extern "C" fn get_size(_factory: *const spa_handle_factory, _params: *const spa_dict) -> usize {
  std::mem::size_of::<State>()
}

// Discover the usable hardware controls and read their ACTUAL state before
// anything is emitted: reporting 1.0 placeholders and correcting later is a
// classic volume-jump source.
fn probe_routes(pcm_devices: &[crate::sound::PcmDevice]) -> (Vec<RouteState>, Vec<MixerHandle>) {

  let mut routes: Vec<RouteState>  = vec![];
  let mut mixers: Vec<MixerHandle> = vec![];
  let mut n_out = 0;
  let mut n_in  = 0;
  let device_count = pcm_devices.len();

  for device in pcm_devices {

    let Some(mixer) = crate::mixer::Mixer::open(device.index) else {
      continue; // no mixer device: the node keeps its softvol
    };

    let mixer_index = mixers.len();
    let mut used = false;

    for (rec, enabled) in [(false, device.play), (true, device.rec)] {
      if !enabled {
        continue;
      }
      let Some(control) = (if rec { mixer.input_control() } else { mixer.output_control() }) else {
        continue; // no usable control for this direction
      };
      let Some(levels) = mixer.level(control) else {
        continue;
      };
      let mute = mixer.muted(control).unwrap_or(false);

      // Names are the session manager's persistence key: stable, no locale.
      // Derived from the pcm unit, not an ordinal - an ordinal shifts every
      // sibling when one unit's mixer fails to probe (attach-order race) or
      // the unit set changes, restoring saved volumes onto the wrong output.
      let (name, description) = if rec {
        n_in += 1;
        if n_in == 1 && device_count == 1 { ("oss-input".to_string(), "Input".to_string()) }
        else { (format!("oss-input-pcm{}", device.index), format!("Input (pcm{})", device.index)) }
      } else {
        n_out += 1;
        if n_out == 1 && device_count == 1 { ("oss-output".to_string(), "Output".to_string()) }
        else { (format!("oss-output-pcm{}", device.index), format!("Output (pcm{})", device.index)) }
      };

      routes.push(RouteState {
        node_id: device.index * 2 + rec as u32,
        rec,
        name,
        description,
        mixer: mixer_index,
        control,
        levels,
        mute,
        save: false
      });
      used = true;
    }

    if used {
      let counter = mixer.modify_counter().unwrap_or(0);
      mixers.push(MixerHandle { mixer, counter });
    }
  }

  (routes, mixers)
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

  // the main loop and system drive the mixer poll timer; both are optional -
  // without them external mixer changes just go unnoticed
  let main_loop = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop  .as_ptr().cast()) as *mut spa_loop;
  let system    = spa_support_find(support, n_support, SPA_TYPE_INTERFACE_System.as_ptr().cast()) as *mut spa_system;
  let main_loop = if main_loop.is_null() { None } else { Some(crate::spa::Loop  ::wrap(main_loop)) };
  let system    = if system   .is_null() { None } else { Some(crate::spa::System::wrap(system)) };

  let state = handle.cast::<State>().as_mut()
    .expect("handle is not supposed to be null");

  let mut pcm_parent_device  = None;
  let mut pcm_device_indexes = vec![];

  if let Some(info) = info.as_ref() {
    #[cfg(debug_assertions)]
    crate::spa::dump_spa_dict(info);

    crate::spa::for_each_dict_item(info, |key, value| {
      match key {
        crate::keys::PCM_PARENT_DEVICE => {
          pcm_parent_device = Some(value.to_string());
        },
        crate::keys::PCM_DEVICE_INDEXES =>
          for part in value.split(',') {
            if let Ok(index) = part.parse::<u32>() {
              pcm_device_indexes.push(index);
            }
          },
        _ => ()
      }
    });
  }

  if pcm_device_indexes.is_empty() {
    crate::error!(log, "{} should contain pcm device indexes", crate::keys::PCM_DEVICE_INDEXES);
    return -libc::EINVAL;
  }

  let pcm_devices = crate::sound::list_pcm_devices(&pcm_device_indexes);

  if pcm_devices.is_empty() {
    crate::error!(log, "can't retrieve pcm device information");
    return -libc::EINVAL;
  }

  let (routes, mixers) = probe_routes(&pcm_devices);

  let mut common_desc = pcm_devices[0].desc.clone();
  for pcm_device in &pcm_devices[1..] {

    let mut count = 0;

    for (a, b) in common_desc.bytes().zip(pcm_device.desc.bytes()) {
      if a == b {
        count += 1;
      } else {
        break;
      }
    }

    common_desc.truncate(count);
  }

  while common_desc.ends_with(' ') || common_desc.ends_with('(') {
    common_desc.truncate(common_desc.len() - 1);
  }

  std::ptr::write(state, State {
    handle: spa_handle {
      version:       SPA_VERSION_HANDLE,
      get_interface: Some(get_interface),
      clear:         Some(clear)
    },

    device: spa_device {
      iface: spa_interface {
        type_:   SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
        version: SPA_VERSION_DEVICE,
        cb: spa_callbacks {
          funcs: &DEVICE_IMPL as *const _ as *const c_void,
          data:  state as *mut _ as *mut c_void
        }
      }
    },

    dev_info: crate::spa::DeviceInfo::new(),

    hooks: spa_hook_list {
      list: spa_list {
        next: std::ptr::null_mut(),
        prev: std::ptr::null_mut()
      }
    },

    pcm_devices,
    description: common_desc,
    profile:      1, // default on until a session manager decides otherwise
    profile_save: false,

    routes,
    mixers,

    main_loop,
    system,

    timer_source: spa_source {
      loop_: std::ptr::null_mut(),
      func:  Some(on_mixer_timeout),
      data:  state as *mut _ as *mut c_void,
      fd:    -1,
      mask:  SPA_IO_IN,
      rmask: 0,
      priv_: std::ptr::null_mut()
    },
    timer_added: false,

    log
  });

  state.dev_info.fix_pointers();
  state.dev_info.add_prop(SPA_KEY_DEVICE_API .as_ptr(), "freebsd-oss");
  state.dev_info.add_prop(SPA_KEY_MEDIA_CLASS.as_ptr(), "Audio/Device");
  if let Some(pcm_parent_device) = pcm_parent_device {
    state.dev_info.add_prop(SPA_KEY_DEVICE_NAME.as_ptr(), pcm_parent_device);
  }
  state.dev_info.add_prop(SPA_KEY_DEVICE_DESCRIPTION.as_ptr(), state.description.as_str());
  state.dev_info.add_param(SPA_PARAM_EnumProfile, SPA_PARAM_INFO_READ);
  state.dev_info.add_param(SPA_PARAM_Profile,     SPA_PARAM_INFO_READWRITE);
  state.dev_info.add_param(SPA_PARAM_EnumRoute,   SPA_PARAM_INFO_READ);
  state.dev_info.add_param(SPA_PARAM_Route,       SPA_PARAM_INFO_READWRITE);

  spa_hook_list_init(&mut state.hooks);

  // ~1 Hz mixer poll; only worth arming when something is routed
  if !state.routes.is_empty() {
    if let (Some(main_loop), Some(system)) = (&state.main_loop, &state.system) {
      let fd = system.timerfd_create(libc::CLOCK_MONOTONIC, (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as c_int);
      if fd >= 0 {
        let timerspec = itimerspec {
          it_value:    timespec { tv_sec: 1, tv_nsec: 0 },
          it_interval: timespec { tv_sec: 1, tv_nsec: 0 }
        };
        system.timerfd_settime(fd, 0, &timerspec, std::ptr::null_mut());
        state.timer_source.fd = fd;
        if main_loop.add_source(&mut state.timer_source) >= 0 {
          state.timer_added = true;
        } else {
          crate::warn!(state.log, "can't watch the mixer; external volume changes won't be noticed");
          system.close(fd);
          state.timer_source.fd = -1;
        }
      }
    }
  }

  0
}

const INTERFACE_INFO: [spa_interface_info; 1] = [
  spa_interface_info {
    type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast()
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

const OSS_DEVICE_FACTORY_INFO: spa_dict = spa_dict {
  flags:   0,
  n_items: 0,
  items:   std::ptr::null()
};

pub const OSS_DEVICE_FACTORY: spa_handle_factory = spa_handle_factory {
  version:             SPA_VERSION_HANDLE_FACTORY,
  name:                c"freebsd-oss.device".as_ptr(),
  info:                &OSS_DEVICE_FACTORY_INFO,
  get_size:            Some(get_size),
  init:                Some(init),
  enum_interface_info: Some(enum_interface_info)
};
