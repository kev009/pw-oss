use std::mem::MaybeUninit;
use std::os::raw::{c_char, c_int, c_void};
use libspa::sys::*;

#[repr(C)]
struct State {
  handle:      spa_handle,
  device:      spa_device,
  dev_info:    crate::spa::DeviceInfo,
  hooks:       spa_hook_list,
  pcm_devices: Vec<crate::sound::PcmDevice>,
  description: String,
  profile:     u32, // 0 = off, 1 = default
  log:         crate::spa::Log
}

// emit (or, with present = false, retract) the node object for every pcm device
unsafe fn emit_objects(f: &spa_device_events, data: *mut c_void, pcm_devices: &[crate::sound::PcmDevice], description: &str, present: bool) {

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

unsafe fn build_profile_info(b: &mut libspa::pod::builder::Builder, id: u32, index: u32) -> Result<(), rustix::io::Errno> {

  let (name, description, priority) = if index == 0 {
    ("off", "Off", 0)
  } else {
    ("default", "Default", 100)
  };

  let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

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

  b.pop(frame.assume_init_mut());

  Ok(())
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

    emit_objects(f, entry.cb.data, &state.pcm_devices, &state.description, state.profile != 0);
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

  assert_ne!(max, 0);

  let mut buffer = vec![];

  let mut index = start;
  let mut count = 0;

  while count < max {

    use libspa::pod::builder::Builder;

    let mut builder = Builder::new(&mut buffer);

    #[allow(non_upper_case_globals)]
    match (id, index) {
      (SPA_PARAM_EnumProfile, 0 | 1) => build_profile_info(&mut builder, SPA_PARAM_EnumProfile, index).unwrap(),
      (SPA_PARAM_EnumProfile, _)     => return 0,
      (SPA_PARAM_Profile, 0)         => build_profile_info(&mut builder, SPA_PARAM_Profile, state.profile).unwrap(),
      (SPA_PARAM_Profile, _)         => return 0,
      _ => return -libc::ENOENT
    };

    let mut result = spa_result_device_params { id, index, next: index + 1, param: std::ptr::null_mut() };

    if spa_pod_filter(builder.as_raw_ptr(), &mut result.param, buffer.as_mut_ptr() as *mut spa_pod, filter) >= 0 {
      crate::spa::dev_emit_result(&mut state.hooks, seq, 0, SPA_RESULT_TYPE_DEVICE_PARAMS, &result);
      count += 1;
    }

    index += 1;
  }

  0
}

unsafe extern "C" fn set_param(object: *mut c_void, id: u32, _flags: u32, param: *const spa_pod) -> c_int {

  let state = object.cast::<State>().as_mut()
    .expect("object is not supposed to be null");

  #[allow(non_upper_case_globals)]
  match id {
    SPA_PARAM_Profile => {

      if param.is_null() {
        return -libc::EINVAL;
      }

      use libspa::pod::{Value, Object, Pod};
      use libspa::pod::deserialize::PodDeserializer;

      let mut index = None;
      match PodDeserializer::deserialize_any_from(Pod::from_raw(param).as_bytes()) {
        Ok((_, Value::Object(Object { type_, properties, .. }))) if type_ == SPA_TYPE_OBJECT_ParamProfile => {
          for p in properties {
            #[allow(non_upper_case_globals)]
            match (p.key, p.value) {
              (SPA_PARAM_PROFILE_index, Value::Int(v)) if (0..=1).contains(&v) => index = Some(v as u32),
              _ => ()
            }
          }
        },
        _ => return -libc::EINVAL
      }

      let Some(index) = index else {
        return -libc::EINVAL;
      };

      if state.profile != index {
        state.profile = index;
        crate::info!(state.log, "profile -> {}", if index == 0 { "off" } else { "default" });

        // add or remove the nodes, then re-announce the Profile param
        crate::spa::for_each_hook(&mut state.hooks, |entry| {
          let f = entry.cb.funcs.cast::<spa_device_events>().as_ref()
            .expect("hook should be initialized");
          assert!(f.version >= SPA_VERSION_DEVICE_EVENTS);
          emit_objects(f, entry.cb.data, &state.pcm_devices, &state.description, index != 0);
        });

        let _ = state.dev_info.replace_change_mask(0);
        state.dev_info.bump_param(SPA_PARAM_Profile);
        emit_device_info(state);
      }

      0
    },
    // Route lands together with mixer/hardware volume support
    _ => -libc::ENOTSUP
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
    profile:     1, // default on until a session manager decides otherwise
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
  // EnumRoute/Route land together with mixer/hardware volume support

  spa_hook_list_init(&mut state.hooks);

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
