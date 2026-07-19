use super::*;

// Route properties accepted from set_param. Unknown properties are ignored
// because session managers may include adapter-owned values such as
// softVolumes. Retaining pod order preserves the caller's mixer-write order,
// including duplicate properties and partial hardware failures.
#[derive(Debug, Default, PartialEq)]
pub(super) struct RouteProps(pub(super) Vec<RouteProp>);

#[derive(Debug, PartialEq)]
pub(super) enum RouteProp {
    Mute(bool),
    ChannelVolumes(Vec<f32>),
}

fn decode_route_props(object: libspa::pod::Object) -> RouteProps {
    use libspa::pod::{Value, ValueArray};

    let mut props = Vec::new();
    for p in object.properties {
        #[allow(non_upper_case_globals)]
        match (p.key, p.value) {
            (SPA_PROP_mute, Value::Bool(mute)) => props.push(RouteProp::Mute(mute)),
            (SPA_PROP_channelVolumes, Value::ValueArray(ValueArray::Float(v))) if !v.is_empty() => {
                props.push(RouteProp::ChannelVolumes(v));
            }
            _ => (),
        }
    }
    RouteProps(props)
}

// Apply route properties to the hardware and cached state.
fn apply_route_props(
    state: &mut Runtime,
    pos: usize,
    props: RouteProps,
    vol_changed: &mut bool,
    mute_changed: &mut bool,
) {
    // the cached control may lag a recording-source change by up to a poll
    // tick; a write must target the CURRENT source
    resolve_recsrc(state, pos);
    let mi = state.routes[pos].mixer;
    // a control-less route is a soft one: writes land in the shadow only, and
    // emit_object_config pushes them into the node's softVolumes
    let control = state.routes[pos].control;

    for prop in props.0 {
        match prop {
            RouteProp::Mute(mute) => {
                if mute != state.routes[pos].mute {
                    let applied = match control {
                        Some(c) => state.mixers[mi].mixer.set_muted(c, mute),
                        None => true, // soft route: the shadow is the state
                    };
                    if applied {
                        state.routes[pos].mute = mute;
                        *mute_changed = true;
                    }
                }
            }
            RouteProp::ChannelVolumes(v) => {
                let Some(&left) = v.first() else {
                    continue;
                };
                let right = v.get(1).copied().unwrap_or(left);
                // any width is accepted: the mixer uses the first two values,
                // with a mono request fanned out to both channels
                crate::debug!(
                    state.log,
                    "route {} channelVolumes {:?}",
                    state.routes[pos].name,
                    v
                );
                let levels = (linear_to_oss(left), linear_to_oss(right));
                if levels != state.routes[pos].levels {
                    let applied = match control {
                        Some(c) => state.mixers[mi].mixer.set_level(c, levels.0, levels.1),
                        None => true,
                    };
                    if applied {
                        state.routes[pos].levels = levels;
                        *vol_changed = true;
                    }
                }
            }
        }
    }
}

// A validated Profile request. Profiles may be addressed by index or name;
// both forms resolve before the device state is borrowed. A NULL pod selects
// the default profile, and an invalid address returns -EINVAL.
#[derive(Debug, PartialEq)]
pub(super) struct ProfileRequest {
    pub(super) index: u32, // 0 = off, 1 = default
    pub(super) save: bool,
}

pub(super) fn decode_profile_request(
    value: Option<libspa::pod::Value>,
) -> Result<ProfileRequest, c_int> {
    use libspa::pod::Value;

    let mut index = None;
    let mut name = None;
    let mut save = false;
    match value {
        None => index = Some(1),
        Some(Value::Object(o)) if o.type_ == SPA_TYPE_OBJECT_ParamProfile => {
            for p in o.properties {
                #[allow(non_upper_case_globals)]
                match (p.key, p.value) {
                    (SPA_PARAM_PROFILE_index, Value::Int(v)) if (0..=1).contains(&v) => {
                        index = Some(v as u32);
                    }
                    (SPA_PARAM_PROFILE_name, Value::String(v)) => name = Some(v),
                    (SPA_PARAM_PROFILE_save, Value::Bool(v)) => save = v,
                    _ => (),
                }
            }
        }
        _ => return Err(-libc::EINVAL),
    }

    // session managers may address profiles by name instead of index
    if index.is_none() {
        index = match name.as_deref() {
            Some("off") => Some(0),
            Some("default") => Some(1),
            _ => None,
        };
    }

    match index {
        Some(index) => Ok(ProfileRequest { index, save }),
        None => Err(-libc::EINVAL),
    }
}

// Apply a resolved Profile request: 0 is Off and 1 is the default profile.
fn set_profile_param(
    state: &mut Runtime,
    request: ProfileRequest,
    notifications: &mut Vec<DeviceNotification>,
) -> c_int {
    let ProfileRequest { index, save } = request;

    let profile_save_changed = state.profile_save != save;
    state.profile_save = save;

    if state.profile != index {
        state.profile = index;
        crate::info!(
            state.log,
            "profile -> {}",
            if index == 0 { "off" } else { "default" }
        );

        // The poll idles while Off, so external mixer changes may have gone
        // unseen; refresh every shadow BEFORE the bump re-announces Route
        // pods, or consumers read stale volumes for up to a tick.
        if index != 0 {
            // the recording source may have moved too; re-derive the active
            // routes before their shadows are read
            for mi in 0..state.mixers.len() {
                let _ = sync_recsrc(state, mi);
            }
            for pos in 0..state.routes.len() {
                refresh_route_shadow(state, pos);
            }
        }

        // add or remove the nodes, then re-announce the params tied to the
        // active profile (Route pods appear/vanish with it)
        notifications.extend(
            object_events(
                &state.pcm_devices,
                &state.routes,
                &state.description,
                index != 0,
            )
            .into_iter()
            .map(DeviceNotification::Object),
        );

        state.events.with_info(|info| {
            let _ = info.replace_change_mask(0);
            info.bump_param(SPA_PARAM_Profile);
            info.bump_param(SPA_PARAM_EnumRoute);
            info.bump_param(SPA_PARAM_Route);
        });
        notifications.push(DeviceNotification::Info(state.events.take_info()));
    } else if profile_save_changed {
        // the save flag is part of the Profile readback; keep it fresh
        state.events.with_info(|info| {
            let _ = info.replace_change_mask(0);
            info.bump_param(SPA_PARAM_Profile);
        });
        notifications.push(DeviceNotification::Info(state.events.take_info()));
    }

    0
}

// resolve a Route pod's (index, name, device) triple to a routes[] position.
// Device consistency is required: a stale in-range index (route set changed
// since the state was saved) must lose to the durable name instead of
// winning and then failing the device check.
fn resolve_route_pos(
    state: &Runtime,
    index: Option<usize>,
    name: Option<&str>,
    device: u32,
) -> Option<usize> {
    index
        .filter(|i| *i < state.routes.len() && state.routes[*i].node_id == device)
        // sibling source routes share node_id, so a stale index passes the
        // device filter; the durable name wins whenever it disagrees
        .filter(|i| match name {
            Some(nm) => state.routes[*i].name == nm,
            None => true,
        })
        .or_else(|| {
            name.and_then(|n| {
                state
                    .routes
                    .iter()
                    .position(|r| r.name == n && r.node_id == device)
            })
        })
}

// A validated Route request. The handler resolves its index/name/device
// address against the live route table before applying properties. The
// device field is required, and Route has no NULL reset operation.
#[derive(Debug, PartialEq)]
pub(super) struct RouteRequest {
    pub(super) index: Option<usize>,
    pub(super) name: Option<String>,
    pub(super) device: u32,
    pub(super) save: bool,
    pub(super) props: Option<RouteProps>,
}

pub(super) fn decode_route_request(
    value: Option<libspa::pod::Value>,
) -> Result<RouteRequest, c_int> {
    use libspa::pod::Value;

    let object = match value {
        Some(Value::Object(o)) if o.type_ == SPA_TYPE_OBJECT_ParamRoute => o,
        // includes None (a NULL pod): there is no route state to reset
        _ => return Err(-libc::EINVAL),
    };

    let mut index = None;
    let mut name = None;
    let mut device = None;
    let mut save = false;
    let mut props = None;

    for p in object.properties {
        #[allow(non_upper_case_globals)]
        match (p.key, p.value) {
            (SPA_PARAM_ROUTE_index, Value::Int(v)) if v >= 0 => index = Some(v as usize),
            (SPA_PARAM_ROUTE_name, Value::String(v)) => name = Some(v),
            (SPA_PARAM_ROUTE_device, Value::Int(v)) if v >= 0 => device = Some(v as u32),
            (SPA_PARAM_ROUTE_save, Value::Bool(v)) => save = v,
            (SPA_PARAM_ROUTE_props, Value::Object(o)) if o.type_ == SPA_TYPE_OBJECT_Props => {
                props = Some(decode_route_props(o));
            }
            _ => (),
        }
    }

    let Some(device) = device else {
        return Err(-libc::EINVAL);
    };
    Ok(RouteRequest {
        index,
        name,
        device,
        save,
        props,
    })
}

// Apply route properties, a recording-source switch, or both.
fn set_route_param(
    state: &mut Runtime,
    request: RouteRequest,
    notifications: &mut Vec<DeviceNotification>,
) -> c_int {
    if state.profile == 0 {
        return -libc::EINVAL; // no routes exist under the Off profile
    }

    let RouteRequest {
        index,
        name,
        device,
        save,
        props,
    } = request;

    let Some(pos) = resolve_route_pos(state, index, name.as_deref(), device) else {
        return -libc::EINVAL;
    };

    let save_changed = state.routes[pos].save != save;
    state.routes[pos].save = save;

    // Selecting an inactive source route is a port switch: write RECSRC
    // with that source's bit. The kernel may strip it or fall back
    // (mixer.c:347-357) and the driver decides what it really applied, so
    // the readback in sync_recsrc names the route that became active.
    let mut switched = None;
    if state.routes[pos].source.is_some() && !state.routes[pos].active {
        let mi = state.routes[pos].mixer;
        let bit = state.routes[pos].source.unwrap_or(0);
        if state.mixers[mi].mixer.set_recsrc(1 << bit) {
            switched = sync_recsrc(state, mi);
            if switched != Some(pos) {
                crate::info!(
                    state.log,
                    "kernel did not move the recording source to route {}",
                    state.routes[pos].name
                );
                // re-announce even so: the session manager applied the switch
                // optimistically and must re-read what really happened
                queue_route_change(state, notifications);
            }
        } else {
            crate::warn!(
                state.log,
                "can't select the recording source for route {}",
                state.routes[pos].name
            );
        }
    }

    // a port-switch message carries no props and must not touch the volume
    let mut vol_changed = false;
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

    // bump only on an observable change: every spurious serial flip costs
    // the session manager a full param re-enumeration
    if vol_changed || mute_changed || save_changed || switched.is_some() {
        queue_route_change(state, notifications);
    }

    // A switch changes which control feeds the node, so push the newly
    // active route's state unless the props above already did. Props that
    // rode a DEFLECTED switch were applied to a now-inactive route; the
    // active gate keeps them off the node.
    if vol_changed && !state.routes[pos].active {
        vol_changed = false;
        mute_changed = false;
    }
    if let Some(active_pos) = switched {
        if !(active_pos == pos && vol_changed) {
            queue_object_config(state, active_pos, true, notifications);
        }
        if !(active_pos == pos && mute_changed) {
            queue_object_config(state, active_pos, false, notifications);
        }
    }

    if vol_changed {
        queue_object_config(state, pos, true, notifications);
    }
    if mute_changed {
        queue_object_config(state, pos, false, notifications);
    }

    0
}

pub(super) unsafe extern "C" fn set_param(
    object: *mut c_void,
    id: u32,
    _flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");

    #[allow(non_upper_case_globals)]
    match id {
        SPA_PARAM_Profile | SPA_PARAM_Route => (),
        _ => return -libc::ENOENT, // unknown param id (ALSA convention)
    }

    // Deserialize the pod before borrowing State. None represents a NULL pod.
    let value = if param.is_null() {
        None
    } else {
        match unsafe { crate::utils::deserialize_pod(param) } {
            Some(value) => Some(value),
            None => return -libc::EINVAL,
        }
    };

    // Validate the request before mutating device state.
    let (events, result, notifications) = unsafe {
        with_runtime_mut(state, |state| {
            let mut notifications = Vec::new();
            #[allow(non_upper_case_globals)]
            let result = match id {
                SPA_PARAM_Profile => match decode_profile_request(value) {
                    Ok(request) => set_profile_param(state, request, &mut notifications),
                    Err(err) => err,
                },
                SPA_PARAM_Route => match decode_route_request(value) {
                    Ok(request) => set_route_param(state, request, &mut notifications),
                    Err(err) => err,
                },
                _ => -libc::ENOENT, // filtered above
            };
            (state.events.clone(), result, notifications)
        })
    };
    // SAFETY: the mutation phase's State borrow ended above.
    unsafe { events.dispatch_all(notifications) };
    result
}
