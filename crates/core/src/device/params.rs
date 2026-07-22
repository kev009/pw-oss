use super::*;
use crate::spa::{self, deserialize_pod, pod_prop};

#[cfg(test)]
mod tests;

// Route properties accepted from set_param. Unknown properties are ignored
// because session managers may include adapter-owned values such as
// softVolumes. Retaining pod order preserves the caller's control-write order,
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
        #[expect(non_upper_case_globals)]
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
                #[expect(non_upper_case_globals)]
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
fn set_profile_param<B: backend::Backend>(
    state: &mut Runtime<B>,
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

        // The poll idles while Off, so external control changes may have gone
        // unseen; refresh every shadow BEFORE the bump re-announces Route
        // pods, or consumers read stale volumes for up to a tick.
        if index != 0 {
            state.route_controller.refresh_all(&mut state.routes);
        }

        // add or remove the nodes, then re-announce the params tied to the
        // active profile (Route pods appear/vanish with it)
        notifications.extend(
            object_events(&state.snapshot, &state.routes, index != 0)
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
fn resolve_route_pos<B: backend::Backend>(
    state: &Runtime<B>,
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
        #[expect(non_upper_case_globals)]
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
fn set_route_param<B: backend::Backend>(
    state: &mut Runtime<B>,
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

    let values = props
        .into_iter()
        .flat_map(|props| props.0)
        .filter_map(|prop| match prop {
            RouteProp::Mute(mute) => Some(backend::RouteValueUpdate::Mute(mute)),
            RouteProp::ChannelVolumes(values) => {
                (!values.is_empty()).then_some(backend::RouteValueUpdate::Volume(values))
            }
        })
        .collect();
    let route_key = state.routes[pos].key;
    let activate = !state.routes[pos].active;
    let change = state.route_controller.apply(
        &mut state.routes,
        route_key,
        backend::RouteUpdate { activate, values },
    );
    if let Some(diagnostic) = &change.diagnostic {
        log_route_diagnostic(&state.log, diagnostic);
    }
    if change.refresh || save_changed {
        queue_route_change(state, notifications);
    }
    if let Some(key) = change.key
        && let Some(changed_pos) = state.routes.iter().position(|route| route.key == key)
    {
        if change.volume {
            queue_object_config(state, changed_pos, true, notifications);
        }
        if change.mute {
            queue_object_config(state, changed_pos, false, notifications);
        }
    }

    0
}

pub(super) unsafe extern "C" fn set_param<B: backend::Backend>(
    object: *mut c_void,
    id: u32,
    _flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state: *mut State<B> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");

    #[expect(non_upper_case_globals)]
    match id {
        SPA_PARAM_Profile | SPA_PARAM_Route => (),
        _ => return -libc::ENOENT, // SPA methods reject an unknown param id
    }

    // Deserialize the pod before borrowing State. None represents a NULL pod.
    let value = if param.is_null() {
        None
    } else {
        match unsafe { deserialize_pod(param) } {
            Some(value) => Some(value),
            None => return -libc::EINVAL,
        }
    };

    // Validate the request before mutating device state.
    let (events, result, notifications) = unsafe {
        with_runtime_mut(state, |state| {
            let mut notifications = Vec::new();
            #[expect(non_upper_case_globals)]
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

pub(super) fn build_profile_info(
    id: u32,
    index: u32,
    snapshot: &backend::DeviceSnapshot,
    profile_save: bool,
    current: bool,
) -> Vec<u8> {
    use libspa::pod::{Object, Value, ValueArray};
    use libspa::utils::Id;

    let (name, description, priority) = if index == 0 {
        ("off", "Off", 0)
    } else {
        ("default", "Default", 100)
    };

    // WirePlumber's route selection walks the classes struct to map nodes to
    // this profile; without it no route is applied. List every node whether
    // or not it currently has a route.
    let mut capture: Vec<i32> = vec![];
    let mut playback: Vec<i32> = vec![];
    for endpoint in &snapshot.endpoints {
        match endpoint.direction {
            backend::StreamDirection::Playback => playback.push(endpoint.object_id as i32),
            backend::StreamDirection::Capture => capture.push(endpoint.object_id as i32),
        }
    }

    let classes: [(&str, &Vec<i32>); 2] = [("Audio/Source", &capture), ("Audio/Sink", &playback)];
    let n_classes = if index == 0 {
        0
    } else {
        classes.iter().filter(|(_, ids)| !ids.is_empty()).count()
    };

    let mut class_fields = vec![Value::Int(n_classes as i32)];
    if index != 0 {
        for (class, ids) in classes {
            if ids.is_empty() {
                continue;
            }
            class_fields.push(Value::Struct(vec![
                Value::String(class.to_string()),
                Value::Int(ids.len() as i32),
                Value::String("card.profile.devices".to_string()),
                Value::ValueArray(ValueArray::Int(ids.clone())),
            ]));
        }
    }

    let mut properties = vec![
        pod_prop(SPA_PARAM_PROFILE_index, Value::Int(index as i32)),
        pod_prop(SPA_PARAM_PROFILE_name, Value::String(name.to_string())),
        pod_prop(
            SPA_PARAM_PROFILE_description,
            Value::String(description.to_string()),
        ),
        pod_prop(SPA_PARAM_PROFILE_priority, Value::Int(priority)),
        pod_prop(
            SPA_PARAM_PROFILE_available,
            Value::Id(Id(SPA_PARAM_AVAILABILITY_yes)),
        ),
        pod_prop(SPA_PARAM_PROFILE_classes, Value::Struct(class_fields)),
    ];

    if current {
        properties.push(pod_prop(SPA_PARAM_PROFILE_save, Value::Bool(profile_save)));
    }

    spa::serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamProfile,
        id,
        properties,
    }))
}

// EnumRoute (full = false) carries the static description only; Route
// (full = true) adds device/profile/save and the volume props object.
pub(super) fn build_route_info(
    id: u32,
    route: &RouteState,
    pos: usize,
    profile: u32,
    full: bool,
) -> Vec<u8> {
    use libspa::pod::{Object, Property, PropertyFlags, Value, ValueArray};
    use libspa::utils::Id;

    let mut properties = vec![
        pod_prop(SPA_PARAM_ROUTE_index, Value::Int(pos as i32)),
        // note: PLAYBACK maps to OUTPUT here (the route points out of the graph)
        pod_prop(
            SPA_PARAM_ROUTE_direction,
            Value::Id(Id(
                if route.direction == backend::StreamDirection::Capture {
                    SPA_DIRECTION_INPUT
                } else {
                    SPA_DIRECTION_OUTPUT
                },
            )),
        ),
        pod_prop(SPA_PARAM_ROUTE_name, Value::String(route.name.clone())),
        pod_prop(
            SPA_PARAM_ROUTE_description,
            Value::String(route.description.clone()),
        ),
        pod_prop(SPA_PARAM_ROUTE_priority, Value::Int(route.priority)),
        pod_prop(
            SPA_PARAM_ROUTE_available,
            Value::Id(Id(match route.availability {
                backend::RouteAvailability::Yes => SPA_PARAM_AVAILABILITY_yes,
                backend::RouteAvailability::No => SPA_PARAM_AVAILABILITY_no,
                backend::RouteAvailability::Unknown => SPA_PARAM_AVAILABILITY_unknown,
            })),
        ),
        pod_prop(
            SPA_PARAM_ROUTE_profiles,
            Value::ValueArray(ValueArray::Int(vec![1])),
        ),
        pod_prop(
            SPA_PARAM_ROUTE_devices,
            Value::ValueArray(ValueArray::Int(vec![route.node_id as i32])),
        ),
    ];

    if full {
        properties.push(pod_prop(
            SPA_PARAM_ROUTE_device,
            Value::Int(route.node_id as i32),
        ));

        // Volume writers (pulse, the session manager) direct volume at the card
        // whenever an ACTIVE Route exists, regardless of props presence
        // (pulse-server.c:3004-3010 gates on active_port) - so even a source
        // with no level control must carry props, backed by a soft shadow that
        // audioconvert applies (the acp softvol model). The HARDWARE flag and
        // unity softVolumes apply only when a real control exists.
        let hw = route.volume.hardware;
        let volume_flag = if hw {
            PropertyFlags::HARDWARE
        } else {
            PropertyFlags::empty()
        };
        let mute_flag = if route.mute.hardware {
            PropertyFlags::HARDWARE
        } else {
            PropertyFlags::empty()
        };
        let volumes = route.volume.values.clone();
        // with hardware attenuation the node's software volume must stay at
        // unity or the signal is attenuated twice; a soft route IS the
        // software volume, so it mirrors the levels
        let soft = if hw {
            vec![1.0; route.volume.channels.len()]
        } else {
            volumes.clone()
        };
        properties.push(pod_prop(
            SPA_PARAM_ROUTE_props,
            Value::Object(Object {
                type_: SPA_TYPE_OBJECT_Props,
                id,
                properties: vec![
                    Property {
                        key: SPA_PROP_mute,
                        flags: mute_flag,
                        value: Value::Bool(route.mute.value),
                    },
                    Property {
                        key: SPA_PROP_channelVolumes,
                        flags: volume_flag,
                        value: Value::ValueArray(ValueArray::Float(volumes)),
                    },
                    Property {
                        key: SPA_PROP_volumeBase,
                        flags: PropertyFlags::READONLY,
                        value: Value::Float(route.volume.base),
                    },
                    Property {
                        key: SPA_PROP_volumeStep,
                        flags: PropertyFlags::READONLY,
                        value: Value::Float(route.volume.step),
                    },
                    pod_prop(
                        SPA_PROP_channelMap,
                        Value::ValueArray(ValueArray::Id(
                            route.volume.channels.iter().map(|&c| Id(c)).collect(),
                        )),
                    ),
                    pod_prop(
                        SPA_PROP_softVolumes,
                        Value::ValueArray(ValueArray::Float(soft)),
                    ),
                ],
            }),
        ));

        properties.push(pod_prop(
            SPA_PARAM_ROUTE_profile,
            Value::Int(profile as i32),
        ));
        properties.push(pod_prop(SPA_PARAM_ROUTE_save, Value::Bool(route.save)));
    }

    spa::serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamRoute,
        id,
        properties,
    }))
}
