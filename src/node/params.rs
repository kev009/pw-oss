use super::events::{emit_node_info, handle_process_latency};
use super::*;

// Updates accepted from a Props pod. None means the property was absent.
// The sink consumes oss_delay and the source ignores it. Capping oss_delay
// and normalizing oss.fragment happen when the update is applied so readback
// reports the effective value.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct PropsUpdate {
    pub latency_offset_ns: Option<i64>,
    pub oss_delay: Option<u32>,
    pub oss_fragment: Option<u32>,
}

// Validated node parameter requests. Raw pods do not cross this boundary.
pub(crate) enum NodeParamRequest {
    ResetProps, // set_param(Props, NULL)
    Props(PropsUpdate),
    ResetProcessLatency, // set_param(ProcessLatency, NULL)
    ProcessLatency(spa_process_latency_info),
}

// Parse a deserialized Props object. The adapter owns soft-volume properties,
// unknown keys are logged and skipped, and invalid oss.* values are ignored.
pub(super) fn parse_props_update(
    properties: Vec<libspa::pod::Property>,
    log: &crate::spa::Log,
) -> PropsUpdate {
    use libspa::pod::Value;

    let mut update = PropsUpdate::default();
    for property in properties {
        #[expect(non_upper_case_globals)]
        match property.key {
            // softvol-handled by the adapter
            SPA_PROP_volume
            | SPA_PROP_mute
            | SPA_PROP_channelVolumes
            | SPA_PROP_channelMap
            | SPA_PROP_monitorMute
            | SPA_PROP_monitorVolumes
            | SPA_PROP_softMute
            | SPA_PROP_softVolumes => (),
            SPA_PROP_latencyOffsetNsec => {
                if let Value::Long(ns) = property.value {
                    update.latency_offset_ns = Some(ns);
                }
            }
            // pw-cli set-param <object-id> Props '{ "params": ["oss.delay", 8]}'
            SPA_PROP_params => parse_oss_params(&property.value, &mut update),
            key => {
                crate::debug!(log, "ignoring unknown prop {}", key);
            }
        }
    }
    update
}

// the SPA_PROP_params payload: a Struct of ("key", value) pairs
pub(super) fn parse_oss_params(value: &libspa::pod::Value, update: &mut PropsUpdate) {
    use libspa::pod::Value;
    let Value::Struct(values) = value else {
        return;
    };
    if values.len() % 2 != 0 {
        return;
    }
    for kv in values.chunks(2) {
        match (&kv[0], &kv[1]) {
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_DELAY && *x >= 0 => {
                update.oss_delay = Some(*x as u32);
            }
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_FRAGMENT && *x >= 0 => {
                update.oss_fragment = Some(*x as u32);
            }
            _ => (),
        }
    }
}

// Apply a validated request to the main-loop model. Data-loop effects cross
// only through DataControl. Props apply in this order: latency offset,
// oss.delay, then oss.fragment. The first failing oss.* update returns its
// errno.
pub(crate) fn apply_node_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    request: NodeParamRequest,
) -> c_int {
    match request {
        NodeParamRequest::ResetProps => {
            let res = D::reset_props(state, data);
            if res == 0 {
                state.events.with_node_info(|info| {
                    let _ = info.replace_change_mask(0);
                    info.bump_param(SPA_PARAM_Props);
                });
                emit_node_info(state);
            }
            res
        }
        NodeParamRequest::Props(update) => {
            if let Some(ns) = update.latency_offset_ns {
                let mut info = state.process_latency;
                info.ns = ns;
                handle_process_latency(state, info);
            }
            if let Some(delay) = update.oss_delay {
                let res = D::apply_oss_delay(state, data, delay);
                if res != 0 {
                    return res;
                }
            }
            if let Some(fragment) = update.oss_fragment {
                // stored normalized, so the Props readback reports the
                // effective (rounded/clamped) value, not the raw request
                let new_fragment = normalize_fragment(fragment);
                if new_fragment != state.oss_fragment {
                    // unchanged echoes must not rebuild a running device
                    let old_fragment = state.oss_fragment;
                    // install_device consumes the main-loop copy while the
                    // data-loop store/rebuild is in progress.
                    state.oss_fragment = new_fragment;
                    let res = apply_props_param(state, data, move |state| {
                        state.oss_fragment = new_fragment;
                    });
                    if res != 0 {
                        state.oss_fragment = old_fragment;
                        return res;
                    }
                }
            }
            0
        }
        NodeParamRequest::ResetProcessLatency => {
            handle_process_latency(state, crate::spa::process_latency_default());
            0
        }
        NodeParamRequest::ProcessLatency(info) => {
            handle_process_latency(state, info);
            0
        }
    }
}

pub(super) unsafe extern "C" fn set_param<D: Direction>(
    object: *mut c_void,
    id: u32,
    _flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let log = unsafe { main_ref(state).log.clone() };

    use libspa::pod::{Object, Value};

    // Reject unknown ids before reading the pod. NULL resets a known
    // parameter; malformed or mistyped pods return -EINVAL.
    #[expect(non_upper_case_globals)]
    let request = match id {
        SPA_PARAM_Props => {
            if param.is_null() {
                // a NULL pod resets the props to their defaults
                NodeParamRequest::ResetProps
            } else {
                // Deserialize before borrowing State.
                match unsafe { crate::spa::deserialize_pod(param) } {
                    Some(Value::Object(Object {
                        type_, properties, ..
                    })) if type_ == SPA_TYPE_OBJECT_Props => {
                        NodeParamRequest::Props(parse_props_update(properties, &log))
                    }
                    _ => return -libc::EINVAL,
                }
            }
        }
        SPA_PARAM_ProcessLatency => {
            if param.is_null() {
                NodeParamRequest::ResetProcessLatency
            } else {
                // Deserialize before borrowing State.
                let value = unsafe { crate::spa::deserialize_pod(param) };
                match crate::spa::parse_process_latency_info(value.as_ref()) {
                    Some(info) => NodeParamRequest::ProcessLatency(info),
                    None => return -libc::EINVAL,
                }
            }
        }
        id => {
            crate::warn!(log, "set_param: unknown param {}", id);
            return -libc::ENOENT;
        }
    };
    let control = unsafe { DataControl::from_raw(state) };
    let (events, result) = {
        // All info emissions produced by the safe phase are queued as owned
        // snapshots. End this State borrow before invoking any listener.
        let state = unsafe { main_mut(state) };
        let events = state.events.clone();
        let result = apply_node_param(state, &control, request);
        (events, result)
    };
    // SAFETY: the scoped State borrow above ended before this dispatch.
    unsafe { events.flush() };
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    // Known Props populate the update; adapter-owned, unknown, and invalid
    // values are ignored.
    #[test]
    fn props_update_parses_known_keys_and_drops_the_rest() {
        use crate::spa::pod_prop;
        use libspa::pod::Value;
        let log = crate::spa::Log::test_null();

        let params = Value::Struct(vec![
            Value::String(crate::keys::OSS_DELAY.into()),
            Value::Int(8),
            Value::String(crate::keys::OSS_FRAGMENT.into()),
            Value::Int(4096),
            Value::String("bogus.key".into()),
            Value::Int(1),
        ]);
        let update = parse_props_update(
            vec![
                pod_prop(SPA_PROP_volume, Value::Float(1.0)), // softvol: adapter's
                pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(250_000)),
                pod_prop(SPA_PROP_params, params),
                pod_prop(0x77777, Value::Int(3)), // unknown key: logged, skipped
            ],
            &log,
        );
        assert_eq!(
            update,
            PropsUpdate {
                latency_offset_ns: Some(250_000),
                oss_delay: Some(8),
                oss_fragment: Some(4096),
            }
        );

        // negative values are ignored, an odd-length struct is ignored whole,
        // and a mistyped latency offset stays None
        let update = parse_props_update(
            vec![
                pod_prop(SPA_PROP_latencyOffsetNsec, Value::Int(250_000)),
                pod_prop(
                    SPA_PROP_params,
                    Value::Struct(vec![
                        Value::String(crate::keys::OSS_DELAY.into()),
                        Value::Int(-1),
                    ]),
                ),
            ],
            &log,
        );
        assert_eq!(update, PropsUpdate::default());
        let update = parse_props_update(
            vec![pod_prop(
                SPA_PROP_params,
                Value::Struct(vec![Value::String(crate::keys::OSS_DELAY.into())]),
            )],
            &log,
        );
        assert_eq!(update, PropsUpdate::default());
    }
}
