use super::events::{emit_node_info, handle_process_latency};
use super::*;
use crate::backend::BackendProperties as _;
use crate::spa::{self, Log, deserialize_pod};

pub(crate) struct PropsUpdate<D: Direction> {
    pub latency_offset_ns: Option<i64>,
    pub backend: Vec<BackendPropertyUpdateOf<D>>,
}

impl<D: Direction> std::fmt::Debug for PropsUpdate<D> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PropsUpdate")
            .field("latency_offset_ns", &self.latency_offset_ns)
            .field("backend", &self.backend)
            .finish()
    }
}

impl<D: Direction> PartialEq for PropsUpdate<D> {
    fn eq(&self, other: &Self) -> bool {
        self.latency_offset_ns == other.latency_offset_ns && self.backend == other.backend
    }
}

impl<D: Direction> Default for PropsUpdate<D> {
    fn default() -> Self {
        Self {
            latency_offset_ns: None,
            backend: Vec::new(),
        }
    }
}

// Validated node parameter requests. Raw pods do not cross this boundary.
pub(crate) enum NodeParamRequest<D: Direction> {
    ResetProps, // set_param(Props, NULL)
    Props(PropsUpdate<D>),
    ResetProcessLatency, // set_param(ProcessLatency, NULL)
    ProcessLatency(spa_process_latency_info),
}

// Parse a deserialized Props object. The adapter owns soft-volume properties,
// unknown keys are logged and skipped, and invalid backend stream values are
// ignored.
pub(super) fn parse_props_update<D: Direction>(
    properties: Vec<libspa::pod::Property>,
    log: &Log,
) -> PropsUpdate<D> {
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
            SPA_PROP_params => parse_stream_params(&property.value, &mut update),
            key => {
                crate::debug!(log, "ignoring unknown prop {}", key);
            }
        }
    }
    update
}

// the SPA_PROP_params payload: a Struct of ("key", value) pairs
pub(super) fn parse_stream_params<D: Direction>(
    value: &libspa::pod::Value,
    update: &mut PropsUpdate<D>,
) {
    use libspa::pod::Value;
    let Value::Struct(values) = value else {
        return;
    };
    if values.len() % 2 != 0 {
        return;
    }
    update.backend = BackendPropertiesOf::<D>::decode_params(value);
}

// Apply a validated request to the main-loop model. Data-loop effects cross
// only through DataControl. The selected property decoder canonicalizes its
// own update order; the first failing backend-specific update returns errno.
pub(crate) fn apply_node_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    request: NodeParamRequest<D>,
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
            for update in update.backend {
                let mut properties = state.backend_properties;
                if properties.apply(update) {
                    let old = state.backend_properties;
                    state.backend_properties = properties;
                    let res = apply_props_param(state, data, move |state| {
                        state.backend_properties = properties;
                        D::sync_backend_properties(&mut state.ext, &properties);
                    });
                    if res != 0 {
                        state.backend_properties = old;
                        return res;
                    }
                }
            }
            0
        }
        NodeParamRequest::ResetProcessLatency => {
            handle_process_latency(state, spa::process_latency_default());
            0
        }
        NodeParamRequest::ProcessLatency(info) => {
            handle_process_latency(state, info);
            0
        }
    }
}

pub(super) fn build_backend_node_param<D: Direction>(
    state: &MainState<D>,
    id: u32,
    index: u32,
) -> ParamBuild {
    #[expect(non_upper_case_globals)]
    let pod = match (id, index) {
        (SPA_PARAM_PropInfo, 0) => spa::build_latency_offset_prop_info(),
        (SPA_PARAM_PropInfo, index) => {
            let Some(descriptor) = state
                .backend_properties
                .descriptors()
                .get(index.saturating_sub(1) as usize)
            else {
                return ParamBuild::Exhausted;
            };
            let value = state
                .backend_properties
                .values()
                .into_iter()
                .find_map(|(key, value)| (key == descriptor.key).then_some(value))
                .unwrap_or(0);
            spa::build_params_prop_info(
                descriptor.key,
                descriptor.description,
                value,
                descriptor.maximum,
            )
        }
        (SPA_PARAM_Props, 0) => spa::build_latency_offset_props(
            state.process_latency.ns,
            &state.backend_properties.values(),
        ),
        (SPA_PARAM_ProcessLatency, 0) => spa::build_process_latency_info(&state.process_latency),
        (SPA_PARAM_Props | SPA_PARAM_ProcessLatency, _) => {
            return ParamBuild::Exhausted;
        }
        _ => return ParamBuild::Unknown,
    };
    ParamBuild::Built(pod)
}

pub(super) fn reset_backend_props<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
) -> c_int {
    let old = state.backend_properties;
    let mut properties = old;
    properties.reset();
    state.backend_properties = properties;
    let res = store_and_rebuild(state, data, move |state| {
        state.backend_properties = properties;
        D::sync_backend_properties(&mut state.ext, &properties);
    });
    if res != 0 {
        state.backend_properties = old;
        return res;
    }
    handle_process_latency(state, spa::process_latency_default());
    0
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
                match unsafe { deserialize_pod(param) } {
                    Some(Value::Object(Object {
                        type_, properties, ..
                    })) if type_ == SPA_TYPE_OBJECT_Props => {
                        NodeParamRequest::Props(parse_props_update::<D>(properties, &log))
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
                let value = unsafe { deserialize_pod(param) };
                match spa::parse_process_latency_info(value.as_ref()) {
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
    use crate::backend::fake::FakePropertyUpdate;
    use crate::spa::pod_prop;
    // Known Props populate the update; adapter-owned, unknown, and invalid
    // values are ignored.
    #[test]
    fn props_update_parses_known_keys_and_drops_the_rest() {
        use libspa::pod::Value;
        use pod_prop;
        let log = Log::test_null();

        let params = Value::Struct(vec![
            Value::String("fake.quantum".into()),
            Value::Int(4096),
            Value::String("bogus.key".into()),
            Value::Int(1),
        ]);
        let update =
            parse_props_update::<crate::node::sink::SinkDir<crate::backend::fake::FakeBackend>>(
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
                backend: vec![FakePropertyUpdate::Quantum(4096)],
            }
        );

        // negative values are ignored, an odd-length struct is ignored whole,
        // and a mistyped latency offset stays None
        let update =
            parse_props_update::<crate::node::sink::SinkDir<crate::backend::fake::FakeBackend>>(
                vec![
                    pod_prop(SPA_PROP_latencyOffsetNsec, Value::Int(250_000)),
                    pod_prop(
                        SPA_PROP_params,
                        Value::Struct(vec![Value::String("fake.quantum".into()), Value::Int(-1)]),
                    ),
                ],
                &log,
            );
        assert_eq!(update, PropsUpdate::default());
        let update =
            parse_props_update::<crate::node::sink::SinkDir<crate::backend::fake::FakeBackend>>(
                vec![pod_prop(
                    SPA_PROP_params,
                    Value::Struct(vec![Value::String("fake.quantum".into())]),
                )],
                &log,
            );
        assert_eq!(update, PropsUpdate::default());
    }
}
