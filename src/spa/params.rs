use super::*;

// Run spa_pod_filter with the output going into `out` through its own builder.
// The source pod must NOT live in `out`: the builder's overflow callback grows
// the Vec by reallocating, which would move the source out from under the
// filter mid-copy. Returns a pointer into `out`, valid until `out` changes.
pub(crate) unsafe fn filter_pod(
    out: &mut Vec<u8>,
    src: *mut spa_pod,
    filter: *const spa_pod,
) -> Option<*mut spa_pod> {
    let builder = libspa::pod::builder::Builder::new(out);
    let mut param: *mut spa_pod = std::ptr::null_mut();
    if unsafe { spa_pod_filter(builder.as_raw_ptr(), &mut param, src, filter) } >= 0 {
        Some(param)
    } else {
        None
    }
}

// one (id, index) step of a param enumeration (enum_params_loop's build closure)
pub(crate) enum ParamStep {
    Built(Vec<u8>), // the serialized pod for this index
    Skip,           // nothing at this index; keep scanning (inactive routes)
    Stop(c_int),    // end the enumeration with this return code
}

/// The shared enum_params frame behind node, port and device param
/// enumeration: walk indices from `start`, build one pod per step, filter it
/// against the host's filter pod and emit up to `max` matches as result
/// events. Each build gets a fresh, short State borrow; that borrow ends
/// before `emit`, so a result listener may safely re-enter and the following
/// index observes any resulting state change.
///
/// # Safety
/// `state` must remain live for the call, and a reentrant listener must not
/// destroy it before enumeration returns. `filter` must be null or point at
/// a valid pod (the spa_pod_filter contract). The emit closure receives a
/// pointer into a buffer valid only for that call.
pub(crate) unsafe fn enum_params_loop<S>(
    state: *mut S,
    (start, max): (u32, u32),
    filter: *const spa_pod,
    mut build: impl FnMut(&mut S, u32) -> ParamStep,
    mut emit: impl FnMut(u32, *mut spa_pod),
) -> c_int {
    assert!(!state.is_null(), "enumerated state must not be null");
    let mut fbuffer = vec![]; // spa_pod_filter output; kept apart from the source pod (see filter_pod)

    let mut index = start;
    let mut count = 0;

    while count < max {
        // Reborrow for one build step only. The reference ends before the
        // listener call below, which may re-enter and mutably borrow S.
        let step = build(
            unsafe { state.as_mut() }.expect("state was checked non-null"),
            index,
        );
        let mut buffer = match step {
            ParamStep::Built(pod) => pod,
            ParamStep::Skip => {
                index += 1;
                continue;
            }
            ParamStep::Stop(res) => return res,
        };

        // the built pod lives in `buffer`, distinct from the filter output
        if let Some(param) =
            unsafe { filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter) }
        {
            emit(index, param);
            count += 1;
        }

        index += 1;
    }

    0
}
// slice::from_raw_parts requires the byte size to fit in isize even when the
// host claims the backing allocation is valid.
pub(crate) fn raw_slice_len_ok<T>(len: usize) -> bool {
    let size = std::mem::size_of::<T>();
    size == 0 || len <= (isize::MAX as usize) / size
}

// Serialize a Value tree into a standalone pod byte buffer. Infallible in
// practice: the output is in-memory and every Value we build is
// serializable, so an error here is a programming bug.
pub(crate) fn serialize_pod(value: &libspa::pod::Value) -> Vec<u8> {
    use libspa::pod::serialize::PodSerializer;
    PodSerializer::serialize(std::io::Cursor::new(Vec::new()), value)
        .expect("serializing a pod Value into a Vec cannot fail")
        .0
        .into_inner()
}

// a flag-less object property (the common case)
pub(crate) fn pod_prop(key: u32, value: libspa::pod::Value) -> libspa::pod::Property {
    libspa::pod::Property::new(key, value)
}

pub(crate) fn pod_int_range(default: i32, min: i32, max: i32) -> libspa::pod::Value {
    use libspa::pod::{ChoiceValue, Value};
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags};
    Value::Choice(ChoiceValue::Int(Choice(
        ChoiceFlags::empty(),
        ChoiceEnum::Range { default, min, max },
    )))
}

// an Int range choice (default, min, max)
pub(crate) fn latency_info_default(
    direction: libspa::sys::spa_direction,
) -> libspa::sys::spa_latency_info {
    libspa::sys::spa_latency_info {
        direction,
        min_quantum: 0.0,
        max_quantum: 0.0,
        min_rate: 0,
        max_rate: 0,
        min_ns: 0,
        max_ns: 0,
    }
}

// spa_latency_parse is static inline C, so reimplemented here; takes the
// already-deserialized Value (the extern fns call deserialize_pod at the
// FFI boundary), so the parse itself is safe code
pub(crate) fn parse_latency_info(
    value: Option<&libspa::pod::Value>,
) -> Option<libspa::sys::spa_latency_info> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    match value {
        Some(Value::Object(Object {
            type_, properties, ..
        })) if *type_ == SPA_TYPE_OBJECT_ParamLatency => {
            let mut info = latency_info_default(SPA_DIRECTION_INPUT);
            for p in properties {
                #[allow(non_upper_case_globals)]
                match (p.key, &p.value) {
                    (SPA_PARAM_LATENCY_direction, Value::Id(v)) => info.direction = v.0 & 1,
                    (SPA_PARAM_LATENCY_minQuantum, Value::Float(v)) => info.min_quantum = *v,
                    (SPA_PARAM_LATENCY_maxQuantum, Value::Float(v)) => info.max_quantum = *v,
                    (SPA_PARAM_LATENCY_minRate, Value::Int(v)) => info.min_rate = *v,
                    (SPA_PARAM_LATENCY_maxRate, Value::Int(v)) => info.max_rate = *v,
                    (SPA_PARAM_LATENCY_minNs, Value::Long(v)) => info.min_ns = *v,
                    (SPA_PARAM_LATENCY_maxNs, Value::Long(v)) => info.max_ns = *v,
                    _ => (),
                }
            }
            Some(info)
        }
        _ => None,
    }
}

// spa_latency_build is static inline C, so reimplemented here
pub(crate) fn build_latency_info(info: &libspa::sys::spa_latency_info) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;
    use libspa::utils::Id;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamLatency,
        id: SPA_PARAM_Latency,
        properties: vec![
            pod_prop(SPA_PARAM_LATENCY_direction, Value::Id(Id(info.direction))),
            pod_prop(SPA_PARAM_LATENCY_minQuantum, Value::Float(info.min_quantum)),
            pod_prop(SPA_PARAM_LATENCY_maxQuantum, Value::Float(info.max_quantum)),
            pod_prop(SPA_PARAM_LATENCY_minRate, Value::Int(info.min_rate)),
            pod_prop(SPA_PARAM_LATENCY_maxRate, Value::Int(info.max_rate)),
            pod_prop(SPA_PARAM_LATENCY_minNs, Value::Long(info.min_ns)),
            pod_prop(SPA_PARAM_LATENCY_maxNs, Value::Long(info.max_ns)),
        ],
    }))
}

pub(crate) fn process_latency_default() -> libspa::sys::spa_process_latency_info {
    libspa::sys::spa_process_latency_info {
        quantum: 0.0,
        rate: 0,
        ns: 0,
    }
}

// spa_process_latency_parse is static inline C, so reimplemented here;
// takes the already-deserialized Value (see parse_latency_info)
pub(crate) fn parse_process_latency_info(
    value: Option<&libspa::pod::Value>,
) -> Option<libspa::sys::spa_process_latency_info> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    match value {
        Some(Value::Object(Object {
            type_, properties, ..
        })) if *type_ == SPA_TYPE_OBJECT_ParamProcessLatency => {
            let mut info = process_latency_default();
            for p in properties {
                #[allow(non_upper_case_globals)]
                match (p.key, &p.value) {
                    (SPA_PARAM_PROCESS_LATENCY_quantum, Value::Float(v)) => info.quantum = *v,
                    (SPA_PARAM_PROCESS_LATENCY_rate, Value::Int(v)) => info.rate = *v,
                    (SPA_PARAM_PROCESS_LATENCY_ns, Value::Long(v)) => info.ns = *v,
                    _ => (),
                }
            }
            Some(info)
        }
        _ => None,
    }
}

// spa_process_latency_build is static inline C, so reimplemented here
pub(crate) fn build_process_latency_info(info: &libspa::sys::spa_process_latency_info) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamProcessLatency,
        id: SPA_PARAM_ProcessLatency,
        properties: vec![
            pod_prop(
                SPA_PARAM_PROCESS_LATENCY_quantum,
                Value::Float(info.quantum),
            ),
            pod_prop(SPA_PARAM_PROCESS_LATENCY_rate, Value::Int(info.rate)),
            pod_prop(SPA_PARAM_PROCESS_LATENCY_ns, Value::Long(info.ns)),
        ],
    }))
}

// spa_process_latency_info_add is static inline C, so reimplemented here
pub(crate) fn process_latency_info_add(
    process: &libspa::sys::spa_process_latency_info,
    info: &mut libspa::sys::spa_latency_info,
) {
    info.min_quantum += process.quantum;
    info.max_quantum += process.quantum;
    info.min_rate += process.rate;
    info.max_rate += process.rate;
    info.min_ns += process.ns;
    info.max_ns += process.ns;
}

pub(crate) fn build_latency_offset_prop_info() -> Vec<u8> {
    use libspa::pod::{ChoiceValue, Object, Value};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_PropInfo,
        id: SPA_PARAM_PropInfo,
        properties: vec![
            pod_prop(SPA_PROP_INFO_id, Value::Id(Id(SPA_PROP_latencyOffsetNsec))),
            pod_prop(
                SPA_PROP_INFO_description,
                Value::String("Latency offset (ns)".to_string()),
            ),
            pod_prop(
                SPA_PROP_INFO_type,
                Value::Choice(ChoiceValue::Long(Choice(
                    ChoiceFlags::empty(),
                    ChoiceEnum::Range {
                        default: 0,
                        min: 0,
                        max: 2 * SPA_NSEC_PER_SEC as i64,
                    },
                ))),
            ),
        ],
    }))
}

pub(crate) fn build_latency_offset_props(ns: i64, params: &[(&str, u32)]) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    let mut properties = vec![pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(ns))];

    // custom key/value props (oss.delay, oss.fragment) ride in the params struct
    if !params.is_empty() {
        let fields = params
            .iter()
            .flat_map(|(key, value)| [Value::String((*key).to_string()), Value::Int(*value as i32)])
            .collect();
        properties.push(pod_prop(SPA_PROP_params, Value::Struct(fields)));
    }

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_Props,
        id: SPA_PARAM_Props,
        properties,
    }))
}

// PropInfo for a custom u32 tunable carried in the Props params struct; the
// advertised default is the CURRENT (effective) value, like the ALSA plugin
pub(crate) fn build_params_prop_info(
    name: &str,
    description: &str,
    current: u32,
    max: u32,
) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_PropInfo,
        id: SPA_PARAM_PropInfo,
        properties: vec![
            pod_prop(SPA_PROP_INFO_name, Value::String(name.to_string())),
            pod_prop(
                SPA_PROP_INFO_description,
                Value::String(description.to_string()),
            ),
            pod_prop(
                SPA_PROP_INFO_type,
                pod_int_range(current as i32, 0, max as i32),
            ),
            // settable through the Props params struct
            pod_prop(SPA_PROP_INFO_params, Value::Bool(true)),
        ],
    }))
}

// Deserialize a host-supplied pod without trusting it: libspa's
// deserializer divides by a pod-declared child size (Choice pods) and
// pre-allocates from declared lengths, so a hostile pod can panic it -
// which must not unwind across our extern "C" boundaries.
pub(crate) unsafe fn deserialize_pod(
    param: *const libspa::sys::spa_pod,
) -> Option<libspa::pod::Value> {
    use libspa::pod::deserialize::PodDeserializer;
    let bytes = unsafe { libspa::pod::Pod::from_raw(param).as_bytes() };
    std::panic::catch_unwind(|| PodDeserializer::deserialize_any_from(bytes).ok())
        .ok()
        .flatten()
        // the parse remainder rode a lifetime fabricated from the raw pod;
        // every caller wants only the owned Value
        .map(|(_, value)| value)
}

// Test-only: parse a serialized pod back through the same PodDeserializer
// consumers run, insisting the buffer holds exactly one complete pod.
#[cfg(test)]
pub(crate) fn parse_back(pod: &[u8]) -> libspa::pod::Value {
    let (rest, value) = libspa::pod::deserialize::PodDeserializer::deserialize_any_from(pod)
        .expect("an advertised pod must deserialize");
    assert!(rest.is_empty(), "trailing bytes after the pod");
    value
}

#[cfg(test)]
mod pod_tests {
    #[test]
    fn latency_offset_props_parse_back_with_params_struct() {
        use libspa::pod::{Object, Value};
        use libspa::sys::*;

        // the sink's Props shape: a long, plus the params struct of
        // string/int pairs
        let pod =
            super::build_latency_offset_props(50_000_000, &[("oss.delay", 4), ("oss.fragment", 0)]);
        assert_eq!(
            super::parse_back(&pod),
            Value::Object(Object {
                type_: SPA_TYPE_OBJECT_Props,
                id: SPA_PARAM_Props,
                properties: vec![
                    super::pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(50_000_000)),
                    super::pod_prop(
                        SPA_PROP_params,
                        Value::Struct(vec![
                            Value::String("oss.delay".to_string()),
                            Value::Int(4),
                            Value::String("oss.fragment".to_string()),
                            Value::Int(0),
                        ]),
                    ),
                ],
            })
        );
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    // A result callback may mutate the enumerated object. The next build
    // step must reacquire State and observe that mutation; retaining &mut S
    // across emit would make this exact pattern formally unsound.
    #[test]
    fn enumeration_reborrows_state_after_reentrant_emit() {
        let mut state = vec![10i32, 20];
        let state_ptr = &raw mut state;
        let mut built = Vec::new();
        let build = |state: &mut Vec<i32>, index: u32| {
            let value = state[index as usize];
            built.push(value);
            ParamStep::Built(crate::spa::serialize_pod(&libspa::pod::Value::Int(value)))
        };
        let emit = |index: u32, _param: *mut spa_pod| {
            if index == 0 {
                // SAFETY: enum_params_loop guarantees its per-step reference
                // ended before emit.
                unsafe { (&mut *state_ptr)[1] = 99 };
            }
        };
        let result = unsafe { enum_params_loop(state_ptr, (0, 2), std::ptr::null(), build, emit) };
        assert_eq!(result, 0);
        assert_eq!(built, [10, 99]);
    }
}
