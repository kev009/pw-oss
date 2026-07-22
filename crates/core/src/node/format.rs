use crate::{
    backend,
    spa::{pod_int_range, pod_prop, serialize_pod},
};

// Snap a requested raw format onto the advertised caps for callers that pass
// SPA_NODE_PARAM_FLAG_NEAREST. Audioadapter negotiates followers this way and
// re-reads Format when the node returns 1, so callers must report whether the
// backend-adjusted value differs from the request.
pub(crate) fn snap_raw_to_caps(
    caps: &backend::StreamCaps,
    raw: &mut libspa::sys::spa_audio_info_raw,
) -> bool {
    let mut changed = false;

    let Some(configuration) = caps
        .configurations
        .iter()
        .enumerate()
        .filter(|(_, configuration)| !configuration.formats.is_empty())
        .min_by_key(|(index, configuration)| {
            let channel_distance = configuration
                .channels
                .layouts()
                .iter()
                .map(|layout| layout.channels.abs_diff(raw.channels))
                .min()
                .unwrap_or(u32::MAX);
            let rate_distance = match &configuration.rates {
                backend::RateSet::Discrete(rates) => rates
                    .iter()
                    .map(|rate| rate.abs_diff(raw.rate))
                    .min()
                    .unwrap_or(u32::MAX),
                backend::RateSet::Range { min, max } => {
                    raw.rate.abs_diff(raw.rate.clamp(*min, *max))
                }
            };
            (
                !configuration.formats.contains(&raw.format),
                channel_distance,
                rate_distance,
                *index != caps.preferred,
            )
        })
        .map(|(_, configuration)| configuration)
    else {
        return false;
    };

    let offered = backend::offered_formats(configuration);
    if !offered.contains(&raw.format)
        && let Some(&best) = offered.first()
    {
        raw.format = best;
        changed = true;
    }
    // A convertless device with no native format stays unchanged here; the
    // exact-format path rejects it.

    let Some(layout) = configuration
        .channels
        .layouts()
        .into_iter()
        .min_by_key(|layout| layout.channels.abs_diff(raw.channels))
    else {
        return changed;
    };
    // the position array is 64 wide; garbage caps must not push past it
    let channels = layout.channels.min(libspa::sys::SPA_AUDIO_MAX_CHANNELS);
    if channels != raw.channels {
        raw.channels = channels;
        // The requested layout no longer applies; use the layout carried by
        // this capability (or AUX slots when the backend reports only count).
        match layout.positions {
            Some(positions) => {
                for (slot, &p) in raw.position.iter_mut().zip(positions.iter()) {
                    *slot = p;
                }
            }
            None => {
                for (i, slot) in raw.position.iter_mut().take(channels as usize).enumerate() {
                    *slot = libspa::sys::SPA_AUDIO_CHANNEL_AUX0 + i as u32;
                }
            }
        }
        changed = true;
    }

    let rate = match &configuration.rates {
        backend::RateSet::Discrete(rates) => {
            // Discrete native rates: nearest wins.
            rates
                .iter()
                .min_by_key(|rate| rate.abs_diff(raw.rate))
                .copied()
                .unwrap_or(raw.rate)
        }
        backend::RateSet::Range { min, max } => raw.rate.clamp(*min, *max),
    };
    if rate != raw.rate {
        raw.rate = rate;
        changed = true;
    }

    changed
}

// One EnumFormat pod per backend-provided channel layout, in backend order.
// None when `index` is past the last result.
pub(crate) fn build_enum_format_info(caps: &backend::StreamCaps, index: u32) -> Option<Vec<u8>> {
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    let mut index = index as usize;
    let (configuration, layout) = caps
        .configurations_in_preference_order()
        .filter(|configuration| !configuration.formats.is_empty())
        .find_map(|configuration| {
            let layouts = configuration.channels.layouts();
            if index < layouts.len() {
                Some((configuration, layouts[index].clone()))
            } else {
                index -= layouts.len();
                None
            }
        })?;
    let channels = layout.channels;

    // Formats supported by this backend configuration, best first.
    let formats = backend::offered_formats(configuration);
    if formats.is_empty() {
        return None;
    }

    let format = if let [format] = formats {
        Value::Id(Id(*format))
    } else {
        Value::Choice(ChoiceValue::Id(Choice(
            ChoiceFlags::empty(),
            ChoiceEnum::Enum {
                default: Id(formats[0]),
                alternatives: formats[1..].iter().map(|f| Id(*f)).collect(),
            },
        )))
    };

    let rate = match &configuration.rates {
        backend::RateSet::Discrete(rates) if rates.len() > 1 => {
            let target = configuration.preferred_rate.unwrap_or(48000);
            let default = *rates.iter().min_by_key(|rate| rate.abs_diff(target))?;
            Value::Choice(ChoiceValue::Int(Choice(
                ChoiceFlags::empty(),
                ChoiceEnum::Enum {
                    default: default as i32,
                    alternatives: rates.iter().map(|rate| *rate as i32).collect(),
                },
            )))
        }
        backend::RateSet::Discrete(rates) => Value::Int(*rates.first()? as i32),
        backend::RateSet::Range { min, max } if min == max => Value::Int(*min as i32),
        backend::RateSet::Range { min, max } => pod_int_range(
            configuration
                .preferred_rate
                .unwrap_or(48000)
                .clamp(*min, *max) as i32,
            *min as i32,
            *max as i32,
        ),
    };

    let positions: Vec<Id> = match layout.positions {
        Some(positions) => positions.into_iter().map(Id).collect(),
        None => (0..channels)
            .map(|i| Id(SPA_AUDIO_CHANNEL_AUX0 + i))
            .collect(),
    };

    Some(serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_Format,
        id: SPA_PARAM_EnumFormat,
        properties: vec![
            pod_prop(SPA_FORMAT_mediaType, Value::Id(Id(SPA_MEDIA_TYPE_audio))),
            pod_prop(
                SPA_FORMAT_mediaSubtype,
                Value::Id(Id(SPA_MEDIA_SUBTYPE_raw)),
            ),
            pod_prop(SPA_FORMAT_AUDIO_format, format),
            pod_prop(SPA_FORMAT_AUDIO_rate, rate),
            pod_prop(SPA_FORMAT_AUDIO_channels, Value::Int(channels as i32)),
            pod_prop(
                SPA_FORMAT_AUDIO_position,
                Value::ValueArray(ValueArray::Id(positions)),
            ),
        ],
    })))
}

pub(crate) fn build_buffers_info(stride: u32) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    // The point here is dataType = MemPtr: process() maps the buffer memory
    // directly, so a MemFd/DmaBuf block would be unusable.
    //
    // Capacity floors at two graph periods (2048 frames at the 1024-frame
    // reference quantum). The capture catch-up path drains a device
    // ring excursion by handing the graph MORE than one period in a cycle; a
    // one-period buffer clamps that read back to a period, so the ring stays
    // pinned at its ceiling and the kernel overruns every late cycle. Two
    // periods of *capacity* cost no latency - we still deliver one period per
    // cycle - it only widens the container so the drain can happen. The adapter
    // sizes the buffer to the graph quantum and clamps up to this floor, so the
    // headroom is present at the common quanta (a quantum coarser than the floor
    // needs the node ring-quantum cap to stay glitch-free anyway).
    let floor = (2048 * stride) as i32;
    let max = (16384 * stride) as i32;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamBuffers,
        id: SPA_PARAM_Buffers,
        properties: vec![
            pod_prop(SPA_PARAM_BUFFERS_buffers, pod_int_range(2, 1, 32)),
            pod_prop(SPA_PARAM_BUFFERS_blocks, Value::Int(1)),
            pod_prop(SPA_PARAM_BUFFERS_size, pod_int_range(floor, floor, max)),
            pod_prop(SPA_PARAM_BUFFERS_stride, Value::Int(stride as i32)),
            pod_prop(SPA_PARAM_BUFFERS_align, Value::Int(16)),
            pod_prop(
                SPA_PARAM_BUFFERS_dataType,
                Value::Int(1i32 << SPA_DATA_MemPtr),
            ),
        ],
    }))
}

#[cfg(test)]
mod tests;
