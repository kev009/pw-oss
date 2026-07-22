use crate::{
    backend,
    spa::{pod_int_range, pod_prop, serialize_pod},
};

// Snap a requested raw format onto the advertised caps for callers that pass
// SPA_NODE_PARAM_FLAG_NEAREST - audioadapter always negotiates the follower
// that way (audioadapter.c:758, :1059) - mirroring alsa's set_*_near handling
// (alsa-pcm.c:2364, :2388). Returns true when anything was adjusted; the
// caller then returns 1 (alsa-pcm.c:2548) so the adapter re-reads our Format
// param for the actual values (audioadapter.c:596).
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
            let channel_distance = raw.channels.abs_diff(
                raw.channels
                    .clamp(configuration.min_channels, configuration.max_channels),
            );
            let rate_distance = if configuration.rates.is_empty() {
                raw.rate.abs_diff(
                    raw.rate
                        .clamp(configuration.min_rate, configuration.max_rate),
                )
            } else {
                configuration
                    .rates
                    .iter()
                    .map(|rate| rate.abs_diff(raw.rate))
                    .min()
                    .unwrap_or(u32::MAX)
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

    // the position array is 64 wide; garbage caps must not push past it
    let channels = raw
        .channels
        .clamp(configuration.min_channels, configuration.max_channels)
        .min(libspa::sys::SPA_AUDIO_MAX_CHANNELS);
    if channels != raw.channels {
        raw.channels = channels;
        // the requested layout no longer applies; hand out the kernel interleave
        // order (or AUX slots), same as EnumFormat
        match backend::channel_positions(channels) {
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

    let rate = if !configuration.rates.is_empty() {
        // discrete native rates (exclusive devices): nearest wins
        *configuration
            .rates
            .iter()
            .min_by_key(|r| r.abs_diff(raw.rate))
            .unwrap()
    } else {
        raw.rate
            .clamp(configuration.min_rate, configuration.max_rate)
    };
    if rate != raw.rate {
        raw.rate = rate;
        changed = true;
    }

    changed
}

// The offered channel widths, in EnumFormat order: standard widths in range,
// then the native max if missing (AUX for non-std), with 2 pinned first (host
// default) and last (pulse-server falls back to the LAST EnumFormat map when
// Format is gone; HW routes always report 2ch volume, so a last width of
// 1/max would thrash cvolume.channels). That can mean two entries for stereo.
fn enum_format_widths(min_channels: u32, max_channels: u32) -> Vec<u32> {
    let mut counts = [2u32, 4, 6, 8, 1]
        .iter()
        .copied()
        .filter(|c| *c >= min_channels && *c <= max_channels)
        .collect::<Vec<_>>();
    if !counts.contains(&max_channels) {
        counts.push(max_channels);
    }
    // pin 2 first and last; no-op when already only [2]
    if min_channels <= 2 && max_channels >= 2 {
        counts.retain(|c| *c != 2);
        counts.insert(0, 2);
        if counts.last() != Some(&2) {
            counts.push(2);
        }
    }
    counts
}

// One EnumFormat pod per offered channel width (enum_format_widths order),
// positions from the kernel interleave. None when `index` is past the last
// result.
pub(crate) fn build_enum_format_info(caps: &backend::StreamCaps, index: u32) -> Option<Vec<u8>> {
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    let mut index = index as usize;
    let (configuration, channels) = caps
        .configurations_in_preference_order()
        .filter(|configuration| !configuration.formats.is_empty())
        .find_map(|configuration| {
            let counts = enum_format_widths(configuration.min_channels, configuration.max_channels);
            if index < counts.len() {
                Some((configuration, counts[index]))
            } else {
                index -= counts.len();
                None
            }
        })?;

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

    let rate = if configuration.rates.len() > 1 {
        // discrete native rates (exclusive devices); a range would admit
        // in-between rates the hardware can't run (see oss::devices::native_rates)
        let target = configuration.preferred_rate.unwrap_or(48000);
        let default = *configuration
            .rates
            .iter()
            .min_by_key(|r| r.abs_diff(target))
            .unwrap();
        Value::Choice(ChoiceValue::Int(Choice(
            ChoiceFlags::empty(),
            ChoiceEnum::Enum {
                default: default as i32,
                alternatives: configuration.rates.iter().map(|r| *r as i32).collect(),
            },
        )))
    } else if let [rate] = configuration.rates[..] {
        Value::Int(rate as i32)
    } else if configuration.min_rate == configuration.max_rate {
        Value::Int(configuration.min_rate as i32)
    } else {
        pod_int_range(
            configuration
                .preferred_rate
                .unwrap_or(48000)
                .clamp(configuration.min_rate, configuration.max_rate) as i32,
            configuration.min_rate as i32,
            configuration.max_rate as i32,
        )
    };

    let positions: Vec<Id> = match backend::channel_positions(channels) {
        Some(positions) => positions.iter().map(|&p| Id(p)).collect(),
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
