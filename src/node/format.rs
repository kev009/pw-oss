#[cfg(test)]
use crate::oss::{advertised_quantum_cap_frames, max_buffer_period_bytes as max_ring_period_bytes};
use crate::spa::{pod_int_range, pod_prop, serialize_pod};

// sys/dev/sound/pcm/matrix.h interleave order; note 5.1/7.1 put FC/LF after
// the rears, unlike WAV/ALSA
// hand-formatted: one line per speaker pair keeps the interleave order legible
#[rustfmt::skip]
pub(crate) fn channel_positions(channels: u32) -> Option<&'static [u32]> {
    use libspa::sys::*;
    static C1: [u32; 1] = [SPA_AUDIO_CHANNEL_MONO];
    static C2: [u32; 2] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR];
    static C4: [u32; 4] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                           SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR];
    static C6: [u32; 6] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                           SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR,
                           SPA_AUDIO_CHANNEL_FC, SPA_AUDIO_CHANNEL_LFE];
    static C8: [u32; 8] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                           SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR,
                           SPA_AUDIO_CHANNEL_FC, SPA_AUDIO_CHANNEL_LFE,
                           SPA_AUDIO_CHANNEL_SL, SPA_AUDIO_CHANNEL_SR];
    match channels {
        1 => Some(&C1),
        2 => Some(&C2),
        4 => Some(&C4),
        6 => Some(&C6),
        8 => Some(&C8),
        _ => None
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct UnsupportedChannelOrder;

// FreeBSD's default application-side CHID_* interleave. Three- and
// seven-channel defaults contain rear-center, which OSS channel-order maps
// cannot represent; those widths stay opaque unless the kernel grows a wider
// map. Kept encoded here so equivalent SPA names (MONO and FL) compare by
// what the ioctl actually means.
fn default_oss_channel_order(channels: usize) -> Option<u64> {
    match channels {
        1 => Some(0x1),
        2 => Some(0x21),
        4 => Some(0x8721),
        5 => Some(0x3_8721),
        6 => Some(0x43_8721),
        8 => Some(0x6543_8721),
        _ => None,
    }
}

// Translate a positioned SPA interleave into sys/soundcard.h CHID_* nibbles.
// None means the kernel's default order needs no change, or the stream is
// explicitly opaque (unpositioned/AUX). A meaningful named map must be wholly
// representable and unique: silently ignoring only part of it would attach
// the wrong speaker labels to the PCM byte stream.
pub(crate) fn oss_channel_order(
    flags: u32,
    positions: &[u32],
) -> Result<Option<u64>, UnsupportedChannelOrder> {
    use libspa::sys::*;

    if flags & SPA_AUDIO_FLAG_UNPOSITIONED != 0 {
        return Ok(None);
    }

    let channel_id = |position| match position {
        SPA_AUDIO_CHANNEL_MONO | SPA_AUDIO_CHANNEL_FL => Some(1u64),
        SPA_AUDIO_CHANNEL_FR => Some(2),
        SPA_AUDIO_CHANNEL_FC => Some(3),
        SPA_AUDIO_CHANNEL_LFE => Some(4),
        SPA_AUDIO_CHANNEL_SL => Some(5),
        SPA_AUDIO_CHANNEL_SR => Some(6),
        SPA_AUDIO_CHANNEL_RL => Some(7),
        SPA_AUDIO_CHANNEL_RR => Some(8),
        _ => None,
    };
    let opaque = |position| {
        position == SPA_AUDIO_CHANNEL_UNKNOWN
            || position == SPA_AUDIO_CHANNEL_NA
            || position >= SPA_AUDIO_CHANNEL_AUX0
    };

    let any_named = positions
        .iter()
        .any(|&position| channel_id(position).is_some());
    if !any_named {
        return positions
            .iter()
            .all(|&position| opaque(position))
            .then_some(None)
            .ok_or(UnsupportedChannelOrder);
    }
    if positions.len() > 8 {
        return Err(UnsupportedChannelOrder);
    }

    let mut order = 0u64;
    let mut seen = 0u16;
    for (index, &position) in positions.iter().enumerate() {
        let Some(id) = channel_id(position) else {
            return Err(UnsupportedChannelOrder);
        };
        let bit = 1u16 << id;
        if seen & bit != 0 {
            return Err(UnsupportedChannelOrder);
        }
        seen |= bit;
        order |= id << (index * 4);
    }

    let Some(default) = default_oss_channel_order(positions.len()) else {
        return Err(UnsupportedChannelOrder);
    };
    let default_set = (0..positions.len()).fold(0u16, |set, index| {
        set | 1u16 << ((default >> (index * 4)) & 0xf)
    });
    if seen != default_set {
        return Err(UnsupportedChannelOrder);
    }

    // Avoid an ioctl when the encoded order already matches FreeBSD's
    // default, including equivalent SPA spellings such as mono FL vs MONO.
    // This also preserves virtual OSS endpoints that expose the default but
    // implement only GET_CHNORDER.
    if order == default {
        Ok(None)
    } else {
        Ok(Some(order))
    }
}

// (OSS AFMT, SPA audio format, bytes per sample) triples we can produce,
// ordered by preference: wide integer, float, 3-byte 24-bit, 16-bit, then U8.
// This is the single source of truth for EnumFormat, negotiation snapping and
// per-config stride.
// (hand-formatted: one triple per line keeps the mapping scannable)
#[rustfmt::skip]
pub(crate) const FORMAT_MAP: [(u32, u32, u32); 9] = [
    (crate::oss::AFMT_S32_LE, libspa::sys::SPA_AUDIO_FORMAT_S32_LE, 4),
    (crate::oss::AFMT_S32_BE, libspa::sys::SPA_AUDIO_FORMAT_S32_BE, 4),
    (crate::oss::AFMT_F32_LE, libspa::sys::SPA_AUDIO_FORMAT_F32_LE, 4),
    (crate::oss::AFMT_F32_BE, libspa::sys::SPA_AUDIO_FORMAT_F32_BE, 4),
    (crate::oss::AFMT_S24_LE, libspa::sys::SPA_AUDIO_FORMAT_S24_LE, 3),
    (crate::oss::AFMT_S24_BE, libspa::sys::SPA_AUDIO_FORMAT_S24_BE, 3),
    (crate::oss::AFMT_S16_LE, libspa::sys::SPA_AUDIO_FORMAT_S16_LE, 2),
    (crate::oss::AFMT_S16_BE, libspa::sys::SPA_AUDIO_FORMAT_S16_BE, 2),
    (crate::oss::AFMT_U8,     libspa::sys::SPA_AUDIO_FORMAT_U8,     1)
];

// the (OSS AFMT, bytes per sample) behind a SPA audio format; None for
// anything outside the map (rejected at negotiation)
pub(crate) fn oss_format_info(spa_format: u32) -> Option<(u32, u32)> {
    FORMAT_MAP
        .iter()
        .find(|(_, f, _)| *f == spa_format)
        .map(|(m, _, b)| (*m, *b))
}

// the formats a device gets offered: native ones when any exist, all of ours
// otherwise (the kernel feeder converts), nothing on a convertless device
// without a native match (bitperfect has no feeder; a snap-and-mismatch
// would just fail negotiation)
fn offered_formats(caps: &crate::oss::DspCaps) -> Vec<u32> {
    let native = FORMAT_MAP
        .iter()
        .filter(|(m, _, _)| caps.formats & m != 0)
        .map(|(_, f, _)| *f)
        .collect::<Vec<_>>();
    if !native.is_empty() {
        native
    } else if caps.convertless {
        vec![]
    } else {
        FORMAT_MAP.iter().map(|(_, f, _)| *f).collect()
    }
}

// Snap a requested raw format onto the advertised caps for callers that pass
// SPA_NODE_PARAM_FLAG_NEAREST - audioadapter always negotiates the follower
// that way (audioadapter.c:758, :1059) - mirroring alsa's set_*_near handling
// (alsa-pcm.c:2364, :2388). Returns true when anything was adjusted; the
// caller then returns 1 (alsa-pcm.c:2548) so the adapter re-reads our Format
// param for the actual values (audioadapter.c:596).
pub(crate) fn snap_raw_to_caps(
    caps: &crate::oss::DspCaps,
    raw: &mut libspa::sys::spa_audio_info_raw,
) -> bool {
    let mut changed = false;

    let offered = offered_formats(caps);
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
        .clamp(caps.min_channels, caps.max_channels)
        .min(libspa::sys::SPA_AUDIO_MAX_CHANNELS);
    if channels != raw.channels {
        raw.channels = channels;
        // the requested layout no longer applies; hand out the kernel interleave
        // order (or AUX slots), same as EnumFormat
        match channel_positions(channels) {
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

    let rate = if !caps.rates.is_empty() {
        // discrete native rates (exclusive devices): nearest wins
        *caps
            .rates
            .iter()
            .min_by_key(|r| r.abs_diff(raw.rate))
            .unwrap()
    } else {
        raw.rate.clamp(caps.min_rate, caps.max_rate)
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
pub(crate) fn build_enum_format_info(caps: &crate::oss::DspCaps, index: u32) -> Option<Vec<u8>> {
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    // formats supported by both us and the device, best first
    let formats = offered_formats(caps);
    if formats.is_empty() {
        return None;
    }

    let counts = enum_format_widths(caps.min_channels, caps.max_channels);
    let &channels = counts.get(index as usize)?;

    let format = if let [format] = formats[..] {
        Value::Id(Id(format))
    } else {
        Value::Choice(ChoiceValue::Id(Choice(
            ChoiceFlags::empty(),
            ChoiceEnum::Enum {
                default: Id(formats[0]),
                alternatives: formats[1..].iter().map(|f| Id(*f)).collect(),
            },
        )))
    };

    let rate = if caps.rates.len() > 1 {
        // discrete native rates (exclusive devices); a range would admit
        // in-between rates the hardware can't run (see oss::devices::native_rates)
        let target = caps.preferred_rate.unwrap_or(48000);
        let default = *caps
            .rates
            .iter()
            .min_by_key(|r| r.abs_diff(target))
            .unwrap();
        Value::Choice(ChoiceValue::Int(Choice(
            ChoiceFlags::empty(),
            ChoiceEnum::Enum {
                default: default as i32,
                alternatives: caps.rates.iter().map(|r| *r as i32).collect(),
            },
        )))
    } else if let [rate] = caps.rates[..] {
        Value::Int(rate as i32)
    } else if caps.min_rate == caps.max_rate {
        Value::Int(caps.min_rate as i32)
    } else {
        pod_int_range(
            caps.preferred_rate
                .unwrap_or(48000)
                .clamp(caps.min_rate, caps.max_rate) as i32,
            caps.min_rate as i32,
            caps.max_rate as i32,
        )
    };

    let positions: Vec<Id> = match channel_positions(channels) {
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
