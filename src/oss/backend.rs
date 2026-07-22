//! FreeBSD OSS stream capabilities and configuration.

use crate::spa::Log;
use libspa::sys::*;
use std::ffi::c_int;

use crate::backend::{
    BufferLayout, ChannelMapError, ConfigureOutcome, ConversionKind, StreamCaps, StreamConfig,
    StreamConfiguration,
};

use super::{Dsp, DspWriter, MIN_BUFFER_BYTES, feeder_rate_round};

// (FreeBSD AFMT, SPA format, bytes/sample), ordered by preference.
#[rustfmt::skip]
const FORMAT_MAP: [(u32, u32, u32); 9] = [
    (super::AFMT_S32_LE, SPA_AUDIO_FORMAT_S32_LE, 4),
    (super::AFMT_S32_BE, SPA_AUDIO_FORMAT_S32_BE, 4),
    (super::AFMT_F32_LE, SPA_AUDIO_FORMAT_F32_LE, 4),
    (super::AFMT_F32_BE, SPA_AUDIO_FORMAT_F32_BE, 4),
    (super::AFMT_S24_LE, SPA_AUDIO_FORMAT_S24_LE, 3),
    (super::AFMT_S24_BE, SPA_AUDIO_FORMAT_S24_BE, 3),
    (super::AFMT_S16_LE, SPA_AUDIO_FORMAT_S16_LE, 2),
    (super::AFMT_S16_BE, SPA_AUDIO_FORMAT_S16_BE, 2),
    (super::AFMT_U8,     SPA_AUDIO_FORMAT_U8,     1),
];

pub(crate) fn all_formats() -> Vec<u32> {
    FORMAT_MAP.iter().map(|(_, spa, _)| *spa).collect()
}

pub(super) fn formats_from_native_mask(mask: u32, convertless: bool) -> Vec<u32> {
    let native = FORMAT_MAP
        .iter()
        .filter(|(format, _, _)| mask & format != 0)
        .map(|(_, spa, _)| *spa)
        .collect::<Vec<_>>();
    if !native.is_empty() || convertless {
        native
    } else {
        all_formats()
    }
}

fn native_format(spa_format: u32) -> Option<(u32, u32)> {
    FORMAT_MAP
        .iter()
        .find(|(_, spa, _)| *spa == spa_format)
        .map(|(native, _, bytes)| (*native, *bytes))
}

fn stream_config_from_native(
    requested: &StreamConfig,
    applied: super::dsp::AppliedNativeConfig,
) -> Option<StreamConfig> {
    let (_, spa_format, bytes_per_sample) = FORMAT_MAP
        .iter()
        .find(|(native, _, _)| *native == applied.format)?;
    let mut actual = requested.clone();
    actual.format = libspa::param::audio::AudioFormat(*spa_format);
    actual.rate = applied.rate;
    actual.channels = applied.channels;
    actual.stride = bytes_per_sample.saturating_mul(applied.channels);
    Some(actual)
}

fn configure_outcome(
    requested: &StreamConfig,
    applied: super::dsp::AppliedNativeConfig,
) -> Result<ConfigureOutcome, c_int> {
    let actual_config = stream_config_from_native(requested, applied).ok_or(-libc::ENOTSUP)?;
    Ok(ConfigureOutcome {
        adjusted: actual_config != *requested,
        actual_config,
    })
}

pub(crate) fn bytes_per_sample(spa_format: u32) -> Option<u32> {
    native_format(spa_format).map(|(_, bytes)| bytes)
}

pub(crate) fn fallback_caps() -> StreamCaps {
    StreamCaps {
        configurations: vec![StreamConfiguration {
            formats: all_formats(),
            min_channels: 1,
            max_channels: 2,
            min_rate: 8000,
            max_rate: 192000,
            preferred_rate: None,
            rates: vec![],
            rate_tolerance: feeder_rate_round(),
        }],
        preferred: 0,
        conversion: ConversionKind::Backend,
    }
}

// sys/dev/sound/pcm/matrix.h application-side order.
#[rustfmt::skip]
pub(crate) fn channel_positions(channels: u32) -> Option<&'static [u32]> {
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
        _ => None,
    }
}

fn default_channel_order(channels: usize) -> Option<u64> {
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

fn native_channel_order(flags: u32, positions: &[u32]) -> Result<Option<u64>, ChannelMapError> {
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

    if !positions
        .iter()
        .any(|&position| channel_id(position).is_some())
    {
        return positions
            .iter()
            .all(|&position| opaque(position))
            .then_some(None)
            .ok_or(ChannelMapError::Unsupported);
    }
    if positions.len() > 8 {
        return Err(ChannelMapError::Unsupported);
    }

    let mut order = 0u64;
    let mut seen = 0u16;
    for (index, &position) in positions.iter().enumerate() {
        let Some(id) = channel_id(position) else {
            return Err(ChannelMapError::Unsupported);
        };
        let bit = 1u16 << id;
        if seen & bit != 0 {
            return Err(ChannelMapError::Unsupported);
        }
        seen |= bit;
        order |= id << (index * 4);
    }

    let Some(default) = default_channel_order(positions.len()) else {
        return Err(ChannelMapError::Unsupported);
    };
    let default_set = (0..positions.len()).fold(0u16, |set, index| {
        set | 1u16 << ((default >> (index * 4)) & 0xf)
    });
    if seen != default_set {
        return Err(ChannelMapError::Unsupported);
    }

    Ok((order != default).then_some(order))
}

#[cfg(test)]
pub(crate) fn test_native_channel_order(
    flags: u32,
    positions: &[u32],
) -> Result<Option<u64>, ChannelMapError> {
    native_channel_order(flags, positions)
}

pub(crate) fn validate_config(
    caps: &StreamCaps,
    config: &StreamConfig,
) -> Result<(), ChannelMapError> {
    let channel_order = native_channel_order(config.flags, &config.positions)?;
    if caps.conversion == ConversionKind::None && channel_order.is_some() {
        Err(ChannelMapError::ConvertlessReorder)
    } else {
        Ok(())
    }
}

pub(crate) fn configure_capture(
    stream: &mut Dsp,
    config: &StreamConfig,
    fragment: u32,
    log: &Log,
) -> Result<ConfigureOutcome, c_int> {
    let Some((format, _)) = native_format(config.format.0) else {
        return Err(-libc::ENOTSUP);
    };
    let Ok(channel_order) = native_channel_order(config.flags, &config.positions) else {
        crate::warn!(
            log,
            "rejecting unsupported channel map: {:?}",
            config.positions
        );
        return Err(-libc::EINVAL);
    };
    if let Err(err) = stream.open() {
        crate::warn!(log, "dsp open: {}", err);
        return Err(-(err as c_int));
    }
    let applied = match stream.configure(format, config.channels, config.rate, channel_order) {
        Ok(applied) => applied,
        Err(error) => {
            crate::warn!(log, "device rejected {:?}: {}", config, error);
            stream.close();
            return Err(-(error as c_int));
        }
    };
    stream.refresh_hw_quantum();
    stream.set_small_fragments(fragment, MIN_BUFFER_BYTES);
    configure_outcome(config, applied)
}

pub(crate) fn configure_capture_buffer(stream: &Dsp, fragment_bytes: u32, capacity_bytes: u32) {
    stream.set_small_fragments(fragment_bytes, capacity_bytes);
}

pub(crate) fn configure_playback_buffer(
    stream: &DspWriter,
    capacity_bytes: u32,
    fragment_bytes: u32,
) -> BufferLayout {
    stream.set_buffer_size(capacity_bytes, fragment_bytes)
}

pub(crate) fn configure_playback(
    stream: &mut DspWriter,
    config: &StreamConfig,
    log: &Log,
) -> Result<ConfigureOutcome, c_int> {
    let Some((format, _)) = native_format(config.format.0) else {
        return Err(-libc::ENOTSUP);
    };
    let Ok(channel_order) = native_channel_order(config.flags, &config.positions) else {
        crate::warn!(
            log,
            "{}: unsupported channel map: {:?}",
            stream.path(),
            config.positions
        );
        return Err(-libc::EINVAL);
    };
    if let Err(err) = stream.open() {
        crate::warn!(log, "{}: open: {}", stream.path(), err);
        return Err(-(err as c_int));
    }
    let applied = stream.configure(
        format,
        config.channels,
        config.rate,
        config.silence_byte(),
        channel_order,
    );
    let applied = match applied {
        Ok(applied) => applied,
        Err(err) => {
            crate::warn!(
                log,
                "{}: device rejected {:?}: {}",
                stream.path(),
                config,
                err
            );
            stream.close();
            return Err(-(err as c_int));
        }
    };
    stream.refresh_delivery_quantum();
    configure_outcome(config, applied)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_channel_orders_need_no_native_override() {
        for channels in [1, 2, 4, 6, 8] {
            let positions = channel_positions(channels).unwrap();
            assert_eq!(native_channel_order(0, positions), Ok(None));
        }
    }

    #[test]
    fn native_readback_updates_the_applied_stream_config() {
        let requested = StreamConfig {
            format: libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_S16_LE),
            rate: 48_000,
            channels: 2,
            positions: channel_positions(2).unwrap().to_vec(),
            flags: 0,
            stride: 4,
        };
        let outcome = configure_outcome(
            &requested,
            super::super::dsp::AppliedNativeConfig {
                format: super::super::AFMT_S16_LE,
                channels: 2,
                rate: 47_999,
            },
        )
        .unwrap();

        assert!(outcome.adjusted);
        assert_eq!(outcome.actual_config.rate, 47_999);
        assert_eq!(outcome.actual_config.channels, 2);
        assert_eq!(outcome.actual_config.stride, 4);
        assert_eq!(
            outcome.actual_config.format.0,
            libspa::sys::SPA_AUDIO_FORMAT_S16_LE
        );
    }
}
