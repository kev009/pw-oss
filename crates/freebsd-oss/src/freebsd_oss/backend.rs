//! FreeBSD OSS stream capabilities and configuration.

use crate::spa::Log;
use libspa::sys::*;
use std::ffi::c_int;

use crate::backend::{
    AdjustmentFlags, AppliedBufferGeometry, BufferConstraints, CaptureBufferGeometry,
    CaptureBufferRequest, CaptureOperations, CaptureRetune, ChannelLayout, ChannelMapError,
    ChannelSet, ConfigurationFlags, ConfigureOutcome, ConversionPath, DeliveryQuantum,
    PlaybackBufferGeometry, PlaybackBufferRequest, PlaybackOperations, PlaybackRetune, RateSet,
    ReadOutcome, StreamCaps, StreamConfig, StreamConfiguration, StreamError, StreamIdentity,
    StreamLifecycle, WakeBufferState, WakeDiagnostic, WakeError, WriteOutcome, XrunObservation,
};

use super::{Dsp, DspWriter, MIN_BUFFER_BYTES, OssNodeProperties, feeder_rate_round};

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
    if applied.channels != requested.channels {
        if requested.flags & SPA_AUDIO_FLAG_UNPOSITIONED != 0 {
            actual.positions = (0..applied.channels)
                .map(|index| SPA_AUDIO_CHANNEL_AUX0 + index)
                .collect();
        } else if let Some(positions) = channel_positions(applied.channels) {
            actual.positions = positions.to_vec();
        } else {
            actual.flags |= SPA_AUDIO_FLAG_UNPOSITIONED;
            actual.positions = (0..applied.channels)
                .map(|index| SPA_AUDIO_CHANNEL_AUX0 + index)
                .collect();
        }
    }
    Some(actual)
}

fn configure_outcome(
    requested: &StreamConfig,
    applied: super::dsp::AppliedNativeConfig,
    delivery: DeliveryQuantum,
) -> Result<ConfigureOutcome, c_int> {
    let actual_config = stream_config_from_native(requested, applied).ok_or(-libc::ENOTSUP)?;
    let quantum_bytes = if delivery.frames == 0 || delivery.rate == 0 {
        None
    } else {
        let frames = u64::from(delivery.frames)
            .saturating_mul(u64::from(actual_config.rate))
            .div_ceil(u64::from(delivery.rate));
        Some(
            frames
                .saturating_mul(u64::from(actual_config.stride))
                .min(u64::from(u32::MAX)) as u32,
        )
    };
    let buffer_constraints = BufferConstraints {
        capacity_limit_bytes: Some(super::buffer::buffer_capacity_limit(
            actual_config.stride,
            actual_config.rate,
        )),
        quantum_cap_frames: super::buffer::advertised_quantum_cap_frames(
            actual_config.stride,
            actual_config.rate,
        ),
        quantum_cap_basis: Some("4 periods"),
    };
    Ok(ConfigureOutcome {
        adjusted: AdjustmentFlags::between(requested, &actual_config),
        actual_config,
        applied_buffer: AppliedBufferGeometry {
            capacity_bytes: None,
            quantum_bytes,
            delivery,
        },
        buffer_constraints,
    })
}

pub(crate) fn bytes_per_sample(spa_format: u32) -> Option<u32> {
    native_format(spa_format).map(|(_, bytes)| bytes)
}

pub(crate) fn fallback_caps() -> StreamCaps {
    StreamCaps {
        configurations: vec![StreamConfiguration {
            formats: all_formats(),
            channels: channel_layouts(1, 2),
            rates: RateSet::Range {
                min: 8000,
                max: 192000,
            },
            preferred_rate: None,
            rate_tolerance: feeder_rate_round(),
            conversion: ConversionPath::Kernel,
            flags: ConfigurationFlags::with_layout_reorder_and_opaque(),
        }],
        preferred: 0,
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

pub(super) fn channel_layouts(min_channels: u32, max_channels: u32) -> ChannelSet {
    let mut counts = [2u32, 4, 6, 8, 1]
        .iter()
        .copied()
        .filter(|channels| *channels >= min_channels && *channels <= max_channels)
        .collect::<Vec<_>>();
    if !counts.contains(&max_channels) {
        counts.push(max_channels);
    }
    // Preserve the long-standing enumeration order: stereo is the graph
    // default and is repeated last for clients that retain the final map.
    if min_channels <= 2 && max_channels >= 2 {
        counts.retain(|channels| *channels != 2);
        counts.insert(0, 2);
        if counts.last() != Some(&2) {
            counts.push(2);
        }
    }
    ChannelSet::Discrete(
        counts
            .into_iter()
            .map(|channels| ChannelLayout {
                channels,
                positions: channel_positions(channels).map(<[u32]>::to_vec),
            })
            .collect(),
    )
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

pub(crate) fn validate_config(
    caps: &StreamCaps,
    config: &StreamConfig,
) -> Result<(), ChannelMapError> {
    let channel_order = native_channel_order(config.flags, &config.positions)?;
    if caps.conversion_for(config.format.0, config.channels, config.rate)
        == Some(ConversionPath::None)
        && channel_order.is_some()
    {
        Err(ChannelMapError::ConvertlessReorder)
    } else {
        Ok(())
    }
}

pub(crate) fn configure_capture(
    stream: &mut Dsp,
    config: &StreamConfig,
    properties: &OssNodeProperties,
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
    crate::debug!(
        log,
        "{}: channel caps 0x{:08x}{}",
        stream.path(),
        stream.hw_caps(),
        if stream.is_virtual_channel() {
            " (virtual)"
        } else {
            ""
        }
    );
    stream.refresh_hw_quantum();
    stream.set_small_fragments(properties.fragment_bytes(), MIN_BUFFER_BYTES);
    match configure_outcome(config, applied, stream.delivery_quantum()) {
        Ok(outcome) => Ok(outcome),
        Err(error) => {
            stream.close();
            Err(error)
        }
    }
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
    let applied = stream.configure(format, config.channels, config.rate, channel_order);
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
    crate::debug!(
        log,
        "{}: channel caps 0x{:08x}{}",
        stream.path(),
        stream.hw_caps(),
        if stream.is_virtual_channel() {
            " (virtual)"
        } else {
            ""
        }
    );
    stream.refresh_delivery_quantum();
    match configure_outcome(config, applied, stream.delivery_quantum()) {
        Ok(outcome) => Ok(outcome),
        Err(error) => {
            stream.close();
            Err(error)
        }
    }
}

impl StreamLifecycle for Dsp {
    type WakeDriver = super::OssWakeDriver;

    fn new(path: &str) -> Self {
        Self::new(path)
    }
    fn path(&self) -> &str {
        Self::path(self)
    }
    fn is_closed(&self) -> bool {
        Self::is_closed(self)
    }
    fn is_running(&self) -> bool {
        Self::is_running(self)
    }
    fn register_wake(
        &self,
        driver: &mut Self::WakeDriver,
        stream: StreamIdentity,
        buffer: WakeBufferState,
    ) -> Result<(), WakeError> {
        Self::register_wake(self, driver, stream, buffer)
    }
    fn wake_available() -> bool {
        super::enriched_sound_kqueue_available()
    }
    fn new_wake_driver() -> Result<Self::WakeDriver, WakeError> {
        super::OssWakeDriver::new()
    }
    fn wake_diagnostic(kind: WakeDiagnostic) -> &'static str {
        super::identity::wake_diagnostic(kind)
    }
    fn close(&mut self) {
        Self::close(self);
    }
    fn suspend(&mut self) -> bool {
        Self::suspend(self)
    }
}

impl StreamLifecycle for DspWriter {
    type WakeDriver = super::OssWakeDriver;

    fn new(path: &str) -> Self {
        Self::new(path)
    }
    fn path(&self) -> &str {
        Self::path(self)
    }
    fn is_closed(&self) -> bool {
        Self::is_closed(self)
    }
    fn is_running(&self) -> bool {
        Self::is_running(self)
    }
    fn register_wake(
        &self,
        driver: &mut Self::WakeDriver,
        stream: StreamIdentity,
        buffer: WakeBufferState,
    ) -> Result<(), WakeError> {
        Self::register_wake(self, driver, stream, buffer)
    }
    fn wake_available() -> bool {
        super::enriched_sound_kqueue_available()
    }
    fn new_wake_driver() -> Result<Self::WakeDriver, WakeError> {
        super::OssWakeDriver::new()
    }
    fn wake_diagnostic(kind: WakeDiagnostic) -> &'static str {
        super::identity::wake_diagnostic(kind)
    }
    fn close(&mut self) {
        Self::close(self);
    }
    fn suspend(&mut self) -> bool {
        Self::suspend(self)
    }
}

impl CaptureOperations for Dsp {
    type Properties = OssNodeProperties;

    fn configure(
        &mut self,
        config: &StreamConfig,
        properties: &Self::Properties,
        log: &Log,
    ) -> Result<ConfigureOutcome, c_int> {
        configure_capture(self, config, properties, log)
    }

    fn prime_buffer(
        &mut self,
        request: CaptureBufferRequest,
        properties: &Self::Properties,
        scratch: &mut [u8],
        log: &Log,
    ) -> CaptureBufferGeometry {
        Dsp::prime_buffer(self, request, properties, scratch, log)
    }

    fn retune_buffer(
        &mut self,
        request: CaptureBufferRequest,
        primed: bool,
        log: &Log,
    ) -> CaptureRetune {
        Dsp::retune_buffer(self, request, primed, log)
    }

    fn read(&mut self, data: &mut [u8]) -> ReadOutcome {
        Dsp::read(self, data)
    }

    fn queued_bytes(&self) -> u32 {
        Dsp::queued_bytes(self)
    }

    fn overruns(&self) -> XrunObservation {
        Dsp::overruns(self)
    }

    fn recover_overrun(
        &mut self,
        overrun_count: u32,
        pre_read_fill: Option<u32>,
        log: &Log,
    ) -> Option<bool> {
        Dsp::recover_overrun(self, overrun_count, pre_read_fill, log)
    }

    fn log_overrun_recovery(&self, count: u32, now_ns: u64, suppressed: u32, log: &Log) {
        Dsp::log_overrun_recovery(self, count, now_ns, suppressed, log);
    }

    fn clear_overrun_observation(&mut self) {
        Dsp::clear_overrun_observation(self);
    }
}

impl PlaybackOperations for DspWriter {
    type Properties = OssNodeProperties;

    fn configure(
        &mut self,
        config: &StreamConfig,
        _properties: &Self::Properties,
        log: &Log,
    ) -> Result<ConfigureOutcome, c_int> {
        configure_playback(self, config, log)
    }

    fn prime_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        properties: &Self::Properties,
        log: &Log,
    ) -> PlaybackBufferGeometry {
        DspWriter::prime_buffer(self, request, properties, log)
    }

    fn retune_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        current_fill_bytes: u32,
        now_ns: u64,
        log: &Log,
    ) -> PlaybackRetune {
        DspWriter::retune_buffer(self, request, current_fill_bytes, now_ns, log)
    }

    fn write(&mut self, data: &[u8]) -> WriteOutcome {
        DspWriter::write(self, data)
    }

    fn write_silence(&mut self, bytes: u32) {
        DspWriter::write_silence(self, bytes);
    }

    fn end_buffer_sequence(&mut self) -> bool {
        DspWriter::end_buffer_sequence(self)
    }

    fn queued_bytes(&self) -> u32 {
        DspWriter::queued_bytes(self)
    }

    fn underruns(&self) -> XrunObservation {
        DspWriter::underruns(self)
    }

    fn log_underrun_recovery(&self, count: u32, now_ns: u64, suppressed: u32, log: &Log) {
        DspWriter::log_underrun_recovery(self, count, now_ns, suppressed, log);
    }

    fn log_ignored_underruns(
        &self,
        count: u32,
        fill_bytes: u32,
        recovery_threshold_bytes: u32,
        log: &Log,
    ) {
        DspWriter::log_ignored_underruns(self, count, fill_bytes, recovery_threshold_bytes, log);
    }

    fn pause(&mut self) -> Result<(), StreamError> {
        DspWriter::pause(self).map_err(|error| StreamError::from_native_code(error as i32))
    }

    fn resume(&mut self) -> Result<(), StreamError> {
        DspWriter::resume(self).map_err(|error| StreamError::from_native_code(error as i32))
    }

    fn underrun_low(
        target_fill: u32,
        delivery_quantum: u32,
        period_bytes: u32,
        drained_bytes: u32,
    ) -> u32 {
        DspWriter::underrun_low(target_fill, delivery_quantum, period_bytes, drained_bytes)
    }

    fn debug_log_priorities(log: &Log) {
        #[cfg(debug_assertions)]
        DspWriter::debug_log_priorities(log);
        #[cfg(not(debug_assertions))]
        let _ = log;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::QuantumQuality;

    fn layout_widths(min: u32, max: u32) -> Vec<u32> {
        let ChannelSet::Discrete(layouts) = channel_layouts(min, max) else {
            unreachable!()
        };
        layouts.into_iter().map(|layout| layout.channels).collect()
    }

    // PulseAudio compatibility: when stereo is available it must open and
    // close EnumFormat. The host defaults to the first map, while its fallback
    // retains the last; hardware-route volume remains stereo in both cases.
    #[test]
    fn stereo_pins_both_ends_of_enum_format() {
        for (min, max) in [(1u32, 2u32), (2, 2), (1, 8), (2, 8), (1, 10), (2, 32)] {
            let widths = layout_widths(min, max);
            assert_eq!(*widths.first().unwrap(), 2);
            assert_eq!(*widths.last().unwrap(), 2);
            assert!(widths.contains(&max));
            assert!(widths.iter().all(|width| *width >= min && *width <= max));
            assert_eq!(
                widths.iter().filter(|width| **width == 2).count().min(2),
                if widths.len() == 1 { 1 } else { 2 }
            );
        }
        assert_eq!(layout_widths(2, 8), [2, 4, 6, 8, 2]);
        assert_eq!(layout_widths(1, 10), [2, 4, 6, 8, 1, 10, 2]);
        assert_eq!(layout_widths(2, 2), [2]);
        assert_eq!(layout_widths(1, 1), [1]);
        assert_eq!(layout_widths(4, 8), [4, 6, 8]);
        assert_eq!(layout_widths(3, 3), [3]);
    }

    #[test]
    fn default_channel_orders_need_no_native_override() {
        for channels in [1, 2, 4, 6, 8] {
            let positions = channel_positions(channels).unwrap();
            assert_eq!(native_channel_order(0, positions), Ok(None));
        }
    }

    #[test]
    fn fallback_formats_cover_the_supported_surface() {
        let caps = fallback_caps();
        assert_eq!(
            caps.preferred_configuration().unwrap().formats,
            all_formats()
        );
        assert!(caps.admits(
            SPA_AUDIO_FORMAT_S16_LE,
            2,
            Some(&[SPA_AUDIO_CHANNEL_AUX0, SPA_AUDIO_CHANNEL_AUX1]),
            48_000
        ));
    }

    #[test]
    fn alternate_named_channel_orders_encode_native_ids() {
        assert_eq!(
            native_channel_order(0, &[SPA_AUDIO_CHANNEL_FR, SPA_AUDIO_CHANNEL_FL]),
            Ok(Some(0x12))
        );
        assert_eq!(
            native_channel_order(
                0,
                &[
                    SPA_AUDIO_CHANNEL_FL,
                    SPA_AUDIO_CHANNEL_FR,
                    SPA_AUDIO_CHANNEL_FC,
                    SPA_AUDIO_CHANNEL_LFE,
                    SPA_AUDIO_CHANNEL_RL,
                    SPA_AUDIO_CHANNEL_RR,
                ],
            ),
            Ok(Some(0x87_4321))
        );
        assert_eq!(
            native_channel_order(
                0,
                &[
                    SPA_AUDIO_CHANNEL_FL,
                    SPA_AUDIO_CHANNEL_FR,
                    SPA_AUDIO_CHANNEL_FC,
                    SPA_AUDIO_CHANNEL_LFE,
                    SPA_AUDIO_CHANNEL_RL,
                    SPA_AUDIO_CHANNEL_RR,
                    SPA_AUDIO_CHANNEL_SL,
                    SPA_AUDIO_CHANNEL_SR,
                ],
            ),
            Ok(Some(0x6587_4321))
        );
    }

    #[test]
    fn opaque_and_unpositioned_channel_orders_stay_opaque() {
        assert_eq!(
            native_channel_order(
                0,
                &[
                    SPA_AUDIO_CHANNEL_AUX0,
                    SPA_AUDIO_CHANNEL_AUX0 + 1,
                    SPA_AUDIO_CHANNEL_AUX0 + 2,
                ],
            ),
            Ok(None)
        );
        assert_eq!(
            native_channel_order(
                SPA_AUDIO_FLAG_UNPOSITIONED,
                &[SPA_AUDIO_CHANNEL_FR, SPA_AUDIO_CHANNEL_FL],
            ),
            Ok(None)
        );
    }

    #[test]
    fn malformed_channel_orders_are_rejected() {
        for positions in [
            &[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FL][..],
            &[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_AUX0],
            &[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FLC],
            &[
                SPA_AUDIO_CHANNEL_FL,
                SPA_AUDIO_CHANNEL_FR,
                SPA_AUDIO_CHANNEL_FC,
                SPA_AUDIO_CHANNEL_LFE,
            ],
            &[
                SPA_AUDIO_CHANNEL_FL,
                SPA_AUDIO_CHANNEL_FR,
                SPA_AUDIO_CHANNEL_FC,
                SPA_AUDIO_CHANNEL_LFE,
                SPA_AUDIO_CHANNEL_RL,
                SPA_AUDIO_CHANNEL_RR,
                SPA_AUDIO_CHANNEL_SL,
                SPA_AUDIO_CHANNEL_SR,
                SPA_AUDIO_CHANNEL_BC,
            ],
        ] {
            assert_eq!(
                native_channel_order(0, positions),
                Err(ChannelMapError::Unsupported)
            );
        }
    }

    #[test]
    fn native_sample_widths_match_the_supported_formats() {
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_F32_LE), Some(4));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_S24_BE), Some(3));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_S16_LE), Some(2));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U8), Some(1));
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
            DeliveryQuantum::unavailable(),
        )
        .unwrap();

        assert!(!outcome.adjusted.is_empty());
        assert_eq!(outcome.actual_config.rate, 47_999);
        assert_eq!(outcome.actual_config.channels, 2);
        assert_eq!(outcome.actual_config.stride, 4);
        assert_eq!(
            outcome.actual_config.format.0,
            libspa::sys::SPA_AUDIO_FORMAT_S16_LE
        );
    }

    #[test]
    fn native_channel_clamp_updates_layout_and_stride_together() {
        let requested = StreamConfig {
            format: libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_S16_LE),
            rate: 48_000,
            channels: 6,
            positions: channel_positions(6).unwrap().to_vec(),
            flags: 0,
            stride: 12,
        };
        let outcome = configure_outcome(
            &requested,
            super::super::dsp::AppliedNativeConfig {
                format: super::super::AFMT_S16_LE,
                channels: 2,
                rate: 48_000,
            },
            DeliveryQuantum::unavailable(),
        )
        .unwrap();

        assert_eq!(outcome.actual_config.channels, 2);
        assert_eq!(outcome.actual_config.stride, 4);
        assert_eq!(
            outcome.actual_config.positions,
            channel_positions(2).unwrap()
        );
        assert!(!outcome.adjusted.is_empty());
    }

    #[test]
    fn native_delivery_quantum_is_scaled_into_applied_stream_bytes() {
        let requested = StreamConfig {
            format: libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_S16_LE),
            rate: 96_000,
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
                rate: 96_000,
            },
            DeliveryQuantum {
                frames: 256,
                rate: 48_000,
                quality: QuantumQuality::Estimated,
            },
        )
        .unwrap();

        assert_eq!(outcome.applied_buffer.quantum_bytes, Some(2_048));
    }
}
