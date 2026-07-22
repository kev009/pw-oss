//! Audio stream capabilities, events, configuration, and I/O outcomes.

use libspa::sys::SPA_AUDIO_FORMAT_U8;
use std::ffi::c_int;

pub(crate) use crate::oss::{
    Dsp as CaptureStream, DspWriter as PlaybackStream, SoundKqueue as WakeQueue,
};
pub(crate) use crate::oss::{
    advertised_quantum_cap_frames, buffer_capacity_limit, capture_buffer_request,
    capture_buffer_required, capture_fill_targets, configure_capture_buffer,
    configure_playback_buffer, enriched_sound_kqueue_available as device_wake_available,
    fallback_caps, max_buffer_period_bytes, normalize_fragment, playback_buffer_request,
    playback_buffer_required, playback_desired_delay, playback_fill_floor, playback_target_delay,
    probe_caps,
};
pub(crate) use crate::oss::{
    bytes_per_sample, channel_positions, configure_capture, configure_playback, validate_config,
};

#[cfg(test)]
pub(crate) use crate::oss::{all_formats, test_native_channel_order};

#[cfg(test)]
pub(crate) mod test_transport;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct DeviceEvent {
    pub(crate) fd: c_int,
    /// Readable capture bytes or writable playback bytes at the wake edge.
    pub(crate) available_bytes: u32,
    /// Playback frames queued in the backend buffer, when supplied.
    pub(crate) queued_frames: Option<u64>,
    pub(crate) xruns: u32,
    pub(crate) eof: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum WakeEvent {
    Timer,
    Device(DeviceEvent),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct WriteOutcome {
    pub(crate) bytes: usize,
    pub(crate) status: IoStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ReadOutcome {
    pub(crate) bytes: usize,
    pub(crate) status: IoStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum IoStatus {
    Progress,
    WouldBlock,
    Interrupted,
    Disconnected,
    Failed,
}

impl IoStatus {
    pub(crate) fn device_lost(self) -> bool {
        matches!(self, Self::Disconnected | Self::Failed)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct BufferLayout {
    pub(crate) queued_bytes: u32,
    pub(crate) quantum_bytes: u32,
    pub(crate) capacity_bytes: u32,
}

impl WriteOutcome {
    pub(crate) fn consumed(bytes: usize) -> Self {
        Self {
            bytes,
            status: IoStatus::Progress,
        }
    }

    pub(crate) fn would_block(&self) -> bool {
        self.bytes == 0 && self.status == IoStatus::WouldBlock
    }

    pub(crate) fn retryable_partial(&self) -> bool {
        matches!(self.status, IoStatus::Progress | IoStatus::WouldBlock)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StreamConfig {
    pub(crate) format: libspa::param::audio::AudioFormat,
    pub(crate) rate: u32,
    pub(crate) channels: u32,
    pub(crate) positions: Vec<u32>,
    pub(crate) flags: u32,
    pub(crate) stride: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ConfigureOutcome {
    pub(crate) actual_config: StreamConfig,
    pub(crate) adjusted: bool,
}

impl StreamConfig {
    pub(crate) fn silence_byte(&self) -> u8 {
        // Every currently supported signed integer and float format represents
        // silence with all-zero bits. Unsigned 8-bit PCM is biased around 0x80.
        if self.format.0 == SPA_AUDIO_FORMAT_U8 {
            0x80
        } else {
            0
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ConversionKind {
    None,
    Backend,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StreamConfiguration {
    /// SPA formats offered to the graph, in preference order.
    pub(crate) formats: Vec<u32>,
    pub(crate) min_channels: u32,
    pub(crate) max_channels: u32,
    pub(crate) min_rate: u32,
    pub(crate) max_rate: u32,
    pub(crate) preferred_rate: Option<u32>,
    /// Discrete rates; empty means the min/max range is dense.
    pub(crate) rates: Vec<u32>,
    /// Backend-specific tolerance used when a dense rate range snaps values.
    pub(crate) rate_tolerance: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct StreamCaps {
    pub(crate) configurations: Vec<StreamConfiguration>,
    pub(crate) preferred: usize,
    pub(crate) conversion: ConversionKind,
}

impl StreamCaps {
    pub(crate) fn admits(&self, spa_format: u32, channels: u32, rate: u32) -> bool {
        self.configurations.iter().any(|configuration| {
            if !configuration.formats.contains(&spa_format) {
                return false;
            }
            if channels < configuration.min_channels || channels > configuration.max_channels {
                return false;
            }
            if !configuration.rates.is_empty() {
                return configuration.rates.contains(&rate);
            }
            rate.saturating_add(configuration.rate_tolerance) >= configuration.min_rate
                && rate
                    <= configuration
                        .max_rate
                        .saturating_add(configuration.rate_tolerance)
        })
    }

    pub(crate) fn preferred_configuration(&self) -> Option<&StreamConfiguration> {
        let preferred = if self.preferred < self.configurations.len() {
            self.preferred
        } else {
            0
        };
        self.configurations.get(preferred)
    }

    pub(crate) fn configurations_in_preference_order(
        &self,
    ) -> impl Iterator<Item = &StreamConfiguration> {
        let preferred = if self.preferred < self.configurations.len() {
            self.preferred
        } else {
            0
        };
        self.preferred_configuration().into_iter().chain(
            self.configurations
                .iter()
                .enumerate()
                .filter(move |(index, _)| *index != preferred)
                .map(|(_, configuration)| configuration),
        )
    }
}

pub(crate) fn offered_formats(configuration: &StreamConfiguration) -> &[u32] {
    &configuration.formats
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ChannelMapError {
    Unsupported,
    ConvertlessReorder,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn configuration(format: u32, channels: u32, rate: u32) -> StreamConfiguration {
        StreamConfiguration {
            formats: vec![format],
            min_channels: channels,
            max_channels: channels,
            min_rate: rate,
            max_rate: rate,
            preferred_rate: None,
            rates: vec![],
            rate_tolerance: 0,
        }
    }

    #[test]
    fn backend_conversion_does_not_cross_configuration_constraints() {
        let caps = StreamCaps {
            configurations: vec![configuration(10, 2, 48_000), configuration(20, 6, 96_000)],
            preferred: 0,
            conversion: ConversionKind::Backend,
        };

        assert!(caps.admits(10, 2, 48_000));
        assert!(caps.admits(20, 6, 96_000));
        assert!(!caps.admits(20, 2, 48_000));
        assert!(!caps.admits(10, 6, 96_000));
        assert!(!caps.admits(30, 2, 48_000));
    }

    #[test]
    fn semantic_io_outcomes_distinguish_progress_and_backpressure() {
        let consumed = WriteOutcome::consumed(128);
        assert_eq!(consumed.status, IoStatus::Progress);
        assert!(consumed.retryable_partial());
        assert!(!consumed.would_block());

        let blocked = WriteOutcome {
            bytes: 0,
            status: IoStatus::WouldBlock,
        };
        assert!(blocked.retryable_partial());
        assert!(blocked.would_block());

        let failed = WriteOutcome {
            bytes: 64,
            status: IoStatus::Failed,
        };
        assert!(!failed.retryable_partial());
        assert!(!failed.would_block());
        assert!(failed.status.device_lost());
        assert!(!IoStatus::Interrupted.device_lost());
    }
}
