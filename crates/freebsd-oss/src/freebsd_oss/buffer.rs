//! FreeBSD PCM soft-ring policy.
//!
//! These calculations encode kernel limits and OSS fragment behavior. Keeping
//! them pure and together makes the policy testable without opening a device
//! and keeps numerical OSS ring policy out of the SPA node.

use crate::backend::{
    CaptureBufferGeometry, CaptureBufferRequest, PlaybackBufferGeometry, PlaybackBufferRequest,
};

// sys/dev/sound/pcm/channel.h
pub(crate) const MAX_BUFFER_BYTES: usize = 131_072;

// Every ring request keeps at least this byte budget in both directions.
pub(crate) const MIN_BUFFER_BYTES: u32 = 65_536;

/// The kernel's per-channel soft-ring byte budget.
///
/// The FreeBSD PCM channel limit is independent of format and rate.
pub(crate) fn buffer_capacity_limit(_stride: u32, _rate: u32) -> u32 {
    MAX_BUFFER_BYTES as u32
}

/// Normalize the public `oss.fragment` value to a conservative kernel request.
pub(crate) fn normalize_fragment(value: u32) -> u32 {
    if value == 0 {
        0
    } else {
        (1u32 << (31 - value.leading_zeros())).clamp(64, 16_384)
    }
}

fn graph_frames_in_device_bytes(
    frames: u64,
    device_rate: u32,
    graph_rate: u32,
    stride: u32,
) -> u32 {
    if graph_rate == 0 {
        return frames.saturating_mul(stride as u64).min(u32::MAX as u64) as u32;
    }
    frames
        .saturating_mul(device_rate as u64)
        .saturating_div(graph_rate as u64)
        .saturating_mul(stride as u64)
        .min(u32::MAX as u64) as u32
}

/// One graph cycle of the largest commonly negotiable quantum in device bytes.
pub(crate) fn max_buffer_period_bytes(stride: u32, device_rate: u32, graph_rate: u32) -> u32 {
    let stride = stride.max(1);
    let default_max = graph_frames_in_device_bytes(2_048, device_rate, graph_rate, stride);
    let cap_frames = buffer_capacity_limit(stride, device_rate) / stride / 4;
    default_max.min(cap_frames.saturating_mul(stride))
}

/// A node.max-latency cap when four graph periods would not fit the OSS ring.
///
/// At fat strides (for example a 20-channel S32 interface), the fixed PCM
/// soft-ring byte cap can hold fewer than two ordinary graph periods. Capture
/// then has no arrival-jitter headroom and playback cannot retain both quanta
/// plus its delay target. The 44.1 kHz comparison publishes a conservative
/// time-domain cap before that geometry becomes structurally unsafe.
pub(crate) fn advertised_quantum_cap_frames(stride: u32, rate: u32) -> Option<u32> {
    let stride = stride.max(1);
    let frames = buffer_capacity_limit(stride, rate) / stride / 4;
    if rate == 0 || frames as u64 * 44_100 >= 2_048 * rate as u64 {
        return None;
    }
    Some(frames)
}

pub(crate) fn playback_desired_delay(period: u32, delay_eighths: u32) -> u32 {
    (period / 8).saturating_mul(delay_eighths)
}

/// Lowest healthy playback fill: one period plus a jitter margin.
///
/// Both buffer admission and the applied target derive from this value, so an
/// in-place retune is accepted only when the granted ring can hold the target.
pub(crate) fn playback_fill_floor(period: u32, blocksize: u32) -> u32 {
    period.saturating_add((period / 4).max(blocksize))
}

pub(crate) fn playback_buffer_required(
    period: u32,
    desired: u32,
    blocksize: u32,
    write_max: u32,
) -> u32 {
    period.saturating_mul(2).saturating_add(desired).max(
        playback_fill_floor(period, blocksize)
            .saturating_add(write_max)
            .saturating_add(blocksize),
    )
}

/// Stable playback ring request for the current and largest graph quantum.
///
/// Capacity is not the playback latency target. The extra room lets later
/// graph-quantum changes retune without resizing the kernel ring, while the
/// applied fill target below controls queued audio. The kernel cap always wins
/// over the preferred minimum.
#[expect(clippy::too_many_arguments)]
pub(crate) fn playback_buffer_request(
    period: u32,
    max_period: u32,
    stride: u32,
    rate: u32,
    fragment: u32,
    hardware_chunk: u32,
    write_max: u32,
    delay_eighths: u32,
) -> u32 {
    let fragment_estimate = if fragment == 0 { 1_024 } else { fragment };
    let transfer = fragment_estimate.max(hardware_chunk);
    let stable = playback_buffer_required(
        max_period,
        playback_desired_delay(max_period, delay_eighths),
        transfer,
        max_period,
    );
    playback_buffer_required(
        period,
        playback_desired_delay(period, delay_eighths),
        transfer,
        write_max,
    )
    .max(stable)
    .max(MIN_BUFFER_BYTES)
    .min(buffer_capacity_limit(stride, rate))
}

/// Playback fill target derived from the buffer grant and write headroom.
///
/// The target keeps one period plus a jitter margin queued and reserves room
/// for the largest expected write plus one delivery chunk. This avoids driving
/// large uaudio rings near full while still making the best of drivers that
/// grant less than two graph periods.
pub(crate) fn playback_target_delay(
    granted: u32,
    period: u32,
    blocksize: u32,
    write_max: u32,
    desired: u32,
) -> (u32, bool) {
    if granted >= period.saturating_mul(2) {
        let floor = playback_fill_floor(period, blocksize);
        let ceiling = granted
            .saturating_sub(write_max.saturating_add(blocksize))
            .max(period);
        let wanted = desired.max(floor);
        (wanted.min(ceiling).max(period), wanted > ceiling)
    } else {
        (granted / 2, false)
    }
}

/// Capture fill target and healthy catch-up peak for an applied ring.
///
/// The target adds half an arrival below the nominal period. Device fill
/// readings move in arrival-sized steps, so a one-period target otherwise
/// bottoms out at zero whenever an arrival is even slightly late.
///
/// The peak adds catch-up slack above the target but remains one arrival below
/// the granted ring. This leaves headroom for the next arrival and prevents a
/// clamped ring from pinning at its ceiling.
pub(crate) fn capture_fill_targets(period: u32, blocksize: u32, ring: u32) -> (u32, u32) {
    let target = period.saturating_add(blocksize / 2);
    let mut peak = target
        .saturating_add(blocksize / 2)
        .saturating_add(period / 2);
    if ring > 0 {
        let ring_peak = ring.saturating_sub(blocksize);
        let min_peak = target.saturating_add(blocksize).min(ring_peak);
        peak = peak.min(ring_peak).max(min_peak);
    }
    (target, peak)
}

/// Capture ring request, large enough for four current or maximum periods.
pub(crate) fn capture_buffer_request(period: u32, max_period: u32, stride: u32, rate: u32) -> u32 {
    period
        .saturating_mul(4)
        .max(max_period.saturating_mul(4))
        .max(MIN_BUFFER_BYTES)
        .min(buffer_capacity_limit(stride, rate))
}

pub(crate) fn capture_buffer_required(period: u32, blocksize: u32) -> u32 {
    // The healthy target, one-arrival catch-up band, and one arrival of top
    // headroom must all fit. The two-period bound covers small block sizes.
    let (target, _) = capture_fill_targets(period, blocksize, 0);
    target
        .saturating_add(blocksize.saturating_mul(2))
        .max(period.saturating_mul(2))
}

pub(crate) fn capture_buffer_plan(
    request: CaptureBufferRequest,
    fragment_bytes: u32,
) -> (u32, u32) {
    let max_period =
        max_buffer_period_bytes(request.stride, request.device_rate, request.graph_rate);
    let m = request.period_bytes.max(1_024);
    let fragment_cap = 1u32 << (31 - m.leading_zeros());
    let fragment = if fragment_bytes == 0 {
        1_024
    } else {
        fragment_bytes.min(fragment_cap)
    };
    let capacity = capture_buffer_request(
        request.period_bytes,
        max_period,
        request.stride,
        request.device_rate,
    );
    (fragment, capacity)
}

pub(crate) fn capture_applied_geometry(
    request: CaptureBufferRequest,
    capacity_bytes: u32,
    granted_quantum_bytes: u32,
    delivery_quantum_ns: u64,
) -> CaptureBufferGeometry {
    let delivery_quantum_bytes =
        delivery_quantum_bytes(delivery_quantum_ns, request.device_rate, request.stride);
    let quantum_bytes = granted_quantum_bytes.max(delivery_quantum_bytes);
    let (target_fill_bytes, peak_fill_bytes) =
        capture_fill_targets(request.period_bytes, quantum_bytes, capacity_bytes);
    CaptureBufferGeometry {
        capacity_bytes,
        quantum_bytes,
        target_fill_bytes,
        peak_fill_bytes,
        required_capacity_bytes: capture_buffer_required(request.period_bytes, quantum_bytes),
        device_lost: false,
    }
}

fn delivery_quantum_bytes(ns: u64, rate: u32, stride: u32) -> u32 {
    let stride = stride.max(1);
    let bytes = ((ns as u128)
        .saturating_mul(rate as u128)
        .saturating_mul(stride as u128)
        / 1_000_000_000)
        .min(u32::MAX as u128) as u32;
    bytes
        .checked_next_multiple_of(stride)
        .unwrap_or(u32::MAX - u32::MAX % stride)
}

pub(crate) fn playback_buffer_plan(
    request: PlaybackBufferRequest,
    delivery_quantum_ns: u64,
    fragment_bytes: u32,
    delay_eighths: u32,
) -> (u32, u32) {
    let hardware_quantum =
        delivery_quantum_bytes(delivery_quantum_ns, request.device_rate, request.stride);
    let write_max = request.period_bytes.max(request.maximum_write_bytes);
    let max_period =
        max_buffer_period_bytes(request.stride, request.device_rate, request.graph_rate);
    let capacity = playback_buffer_request(
        request.period_bytes,
        max_period,
        request.stride,
        request.device_rate,
        fragment_bytes,
        hardware_quantum,
        write_max,
        delay_eighths,
    );
    (capacity, hardware_quantum)
}

pub(crate) fn playback_applied_geometry(
    request: PlaybackBufferRequest,
    capacity_bytes: u32,
    granted_quantum_bytes: u32,
    delivery_quantum_ns: u64,
    delay_eighths: u32,
) -> PlaybackBufferGeometry {
    let hardware_quantum =
        delivery_quantum_bytes(delivery_quantum_ns, request.device_rate, request.stride);
    let quantum_bytes = granted_quantum_bytes.max(hardware_quantum);
    let write_max = request.period_bytes.max(request.maximum_write_bytes);
    let desired = playback_desired_delay(request.period_bytes, delay_eighths);
    let (target_fill_bytes, delay_capped) = playback_target_delay(
        capacity_bytes,
        request.period_bytes,
        quantum_bytes,
        write_max,
        desired,
    );
    PlaybackBufferGeometry {
        capacity_bytes,
        quantum_bytes,
        target_fill_bytes,
        target_goal_bytes: target_fill_bytes,
        minimum_fill_bytes: playback_fill_floor(request.period_bytes, quantum_bytes),
        required_capacity_bytes: playback_buffer_required(
            request.period_bytes,
            desired,
            quantum_bytes,
            write_max,
        ),
        delay_capped,
    }
}

pub(crate) fn playback_retuned_geometry(
    request: PlaybackBufferRequest,
    capacity_bytes: u32,
    granted_quantum_bytes: u32,
    delivery_quantum_ns: u64,
    current_fill_bytes: u32,
    delay_eighths: u32,
) -> PlaybackBufferGeometry {
    let mut geometry = playback_applied_geometry(
        request,
        capacity_bytes,
        granted_quantum_bytes,
        delivery_quantum_ns,
        delay_eighths,
    );
    let predicted = current_fill_bytes
        .saturating_add(request.write_bytes)
        .saturating_sub(request.period_bytes);
    geometry.target_fill_bytes = geometry.target_goal_bytes.min(predicted);
    geometry
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fragment_normalization_rounds_down_and_clamps() {
        for (input, expected) in [
            (0, 0),
            (1, 64),
            (63, 64),
            (64, 64),
            (65, 64),
            (1_000, 512),
            (4_096, 4_096),
            (16_384, 16_384),
            (30_000, 16_384),
            (1 << 31, 16_384),
            (u32::MAX, 16_384),
        ] {
            assert_eq!(normalize_fragment(input), expected);
        }
    }

    #[test]
    fn maximum_period_respects_rate_scaling_and_ring_capacity() {
        assert_eq!(max_buffer_period_bytes(8, 48_000, 48_000), 16_384);
        assert_eq!(max_buffer_period_bytes(8, 96_000, 48_000), 32_768);
        assert_eq!(max_buffer_period_bytes(40, 48_000, 48_000), 819 * 40);
        assert_eq!(max_buffer_period_bytes(8, 48_000, 0), 16_384);
    }

    #[test]
    fn advertised_quantum_cap_is_time_based() {
        assert_eq!(advertised_quantum_cap_frames(8, 48_000), None);
        assert_eq!(advertised_quantum_cap_frames(8, 192_000), Some(4_096));
        assert_eq!(advertised_quantum_cap_frames(8, 96_000), Some(4_096));
        assert_eq!(advertised_quantum_cap_frames(40, 48_000), Some(819));
        assert_eq!(advertised_quantum_cap_frames(8, 44_100), None);
    }

    #[test]
    fn delivery_quantum_rounds_to_frames_and_stays_saturated() {
        assert_eq!(delivery_quantum_bytes(5_333_333, 48_000, 8), 2_048);
        assert_eq!(delivery_quantum_bytes(0, 48_000, 8), 0);
        assert_eq!(
            delivery_quantum_bytes(u64::MAX, u32::MAX, 8),
            u32::MAX - u32::MAX % 8
        );
        assert_eq!(delivery_quantum_bytes(u64::MAX, u32::MAX, 1), u32::MAX);
    }

    #[test]
    fn playback_buffer_request_covers_the_largest_negotiable_quantum() {
        let request = playback_buffer_request(4_096, 16_384, 8, 48_000, 0, 2_048, 4_096, 4);
        assert!(
            request
                >= playback_buffer_required(
                    16_384,
                    playback_desired_delay(16_384, 4),
                    2_048,
                    16_384,
                )
        );
        assert!(request >= MIN_BUFFER_BYTES.min(buffer_capacity_limit(8, 48_000)));
        assert!(request <= buffer_capacity_limit(8, 48_000));
    }

    #[test]
    fn playback_target_matches_live_geometry() {
        assert_eq!(
            playback_target_delay(65_536, 16_384, 2_048, 16_384, 0),
            (20_480, false)
        );
        assert_eq!(playback_fill_floor(16_384, 8_192), 24_576);
    }

    #[test]
    fn playback_grant_at_required_has_safe_headroom() {
        for period in [1_024u32, 4_096, 16_384, 65_536] {
            for quantum in [512u32, 1_024, 2_047, 2_048, 16_384, 65_536] {
                for write_max in [period, period.saturating_mul(2), period.saturating_mul(4)] {
                    for delay_eighths in [0u32, 4, 32, 1_024] {
                        let desired = playback_desired_delay(period, delay_eighths);
                        let required =
                            playback_buffer_required(period, desired, quantum, write_max);
                        for granted in [
                            required,
                            required.saturating_add(1),
                            required.saturating_mul(2),
                        ] {
                            let (target, _) =
                                playback_target_delay(granted, period, quantum, write_max, desired);
                            assert!(target >= playback_fill_floor(period, quantum));
                            assert!(
                                target.saturating_add(write_max).saturating_add(quantum) <= granted
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn playback_small_grant_and_oversized_delay_are_bounded() {
        assert_eq!(
            playback_target_delay(8_192, 16_384, 1_024, 16_384, 0),
            (4_096, false)
        );
        let (target, capped) = playback_target_delay(65_536, 4_096, 1_024, 4_096, u32::MAX);
        assert_eq!(target, 65_536 - 4_096 - 1_024);
        assert!(capped);
    }

    #[test]
    fn capture_buffer_request_floors_and_caps() {
        assert_eq!(capture_buffer_request(1_024, 16_384, 8, 48_000), 16_384 * 4);
        assert_eq!(
            capture_buffer_request(32_768, 16_384, 8, 48_000),
            32_768 * 4
        );
        assert!(capture_buffer_request(64, 64, 8, 48_000) >= MIN_BUFFER_BYTES);
        assert_eq!(
            capture_buffer_request(65_536, 65_536, 8, 48_000),
            MAX_BUFFER_BYTES as u32
        );
    }

    #[test]
    fn capture_targets_track_arrival_granularity() {
        for period in [1_024u32, 4_096, 16_384, 65_536] {
            for quantum in [512u32, 1_024, 2_047, 2_048, 16_384, 65_536] {
                let (target, peak) = capture_fill_targets(period, quantum, 0);
                assert_eq!(target, period + quantum / 2);
                assert_eq!(peak, target + quantum / 2 + period / 2);

                let ring = capture_buffer_required(period, quantum);
                let (bounded_target, bounded_peak) = capture_fill_targets(period, quantum, ring);
                assert_eq!(bounded_target, target);
                assert!(bounded_peak >= target + quantum);
                assert!(bounded_peak <= ring - quantum);

                let (_, degenerate_peak) = capture_fill_targets(period, quantum, period);
                assert!(degenerate_peak <= period.saturating_sub(quantum));
            }
        }
    }
}
