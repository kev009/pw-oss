//! FreeBSD PCM soft-ring policy.
//!
//! These calculations encode kernel limits and OSS fragment behavior. Keeping
//! them pure and together makes the policy testable without opening a device
//! and keeps the SPA node focused on graph lifecycle and servo state.

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
}
