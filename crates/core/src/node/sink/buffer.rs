use super::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum RetuneOutcome {
    /// The current period remains in effect.
    Unchanged,
    /// Geometry changed, either live or after trigger-suspending for re-prime.
    Retuned,
    /// The ring cannot be retuned and the device refused trigger suspension.
    Rebuild,
}

pub(super) fn predicted_next_fill(fill: u32, write_now: u32, period: u32) -> u32 {
    fill.saturating_add(write_now).saturating_sub(period)
}

// the resampler's per-cycle output can exceed a quantum; its size bounds the
// largest single write and so the headroom the fill ceiling must reserve
pub(super) fn rate_match_bytes(rate_match: &spa::IoArea<spa_io_rate_match>, stride: u32) -> u32 {
    rate_match
        .with_ref(|rm| rm.size.saturating_mul(stride))
        .unwrap_or(0)
}

pub(super) fn commit_geometry<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    period: u32,
    geometry: PlaybackBufferGeometry,
) -> bool {
    // defense in depth (the prime path gates on the period too): a committed
    // setup_period of 0 would flip the channel to Running with degenerate
    // geometry, and retune_period's setup_period == 0 early-exit would never
    // re-commit - stuck until a full rebuild
    if period == 0 {
        return false;
    }
    port.setup_period = period;
    port.delivery_quantum_bytes = geometry.quantum_bytes;
    port.ext.buffer_size = geometry.capacity_bytes;
    port.ext.target_delay = geometry.target_fill_bytes;
    port.ext.target_goal = geometry.target_goal_bytes;
    port.ext.minimum_fill = geometry.minimum_fill_bytes;
    port.dll.init();
    port.bw_adapt.reset();
    let (stride, rate) = port.stride_rate().unwrap_or((1, 0));
    port.bw_adapt.configure(
        stride,
        geometry.quantum_bytes,
        period,
        rate.saturating_mul(stride),
    );
    true
}

pub(super) fn retune_period<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    period_in_bytes: u32,
    stride: u32,
    write_now: u32,
    now: u64,
    log: &Log,
) -> RetuneOutcome {
    let current_fill = measured_fill(port);
    let request = PlaybackBufferRequest {
        period_bytes: period_in_bytes,
        // A live retune reuses the established backend capacity; graph_rate
        // is only an input to the initial ring plan.
        graph_rate: 0,
        stride,
        device_rate: port.stride_rate().map_or(0, |(_, rate)| rate),
        write_bytes: write_now,
        maximum_write_bytes: rate_match_bytes(&port.rate_match, stride),
    };
    match port.dsp.retune_buffer(request, current_fill, now, log) {
        PlaybackRetune::Unchanged => {
            port.ext.retune_pending = false;
            RetuneOutcome::Unchanged
        }
        PlaybackRetune::Pending => {
            port.ext.retune_pending = true;
            RetuneOutcome::Unchanged
        }
        PlaybackRetune::Applied(geometry) => {
            commit_geometry(port, period_in_bytes, geometry);
            port.ext.retune_pending = false;
            RetuneOutcome::Retuned
        }
        PlaybackRetune::Reprime => {
            port.ext.retune_pending = false;
            port.ext.xrun_timestamp = 0;
            port.was_matching = false;
            reset_stream_epoch(port);
            RetuneOutcome::Retuned
        }
        PlaybackRetune::Rebuild => {
            port.ext.retune_pending = true;
            RetuneOutcome::Rebuild
        }
    }
}

// The prime phase: the channel is in setup (first cycle, or a trigger
// suspend from the retune/resize path), so the ring layout can be applied.
// Size the ring, commit the fill geometry and pre-fill to target; the
// cycle's real write then arms the channel.
pub(super) fn prime_playback<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    period_in_bytes: u32,
    graph_rate: u32,
    properties: &BackendPropertiesOf<SinkDir<B>>,
    log: &Log,
) {
    #[cfg(debug_assertions)]
    <B::Playback as backend::PlaybackOperations>::debug_log_priorities(log);

    let Some((stride, cfg_rate)) = port.stride_rate() else {
        return;
    };
    if period_in_bytes == 0 {
        return; // see commit_geometry: zero-period geometry is never committed
    }

    let request = PlaybackBufferRequest {
        period_bytes: period_in_bytes,
        graph_rate,
        stride,
        device_rate: cfg_rate,
        write_bytes: period_in_bytes,
        maximum_write_bytes: rate_match_bytes(&port.rate_match, stride),
    };
    let geometry = port.dsp.prime_buffer(request, properties, log);
    commit_geometry(port, period_in_bytes, geometry);
}
