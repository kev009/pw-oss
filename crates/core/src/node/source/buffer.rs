use super::*;

// Apply the fill targets for `period` against the granted ring and relock the
// servo.
pub(super) fn commit_geometry<B: backend::Backend>(
    port: &mut Port<SourceDir<B>>,
    period: u32,
    geometry: backend::CaptureBufferGeometry,
) -> bool {
    // The process prime arm and current backends reject a zero period first.
    // Keep the shared commit boundary defensive so a future backend cannot
    // mark capture primed with degenerate fill and servo geometry.
    if period == 0 {
        return false;
    }
    port.setup_period = period;
    port.delivery_quantum_bytes = geometry.quantum_bytes;
    port.ext.ring_size = geometry.capacity_bytes;
    port.ext.target_fill = geometry.target_fill_bytes;
    port.ext.read_peak = geometry.peak_fill_bytes;
    port.dll.init();
    port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle
    let (stride, rate) = port.stride_rate().unwrap_or((1, 0));
    port.bw_adapt.configure(
        stride,
        geometry.quantum_bytes,
        period,
        rate.saturating_mul(stride),
    );
    true
}

// A period change retunes the servo because stale fill targets would steer it
// toward the old cycle latency indefinitely. The backend decides whether the
// new geometry applies live, requires a same-cycle re-prime, or needs a full
// rebuild; every applied path relocks the servo from the returned geometry.
pub(super) fn retune_period<B: backend::Backend>(
    port: &mut Port<SourceDir<B>>,
    period_in_bytes: u32,
    log: &Log,
) -> bool {
    let Some((stride, device_rate)) = port.stride_rate() else {
        port.ext.retune_pending = false;
        return false;
    };
    let request = backend::CaptureBufferRequest {
        period_bytes: period_in_bytes,
        // A live retune reuses the established backend capacity; graph_rate
        // is only an input to the initial ring plan.
        graph_rate: 0,
        stride,
        device_rate,
    };
    match port.dsp.retune_buffer(request, port.ext.primed, log) {
        backend::CaptureRetune::Unchanged => {
            port.ext.retune_pending = false;
            false
        }
        backend::CaptureRetune::Pending => {
            port.ext.retune_pending = true;
            false
        }
        backend::CaptureRetune::Applied(geometry) => {
            port.ext.retune_pending = false;
            commit_geometry(port, period_in_bytes, geometry);
            false
        }
        backend::CaptureRetune::Reprime => {
            port.ext.retune_pending = false;
            port.ext.primed = false;
            reset_stream_epoch(port);
            false
        }
        backend::CaptureRetune::Rebuild => {
            port.ext.retune_pending = true;
            true
        }
    }
}

// The prime phase - the capture analogue of the sink's zero priming: trigger
// the device, discard any backlog so the fill level starts out known, and
// hand the graph one period of silence while the ring fills. Don't wait for
// real data: an empty first cycle reads as a missed deadline to the graph.
// Re-apply the backend buffer layout while the channel is in setup. Returns
// the cycle's byte count (the period of silence), or
// EMPTY_CYCLE before a format is negotiated (unreachable past the caller's
// gate).
pub(super) fn prime_capture<B: backend::Backend>(
    port: &mut Port<SourceDir<B>>,
    period_in_bytes: u32,
    graph_rate: u32,
    properties: &BackendPropertiesOf<SourceDir<B>>,
    data: &mut [u8],
    log: &Log,
) -> isize {
    let Some((stride, rate)) = port.stride_rate() else {
        return EMPTY_CYCLE;
    };
    let geometry = port.dsp.prime_buffer(
        backend::CaptureBufferRequest {
            period_bytes: period_in_bytes,
            graph_rate,
            stride,
            device_rate: rate,
        },
        properties,
        data,
        log,
    );
    port.rebuild_required |= geometry.device_lost;
    port.ext.primed = true;
    commit_geometry(port, period_in_bytes, geometry);
    port.dsp.clear_overrun_observation();

    let len = period_in_bytes.min(data.len() as u32);
    fill_silence(port, &mut data[..len as usize]);
    len as isize
}
