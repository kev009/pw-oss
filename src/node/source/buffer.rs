use super::*;

// Apply the fill targets for `period` against the granted ring and relock the
// servo.
pub(super) fn commit_geometry(port: &mut Port<SourceDir>, period: u32, blocksize: u32) {
    let (target, peak) = fill_targets(period, blocksize, port.ext.ring_size);
    port.setup_period = period;
    port.setup_blocksize = blocksize;
    port.ext.target_fill = target;
    port.ext.read_peak = peak;
    port.dll.init();
    port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle
    let (stride, rate) = port.stride_rate().unwrap_or((1, 0));
    port.bw_adapt
        .configure(stride, blocksize, period, rate.saturating_mul(stride));
}

// The retune phase. A period change re-tunes the servo: the fill target and
// catch-up peak derive from the period, and stale ones steer the servo to
// the OLD quantum's latency forever (ALSA compensates the error by the
// threshold delta, we relock fast instead). The ring IS SETFRAGMENT-sized
// (at prime), so the retune stays in place only while the grant still holds
// the new period's geometry (ring_required); a smaller ring suspends the
// channel so the prime phase re-applies the fragment layout at the new size
// in this very cycle. Returns true when the driver refused the trigger stop
// (dying fd) and only a main-thread rebuild remains.
pub(super) fn retune_period(port: &mut Port<SourceDir>, period_in_bytes: u32, log: &Log) -> bool {
    if !port.ext.primed
        || port.setup_period == 0
        || period_in_bytes == 0
        || period_in_bytes == port.setup_period
    {
        port.ext.period_mismatch = 0;
        return false;
    }
    // debounce (as the sink): a single-cycle flip usually means a
    // renegotiation is in flight; read at the old geometry for one cycle
    // rather than relock the servo on every flip of a storm
    port.ext.period_mismatch += 1;
    if port.ext.period_mismatch < 2 {
        return false;
    }
    // cached blocksize: the triggered channel refuses SETFRAGMENT and the
    // hw cadence is per-session, so the grant can't have changed since
    // prime (the sink reuses its cache for the same reason)
    let blocksize = port.setup_blocksize;
    if port.ext.ring_size >= ring_required(period_in_bytes, blocksize) {
        commit_geometry(port, period_in_bytes, blocksize);
        port.ext.period_mismatch = 0;
        false
    } else if port.dsp.suspend() {
        // re-enter priming IN THIS CYCLE: it re-applies the fragment layout at
        // the new size, discards the backlog, hands the graph one period of
        // silence and relocks (commit_geometry at prime) - the cost the overrun
        // recovery already pays, and one cycle cheaper than a close/reopen
        crate::info!(
            log,
            "capture period {} -> {} bytes exceeds the ring ({}); re-priming",
            port.setup_period,
            period_in_bytes,
            port.ext.ring_size
        );
        port.ext.period_mismatch = 0;
        port.ext.primed = false;
        // SETTRIGGER starts a new kernel xrun epoch; SETFRAGMENT also resets
        // the low-water mark during the prime that follows.
        reset_device_event(port);
        false
    } else {
        // period_mismatch stays >= 2 on purpose: if the caller can't queue the
        // rebuild (no main loop), the next cycle retries this retune
        // immediately instead of re-running the debounce
        true
    }
}

// The prime phase - the capture analogue of the sink's zero priming: trigger
// the device, discard any backlog so the fill level starts out known, and
// hand the graph one period of silence while the ring fills. Don't wait for
// real data: an empty first cycle reads as a missed deadline to the graph.
// Re-apply the fragment layout while the channel is in setup (legal after a
// trigger suspend too, so live platform::FRAGMENT changes reach a suspended
// source). Returns the cycle's byte count (the period of silence), or
// EMPTY_CYCLE before a format is negotiated (unreachable past the caller's
// gate).
pub(super) fn prime_capture(
    port: &mut Port<SourceDir>,
    period_in_bytes: u32,
    graph_rate: u32,
    fragment_bytes: u32,
    data: &mut [u8],
    log: &Log,
) -> isize {
    let maxsize = data.len() as u32;
    let Some((stride, rate)) = port.stride_rate() else {
        return EMPTY_CYCLE;
    };
    // The capture fragment is capped at the period: queued readings move in
    // fragment steps, and a fragment far above the period makes the servo
    // target unreachable - the error pegs at the clamp and the integrator
    // ramps. The ring scales with the period so large quanta keep some
    // overrun slack.
    if !port.dsp.is_running() {
        let m = period_in_bytes.max(1024);
        let cap = 1u32 << (31 - m.leading_zeros());
        let frag = if fragment_bytes == 0 {
            1024
        } else {
            fragment_bytes.min(cap)
        };
        backend::configure_capture_buffer(
            &port.dsp,
            frag,
            ring_request(
                period_in_bytes,
                backend::max_buffer_period_bytes(stride, rate, graph_rate),
                stride,
                rate,
            ),
        );
    }
    let ready = port.dsp.ready_for_reading(0);
    // one GETISPACE serves the backlog, the granted fragment and the ring
    // total: they come from the same struct, and the layout fields are
    // final now that ready_for_reading has triggered the channel. The
    // ACTUAL grant, not the request - the kernel clamps the ring silently,
    // and the fill geometry must fit reality.
    let layout = port.dsp.buffer_layout();
    let (backlog, fragsize, ring) = (
        layout.queued_bytes,
        layout.quantum_bytes,
        layout.capacity_bytes,
    );
    if ready {
        let mut backlog = backlog;
        while backlog > 0 {
            // whole frames only: GETISPACE is byte-granular and can sit
            // mid-frame; an unaligned read would tear every later sample
            let chunk = (backlog.min(maxsize) / stride) * stride;
            if chunk == 0 {
                break; // a sub-frame tail; it completes into a frame later
            }
            let outcome = port.dsp.read(&mut data[..chunk as usize]);
            if outcome.status.device_lost() {
                port.device_eof = true;
            }
            if outcome.bytes == 0 {
                break;
            }
            backlog -= outcome.bytes as u32;
        }
    }
    port.ext.primed = true;
    port.ext.ring_size = ring;
    // the measurement/arrival quantum is the granted fragment or the
    // hardware cadence sndstat reports, whichever is coarser (see the
    // sink); data arrives in these lumps regardless of the soft fragment
    let chunk = ns_to_frame_bytes(port.dsp.delivery_quantum_ns(), rate, stride);
    commit_geometry(port, period_in_bytes, fragsize.max(chunk));
    port.ext.pinned_cycles = 0;
    if port.ext.ring_size > 0
        && port.ext.ring_size < ring_required(period_in_bytes, port.setup_blocksize)
    {
        crate::warn!(
            log,
            "granted OSS capture ring ({}) is smaller than the fill geometry needs ({}); \
      audio will glitch. Lower the PipeWire quantum; we set the fragment size \
      explicitly, so hw.snd.latency has no effect",
            port.ext.ring_size,
            ring_required(period_in_bytes, port.setup_blocksize)
        );
    }

    let len = period_in_bytes.min(maxsize);
    data[..len as usize].fill(silence_byte(port));
    len as isize
}
