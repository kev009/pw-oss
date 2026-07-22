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

pub(super) fn retune_seed(target_goal: u32, fill: u32, write_now: u32, period: u32) -> u32 {
    target_goal.min(predicted_next_fill(fill, write_now, period))
}

pub(super) fn desired_delay(period: u32, playback_delay_eighths: u32) -> u32 {
    (period / 8).saturating_mul(playback_delay_eighths)
}

// the resampler's per-cycle output can exceed a quantum; its size bounds the
// largest single write and so the headroom the fill ceiling must reserve
pub(super) fn rate_match_bytes(
    rate_match: &crate::spa::IoArea<spa_io_rate_match>,
    stride: u32,
) -> u32 {
    rate_match
        .with_ref(|rm| rm.size.saturating_mul(stride))
        .unwrap_or(0)
}

// the fill target's floor: one period plus a jitter margin (a quarter period,
// or one device fragment when the fragment dwarfs the quantum), so a small
// oss.delay or a tiny quantum can't starve the wakeup fill. buffer_required()
// and target_delay() must derive this identically: the in-place retune gate
// (buffer_size >= required) guarantees the fill ceiling clears this floor
// only while the two agree.
pub(super) fn fill_floor(period: u32, blocksize: u32) -> u32 {
    period.saturating_add((period / 4).max(blocksize))
}

pub(super) fn buffer_required(period: u32, desired: u32, blocksize: u32, write_max: u32) -> u32 {
    period.saturating_mul(2).saturating_add(desired).max(
        fill_floor(period, blocksize)
            .saturating_add(write_max)
            .saturating_add(blocksize),
    )
}

// The prime-time ring request: what the current period needs, floored at
// what the LARGEST negotiable quantum needs (max_period comes from
// node::max_ring_period_bytes - the shared policy behind this floor, the
// capture ring request and the advertised node.max-latency), never below
// MIN_RING_BYTES, never above the kernel cap (which always wins). Capacity
// is not latency: the fill target below still controls queued audio, while
// a ring sized for every negotiable quantum lets period changes retune in
// place instead of resizing the device.
pub(super) fn buffer_request(
    period: u32,
    max_period: u32,
    cap: u32,
    fragment: u32,
    chunk: u32,
    write_max: u32,
    playback_delay_eighths: u32,
) -> u32 {
    let frag_est = if fragment == 0 { 1024 } else { fragment };
    let transfer = frag_est.max(chunk);
    let stable = buffer_required(
        max_period,
        desired_delay(max_period, playback_delay_eighths),
        transfer,
        max_period,
    );
    buffer_required(
        period,
        desired_delay(period, playback_delay_eighths),
        transfer,
        write_max,
    )
    .max(stable)
    .max(crate::oss::MIN_RING_BYTES)
    .min(cap)
}

pub(super) fn target_delay(
    granted: u32,
    period: u32,
    blocksize: u32,
    write_max: u32,
    desired: u32,
) -> (u32, bool) {
    if granted >= period.saturating_mul(2) {
        // Calibrated: period/8 per oss.delay step, floored per fill_floor().
        // The ceiling always leaves room above target for the largest expected
        // write (write_max: a quantum, or the resampler's size if larger) plus
        // one fragment of servo wander: the OSS write is non-blocking, so a
        // write that doesn't fit must retain and retry its tail. A driver that
        // grants many small fragments in a large buffer (uaudio) must
        // not be fill-targeted near-full - that both adds 100+ ms of
        // latency and leaves one fragment of headroom, dropping a chunk on
        // every normal servo excursion. (uaudio drains buffer_ms-sized
        // transfers, folded into blocksize above.) On a genuinely small grant
        // (snd_hdspe forces both the fragment and the total) the ceiling
        // lands just under near-full, which is the best a two-quanta
        // buffer can do.
        let floor = fill_floor(period, blocksize);
        let ceil = granted
            .saturating_sub(write_max.saturating_add(blocksize))
            .max(period);
        let want = desired.max(floor);
        (want.min(ceil).max(period), want > ceil)
    } else {
        (granted / 2, false) // buffer too small for two quanta; best-effort, will drop (prime_playback warns)
    }
}

// the shared geometry-commit tail of the prime and in-place retune paths:
// apply the fill target for a `granted`-byte ring at `period` and relock the
// servo. Returns whether the oss.delay target was capped by the ring.
pub(super) fn commit_geometry(
    port: &mut crate::node::Port<SinkDir>,
    granted: u32,
    period: u32,
    blocksize: u32,
    write_max: u32,
    desired: u32,
) -> bool {
    // defense in depth (the prime path gates on the period too): a committed
    // setup_period of 0 would flip the channel to Running with degenerate
    // geometry, and retune_period's setup_period == 0 early-exit would never
    // re-commit - stuck until a full rebuild
    if period == 0 {
        return false;
    }
    let (target, delay_capped) = target_delay(granted, period, blocksize, write_max, desired);
    port.setup_period = period;
    port.setup_blocksize = blocksize; // the effective quantum, incl. hw chunk
    port.ext.target_delay = target;
    port.ext.target_goal = target;
    port.dll.init();
    port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle
    let (stride, rate) = port.stride_rate().unwrap_or((1, 0));
    port.bw_adapt
        .configure(stride, blocksize, period, rate.saturating_mul(stride));
    delay_capped
}

pub(super) fn log_delay_capped(log: &crate::spa::Log, path: &str, granted: u32) {
    crate::info!(
        log,
        "{}: the oss.delay target is capped by the granted buffer ({})",
        path,
        granted
    );
}

// The retune phase. A quantum or graph-rate change needs new servo geometry.
// If the current OSS ring is already large enough, retune that geometry in
// place: the triggered channel can't accept SETFRAGMENT, but it does not need
// to when the existing grant still has the headroom the new period requires.
// A grant too small re-primes in place via a trigger suspend (SETFRAGMENT
// becomes legal again). The outcome tells process() whether this cycle
// committed new geometry, so followers do not immediately overwrite its
// predicted live target with a pre-write fill sample.
pub(super) fn retune_period(
    port: &mut crate::node::Port<SinkDir>,
    period_in_bytes: u32,
    stride: u32,
    write_now: u32,
    playback_delay_eighths: u32,
    now: u64,
    log: &crate::spa::Log,
) -> RetuneOutcome {
    if !port.dsp.is_running()
        || port.setup_period == 0
        || period_in_bytes == 0
        || period_in_bytes == port.setup_period
    {
        port.ext.period_mismatch = 0;
        return RetuneOutcome::Unchanged;
    }
    // debounce BOTH paths: a single-cycle flip usually means a renegotiation is
    // in flight (which re-primes anyway); a rebuild on it costs an audible gap,
    // and even the in-place retune relocks the servo. Keep the old geometry for
    // one cycle instead.
    port.ext.period_mismatch += 1;
    if port.ext.period_mismatch < 2 {
        return RetuneOutcome::Unchanged;
    }
    // cached blocksize: the triggered channel refuses SETFRAGMENT, so the
    // granted fragment (and the session-fixed hw cadence folded in at
    // prime) cannot have changed; reusing it avoids an ioctl here
    let blocksize = port.setup_blocksize;
    let desired = desired_delay(period_in_bytes, playback_delay_eighths);
    let write_max = period_in_bytes.max(rate_match_bytes(&port.rate_match, stride));
    if port.ext.buffer_size >= buffer_required(period_in_bytes, desired, blocksize, write_max) {
        let old_period = port.setup_period;
        // Measure the real queued audio before committing the new geometry.
        // The live target below predicts what remains at the next wake after
        // this cycle's write; advancing immediately to the larger goal with
        // silence creates an audible hole in a continuous stream.
        let odelay = measured_fill(port);
        let delay_capped = commit_geometry(
            port,
            port.ext.buffer_size,
            period_in_bytes,
            blocksize,
            write_max,
            desired,
        );
        let target_goal = port.ext.target_goal;
        // `write_now` normally equals the new period and therefore maintains
        // the pre-write fill at the next wake. A short graph buffer can reduce
        // it, so do not seed the live target above that predicted next fill.
        port.ext.target_delay = retune_seed(target_goal, odelay, write_now, period_in_bytes);
        port.ext.period_mismatch = 0;
        // commit_geometry cold-started the servo. It will now move the live
        // target from this seed to target_goal as actual audio accumulates.

        crate::info!(
            log,
            "{}: period {} -> {} bytes; retuned in place (granted {}, target delay {} -> {})",
            port.dsp.path,
            old_period,
            period_in_bytes,
            port.ext.buffer_size,
            port.ext.target_delay,
            target_goal
        );
        if delay_capped {
            log_delay_capped(log, &port.dsp.path, port.ext.buffer_size);
        }
        RetuneOutcome::Retuned
    } else if port.dsp.suspend() {
        // Too small for the new period: stop the channel in place.
        // SETTRIGGER(0) discards the queued audio exactly like the
        // rebuild's HALT and clears TRIGGERED, so the prime phase
        // re-runs SETFRAGMENT at the new layout IN THIS CYCLE and this
        // cycle's real write re-arms - one prime-sized gap instead of the
        // multi-cycle main-thread close/reopen, and no main-loop
        // dependency (the source resizes the same way).
        crate::info!(
            log,
            "{}: period {} -> {} bytes exceeds the ring ({}); re-priming",
            port.dsp.path,
            port.setup_period,
            period_in_bytes,
            port.ext.buffer_size
        );
        port.ext.period_mismatch = 0;
        port.ext.xrun_timestamp = 0; // a stale recovery hold must not defer the re-arm
        port.was_matching = false;
        // SETTRIGGER starts a new kernel xrun epoch; SETFRAGMENT also resets
        // the low-water mark during the prime that follows.
        crate::node::reset_device_event(port);
        RetuneOutcome::Retuned
    } else {
        // period_mismatch stays >= 2 on purpose: if the caller can't queue the
        // rebuild (no main loop), the next cycle retries this retune
        // immediately instead of re-running the debounce - so this arm can run
        // every cycle; rate-limit the log (on its own limiter - see
        // SinkPortExt::retune_limit)
        if let Some(suppressed) = port.ext.retune_limit.check(now) {
            crate::info!(
                log,
                "{}: period {} -> {} bytes; reconfiguring (+{} messages suppressed)",
                port.dsp.path,
                port.setup_period,
                period_in_bytes,
                suppressed
            );
        }
        RetuneOutcome::Rebuild
    }
}

// debug-build diagnostics: the scheduling class/priority the data loop
// actually runs at (RT setup problems show up here first)
#[cfg(debug_assertions)]
pub(super) fn debug_log_priorities(log: &crate::spa::Log) {
    fn prio_type(type_: std::ffi::c_ushort) -> &'static str {
        match type_ {
            libc::RTP_PRIO_REALTIME => "realtime",
            libc::RTP_PRIO_NORMAL => "normal",
            libc::RTP_PRIO_IDLE => "idle",
            _ => unreachable!(),
        }
    }

    fn gettid() -> i32 {
        let mut tid = 0;
        if unsafe { libc::thr_self(&mut tid) } != -1 {
            assert!(tid <= i32::MAX as i64);
            tid as i32
        } else {
            0
        }
    }

    let mut rtp = libc::rtprio { type_: 0, prio: 0 };

    let pid = unsafe { libc::getpid() };
    if unsafe { libc::rtprio(libc::RTP_LOOKUP, pid, &mut rtp) } != -1 {
        crate::warn!(
            log,
            "process priority ({:5}): type = {}, prio = {}",
            pid,
            prio_type(rtp.type_),
            rtp.prio
        );
    }

    let tid = gettid();
    if unsafe { libc::rtprio_thread(libc::RTP_LOOKUP, tid, &mut rtp) } != -1 {
        crate::warn!(
            log,
            "thread priority ({:6}): type = {}, prio = {}",
            tid,
            prio_type(rtp.type_),
            rtp.prio
        );
    }
}

// The prime phase: the channel is in setup (first cycle, or a trigger
// suspend from the retune/resize path), so the ring layout can be applied.
// Size the ring, commit the fill geometry and pre-fill to target; the
// cycle's real write then arms the channel.
pub(super) fn prime_playback(
    port: &mut crate::node::Port<SinkDir>,
    period_in_bytes: u32,
    graph_rate: u32,
    playback_delay_eighths: u32,
    fragment_bytes: u32,
    log: &crate::spa::Log,
) {
    #[cfg(debug_assertions)]
    debug_log_priorities(log);

    let Some((stride, cfg_rate)) = port.stride_rate() else {
        return;
    };
    if period_in_bytes == 0 {
        return; // see commit_geometry: zero-period geometry is never committed
    }

    // Size the fill to the granted buffer and the device's real fragment.
    // fragment_bytes (0 = automatic 1 KiB) only mutates on this loop, so the
    // read is race-free; no ioctls beyond what the prime always issued
    // The measurement/drain quantum is the granted fragment - unless the
    // device's hardware cadence is coarser (drivers that ignore
    // SETFRAGMENT and pull fixed transfers; vchan parents), which the
    // soft fragsize can't see and sndstat can. Floor, headroom and the
    // servo noise model key on the larger - and the buffer REQUEST must
    // include it, or a device that honors the request grants no room for
    // the ceiling above the floor.
    let desired = desired_delay(period_in_bytes, playback_delay_eighths);
    let chunk = crate::node::ns_to_frame_bytes(port.dsp.hw_quantum_ns, cfg_rate, stride);
    let write_max = period_in_bytes.max(rate_match_bytes(&port.rate_match, stride));
    let max_period = crate::node::max_ring_period_bytes(stride, cfg_rate, graph_rate);
    let request = buffer_request(
        period_in_bytes,
        max_period,
        crate::oss::ring_byte_cap(stride, cfg_rate),
        fragment_bytes,
        chunk,
        write_max,
        playback_delay_eighths,
    );
    let granted = port.dsp.set_buffer_size(request, fragment_bytes);
    let blocksize = port.dsp.blocksize().max(chunk);

    // saturating arithmetic: blocksize/rate_match.size are device-provided and
    // an overflow here would abort the data loop.
    let delay_capped = commit_geometry(
        port,
        granted,
        period_in_bytes,
        blocksize,
        write_max,
        desired,
    );
    port.ext.buffer_size = granted;

    crate::warn!(
        log,
        "{}: granted {}, blocksize {}, period {}, target delay {}",
        port.dsp.path,
        granted,
        blocksize,
        period_in_bytes,
        port.ext.target_delay
    );
    if delay_capped {
        log_delay_capped(log, &port.dsp.path, granted);
    }
    if granted < period_in_bytes.saturating_mul(2) {
        crate::warn!(
            log,
            "{}: granted OSS buffer ({}) is smaller than two quanta ({}); \
      audio will glitch. Lower the PipeWire quantum; we set the fragment size \
      explicitly, so hw.snd.latency has no effect",
            port.dsp.path,
            granted,
            period_in_bytes.saturating_mul(2)
        );
    }

    port.dsp.write_silence(port.ext.target_delay);
}
