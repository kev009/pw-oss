use std::os::raw::c_int;

use libspa::sys::*;

use crate::node::{
    DataControl, DataState, Direction, MAX_PORTS, MainState, ParamBuild, PortConfig,
};

mod buffer;

use buffer::*;

// PortInfo and clock-delay state currently support one playback port.
const _: () = assert!(MAX_PORTS == 1);

pub(crate) enum SinkDir {}

// main-loop property model
pub(crate) struct SinkMainExt {
    pub oss_delay: u32,
    pub oss_delay_default: u32,
}

impl Default for SinkMainExt {
    fn default() -> Self {
        Self {
            oss_delay: 10,
            oss_delay_default: 10,
        }
    }
}

// direction-specific data-loop fields (DataState.ext)
pub(crate) struct SinkDataExt {
    pub cur_timestamp: u64, // method invocation timestamp for `process`
    pub old_timestamp: u64,
    pub oss_delay: u32, // additional delay in 1/8ths of period
}

impl Default for SinkDataExt {
    fn default() -> Self {
        Self {
            cur_timestamp: 0,
            old_timestamp: 0,
            oss_delay: 10,
        }
    }
}

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SinkPortExt {
    pub xrun_timestamp: u64, // the moment we noticed an underrun (which is a bit later than the start of it)
    pub target_delay: u32,   // live servo target in bytes
    pub target_goal: u32, // geometry target retained for rate-controlled settling and future primes
    pub buffer_size: u32, // granted OSS playback ring capacity in bytes
    pub period_mismatch: u32, // consecutive cycles at a different period (debounce)
    pub resuming: bool,   // first real buffer after Pause must bypass generic xrun hold
    pub rebuild_after_start: bool, // Pause could neither preserve nor reset this device
    // OSS may accept only a prefix from its nonblocking fd. Keep the host
    // buffer until every byte is accepted, and resume at this byte offset.
    pub pending_buffer: Option<u32>,
    pub pending_offset: u32,
    // the rebuild-pending arm of retune_period can run every cycle; its own
    // limiter, because sharing port.warn_limit would let a persistent refusal
    // consume the dropped-bytes/underrun warnings' emission slots and fold
    // unrelated events into their suppressed counts
    pub retune_limit: crate::node::RateLimit,
}

fn measured_fill(port: &crate::node::Port<SinkDir>) -> u32 {
    crate::node::device_event_fill(port).unwrap_or_else(|| port.dsp.odelay())
}

fn measured_underruns(port: &mut crate::node::Port<SinkDir>) -> u32 {
    if let Some(count) = crate::node::take_device_event_xruns(port) {
        count
    } else {
        let total = port.dsp.underruns();
        crate::node::take_fallback_xruns(port, total)
    }
}

// The xrun-detection phase, on a running channel. The vchan mixer counts a
// momentarily-short child as an xrun and pads it with silence
// (feeder_mixer.c); with the fill still healthy that's accounting noise, not
// a dropout - only a genuinely low fill at wakeup is a real underrun worth
// recovery and reporting. "Low" is a period, capped by the healthy sawtooth
// floor (target minus one fragment): with a fragment wider than the period
// the fill routinely dips under one fragment while perfectly locked, and
// gating on the fragment size would fire recovery on every accounting tick
// there. Arms the recovery hold (xrun_timestamp) and reports the EVENT to
// the host once, not per held cycle.
// `underrun_count` is the counter the caller read this cycle (nonzero, or
// this isn't called); measured outside so tests can drive the gate.
// the "genuinely low at wakeup" threshold: a period, capped by the healthy
// sawtooth floor (target minus one fragment). A late cycle finds a
// legitimately lower fill (the device kept draining, `drained` bytes over
// the lateness), so the threshold tracks the expected healthy fill at THIS
// moment; the floor keeps a true empty ring (a real underrun reads 0 until
// we write) detectable at any lateness.
fn underrun_low(target_delay: u32, blocksize: u32, period_in_bytes: u32, drained: u32) -> u32 {
    let low = period_in_bytes
        .min(target_delay.saturating_sub(blocksize))
        .max(period_in_bytes / 4);
    let wander = (period_in_bytes / 4).max(blocksize);
    low.min(target_delay.saturating_sub(drained).saturating_sub(wander))
        .max(period_in_bytes / 16)
}

fn detect_underrun(
    port: &mut crate::node::Port<SinkDir>,
    period_in_bytes: u32,
    underrun_count: u32,
    cur_timestamp: u64,
    clock_nsec: u64,
    log: &crate::spa::Log,
) {
    let Some((stride, cfg_rate)) = port.stride_rate() else {
        return;
    };
    // cached blocksize: the channel can't be retuned while triggered, and
    // the gate must not cost ioctls on healthy cycles
    let elapsed = cur_timestamp.saturating_sub(clock_nsec);
    let drained = crate::node::ns_to_bytes(elapsed, cfg_rate, stride);
    let low = underrun_low(
        port.ext.target_delay,
        port.setup_blocksize,
        period_in_bytes,
        drained,
    );
    let odelay_now = measured_fill(port);
    if odelay_now < low {
        if let Some(suppressed) = port.warn_limit.check(cur_timestamp) {
            crate::warn!(
                log,
                "{}: OSS reported {:3} underruns @ {} (+{} warnings suppressed)",
                port.dsp.path,
                underrun_count,
                cur_timestamp,
                suppressed
            );
        }
        if port.ext.xrun_timestamp == 0 {
            // snapshot the DRIVER clock, not wall time: the recovery
            // condition compares against driver_clock.nsec (idealized cycle
            // start, which lags wall time by any lateness); a wall snapshot
            // deferred recovery by the lateness, discarding a buffer per
            // late cycle
            port.ext.xrun_timestamp = clock_nsec.max(1);

            // once per event, not per held cycle; deposited, not called -
            // process() notifies the host after the State borrows end
            // (collect-then-notify, see node::process)
            port.pending_xrun = Some(cur_timestamp / 1000);
        }
    } else {
        // suppressed counts stay diagnosable: a marginal system that
        // ticks the counter while self-healing shows up at debug level
        crate::debug!(
            log,
            "{}: {} underrun counts ignored (fill {} >= {})",
            port.dsp.path,
            underrun_count,
            odelay_now,
            low
        );
    }
}

// The recovery phase, entered while an underrun hold is pending
// (xrun_timestamp != 0). Recover on the first data cycle past the event
// (ALSA does the same: snap the fill, resume immediately): relock the servo,
// re-prime the fill to target and write this cycle's data in the SAME cycle.
// Waiting for a particular process cadence discards real buffers per failed
// attempt, and a follower under a corr-steered driver may never hit a fixed
// window at all. Until the recovery cycle arrives the buffer is consumed
// unwritten (the skip-buffer hold). Returns the cycle's write result
// (`size` when held).
fn recover_or_hold(
    port: &mut crate::node::Port<SinkDir>,
    clock_nsec: u64,
    clock_flags: u32,
    data: &[u8],
) -> crate::oss::PlaybackWrite {
    let size = data.len() as u32;
    if clock_nsec > port.ext.xrun_timestamp && clock_flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER == 0 {
        port.ext.xrun_timestamp = 0;

        port.dll.init();
        port.bw_adapt.reset();
        // A real xrun has already broken continuity. Ensure the live target
        // reaches the geometry's safe floor, but do not raise a soft-settling
        // target toward its optional goal. A steady target already equals the
        // goal and is therefore fully restored.
        let safe = fill_floor(port.setup_period, port.setup_blocksize);
        port.ext.target_delay = port
            .ext
            .target_delay
            .max(safe.min(port.ext.target_goal))
            .min(port.ext.target_goal);

        // buffer's already sized; re-prime only up to target, accounting for what's
        // still queued (a full target_delay would push odelay past the buffer)
        let odelay = measured_fill(port);
        let refill = port.ext.target_delay.saturating_sub(odelay);

        #[cfg(debug_assertions)]
        eprintln!(
            "{}: re-priming with {} bytes of silence (odelay {})",
            port.dsp.path, refill, odelay
        );

        port.dsp.write_silence(refill);
        // write the slice, not the period: only these bytes at the offset are owned
        port.dsp.write(data)
    } else {
        #[cfg(debug_assertions)]
        eprintln!("{}: skipping buffer @ {}", port.dsp.path, clock_nsec);

        crate::oss::PlaybackWrite::consumed(size as isize)
    }
}

// The first real buffer after Pause is not an unexpected xrun. SKIP has already
// restored the real audio that SILENCE moved into FreeBSD's shadow buffer; seed
// the live target from that restored queue and write the current graph buffer
// immediately instead of consuming it with the generic one-cycle xrun hold.
// An empty queue plus a real driver underrun is the fallback for a Pause that
// found no soft-buffer audio to shadow.
fn resume_playback(
    port: &mut crate::node::Port<SinkDir>,
    queued: u32,
    paused_underruns: u32,
    data: &[u8],
    log: &crate::spa::Log,
) -> crate::oss::PlaybackWrite {
    // GETODELAY excludes the hardware buffer, so zero soft fill by itself does
    // not prove playback stopped. Require the driver's underrun count too;
    // otherwise appending real data is the only gap-free choice.
    let drained = queued == 0 && paused_underruns > 0;
    if drained {
        // A deliberate pause that lost its saved queue is a cold restart:
        // restore the full configured goal before appending real audio.
        port.ext.target_delay = port.ext.target_goal;
        port.dll.init();
        port.bw_adapt.reset();
        crate::info!(
            log,
            "{}: playback queue drained while paused; re-priming",
            port.dsp.path
        );
        port.dsp.write_silence(port.ext.target_delay);
    }
    let result = port.dsp.write(data);
    if !drained && result.bytes > 0 {
        // Preserve whatever real audio survived the pause as the live target.
        // Seed from bytes OSS actually accepted: a blocked or short write must
        // not claim that its unaccepted suffix is already queued.
        port.ext.target_delay = port.ext.target_goal.min(predicted_next_fill(
            queued,
            result.bytes as u32,
            port.setup_period,
        ));
    }
    if !result.would_block() {
        port.ext.resuming = false;
    }
    result
}

// Move a live-retuned fill target toward its geometry goal without splicing
// synthetic samples into the queued stream. The target stays at most a quarter
// period ahead of measured fill, inside the normal DLL band; as real audio
// accumulates under rate steering, the target follows it to the goal.
fn settle_target(port: &mut crate::node::Port<SinkDir>, fill: u32, stride: u32) {
    if port.ext.target_delay >= port.ext.target_goal {
        return;
    }
    let lead = (port.setup_period / 4).max(stride);
    let capped_lead = fill.saturating_add(lead).min(port.ext.target_goal);
    port.ext.target_delay = port.ext.target_delay.max(capped_lead);
}

// The follower-servo phase, matching a foreign clock: the DLL serves rate
// matching only (when driving, the servo runs at the device wake where the clock
// is published, and a same-device follower has nothing to correct - updating
// anyway would wind the integrator; ALSA gates the same way). `odelay` is
// the fill the caller measured this cycle. Returns the rate correction and
// whether this cycle's buffer must be skipped (overfill drain).
fn follower_servo(
    port: &mut crate::node::Port<SinkDir>,
    odelay: u32,
    stride: u32,
    nsec: u64,
) -> (f64, bool) {
    let mut corr: f64 = 1.0;
    let mut skip_write = false;
    settle_target(port, odelay, stride);
    if !port.was_matching {
        // matching just engaged; relock rather than apply stale state
        port.dll.init();
        port.bw_adapt.reset();
    }
    let err_raw = odelay as f64 - port.ext.target_delay as f64;
    if err_raw.abs() > port.setup_period as f64 {
        // Fill snap (ALSA's max_resync): a level error past one period is
        // beyond what the +/-1% actuator removes promptly and would wind the
        // integrator against the clamp. Correct the level directly -
        // refill on underfill, drain a cycle on overfill - and relock.
        port.dll.init();
        port.bw_adapt.reset();
        if err_raw < 0.0 {
            port.dsp
                .write_silence(port.ext.target_delay.saturating_sub(odelay));
        } else {
            skip_write = true;
        }
    } else {
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = err_raw.clamp(-max_err, max_err);
        corr = port.dll.update(err);
        port.bw_adapt.update_fill(&mut port.dll, err, nsec);
    }

    #[cfg(debug_assertions)]
    eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, err_raw);

    (corr, skip_write)
}

// Same-device follower: there is no rate actuator that can accumulate real
// audio toward a larger transition goal. Keep the seeded live target instead
// of inserting silence; correct only later drift around that reachable level.
// A genuine underrun is the only path that raises it to the geometry floor.
// Returns whether this cycle's buffer must be skipped (overfill drain).
fn level_correct(port: &mut crate::node::Port<SinkDir>, odelay: u32) -> bool {
    let err_raw = odelay as f64 - port.ext.target_delay as f64;
    if err_raw < -(port.setup_period as f64) {
        port.dsp
            .write_silence(port.ext.target_delay.saturating_sub(odelay));
    } else if err_raw > port.setup_period as f64 {
        return true;
    }
    false
}

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(
    dsp: &mut crate::oss::DspWriter,
    config: &PortConfig,
    log: &crate::spa::Log,
) -> c_int {
    let Ok(channel_order) = config.oss_channel_order() else {
        crate::warn!(
            log,
            "{}: unsupported channel map: {:?}",
            dsp.path,
            config.positions
        );
        return -libc::EINVAL;
    };
    // a busy or vanished device must fail negotiation, not abort
    if let Err(err) = dsp.open() {
        crate::warn!(log, "{}: open: {}", dsp.path, err);
        return -(err as c_int);
    }
    // ditto for a device that won't take the format exactly
    if let Err(err) = dsp.configure(
        config.oss_format(),
        config.channels,
        config.rate,
        config.silence_byte(),
        channel_order,
    ) {
        crate::warn!(log, "{}: device rejected {:?}: {}", dsp.path, config, err);
        dsp.close();
        return -(err as c_int);
    }
    // on direct opens the hardware blocksize is per-session state; re-read it
    // now that THIS configuration is in effect (vchan/uaudio values are stable)
    dsp.hw_quantum_ns = crate::oss::drain_quantum_ns(&dsp.path, true);
    0
}

// the shared not-ready/consumed exit of the sink cycle: publish NEED_DATA on
// the port io AND fold it into the returned status - the host prefetches the
// next buffer only on the return bit
fn need_data(io: &mut crate::spa::IoArea<spa_io_buffers>, result: &mut c_int) {
    io.with(|io| io.status = SPA_STATUS_NEED_DATA as i32);
    *result |= SPA_STATUS_NEED_DATA as i32;
}

fn clear_pending_write(ext: &mut SinkPortExt) {
    ext.pending_buffer = None;
    ext.pending_offset = 0;
}

fn end_input_sequence(port: &mut crate::node::Port<SinkDir>) {
    // This buffer will no longer supply a retained suffix. Close any frame
    // that its accepted prefix left open before a different buffer arrives.
    if port.dsp.end_buffer_sequence() {
        crate::node::reset_device_event(port);
    }
    clear_pending_write(&mut port.ext);
}

fn release_input(port: &mut crate::node::Port<SinkDir>, result: &mut c_int) {
    end_input_sequence(port);
    need_data(&mut port.io, result);
}

fn consume_freewheel_input(port: &mut crate::node::Port<SinkDir>) {
    end_input_sequence(port);
    port.io.with(|io| io.status = SPA_STATUS_NEED_DATA as i32);
}

// Return the first byte OSS has not accepted from this host buffer. A new
// buffer identity, or a chunk shorter than its previously accepted prefix,
// invalidates the old offset and starts from the new slice's beginning.
fn pending_write_offset(ext: &mut SinkPortExt, buffer_id: u32, size: usize) -> usize {
    if ext.pending_buffer != Some(buffer_id) || ext.pending_offset as usize > size {
        ext.pending_buffer = Some(buffer_id);
        ext.pending_offset = 0;
    }
    ext.pending_offset as usize
}

fn prepare_pending_write(
    port: &mut crate::node::Port<SinkDir>,
    buffer_id: u32,
    size: usize,
) -> usize {
    let changed = port.ext.pending_offset != 0
        && (port.ext.pending_buffer != Some(buffer_id) || port.ext.pending_offset as usize > size);
    if changed {
        // HAVE_DATA normally preserves both identity and chunk size. If a
        // host breaks that contract, close the accepted prefix before the
        // replacement bytes can complete its open PCM frame.
        end_input_sequence(port);
    }
    pending_write_offset(&mut port.ext, buffer_id, size)
}

// A positive short write is normal for FreeBSD's nonblocking chn_write: it
// returns the bytes copied before the soft ring filled, without an errno.
// Retain the untouched suffix; EAGAIN can accompany a prefix when a split
// frame's bounded completion retry runs out of room.
fn retain_partial_write(
    ext: &mut SinkPortExt,
    requested: u32,
    write: crate::oss::PlaybackWrite,
) -> bool {
    if write.bytes > 0
        && write.bytes < requested as isize
        && (write.error.is_none() || write.error == Some(nix::errno::Errno::EAGAIN))
    {
        ext.pending_offset = ext.pending_offset.saturating_add(write.bytes as u32);
        true
    } else {
        false
    }
}

// Once OSS has accepted a prefix, the suffix is part of the same PCM byte
// sequence. Finish it before any fill correction can skip it or insert
// synthetic audio between the two pieces.
fn write_retained_tail(
    port: &mut crate::node::Port<SinkDir>,
    data: &[u8],
) -> Option<crate::oss::PlaybackWrite> {
    (port.ext.pending_offset != 0).then(|| port.dsp.write(data))
}

fn process_ports(state: &mut DataState<SinkDir>) -> c_int {
    state.ext.old_timestamp = state.ext.cur_timestamp;
    // on a failed clock read reuse the previous stamp (rate limits and the
    // underrun gate degrade for a cycle) rather than abort the data loop
    state.ext.cur_timestamp =
        crate::node::try_now_ns(&state.data_system).unwrap_or(state.ext.old_timestamp);

    // Freewheeling: the graph runs faster than realtime, so consume the input
    // without touching the device. The io NEED_DATA + return HAVE_DATA pair
    // looks odd for a sink but matches alsa-pcm-sink.c:788-791; it is what
    // keeps the freewheel pump running.
    // position is non-null on the process path (checked by on_wake/process)
    if state.position.with_ref(|p| p.clock.flags).unwrap_or(0) & SPA_IO_CLOCK_FLAG_FREEWHEEL != 0 {
        for port in &mut state.ports {
            consume_freewheel_input(port);
        }
        return SPA_STATUS_HAVE_DATA as i32;
    }

    let mut result = SPA_STATUS_OK as i32;

    // indexed (not iter_mut) so the rebuild arms below can end the &mut port
    // borrow, borrow the whole State, and re-borrow the port
    for port_idx in 0..state.ports.len() {
        // Consume any completed background rebuild before the cycle reads the
        // port (it may swap in a fresh device or clear the config); a rebuild
        // still in flight skips the cycle.
        if crate::node::poll_rebuild(state, port_idx) {
            let port = &mut state.ports[port_idx];
            release_input(port, &mut result);
            continue;
        }
        let port = &mut state.ports[port_idx];
        let Some((stride, cfg_rate)) = port.stride_rate() else {
            continue; // no format negotiated yet
        };

        if port.buffers.is_empty() || port.io.is_null() {
            continue; // not (fully) negotiated yet
        }

        if port.dsp.is_closed() {
            // Suspend closed the device but the host restarted without a fresh
            // format; rebuild off-loop instead of tripping the dsp state
            // asserts (the &mut port borrow ends here: queue_rebuild snapshots
            // an owned request and owns the pending claim)
            crate::node::queue_rebuild(state, port_idx);
            let port = &mut state.ports[port_idx];
            release_input(port, &mut result);
            continue;
        }

        if port.ext.rebuild_after_start {
            // Pause could neither preserve nor reset the old queue. Do not
            // append audio behind unknown contents; replace the device first.
            let queued = crate::node::queue_rebuild(state, port_idx);
            let port = &mut state.ports[port_idx];
            if queued {
                port.ext.rebuild_after_start = false;
            }
            release_input(port, &mut result);
            continue;
        }

        if port.io.with_ref(|io| io.status) != Some(SPA_STATUS_HAVE_DATA as i32) {
            // no input this cycle (e.g. draining after stop); the clock (incl. the
            // draining delay) is published from on_wake now, so just ask for data
            release_input(port, &mut result);
            continue;
        }

        // io is non-null here (the HAVE_DATA gate above read through it)
        let buffer_id = port.io.with_ref(|io| io.buffer_id).unwrap_or(u32::MAX);
        // SAFETY: the host keeps the registered buffer pointers valid until
        // the next port_use_buffers (its contract), and the returned block is
        // used within this cycle only
        let Some(data_0) = (unsafe { crate::node::valid_data_block(port, buffer_id, &state.log) })
        else {
            // return status, not just io, so the host refills
            release_input(port, &mut result);
            continue;
        };

        // The chunk-clamped view remains valid for this cycle. A previous
        // nonblocking write may have accepted only its prefix; retry exactly
        // the untouched suffix while leaving io.status HAVE_DATA.
        let input_data = data_0.input_slice();
        let input_size = input_data.len() as u32;
        let offset = prepare_pending_write(port, buffer_id, input_data.len());
        let mut cycle_data = &input_data[offset..];
        let mut size = cycle_data.len() as u32;
        if size == 0 {
            release_input(port, &mut result);
            continue;
        }

        debug_assert_eq!(data_0.chunk_stride(), stride as i32);

        #[cfg(debug_assertions)]
        if state.position.with_ref(|p| p.clock.flags).unwrap_or(0) & SPA_IO_CLOCK_FLAG_XRUN_RECOVER
            != 0
        {
            crate::warn!(
                state.log,
                "{}: SPA_IO_CLOCK_FLAG_XRUN_RECOVER @ {}",
                port.dsp.path,
                state.ext.cur_timestamp
            );
        }

        #[cfg(debug_assertions)]
        if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
            crate::trace!(
                state.log,
                "chunk size: {}, write offset: {}",
                input_size,
                offset
            );
            // the slice head is in bounds by construction (input_slice)
            unsafe { spa_debug_mem(0, cycle_data.as_ptr().cast(), 16.min(size) as usize) };
        }

        // position is non-null on the process path (checked by process); the
        // else arm is pure defense
        let Some(driver_clock) = state.position.with_ref(|p| p.clock) else {
            release_input(port, &mut result);
            continue;
        };
        let matching = state.following
            && !state
                .position
                .with_ref(|p| crate::node::same_clock(p, &state.clock_name))
                .unwrap_or(false);

        // the resampler can legitimately hand us a few frames over a quantum; warn
        // rather than debug_assert!, which would abort the process (panic across the
        // extern "C" boundary).
        // (u64 math for the same reason: a garbage host duration must not
        // overflow-panic the diagnostic that exists to report it)
        #[cfg(debug_assertions)]
        {
            let quantum_bytes = driver_clock.target_duration.saturating_mul(stride as u64);
            if input_size as u64 > quantum_bytes {
                crate::warn!(
                    state.log,
                    "{}: chunk size {} exceeds one quantum {}",
                    port.dsp.path,
                    input_size,
                    quantum_bytes
                );
            }
        }

        // one graph cycle in device bytes (see node::device_period_bytes)
        let period_in_bytes = crate::node::device_period_bytes(
            driver_clock.target_duration,
            cfg_rate,
            driver_clock.target_rate.denom,
            stride,
        );

        let retune = retune_period(
            port,
            period_in_bytes,
            stride,
            size,
            state.ext.oss_delay,
            state.ext.cur_timestamp,
            &state.log,
        );
        if retune == RetuneOutcome::Rebuild {
            // the driver refused the trigger stop (dying fd): rebuild off-loop
            // (the &mut port borrow ends here: queue_rebuild snapshots an
            // owned request and owns the pending claim)
            let pending = crate::node::queue_rebuild(state, port_idx);
            if pending {
                let port = &mut state.ports[port_idx];
                port.was_matching = false; // the gap invalidates matching history
                release_input(port, &mut result);
                continue;
            }
            // No main loop (unusual host): continue at the stale geometry;
            // normal backpressure and underrun handling remain available.
        }
        let retuned = retune == RetuneOutcome::Retuned;
        // re-borrow: the retune arm above may have borrowed the whole State
        let port = &mut state.ports[port_idx];

        let mut resumed_write = None;
        if !port.dsp.is_running() {
            // A trigger suspend or replacement discarded the accepted prefix
            // along with the OSS queue. Replay this host buffer from byte zero
            // after priming the fresh ring.
            if port.ext.pending_offset != 0 {
                port.ext.pending_offset = 0;
                cycle_data = input_data;
                size = input_size;
            }
            if period_in_bytes == 0 {
                // No usable position yet (the source's prime arm gates on the
                // period the same way): priming now would commit setup_period
                // == 0 and the channel would run with degenerate geometry that
                // retune_period never corrects. Not ready this cycle; ask for
                // data like the other not-ready paths.
                release_input(port, &mut result);
                continue;
            }
            port.ext.resuming = false;
            prime_playback(
                port,
                period_in_bytes,
                driver_clock.target_rate.denom,
                state.ext.oss_delay,
                state.oss_fragment,
                &state.log,
            );
        } else if port.ext.resuming {
            // Consume the event-count delta accrued while deliberately
            // paused. Do not turn it into a host xrun or discard this first
            // resumed buffer.
            let paused_underruns = measured_underruns(port);
            if paused_underruns > 0 {
                crate::debug!(
                    state.log,
                    "{}: {} underruns accrued while paused",
                    port.dsp.path,
                    paused_underruns
                );
            }
            if port.ext.pending_offset == 0 {
                let queued = measured_fill(port);
                resumed_write = Some(resume_playback(
                    port,
                    queued,
                    paused_underruns,
                    cycle_data,
                    &state.log,
                ));
            }
        } else {
            let underruns = measured_underruns(port);
            if underruns > 0 {
                detect_underrun(
                    port,
                    period_in_bytes,
                    underruns,
                    state.ext.cur_timestamp,
                    driver_clock.nsec,
                    &state.log,
                );
            }
        }

        let mut corr: f64 = 1.0; // DLL rate correction, published through rate_match below
        let write_result = if let Some(result) = write_retained_tail(port, cycle_data) {
            result
        } else if let Some(result) = resumed_write {
            result
        } else if port.ext.xrun_timestamp != 0 {
            recover_or_hold(port, driver_clock.nsec, driver_clock.flags, cycle_data)
        } else {
            let mut skip_write = false;
            if !retuned && matching && port.setup_period != 0 && port.ext.period_mismatch == 0 {
                (corr, skip_write) =
                    follower_servo(port, measured_fill(port), stride, state.ext.cur_timestamp);
            }

            if !retuned
                && state.following
                && !matching
                && port.setup_period != 0
                && port.ext.period_mismatch == 0
            {
                skip_write = level_correct(port, measured_fill(port));
            }

            if skip_write {
                // consumed; the device drains toward target meanwhile
                crate::oss::PlaybackWrite::consumed(size as isize)
            } else {
                port.dsp.write(cycle_data)
            }
        };
        if port.ext.resuming && port.ext.pending_offset != 0 && !write_result.would_block() {
            port.ext.resuming = false;
        }

        // Rate-match only as a follower on a foreign clock: when driving, the
        // timer steering applies the correction, and a same-device follower ticks
        // from our clock so there is nothing to match (ALSA gates on the clock
        // name the same way).
        port.was_matching = matching;
        port.rate_match.with(|rm| {
            if matching {
                rm.flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                rm.rate = corr.clamp(0.99, 1.01);
            } else {
                rm.flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                rm.rate = 1.0;
            }
        });

        if write_result.would_block() {
            crate::debug!(
                state.log,
                "{}: playback ring full; retaining {}-byte graph buffer at offset {}",
                port.dsp.path,
                input_size,
                port.ext.pending_offset
            );
            // Keep io.status HAVE_DATA. The peer retains this exact input
            // buffer and process() retries it after the device drains.
            continue;
        }

        let nbytes = write_result.bytes;
        if retain_partial_write(&mut port.ext, size, write_result) {
            crate::debug!(
                state.log,
                "{}: playback accepted {} of {} bytes; retaining {}-byte tail",
                port.dsp.path,
                nbytes,
                size,
                size - nbytes as u32
            );
            // Keep io.status HAVE_DATA. The peer retains this exact input
            // buffer, and pending_offset advances the next write to its tail.
            continue;
        }

        if nbytes < size as isize {
            if let Some(suppressed) = port.warn_limit.check(state.ext.cur_timestamp) {
                crate::warn!(
                    state.log,
                    "{}: dropped {} bytes (write returned {}, error {:?}) (+{} warnings suppressed)",
                    port.dsp.path,
                    if nbytes > 0 {
                        size - nbytes as u32
                    } else {
                        size
                    },
                    nbytes,
                    write_result.error,
                    suppressed
                );
            }
        }

        // a sink has no output, so the return bit is NEED_DATA ("can accept input
        // next cycle"), matching the port io status, not HAVE_DATA.
        release_input(port, &mut result);
    }

    result
}

impl Direction for SinkDir {
    const DIRECTION: spa_direction = SPA_DIRECTION_INPUT;
    const PLAYBACK: bool = true;
    const MEDIA_CLASS: &'static str = "Audio/Sink";
    const READY_STATUS: i32 = SPA_STATUS_NEED_DATA as i32;
    const CMD_WARN_PREFIX: &'static str = "";

    type Device = crate::oss::DspWriter;
    type MainExt = SinkMainExt;
    type DataExt = SinkDataExt;
    type PortExt = SinkPortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        std::ptr::NonNull::new(&raw mut OSS_SINK_TOPIC).expect("a static's address is never null")
    }

    fn info_item(ext: &mut SinkMainExt, key: &str, value: &str) {
        if key == crate::keys::OSS_DELAY {
            // per-device default, e.g. from a wireplumber node rule
            if let Ok(v) = value.parse::<u32>() {
                ext.oss_delay = v.min(1024);
            }
        }
    }

    fn ext_ready(ext: &mut SinkMainExt) {
        ext.oss_delay_default = ext.oss_delay;
    }

    fn data_ext(ext: &SinkMainExt) -> SinkDataExt {
        SinkDataExt {
            oss_delay: ext.oss_delay,
            ..Default::default()
        }
    }

    fn build_node_param(state: &mut MainState<SinkDir>, id: u32, index: u32) -> ParamBuild {
        #[allow(non_upper_case_globals)]
        let pod = match (id, index) {
            (SPA_PARAM_PropInfo, 0) => crate::spa::build_latency_offset_prop_info(),
            (SPA_PARAM_PropInfo, 1) => crate::spa::build_params_prop_info(
                crate::keys::OSS_DELAY,
                "OSS buffer fill target (1/8ths of a period)",
                state.ext.oss_delay,
                1024,
            ),
            (SPA_PARAM_PropInfo, 2) => crate::spa::build_params_prop_info(
                crate::keys::OSS_FRAGMENT,
                "OSS fragment size (bytes, power of two, 0 = automatic)",
                state.oss_fragment,
                16384,
            ),
            (SPA_PARAM_Props, 0) => crate::spa::build_latency_offset_props(
                state.process_latency.ns,
                &[
                    (crate::keys::OSS_DELAY, state.ext.oss_delay),
                    (crate::keys::OSS_FRAGMENT, state.oss_fragment),
                ],
            ),
            (SPA_PARAM_ProcessLatency, 0) => {
                crate::spa::build_process_latency_info(&state.process_latency)
            }
            (SPA_PARAM_PropInfo | SPA_PARAM_Props | SPA_PARAM_ProcessLatency, _) => {
                return ParamBuild::Exhausted;
            }
            _ => return ParamBuild::Unknown,
        };
        ParamBuild::Built(pod)
    }

    // a NULL Props pod resets the props to their defaults and re-applies them
    fn reset_props(state: &mut MainState<SinkDir>, data: &DataControl<SinkDir>) -> c_int {
        let delay = state.ext.oss_delay_default;
        let fragment = state.oss_fragment_default;
        let old_delay = state.ext.oss_delay;
        let old_fragment = state.oss_fragment;
        state.ext.oss_delay = delay;
        state.oss_fragment = fragment;
        let res = crate::node::store_and_rebuild(state, data, move |state| {
            state.ext.oss_delay = delay;
            state.oss_fragment = fragment;
        });
        if res != 0 {
            state.ext.oss_delay = old_delay;
            state.oss_fragment = old_fragment;
            return res;
        }
        crate::node::handle_process_latency(state, crate::spa::process_latency_default());
        0
    }

    fn apply_oss_delay(
        state: &mut MainState<SinkDir>,
        data: &DataControl<SinkDir>,
        delay: u32,
    ) -> c_int {
        // cap it: period/8 * oss_delay runs in the RT path and must not overflow
        let new_delay = delay.min(1024);
        if new_delay == state.ext.oss_delay {
            return 0; // unchanged echoes must not rebuild a running device
        }
        let old_delay = state.ext.oss_delay;
        state.ext.oss_delay = new_delay;
        let res = crate::node::apply_props_param(state, data, move |state| {
            state.ext.oss_delay = new_delay;
        });
        if res != 0 {
            state.ext.oss_delay = old_delay;
        }
        res
    }

    fn try_open_configure(
        dsp: &mut crate::oss::DspWriter,
        config: &PortConfig,
        _fragment: u32,
        log: &crate::spa::Log,
    ) -> c_int {
        // the sink's SETFRAGMENT happens at prime time (process_ports), where
        // the graph period the layout depends on is known
        try_open_configure(dsp, config, log)
    }

    fn on_device_swapped(state: &mut DataState<SinkDir>, port_idx: usize) {
        crate::node::reset_device_event(&mut state.ports[port_idx]);
        let ext = &mut state.ports[port_idx].ext;
        ext.xrun_timestamp = 0;
        ext.resuming = false;
        ext.rebuild_after_start = false;
        clear_pending_write(ext);
    }

    fn on_buffers_swapped(state: &mut DataState<SinkDir>, port_idx: usize) {
        let port = &mut state.ports[port_idx];
        if port.dsp.end_buffer_sequence() {
            crate::node::reset_device_event(port);
        }
        clear_pending_write(&mut port.ext);
    }

    fn on_start_loop(state: &mut DataState<SinkDir>) {
        let resumes_pause = !state.started;
        for port in &mut state.ports {
            port.ext.xrun_timestamp = 0;
            let mut resume_running = resumes_pause
                && !port.ext.rebuild_after_start
                && port.dsp.is_running()
                && port.setup_period != 0;
            if resume_running {
                if let Err(err) = port.dsp.resume() {
                    crate::warn!(
                        state.log,
                        "{}: restoring paused playback: {}",
                        port.dsp.path,
                        err
                    );
                    // Do not append real audio behind a possibly full buffer
                    // of pause silence. Reset in place so process() takes the
                    // normal prime path; a refused reset requires replacement
                    // before process() may write again.
                    resume_running = false;
                    if port.dsp.suspend() {
                        crate::node::reset_device_event(port);
                    } else {
                        port.ext.rebuild_after_start = true;
                    }
                }
            }
            port.ext.resuming = resume_running;
            // Start recomputes `following`, and a role flip that happened
            // while stopped never went through on_role_flip (set_io only
            // detects flips while started): relock the same way, so e.g. a
            // paused follower promoted to driver can't start the timer servo
            // on the follower's integrator state (the source's on_start_loop
            // relocks for the same reason)
            port.dll.init();
            port.bw_adapt.reset();
            port.was_matching = false;
        }
        state.ext.cur_timestamp = 0;
        state.ext.old_timestamp = 0;
    }

    fn on_pause_loop(state: &mut DataState<SinkDir>) {
        for port in &mut state.ports {
            // A soft Pause preserves a partially accepted host buffer: SKIP
            // restores its accepted prefix, and Start continues the suffix.
            if let Err(err) = port.dsp.pause() {
                crate::warn!(
                    state.log,
                    "{}: preserving playback for Pause: {}",
                    port.dsp.path,
                    err
                );
                // A failed SILENCE cannot provide pause semantics. Reset the
                // ring so Start primes cleanly; if the device also refuses
                // that, force replacement before another playback write.
                if port.dsp.suspend() {
                    crate::node::reset_device_event(port);
                } else {
                    port.ext.rebuild_after_start = true;
                }
            }
        }
    }

    fn on_suspend_loop(state: &mut DataState<SinkDir>) {
        for port in &mut state.ports {
            port.ext.resuming = false;
            port.ext.rebuild_after_start = false;
            clear_pending_write(&mut port.ext);
        }
    }

    fn on_role_flip(state: &mut DataState<SinkDir>) {
        // a role flip shifts the servo's measurement phase, not the fill:
        // relock the DLL instead of holding playback like an underrun (the
        // fill snap in the write path corrects any real level error)
        for port in &mut state.ports {
            port.dll.init();
            port.bw_adapt.reset();
            port.was_matching = false;
        }
    }

    fn debug_cycle(state: &DataState<SinkDir>, now: u64, nsec: u64) {
        if cfg!(debug_assertions) {
            eprintln!(
                "cycle: {}, delay: {} ms @ {}",
                // position is non-null on the process path (as in process_ports)
                state.position.with_ref(|p| p.clock.cycle).unwrap_or(0),
                now.saturating_sub(nsec) as f64 / 1000000.0,
                now
            );
        }
    }

    fn servo_ready(_port: &crate::node::Port<SinkDir>) -> bool {
        true
    }

    // One FreeBSD note: GETODELAY reports the soft buffer only - the kernel
    // pre-fills the hardware buffer at trigger and never counts it - so the
    // absolute delay is understated by bufhard; the servo only needs
    // cycle-to-cycle consistency and is unaffected.
    fn servo_fill(port: &mut crate::node::Port<SinkDir>) -> u32 {
        let fill = measured_fill(port);
        let stride = port.stride_rate().map(|(stride, _)| stride).unwrap_or(1);
        settle_target(port, fill, stride);
        fill
    }

    fn servo_hold(port: &crate::node::Port<SinkDir>) -> bool {
        port.ext.xrun_timestamp != 0
    }

    fn servo_err(port: &crate::node::Port<SinkDir>, fill: u32) -> f64 {
        fill as f64 - port.ext.target_delay as f64
    }

    fn wake_threshold(port: &crate::node::Port<SinkDir>) -> u32 {
        // EVFILT_WRITE fires when free >= LOW_WATER. A healthy cycle wakes
        // with target_delay queued, writes one period, then goes inactive
        // until that period drains and the queue returns to the live target.
        port.ext
            .buffer_size
            .saturating_sub(port.ext.target_delay)
            .max(1)
    }

    fn process_ports(state: &mut DataState<SinkDir>) -> c_int {
        process_ports(state)
    }
}

const OSS_SINK_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub(crate) const OSS_SINK_FACTORY: spa_handle_factory = spa_handle_factory {
    version: SPA_VERSION_HANDLE_FACTORY,
    name: c"freebsd-oss.sink".as_ptr(),
    info: &OSS_SINK_FACTORY_INFO,
    get_size: Some(crate::node::get_size::<SinkDir>),
    init: Some(crate::node::init::<SinkDir>),
    enum_interface_info: Some(crate::node::enum_interface_info),
};

// mut: the host logger writes level/has_custom_level back after registration
pub(crate) static mut OSS_SINK_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: c"spa.oss.sink".as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};

#[cfg(test)]
mod tests;
