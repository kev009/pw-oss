use std::ffi::{c_char, c_int};

use libspa::sys::*;

use crate::backend::{
    self, PlaybackBufferGeometry, PlaybackBufferRequest, PlaybackOperations as _, PlaybackRetune,
    StreamLifecycle as _,
};
use crate::spa::{self, Log};

use super::{
    BackendPropertiesOf, DataControl, DataState, Direction, MAX_PORTS, MainState, ParamBuild, Port,
    PortConfig, build_backend_node_param, device_period_bytes, enum_interface_info, get_size, init,
    latch_rebuild_required, ns_to_bytes, pending_xrun, poll_rebuild, queue_rebuild,
    reset_backend_props, reset_stream_epoch, same_clock, take_polled_xruns, take_wake_xruns,
    try_now_ns, valid_data_block, wake_queue_fill,
};

mod buffer;

use buffer::*;

// PortInfo and clock-delay state currently support one playback port.
const _: () = assert!(MAX_PORTS == 1);

pub(crate) struct SinkDir<B>(std::marker::PhantomData<B>);

// main-loop property model
// direction-specific data-loop fields (DataState.ext)
#[derive(Default)]
pub(crate) struct SinkDataExt {
    pub cur_timestamp: u64, // method invocation timestamp for `process`
    pub old_timestamp: u64,
}

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SinkPortExt {
    pub xrun_timestamp: u64, // the moment we noticed an underrun (which is a bit later than the start of it)
    pub target_delay: u32,   // live servo target in bytes
    pub target_goal: u32,    // retained geometry target for settling and subsequent primes
    pub minimum_fill: u32,   // safe recovery floor supplied by the selected backend
    pub buffer_size: u32,    // granted playback ring capacity in bytes
    pub retune_pending: bool,
    pub resuming: bool, // first real buffer after Pause must bypass generic xrun hold
    pub rebuild_after_start: bool, // Pause could neither preserve nor reset this device
    // A nonblocking backend may accept only a prefix. Keep the host
    // buffer until every byte is accepted, and resume at this byte offset.
    pub pending_buffer: Option<u32>,
    pub pending_offset: u32,
}

fn measured_fill<B: backend::Backend>(port: &Port<SinkDir<B>>) -> u32 {
    wake_queue_fill(port).unwrap_or_else(|| port.dsp.queued_bytes())
}

fn measured_underruns<B: backend::Backend>(port: &mut Port<SinkDir<B>>) -> backend::XrunDelta {
    if let Some(count) = take_wake_xruns(port) {
        count
    } else {
        let observation = port.dsp.underruns();
        take_polled_xruns(port, observation)
    }
}

// Classify one semantic underrun observation and arm the shared graph hold
// only when the selected backend says its queue state requires recovery.
fn detect_underrun<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    period_in_bytes: u32,
    underrun: backend::XrunDelta,
    cur_timestamp: u64,
    clock_nsec: u64,
    log: &Log,
) {
    let underrun_count = underrun.events;
    let Some((stride, cfg_rate)) = port.stride_rate() else {
        return;
    };
    // Cached delivery geometry keeps the healthy gate allocation- and
    // query-free on the real-time path.
    let elapsed = cur_timestamp.saturating_sub(clock_nsec);
    let drained = ns_to_bytes(elapsed, cfg_rate, stride);
    let fill = measured_fill(port);
    let recovery_threshold = <B::Playback as backend::PlaybackOperations>::underrun_low(
        port.ext.target_delay,
        port.delivery_quantum_bytes,
        period_in_bytes,
        drained,
    );
    if fill < recovery_threshold {
        if let Some(suppressed) = port.warn_limit.check(cur_timestamp) {
            port.dsp
                .log_underrun_recovery(underrun_count, cur_timestamp, suppressed, log);
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
            port.pending_xrun = Some(pending_xrun(
                cur_timestamp / 1000,
                underrun,
                port.config.as_ref(),
            ));
        }
    } else {
        port.dsp
            .log_ignored_underruns(underrun_count, fill, recovery_threshold, log);
    }
}

// The recovery phase, entered while an underrun hold is pending
// (xrun_timestamp != 0). Recover on the first data cycle past the event
// (snap the fill and resume immediately): relock the servo,
// re-prime the fill to target and write this cycle's data in the SAME cycle.
// Waiting for a particular process cadence discards real buffers per failed
// attempt, and a follower under a corr-steered driver may never hit a fixed
// window at all. Until the recovery cycle arrives the buffer is consumed
// unwritten (the skip-buffer hold). Returns the cycle's write result
// (`size` when held).
fn recover_or_hold<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    clock_nsec: u64,
    clock_flags: u32,
    data: &[u8],
) -> backend::WriteOutcome {
    let size = data.len() as u32;
    if clock_nsec > port.ext.xrun_timestamp && clock_flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER == 0 {
        port.ext.xrun_timestamp = 0;

        port.dll.init();
        port.bw_adapt.reset();
        // A real xrun has already broken continuity. Ensure the live target
        // reaches the geometry's safe floor, but do not raise a soft-settling
        // target toward its optional goal. A steady target already equals the
        // goal and is therefore fully restored.
        let safe = port.ext.minimum_fill;
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
            port.dsp.path(),
            refill,
            odelay
        );

        port.dsp.write_silence(refill);
        // write the slice, not the period: only these bytes at the offset are owned
        port.dsp.write(data)
    } else {
        #[cfg(debug_assertions)]
        eprintln!("{}: skipping buffer @ {}", port.dsp.path(), clock_nsec);

        backend::WriteOutcome::consumed(size as usize)
    }
}

// The first real buffer after Pause is not an unexpected xrun. The backend has
// already restored any queue it preserved during Pause; seed the live target
// from that queue and write the current graph buffer immediately instead of
// consuming it with the generic one-cycle xrun hold. An empty queue plus a real
// driver underrun is the fallback when the backend preserved no audio.
fn resume_playback<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    queued: u32,
    paused_underruns: u32,
    data: &[u8],
    log: &Log,
) -> backend::WriteOutcome {
    // A backend may report only its observable queue, so zero fill by itself
    // does not prove playback stopped. Require its underrun count too;
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
            port.dsp.path()
        );
        port.dsp.write_silence(port.ext.target_delay);
    }
    let result = port.dsp.write(data);
    if !drained && result.bytes > 0 {
        // Preserve whatever real audio survived the pause as the live target.
        // Seed from bytes the backend actually accepted: a blocked or short write must
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
fn settle_target<B: backend::Backend>(port: &mut Port<SinkDir<B>>, fill: u32, stride: u32) {
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
// anyway would wind the integrator). `odelay` is
// the fill the caller measured this cycle. Returns the rate correction and
// whether this cycle's buffer must be skipped (overfill drain).
fn follower_servo<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
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
        // Fill snap: a level error past one period is
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
    eprintln!("{}: corr = {}, err = {}", port.dsp.path(), corr, err_raw);

    (corr, skip_write)
}

// Same-device follower: there is no rate actuator that can accumulate real
// audio toward a larger transition goal. Keep the seeded live target instead
// of inserting silence; correct only later drift around that reachable level.
// A genuine underrun is the only path that raises it to the geometry floor.
// Returns whether this cycle's buffer must be skipped (overfill drain).
fn level_correct<B: backend::Backend>(port: &mut Port<SinkDir<B>>, odelay: u32) -> bool {
    let err_raw = odelay as f64 - port.ext.target_delay as f64;
    if err_raw < -(port.setup_period as f64) {
        port.dsp
            .write_silence(port.ext.target_delay.saturating_sub(odelay));
    } else if err_raw > port.setup_period as f64 {
        return true;
    }
    false
}

// the shared not-ready/consumed exit of the sink cycle: publish NEED_DATA on
// the port io AND fold it into the returned status - the host prefetches the
// next buffer only on the return bit
fn need_data(io: &mut spa::IoArea<spa_io_buffers>, result: &mut c_int) {
    io.with(|io| io.status = SPA_STATUS_NEED_DATA as i32);
    *result |= SPA_STATUS_NEED_DATA as i32;
}

fn clear_pending_write(ext: &mut SinkPortExt) {
    ext.pending_buffer = None;
    ext.pending_offset = 0;
}

fn finish_input_sequence<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    event_epoch_reset: bool,
) {
    // A native reset establishes a fresh measurement epoch, but it must not
    // forgive a fatal I/O outcome that already marked this descriptor for
    // replacement.
    let rebuild_required = port.rebuild_required;
    if event_epoch_reset {
        reset_stream_epoch(port);
    }
    port.rebuild_required |= rebuild_required;
    clear_pending_write(&mut port.ext);
}

fn reset_unpreserved_pause<B: backend::Backend>(port: &mut Port<SinkDir<B>>) -> bool {
    if !port.dsp.suspend() {
        return false;
    }
    // The reset discarded both the native queue and any accepted prefix of a
    // retained host buffer. Restart that still-owned buffer at byte zero after
    // Start, while preserving any fatal rebuild latch from this epoch.
    finish_input_sequence(port, true);
    true
}

fn pause_playback<B: backend::Backend>(port: &mut Port<SinkDir<B>>, log: &Log) {
    // When Pause preserves the native queue, Start resumes its accepted
    // prefix and continues the retained host-buffer suffix.
    match port.dsp.pause() {
        Ok(backend::PauseOutcome::Preserved) => return,
        Ok(backend::PauseOutcome::Reprime) => {}
        Err(err) => crate::warn!(
            log,
            "{}: preserving playback for Pause: {}",
            port.dsp.path(),
            err
        ),
    }
    // The backend cannot preserve this queue. Reset it so Start primes
    // cleanly; if the device also refuses that, force replacement before
    // another playback write.
    if !reset_unpreserved_pause(port) {
        port.ext.rebuild_after_start = true;
    }
}

fn end_input_sequence<B: backend::Backend>(port: &mut Port<SinkDir<B>>) {
    // This buffer will no longer supply a retained suffix. Close any frame
    // that its accepted prefix left open before a different buffer arrives.
    let event_epoch_reset = port.dsp.end_buffer_sequence();
    finish_input_sequence(port, event_epoch_reset);
}

fn release_input<B: backend::Backend>(port: &mut Port<SinkDir<B>>, result: &mut c_int) {
    end_input_sequence(port);
    need_data(&mut port.io, result);
}

fn consume_freewheel_input<B: backend::Backend>(port: &mut Port<SinkDir<B>>) {
    end_input_sequence(port);
    port.io.with(|io| io.status = SPA_STATUS_NEED_DATA as i32);
}

// Return the first byte the backend has not accepted from this host buffer. A new
// buffer identity, or a chunk shorter than its previously accepted prefix,
// invalidates the old offset and starts from the new slice's beginning.
fn pending_write_offset(ext: &mut SinkPortExt, buffer_id: u32, size: usize) -> usize {
    if ext.pending_buffer != Some(buffer_id) || ext.pending_offset as usize > size {
        ext.pending_buffer = Some(buffer_id);
        ext.pending_offset = 0;
    }
    ext.pending_offset as usize
}

fn prepare_pending_write<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
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

// Retain the untouched suffix of a semantic partial write. A retryable
// zero-byte outcome retains the whole graph buffer for the next process cycle.
fn retain_partial_write(
    ext: &mut SinkPortExt,
    requested: u32,
    write: backend::WriteOutcome,
) -> bool {
    if write.bytes < requested as usize && write.retryable_partial() {
        ext.pending_offset = ext.pending_offset.saturating_add(write.bytes as u32);
        true
    } else {
        false
    }
}

// Once the backend accepts a prefix, the suffix is part of the same PCM byte
// sequence. Finish it before any fill correction can skip it or insert
// synthetic audio between the two pieces.
fn write_retained_tail<B: backend::Backend>(
    port: &mut Port<SinkDir<B>>,
    data: &[u8],
) -> Option<backend::WriteOutcome> {
    (port.ext.pending_offset != 0).then(|| port.dsp.write(data))
}

fn process_ports<B: backend::Backend>(state: &mut DataState<SinkDir<B>>) -> c_int {
    state.ext.old_timestamp = state.ext.cur_timestamp;
    // on a failed clock read reuse the previous stamp (rate limits and the
    // underrun gate degrade for a cycle) rather than abort the data loop
    state.ext.cur_timestamp = try_now_ns(&state.data_system).unwrap_or(state.ext.old_timestamp);

    // Freewheeling: the graph runs faster than realtime, so consume the input
    // without touching the device. The io NEED_DATA + return HAVE_DATA pair
    // looks odd for a sink, but it is what keeps the freewheel pump running.
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
        if poll_rebuild(state, port_idx) {
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
            queue_rebuild(state, port_idx);
            let port = &mut state.ports[port_idx];
            release_input(port, &mut result);
            continue;
        }

        if port.ext.rebuild_after_start {
            // Pause could neither preserve nor reset the old queue. Do not
            // append audio behind unknown contents; replace the device first.
            let queued = queue_rebuild(state, port_idx);
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
        let Some(data_0) = (unsafe { valid_data_block(port, buffer_id, &state.log) }) else {
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
                port.dsp.path(),
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
                .with_ref(|p| same_clock(p, &state.clock_name))
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
                    port.dsp.path(),
                    input_size,
                    quantum_bytes
                );
            }
        }

        // one graph cycle in device bytes (see node::device_period_bytes)
        let period_in_bytes = device_period_bytes(
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
            state.ext.cur_timestamp,
            &state.log,
        );
        if retune == RetuneOutcome::Rebuild {
            // the driver refused the trigger stop (dying fd): rebuild off-loop
            // (the &mut port borrow ends here: queue_rebuild snapshots an
            // owned request and owns the pending claim)
            let pending = queue_rebuild(state, port_idx);
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
            // along with the backend queue. Replay this host buffer from byte zero
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
                &state.backend_properties,
                &state.log,
            );
        } else if port.ext.resuming {
            // Consume the event-count delta accrued while deliberately
            // paused. Do not turn it into a host xrun or discard this first
            // resumed buffer.
            let paused_underruns = measured_underruns(port);
            if paused_underruns.events > 0 {
                crate::debug!(
                    state.log,
                    "{}: {} underruns accrued while paused",
                    port.dsp.path(),
                    paused_underruns.events
                );
            }
            if port.ext.pending_offset == 0 {
                let queued = measured_fill(port);
                resumed_write = Some(resume_playback(
                    port,
                    queued,
                    paused_underruns.events,
                    cycle_data,
                    &state.log,
                ));
            }
        } else {
            let underruns = measured_underruns(port);
            if underruns.events > 0 {
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
            if !retuned && matching && port.setup_period != 0 && !port.ext.retune_pending {
                (corr, skip_write) =
                    follower_servo(port, measured_fill(port), stride, state.ext.cur_timestamp);
            }

            if !retuned
                && state.following
                && !matching
                && port.setup_period != 0
                && !port.ext.retune_pending
            {
                skip_write = level_correct(port, measured_fill(port));
            }

            if skip_write {
                // consumed; the device drains toward target meanwhile
                backend::WriteOutcome::consumed(size as usize)
            } else {
                port.dsp.write(cycle_data)
            }
        };
        if port.ext.resuming && port.ext.pending_offset != 0 && !write_result.would_block() {
            port.ext.resuming = false;
        }
        latch_rebuild_required(port, write_result.status);

        // Rate-match only as a follower on a foreign clock: when driving, the
        // timer steering applies the correction, and a same-device follower
        // ticks from our clock so there is nothing to match.
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
                port.dsp.path(),
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
                port.dsp.path(),
                nbytes,
                size,
                size - nbytes as u32
            );
            // Keep io.status HAVE_DATA. The peer retains this exact input
            // buffer, and pending_offset advances the next write to its tail.
            continue;
        }

        if nbytes < size as usize
            && let Some(suppressed) = port.warn_limit.check(state.ext.cur_timestamp)
        {
            crate::warn!(
                state.log,
                "{}: dropped {} bytes (write returned {}, status {:?}) (+{} warnings suppressed)",
                port.dsp.path(),
                if nbytes > 0 {
                    size - nbytes as u32
                } else {
                    size
                },
                nbytes,
                write_result.status,
                suppressed
            );
        }

        // a sink has no output, so the return bit is NEED_DATA ("can accept input
        // next cycle"), matching the port io status, not HAVE_DATA.
        release_input(port, &mut result);
    }

    result
}

impl<B: backend::Backend> Direction for SinkDir<B> {
    const DIRECTION: spa_direction = SPA_DIRECTION_INPUT;
    const PLAYBACK: bool = true;
    const MEDIA_CLASS: &'static str = "Audio/Sink";
    const READY_STATUS: i32 = SPA_STATUS_NEED_DATA as i32;
    const CMD_WARN_PREFIX: &'static str = "";

    type Backend = B;
    type Device = B::Playback;
    type DataExt = SinkDataExt;
    type PortExt = SinkPortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        B::sink_log_topic()
    }

    fn data_ext(properties: &BackendPropertiesOf<SinkDir<B>>) -> SinkDataExt {
        let _ = properties;
        SinkDataExt::default()
    }

    fn sync_backend_properties(
        _ext: &mut SinkDataExt,
        _properties: &BackendPropertiesOf<SinkDir<B>>,
    ) {
    }

    fn build_node_param(state: &mut MainState<SinkDir<B>>, id: u32, index: u32) -> ParamBuild {
        build_backend_node_param(state, id, index)
    }

    // a NULL Props pod resets the props to their defaults and re-applies them
    fn reset_props(state: &mut MainState<SinkDir<B>>, data: &DataControl<SinkDir<B>>) -> c_int {
        reset_backend_props(state, data)
    }

    fn try_open_configure(
        stream: &mut B::Playback,
        config: &PortConfig,
        properties: &BackendPropertiesOf<SinkDir<B>>,
        log: &Log,
    ) -> Result<backend::ConfigureOutcome, c_int> {
        // Playback buffer layout is applied at prime time, when the graph
        // period it depends on is known.
        stream.configure(config, properties, log)
    }

    fn on_device_swapped(state: &mut DataState<SinkDir<B>>, port_idx: usize) {
        reset_stream_epoch(&mut state.ports[port_idx]);
        let ext = &mut state.ports[port_idx].ext;
        ext.xrun_timestamp = 0;
        ext.resuming = false;
        ext.rebuild_after_start = false;
        clear_pending_write(ext);
    }

    fn on_buffers_swapped(state: &mut DataState<SinkDir<B>>, port_idx: usize) {
        let port = &mut state.ports[port_idx];
        if port.dsp.end_buffer_sequence() {
            reset_stream_epoch(port);
        }
        clear_pending_write(&mut port.ext);
    }

    fn on_start_loop(state: &mut DataState<SinkDir<B>>) {
        let resumes_pause = !state.started;
        for port in &mut state.ports {
            port.ext.xrun_timestamp = 0;
            let mut resume_running = resumes_pause
                && !port.ext.rebuild_after_start
                && port.dsp.is_running()
                && port.setup_period != 0;
            if resume_running && let Err(err) = port.dsp.resume() {
                crate::warn!(
                    state.log,
                    "{}: restoring paused playback: {}",
                    port.dsp.path(),
                    err
                );
                // Do not append real audio behind a possibly full buffer
                // of pause silence. Reset in place so process() takes the
                // normal prime path; a refused reset requires replacement
                // before process() may write again.
                resume_running = false;
                if port.dsp.suspend() {
                    reset_stream_epoch(port);
                } else {
                    port.ext.rebuild_after_start = true;
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

    fn on_pause_loop(state: &mut DataState<SinkDir<B>>) {
        for port in &mut state.ports {
            pause_playback(port, &state.log);
        }
    }

    fn on_suspend_loop(state: &mut DataState<SinkDir<B>>) {
        for port in &mut state.ports {
            port.ext.resuming = false;
            port.ext.rebuild_after_start = false;
            clear_pending_write(&mut port.ext);
        }
    }

    fn on_role_flip(state: &mut DataState<SinkDir<B>>) {
        // a role flip shifts the servo's measurement phase, not the fill:
        // relock the DLL instead of holding playback like an underrun (the
        // fill snap in the write path corrects any real level error)
        for port in &mut state.ports {
            port.dll.init();
            port.bw_adapt.reset();
            port.was_matching = false;
        }
    }

    fn debug_cycle(state: &DataState<SinkDir<B>>, now: u64, nsec: u64) {
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

    fn servo_ready(_port: &Port<SinkDir<B>>) -> bool {
        true
    }

    // Queue observations may omit backend-internal staging. The servo needs
    // cycle-to-cycle consistency rather than an absolute physical-latency
    // estimate, so a stable omission does not bias its correction.
    fn servo_fill(port: &mut Port<SinkDir<B>>) -> u32 {
        let fill = measured_fill(port);
        let stride = port.stride_rate().map(|(stride, _)| stride).unwrap_or(1);
        settle_target(port, fill, stride);
        fill
    }

    fn servo_hold(port: &Port<SinkDir<B>>) -> bool {
        port.ext.xrun_timestamp != 0
    }

    fn servo_err(port: &Port<SinkDir<B>>, fill: u32) -> f64 {
        fill as f64 - port.ext.target_delay as f64
    }

    fn wake_buffer_state(port: &Port<SinkDir<B>>) -> backend::WakeBufferState {
        backend::WakeBufferState {
            frame_stride: port.stride_rate().map_or(1, |(stride, _)| stride),
            period_bytes: port.setup_period,
            quantum_bytes: port.delivery_quantum_bytes,
            capacity_bytes: port.ext.buffer_size,
            target_fill_bytes: port.ext.target_delay,
        }
    }

    fn process_ports(state: &mut DataState<SinkDir<B>>) -> c_int {
        process_ports(state)
    }
}

const SINK_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub const fn factory<B: backend::Backend>(name: *const c_char) -> spa_handle_factory {
    spa_handle_factory {
        version: SPA_VERSION_HANDLE_FACTORY,
        name,
        info: &SINK_FACTORY_INFO,
        get_size: Some(get_size::<SinkDir<B>>),
        init: Some(init::<SinkDir<B>>),
        enum_interface_info: Some(enum_interface_info),
    }
}

#[cfg(test)]
mod tests;
