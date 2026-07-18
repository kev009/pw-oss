use std::os::raw::c_int;

use libspa::sys::*;

use crate::node::{Direction, ParamBuild, State, MAX_PORTS};

// several State fields are per-port in disguise (the single PortInfo,
// on_timeout's last-port-wins clock delay); fix those before raising this
const _: () = assert!(MAX_PORTS == 1);

pub(crate) enum SinkDir {}

// direction-specific State fields (State.ext)
pub(crate) struct SinkExt {
    pub cur_timestamp: u64, // method invocation timestamp for `process`
    pub old_timestamp: u64,
    pub oss_delay: u32,         // additional delay in 1/8ths of period
    pub oss_delay_default: u32, // init-time value, restored by a NULL Props reset
}

impl Default for SinkExt {
    fn default() -> Self {
        Self {
            cur_timestamp: 0,
            old_timestamp: 0,
            // default fill target: 10/8 of a period
            oss_delay: 10,
            oss_delay_default: 10,
        }
    }
}

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SinkPortExt {
    pub xrun_timestamp: u64, // the moment we noticed an underrun (which is a bit later than the start of it)
    pub target_delay: u32,   // OSS buffer fill target in bytes, clamped to the granted buffer
    pub buffer_size: u32,    // granted OSS playback ring capacity in bytes
    pub period_mismatch: u32, // consecutive cycles at a different period (debounce)
}

#[derive(Debug, Clone)]
pub(crate) struct PortConfig {
    pub format: libspa::param::audio::AudioFormat,
    pub rate: u32,
    pub channels: u32,
    pub positions: Vec<u32>, // the negotiated channel positions, replayed in the Format readback
    pub flags: u32,
}

impl PortConfig {
    fn bytes_per_sample(&self) -> u32 {
        match self.format {
            libspa::param::audio::AudioFormat::S32LE => 4,
            libspa::param::audio::AudioFormat::S32BE => 4,
            libspa::param::audio::AudioFormat::S16LE => 2,
            libspa::param::audio::AudioFormat::S16BE => 2,
            _ => unreachable!(),
        }
    }

    fn stride(&self) -> u32 {
        self.bytes_per_sample() * self.channels
    }

    fn oss_format(&self) -> u32 {
        match self.format {
            libspa::param::audio::AudioFormat::S32LE => crate::sound::AFMT_S32_LE,
            libspa::param::audio::AudioFormat::S32BE => crate::sound::AFMT_S32_BE,
            libspa::param::audio::AudioFormat::S16LE => crate::sound::AFMT_S16_LE,
            libspa::param::audio::AudioFormat::S16BE => crate::sound::AFMT_S16_BE,
            _ => unreachable!(), // rejected at negotiation
        }
    }
}

impl crate::node::ConfigOps for PortConfig {
    fn oss_format(&self) -> u32 {
        PortConfig::oss_format(self)
    }
    fn rate(&self) -> u32 {
        self.rate
    }
    fn channels(&self) -> u32 {
        self.channels
    }
    fn stride(&self) -> u32 {
        PortConfig::stride(self)
    }
    fn format_raw(&self) -> u32 {
        self.format.0
    }
    fn flags(&self) -> u32 {
        self.flags
    }
    fn positions(&self) -> &[u32] {
        &self.positions
    }
}

fn desired_delay(period: u32, oss_delay: u32) -> u32 {
    (period / 8).saturating_mul(oss_delay)
}

// the resampler's per-cycle output can exceed a quantum; its size bounds the
// largest single write and so the headroom the fill ceiling must reserve
unsafe fn rate_match_bytes(rate_match: *const spa_io_rate_match, stride: u32) -> u32 {
    if rate_match.is_null() {
        0
    } else {
        (*rate_match).size.saturating_mul(stride)
    }
}

// the fill target's floor: one period plus a jitter margin (a quarter period,
// or one device fragment when the fragment dwarfs the quantum), so a small
// oss.delay or a tiny quantum can't starve the wakeup fill. buffer_required()
// and target_delay() must derive this identically: the in-place retune gate
// (buffer_size >= required) guarantees the fill ceiling clears this floor
// only while the two agree.
fn fill_floor(period: u32, blocksize: u32) -> u32 {
    period.saturating_add((period / 4).max(blocksize))
}

fn buffer_required(period: u32, desired: u32, blocksize: u32, write_max: u32) -> u32 {
    period.saturating_mul(2).saturating_add(desired).max(
        fill_floor(period, blocksize)
            .saturating_add(write_max)
            .saturating_add(blocksize),
    )
}

// The prime-time ring request: what the current period needs, floored at
// what the LARGEST negotiable quantum needs (max_period comes from
// sound::max_ring_period_bytes - the shared policy behind this floor, the
// capture ring request and the advertised node.max-latency), never below
// MIN_RING_BYTES, never above the kernel cap (which always wins). Capacity
// is not latency: the fill target below still controls queued audio, while
// a ring sized for every negotiable quantum lets period changes retune in
// place instead of resizing the device.
fn buffer_request(
    period: u32,
    max_period: u32,
    cap: u32,
    fragment: u32,
    chunk: u32,
    write_max: u32,
    oss_delay: u32,
) -> u32 {
    let frag_est = if fragment == 0 { 1024 } else { fragment };
    let transfer = frag_est.max(chunk);
    let stable = buffer_required(
        max_period,
        desired_delay(max_period, oss_delay),
        transfer,
        max_period,
    );
    buffer_required(
        period,
        desired_delay(period, oss_delay),
        transfer,
        write_max,
    )
    .max(stable)
    .max(crate::sound::MIN_RING_BYTES)
    .min(cap)
}

fn target_delay(
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
        // write that doesn't fit short-writes and DROPS the tail. A driver that
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
fn commit_geometry(
    port: &mut crate::node::Port<SinkDir>,
    granted: u32,
    period: u32,
    blocksize: u32,
    write_max: u32,
    desired: u32,
) -> bool {
    let (target, delay_capped) = target_delay(granted, period, blocksize, write_max, desired);
    port.setup_period = period;
    port.setup_blocksize = blocksize; // the effective quantum, incl. hw chunk
    port.ext.target_delay = target;
    port.dll.init();
    port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle
    let (stride, rate) = port.stride_rate().unwrap_or((1, 0));
    port.bw_adapt
        .configure(stride, blocksize, period, rate.saturating_mul(stride));
    delay_capped
}

fn log_delay_capped(log: &crate::spa::Log, path: &str, granted: u32) {
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
// becomes legal again). Returns true when the driver refused the trigger stop
// (dying fd) and only a main-thread rebuild remains.
fn retune_period(
    port: &mut crate::node::Port<SinkDir>,
    period_in_bytes: u32,
    stride: u32,
    oss_delay: u32,
    log: &crate::spa::Log,
) -> bool {
    if !port.dsp.is_running()
        || port.setup_period == 0
        || period_in_bytes == 0
        || period_in_bytes == port.setup_period
    {
        port.ext.period_mismatch = 0;
        return false;
    }
    // debounce BOTH paths: a single-cycle flip usually means a renegotiation is
    // in flight (which re-primes anyway); a rebuild on it costs an audible gap,
    // and even the in-place retune relocks the servo and snaps the fill. Write
    // at the old size for one cycle instead.
    port.ext.period_mismatch += 1;
    if port.ext.period_mismatch < 2 {
        return false;
    }
    // cached blocksize: the triggered channel refuses SETFRAGMENT, so the
    // granted fragment (and the session-fixed hw cadence folded in at
    // prime) cannot have changed; reusing it avoids an ioctl here
    let blocksize = port.setup_blocksize;
    let desired = desired_delay(period_in_bytes, oss_delay);
    let write_max = period_in_bytes.max(unsafe { rate_match_bytes(port.rate_match, stride) });
    if port.ext.buffer_size >= buffer_required(period_in_bytes, desired, blocksize, write_max) {
        let old_period = port.setup_period;
        let delay_capped = commit_geometry(
            port,
            port.ext.buffer_size,
            period_in_bytes,
            blocksize,
            write_max,
            desired,
        );
        port.ext.period_mismatch = 0;
        port.was_matching = false;

        // Level snap (ALSA's resync does the same): a sustained quantum
        // change is a re-prime in place. When driving, nothing else
        // corrects the level - the timer servo only rate-steers with a
        // clamped error - so an under-filled ring after a quantum growth
        // would sit one late wakeup away from an underrun for the seconds
        // the skew needs; fill to target like the prime and xrun-recovery
        // paths do. Overfill after a shrink is latency only and drains via
        // the servo (or follower_servo's fill snap).
        let odelay = port.dsp.odelay();
        port.dsp
            .write_zeroes(port.ext.target_delay.saturating_sub(odelay));

        crate::info!(
            log,
            "{}: period {} -> {} bytes; retuned in place (granted {}, target delay {})",
            port.dsp.path,
            old_period,
            period_in_bytes,
            port.ext.buffer_size,
            port.ext.target_delay
        );
        if delay_capped {
            log_delay_capped(log, &port.dsp.path, port.ext.buffer_size);
        }
        false
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
        false
    } else {
        // period_mismatch stays >= 2 on purpose: if the caller can't queue the
        // rebuild (no main loop), the next cycle retries this retune
        // immediately instead of re-running the debounce
        crate::info!(
            log,
            "{}: period {} -> {} bytes; re-setting up",
            port.dsp.path,
            port.setup_period,
            period_in_bytes
        );
        true
    }
}

// debug-build diagnostics: the scheduling class/priority the data loop
// actually runs at (RT setup problems show up here first)
#[cfg(debug_assertions)]
fn debug_log_priorities(log: &crate::spa::Log) {
    fn prio_type(type_: libc::c_ushort) -> &'static str {
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
fn prime_playback(
    port: &mut crate::node::Port<SinkDir>,
    period_in_bytes: u32,
    graph_rate: u32,
    oss_delay: u32,
    oss_fragment: u32,
    log: &crate::spa::Log,
) {
    #[cfg(debug_assertions)]
    debug_log_priorities(log);

    let Some((stride, cfg_rate)) = port.stride_rate() else {
        return;
    };

    // Size the fill to the granted buffer and the device's real fragment.
    // oss_fragment (0 = automatic 1 KiB) only mutates on this loop, so the
    // read is race-free; no ioctls beyond what the prime always issued
    // The measurement/drain quantum is the granted fragment - unless the
    // device's hardware cadence is coarser (drivers that ignore
    // SETFRAGMENT and pull fixed transfers; vchan parents), which the
    // soft fragsize can't see and sndstat can. Floor, headroom and the
    // servo noise model key on the larger - and the buffer REQUEST must
    // include it, or a device that honors the request grants no room for
    // the ceiling above the floor.
    let desired = desired_delay(period_in_bytes, oss_delay);
    let chunk = crate::utils::ns_to_frame_bytes(port.dsp.hw_quantum_ns, cfg_rate, stride);
    let write_max = period_in_bytes.max(unsafe { rate_match_bytes(port.rate_match, stride) });
    let max_period = crate::sound::max_ring_period_bytes(stride, cfg_rate, graph_rate);
    let request = buffer_request(
        period_in_bytes,
        max_period,
        crate::sound::ring_byte_cap(stride, cfg_rate),
        oss_fragment,
        chunk,
        write_max,
        oss_delay,
    );
    let granted = port.dsp.set_buffer_size(request, oss_fragment);
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
            period_in_bytes * 2
        );
    }

    port.dsp.write_zeroes(port.ext.target_delay);
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
fn detect_underrun(
    port: &mut crate::node::Port<SinkDir>,
    period_in_bytes: u32,
    underrun_count: u32,
    cur_timestamp: u64,
    clock_nsec: u64,
    callbacks: &spa_callbacks,
    log: &crate::spa::Log,
) {
    let Some((stride, cfg_rate)) = port.stride_rate() else {
        return;
    };
    // (cached blocksize: the channel can't be retuned while triggered,
    // and the gate must not cost ioctls on healthy cycles)
    let low = period_in_bytes
        .min(port.ext.target_delay.saturating_sub(port.setup_blocksize))
        .max(period_in_bytes / 4);
    // A late cycle finds a legitimately lower fill (the device kept
    // draining), so the threshold tracks the expected healthy fill at
    // THIS moment; the floor keeps a true empty ring (a real underrun
    // reads 0 until we write) detectable at any lateness.
    let elapsed = cur_timestamp.saturating_sub(clock_nsec);
    let drained = crate::utils::ns_to_bytes(elapsed, cfg_rate, stride);
    let wander = (period_in_bytes / 4).max(port.setup_blocksize);
    let low = low
        .min(
            port.ext
                .target_delay
                .saturating_sub(drained)
                .saturating_sub(wander),
        )
        .max(period_in_bytes / 16);
    let odelay_now = port.dsp.odelay();
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

            // once per event, not per held cycle
            // the host callback table outlives the node (set_callbacks contract)
            unsafe { crate::node::emit_xrun(callbacks, cur_timestamp / 1000) };
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
unsafe fn recover_or_hold(
    port: &mut crate::node::Port<SinkDir>,
    clock_nsec: u64,
    clock_flags: u32,
    data: *const std::os::raw::c_void,
    size: u32,
) -> isize {
    if clock_nsec > port.ext.xrun_timestamp && clock_flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER == 0 {
        port.ext.xrun_timestamp = 0;

        port.dll.init();
        port.bw_adapt.reset();

        // buffer's already sized; re-prime only up to target, accounting for what's
        // still queued (a full target_delay would push odelay past the buffer)
        let odelay = port.dsp.odelay();
        let refill = port.ext.target_delay.saturating_sub(odelay);

        #[cfg(debug_assertions)]
        eprintln!(
            "{}: re-priming with {} zeroes (odelay {})",
            port.dsp.path, refill, odelay
        );

        port.dsp.write_zeroes(refill);
        // write `size`, not the period: only `size` bytes at the offset are owned
        port.dsp.write(data, size)
    } else {
        #[cfg(debug_assertions)]
        eprintln!("{}: skipping buffer @ {}", port.dsp.path, clock_nsec);

        size as isize
    }
}

// The follower-servo phase, matching a foreign clock: the DLL serves rate
// matching only (when driving, the servo runs in on_timeout where the clock
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
                .write_zeroes(port.ext.target_delay.saturating_sub(odelay));
        } else {
            skip_write = true;
        }
    } else {
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = err_raw.clamp(-max_err, max_err);
        corr = port.dll.update(err);
        port.bw_adapt.update(&mut port.dll, err, nsec);
    }

    #[cfg(debug_assertions)]
    eprintln!("{}: corr = {}, err = {}", port.dsp.path, corr, err_raw);

    (corr, skip_write)
}

// same-device follower: no rate to match, but the level can still drift on
// missed cycles; correct it directly. Returns whether this cycle's buffer
// must be skipped (overfill drain).
fn level_correct(port: &mut crate::node::Port<SinkDir>, odelay: u32) -> bool {
    let err_raw = odelay as f64 - port.ext.target_delay as f64;
    if err_raw < -(port.setup_period as f64) {
        port.dsp
            .write_zeroes(port.ext.target_delay.saturating_sub(odelay));
    } else if err_raw > port.setup_period as f64 {
        return true;
    }
    false
}

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(
    dsp: &mut crate::sound::DspWriter,
    config: &PortConfig,
    log: &crate::spa::Log,
) -> c_int {
    // a busy or vanished device must fail negotiation, not abort
    if let Err(err) = dsp.open() {
        crate::warn!(log, "{}: open: {}", dsp.path, err);
        return -(err as c_int);
    }
    // ditto for a device that won't take the format exactly
    if let Err(err) = dsp.configure(config.oss_format(), config.channels, config.rate) {
        crate::warn!(log, "{}: device rejected {:?}: {}", dsp.path, config, err);
        dsp.close();
        return -(err as c_int);
    }
    // on direct opens the hardware blocksize is per-session state; re-read it
    // now that THIS configuration is in effect (vchan/uaudio values are stable)
    dsp.hw_quantum_ns = crate::sound::drain_quantum_ns(&dsp.path, true);
    0
}

unsafe fn process_ports(state: &mut State<SinkDir>) -> c_int {
    state.ext.old_timestamp = state.ext.cur_timestamp;
    state.ext.cur_timestamp = crate::utils::now_ns(&state.data_system);

    // Freewheeling: the graph runs faster than realtime, so consume the input
    // without touching the device. The io NEED_DATA + return HAVE_DATA pair
    // looks odd for a sink but matches alsa-pcm-sink.c:788-791; it is what
    // keeps the freewheel pump running.
    if (*state.position).clock.flags & SPA_IO_CLOCK_FLAG_FREEWHEEL != 0 {
        for port in &mut state.ports {
            if !port.io.is_null() {
                (*port.io).status = SPA_STATUS_NEED_DATA as i32;
            }
        }
        return SPA_STATUS_HAVE_DATA as i32;
    }

    let mut result = SPA_STATUS_OK as i32;
    let state_ptr: *mut State<SinkDir> = state;

    for (port_idx, port) in state.ports.iter_mut().enumerate() {
        let Some((stride, cfg_rate)) = port.stride_rate() else {
            continue; // no format negotiated yet
        };

        if port.buffers.is_empty() || port.io.is_null() {
            continue; // not (fully) negotiated yet
        }

        if port.resetup_pending {
            // the main thread is rebuilding the device; drop cycles until it lands
            (*port.io).status = SPA_STATUS_NEED_DATA as i32;
            result |= SPA_STATUS_NEED_DATA as i32;
            continue;
        }

        if port.dsp.is_closed() {
            // Suspend closed the device but the host restarted without a fresh
            // format; rebuild off-loop instead of tripping the dsp state asserts
            port.resetup_pending = crate::node::queue_resetup(state_ptr, port_idx);
            (*port.io).status = SPA_STATUS_NEED_DATA as i32;
            result |= SPA_STATUS_NEED_DATA as i32;
            continue;
        }

        if (*port.io).status != SPA_STATUS_HAVE_DATA as i32 {
            // no input this cycle (e.g. draining after stop); the clock (incl. the
            // draining delay) is published from on_timeout now, so just ask for data
            (*port.io).status = SPA_STATUS_NEED_DATA as i32;
            result |= SPA_STATUS_NEED_DATA as i32; // in the return too: the host prefetches only on this bit
            continue;
        }

        let buffer_id = (*port.io).buffer_id;
        let Some(data_0) = crate::node::valid_data_block(port, buffer_id, &state.log) else {
            (*port.io).status = SPA_STATUS_NEED_DATA as i32;
            result |= SPA_STATUS_NEED_DATA as i32; // return status, not just io, so the host refills
            continue;
        };

        // chunk non-null and maxsize > 0 guaranteed above
        let offset = (*data_0.chunk.as_ptr()).offset % data_0.maxsize;
        let size = (*data_0.chunk.as_ptr()).size.min(data_0.maxsize - offset);

        debug_assert_eq!((*data_0.chunk.as_ptr()).stride, stride as i32);

        #[cfg(debug_assertions)]
        if (*state.position).clock.flags & SPA_IO_CLOCK_FLAG_XRUN_RECOVER != 0 {
            crate::warn!(
                state.log,
                "{}: SPA_IO_CLOCK_FLAG_XRUN_RECOVER @ {}",
                port.dsp.path,
                state.ext.cur_timestamp
            );
        }

        #[cfg(debug_assertions)]
        if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
            crate::trace!(state.log, "offset: {}, chunk size: {}", offset, size);
            spa_debug_mem(
                0,
                data_0.data.as_ptr().offset(offset as isize),
                16.min(size) as usize,
            );
        }

        let driver_clock = (*state.position).clock;
        let matching =
            state.following && !crate::utils::same_clock(state.position, &state.clock_name);

        // the resampler can legitimately hand us a few frames over a quantum; warn
        // rather than debug_assert!, which would abort the process (panic across the
        // extern "C" boundary). The write path below caps and drops the excess.
        #[cfg(debug_assertions)]
        if size > driver_clock.target_duration as u32 * stride {
            crate::warn!(
                state.log,
                "{}: chunk size {} exceeds one quantum {}",
                port.dsp.path,
                size,
                driver_clock.target_duration as u32 * stride
            );
        }

        // one graph cycle in device bytes (see utils::device_period_bytes)
        let period_in_bytes = crate::utils::device_period_bytes(
            driver_clock.target_duration,
            cfg_rate,
            driver_clock.target_rate.denom,
            stride,
        );

        if retune_period(
            port,
            period_in_bytes,
            stride,
            state.ext.oss_delay,
            &state.log,
        ) {
            // the driver refused the trigger stop (dying fd): rebuild off-loop
            port.resetup_pending = crate::node::queue_resetup(state_ptr, port_idx);
            if port.resetup_pending {
                port.was_matching = false; // the gap invalidates matching history
                (*port.io).status = SPA_STATUS_NEED_DATA as i32;
                result |= SPA_STATUS_NEED_DATA as i32;
                continue;
            }
            // no main loop (unusual host): keep running at the stale size; the
            // write path drops or underruns but nothing stalls or aborts
        }

        if !port.dsp.is_running() {
            prime_playback(
                port,
                period_in_bytes,
                driver_clock.target_rate.denom,
                state.ext.oss_delay,
                state.oss_fragment,
                &state.log,
            );
        } else {
            let underruns = port.dsp.underruns();
            if underruns > 0 {
                detect_underrun(
                    port,
                    period_in_bytes,
                    underruns,
                    state.ext.cur_timestamp,
                    driver_clock.nsec,
                    &state.callbacks,
                    &state.log,
                );
            }
        }

        let mut corr: f64 = 1.0; // DLL rate correction, published through rate_match below
        let nbytes = if port.ext.xrun_timestamp != 0 {
            recover_or_hold(
                port,
                driver_clock.nsec,
                driver_clock.flags,
                data_0.data.as_ptr().offset(offset as isize),
                size,
            )
        } else {
            let mut skip_write = false;
            if matching && port.setup_period != 0 && port.ext.period_mismatch == 0 {
                (corr, skip_write) =
                    follower_servo(port, port.dsp.odelay(), stride, state.ext.cur_timestamp);
            }

            if state.following
                && !matching
                && port.setup_period != 0
                && port.ext.period_mismatch == 0
            {
                skip_write = level_correct(port, port.dsp.odelay());
            }

            if skip_write {
                size as isize // consumed; the device drains toward target meanwhile
            } else {
                port.dsp.write(data_0.data.as_ptr().offset(offset as isize), size)
            }
        };

        // Rate-match only as a follower on a foreign clock: when driving, the
        // timer steering applies the correction, and a same-device follower ticks
        // from our clock so there is nothing to match (ALSA gates on the clock
        // name the same way).
        port.was_matching = matching;
        if !port.rate_match.is_null() {
            if matching {
                (*port.rate_match).flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                (*port.rate_match).rate = corr.clamp(0.99, 1.01);
            } else {
                (*port.rate_match).flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                (*port.rate_match).rate = 1.0;
            }
        }

        if nbytes < size as isize {
            if let Some(suppressed) = port.warn_limit.check(state.ext.cur_timestamp) {
                crate::warn!(
                    state.log,
                    "{}: dropped {} bytes (+{} warnings suppressed)",
                    port.dsp.path,
                    if nbytes > 0 {
                        size - nbytes as u32
                    } else {
                        size
                    },
                    suppressed
                );
            }
        }

        (*port.io).status = SPA_STATUS_NEED_DATA as i32;

        // a sink has no output, so the return bit is NEED_DATA ("can accept input
        // next cycle"), matching the port io status, not HAVE_DATA.
        result |= SPA_STATUS_NEED_DATA as i32;
    }

    result
}

impl Direction for SinkDir {
    const DIRECTION: spa_direction = SPA_DIRECTION_INPUT;
    const PLAYBACK: bool = true;
    const MEDIA_CLASS: &'static str = "Audio/Sink";
    const READY_STATUS: i32 = SPA_STATUS_NEED_DATA as i32;
    const CMD_WARN_PREFIX: &'static str = "";

    type Device = crate::sound::DspWriter;
    type Config = PortConfig;
    type Ext = SinkExt;
    type PortExt = SinkPortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        std::ptr::NonNull::new(&raw mut OSS_SINK_TOPIC).expect("a static's address is never null")
    }

    fn info_item(ext: &mut SinkExt, key: &str, value: &str) {
        if key == crate::keys::OSS_DELAY {
            // per-device default, e.g. from a wireplumber node rule
            if let Ok(v) = value.parse::<u32>() {
                ext.oss_delay = v.min(1024);
            }
        }
    }

    fn ext_ready(ext: &mut SinkExt) {
        ext.oss_delay_default = ext.oss_delay;
    }

    unsafe fn build_node_param(
        state: &mut State<SinkDir>,
        b: &mut libspa::pod::builder::Builder,
        id: u32,
        index: u32,
    ) -> ParamBuild {
        #[allow(non_upper_case_globals)]
        match (id, index) {
            (SPA_PARAM_PropInfo, 0) => crate::utils::build_latency_offset_prop_info(b).unwrap(),
            (SPA_PARAM_PropInfo, 1) => crate::utils::build_params_prop_info(
                b,
                crate::keys::OSS_DELAY,
                "OSS buffer fill target (1/8ths of a period)",
                state.ext.oss_delay,
                1024,
            )
            .unwrap(),
            (SPA_PARAM_PropInfo, 2) => crate::utils::build_params_prop_info(
                b,
                crate::keys::OSS_FRAGMENT,
                "OSS fragment size (bytes, power of two, 0 = automatic)",
                state.oss_fragment,
                16384,
            )
            .unwrap(),
            (SPA_PARAM_PropInfo, _) => return ParamBuild::Exhausted,
            (SPA_PARAM_Props, 0) => crate::utils::build_latency_offset_props(
                b,
                state.process_latency.ns,
                &[
                    (crate::keys::OSS_DELAY, state.ext.oss_delay),
                    (crate::keys::OSS_FRAGMENT, state.oss_fragment),
                ],
            )
            .unwrap(),
            (SPA_PARAM_Props, _) => return ParamBuild::Exhausted,
            (SPA_PARAM_ProcessLatency, 0) => {
                crate::utils::build_process_latency_info(b, &state.process_latency).unwrap();
            }
            (SPA_PARAM_ProcessLatency, _) => return ParamBuild::Exhausted,
            _ => return ParamBuild::Unknown,
        };
        ParamBuild::Built
    }

    // a NULL Props pod resets the props to their defaults and re-applies them
    unsafe fn reset_props(state: &mut State<SinkDir>) -> c_int {
        let res = crate::node::store_and_rebuild(state, |state| {
            state.ext.oss_delay = state.ext.oss_delay_default; // read by process()
            state.oss_fragment = state.oss_fragment_default; // ditto (the prime path)
        });
        if res != 0 {
            return res;
        }
        crate::node::handle_process_latency(state, crate::utils::process_latency_default());
        0
    }

    unsafe fn set_props_params(state: &mut State<SinkDir>, value: &libspa::pod::Value) -> c_int {
        use libspa::pod::Value;
        match value {
            Value::Struct(values) if values.len() % 2 == 0 => {
                for kv in values.chunks(2) {
                    match (&kv[0], &kv[1]) {
                        // pw-cli set-param <object-id> Props '{ "params": ["oss.delay", 8]}'
                        (Value::String(s), Value::Int(x))
                            if s == crate::keys::OSS_DELAY && *x >= 0 =>
                        {
                            // cap it: period/8 * oss_delay runs in the RT path and must not overflow
                            let new_delay = (*x as u32).min(1024);
                            if new_delay != state.ext.oss_delay {
                                // unchanged echoes must not rebuild a running device
                                let res = crate::node::apply_props_param(state, move |state| {
                                    state.ext.oss_delay = new_delay;
                                });
                                if res != 0 {
                                    return res;
                                }
                            }
                        }
                        (Value::String(s), Value::Int(x))
                            if s == crate::keys::OSS_FRAGMENT && *x >= 0 =>
                        {
                            // stored normalized, so the Props readback reports the
                            // effective (rounded/clamped) value, not the raw request
                            let new_fragment = crate::node::normalize_fragment(*x as u32);
                            if new_fragment != state.oss_fragment {
                                let res = crate::node::apply_props_param(state, move |state| {
                                    state.oss_fragment = new_fragment;
                                });
                                if res != 0 {
                                    return res;
                                }
                            }
                        }
                        _ => (),
                    }
                }
            }
            _ => (),
        }
        0
    }

    unsafe fn parse_config(
        state: &mut State<SinkDir>,
        raw: &spa_audio_info_raw,
    ) -> Result<PortConfig, c_int> {
        let format = libspa::param::audio::AudioFormat(raw.format);

        let config = PortConfig {
            format,
            rate: raw.rate,
            channels: raw.channels,
            positions: raw.position[..raw.channels as usize].to_vec(),
            flags: raw.flags,
        };

        crate::debug!(state.log, "reconfiguring with {:?}", config);

        // only formats from our EnumFormat are expected; reject the rest
        let oss_format = match config.format {
            libspa::param::audio::AudioFormat::S32LE => crate::sound::AFMT_S32_LE,
            libspa::param::audio::AudioFormat::S32BE => crate::sound::AFMT_S32_BE,
            libspa::param::audio::AudioFormat::S16LE => crate::sound::AFMT_S16_LE,
            libspa::param::audio::AudioFormat::S16BE => crate::sound::AFMT_S16_BE,
            _ => {
                crate::warn!(
                    state.log,
                    "rejecting unsupported format {:?}",
                    config.format
                );
                return Err(-libc::ENOTSUP);
            }
        };

        let _ = oss_format;
        Ok(config)
    }

    fn try_open_configure(
        dsp: &mut crate::sound::DspWriter,
        config: &PortConfig,
        _fragment: u32,
        log: &crate::spa::Log,
    ) -> c_int {
        // the sink's SETFRAGMENT happens at prime time (process_ports), where
        // the graph period the layout depends on is known
        try_open_configure(dsp, config, log)
    }

    fn on_device_swapped(state: &mut State<SinkDir>, port_idx: usize) {
        state.ports[port_idx].ext.xrun_timestamp = 0;
    }

    fn on_buffers_swapped(_state: &mut State<SinkDir>, _port_idx: usize) {}

    fn on_start_loop(state: &mut State<SinkDir>) {
        for port in &mut state.ports {
            port.ext.xrun_timestamp = 0;
        }
        state.ext.cur_timestamp = 0;
        state.ext.old_timestamp = 0;
    }

    fn on_suspend_loop(_state: &mut State<SinkDir>) {}

    fn on_role_flip(state: &mut State<SinkDir>) {
        // a role flip shifts the servo's measurement phase, not the fill:
        // relock the DLL instead of holding playback like an underrun (the
        // fill snap in the write path corrects any real level error)
        for port in &mut state.ports {
            port.dll.init();
            port.bw_adapt.reset();
            port.was_matching = false;
        }
    }

    // data loop only
    unsafe fn update_timers(state: &mut State<SinkDir>) {
        #[cfg(debug_assertions)]
        crate::trace!(state.log, "update_timers");

        if state.started && !state.following && !state.position.is_null() {
            state.next_time = crate::utils::now_ns(&state.data_system);
            #[cfg(debug_assertions)]
            crate::trace!(state.log, "next time {}", state.next_time);
            crate::node::set_timeout(state, state.next_time);
        } else {
            #[cfg(debug_assertions)]
            crate::trace!(state.log, "next time {}", 0);
            crate::node::set_timeout(state, 0);
        }
    }

    unsafe fn debug_cycle(_state: &State<SinkDir>, _now: u64, _nsec: u64) {
        #[cfg(debug_assertions)]
        eprintln!(
            "cycle: {}, delay: {} ms @ {}",
            (*_state.position).clock.cycle,
            _now.saturating_sub(_nsec) as f64 / 1000000.0,
            _now
        );
    }

    fn servo_ready(_port: &crate::node::Port<SinkDir>) -> bool {
        true
    }

    // One FreeBSD note: GETODELAY reports the soft buffer only - the kernel
    // pre-fills the hardware buffer at trigger and never counts it - so the
    // absolute delay is understated by bufhard; the servo only needs
    // cycle-to-cycle consistency and is unaffected.
    fn servo_fill(port: &mut crate::node::Port<SinkDir>) -> u32 {
        port.dsp.odelay()
    }

    fn servo_hold(port: &crate::node::Port<SinkDir>) -> bool {
        port.ext.xrun_timestamp != 0
    }

    fn servo_err(port: &crate::node::Port<SinkDir>, fill: u32) -> f64 {
        fill as f64 - port.ext.target_delay as f64
    }

    unsafe fn process_ports(state: &mut State<SinkDir>) -> c_int {
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
mod tests {
    use super::{buffer_request, buffer_required, desired_delay, fill_floor, target_delay};
    use super::{
        detect_underrun, follower_servo, level_correct, recover_or_hold, retune_period, SinkDir,
        SinkPortExt,
    };
    use crate::sound::test_util::{drain, fill_pipe, free_space, pattern, pipe_pair};
    use libspa::sys::{spa_callbacks, SPA_IO_CLOCK_FLAG_XRUN_RECOVER};

    // a Port on a pipe-backed device: the pipe's buffer plays the OSS ring
    // (byte-exact accounting, short writes on a full ring), GETODELAY reads 0
    // (the ioctl fails on a pipe), so the phase functions get the fill level
    // passed explicitly where a decision needs it
    fn test_port(
        write_fd: libc::c_int,
        target_delay: u32,
        period: u32,
    ) -> crate::node::Port<SinkDir> {
        crate::node::Port {
            config: None,
            buffers: vec![],
            io: std::ptr::null_mut(),
            rate_match: std::ptr::null_mut(),
            dsp: crate::sound::DspWriter::test_on_fd(write_fd, 8),
            dll: Default::default(),
            setup_period: period,
            bw_adapt: Default::default(),
            setup_blocksize: 1024,
            resetup_pending: false,
            was_matching: false,
            warn_limit: crate::utils::RateLimit::new(),
            ext: SinkPortExt {
                target_delay,
                ..Default::default()
            },
        }
    }

    #[test]
    fn target_matches_live_geometry() {
        // the production log shape: granted 65536, blocksize 2048, period 16384
        // -> target delay 20480 (fill_floor binds: period + period/4)
        assert_eq!(target_delay(65536, 16384, 2048, 16384, 0), (20480, false));
        // a fragment wider than the jitter margin takes over the floor
        assert_eq!(fill_floor(16384, 8192), 16384 + 8192);
    }

    // "buffer_required() and target_delay() must derive this identically": any
    // grant that passes the retune gate (buffer_size >= required) must yield a
    // fill target at or above the floor (no starvation) with a full write plus
    // one fragment of wander of headroom above it (no short-write drops)
    #[test]
    fn granted_at_required_never_starves_or_drops() {
        for period in [1024u32, 4096, 16384, 65536] {
            for blocksize in [512u32, 1024, 2047, 2048, 16384, 65536] {
                for write_max in [period, period * 2, period * 4] {
                    for oss_delay in [0u32, 4, 32, 1024] {
                        let desired = desired_delay(period, oss_delay);
                        let required = buffer_required(period, desired, blocksize, write_max);
                        for granted in [required, required + 1, required.saturating_mul(2)] {
                            let (target, _) =
                                target_delay(granted, period, blocksize, write_max, desired);
                            assert!(target >= fill_floor(period, blocksize),
                "starved: target {} < floor {} (granted {}, period {}, blocksize {}, write_max {}, desired {})",
                target, fill_floor(period, blocksize), granted, period, blocksize, write_max, desired);
                            assert!(target.saturating_add(write_max).saturating_add(blocksize) <= granted,
                "will drop: target {target} + write_max {write_max} + blocksize {blocksize} > granted {granted} (period {period}, desired {desired})");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn small_grant_is_best_effort_half() {
        // under two quanta there is no workable geometry; half the ring, and the
        // caller warns
        assert_eq!(target_delay(8192, 16384, 1024, 16384, 0), (4096, false));
    }

    #[test]
    fn oversized_delay_is_capped_and_reported() {
        // oss.delay pushing past the ceiling: clamped to it, flagged for the log
        let (target, capped) = target_delay(65536, 4096, 1024, 4096, u32::MAX);
        assert_eq!(target, 65536 - 4096 - 1024);
        assert!(capped);
    }

    // The recovery sequencing behind the 0.9.7 underrun fix: on the first
    // data cycle past the event, the fill re-primes to target FIRST and the
    // cycle's data follows in the SAME cycle - into a ring that is already
    // near-full, so both writes short-write and must stay frame-aligned
    // while the tail drops as whole frames.
    #[test]
    fn recovery_reprimes_then_writes_into_a_near_full_ring() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);
        port.dsp.write_zeroes(0); // a recovering channel is already running
        port.ext.xrun_timestamp = 1_000;

        // near-full ring: room for the full re-prime (odelay reads 0 on a pipe,
        // so the refill is the whole target) but only half this cycle's buffer
        let capacity = fill_pipe(w);
        free_space(r, 4096 + 1024);

        let data = pattern(2048, 1);
        let n = unsafe { recover_or_hold(&mut port, 2_000, 0, data.as_ptr().cast(), 2048) };

        // the hold cleared and the overfull ring dropped the tail: only the
        // frames that fit after the re-prime were consumed
        assert_eq!(port.ext.xrun_timestamp, 0);
        assert_eq!(n, 1024);
        let out = drain(r);
        assert_eq!(out.len(), capacity); // filler + re-prime zeroes + data head
        let tail = &out[out.len() - 5120..];
        assert!(
            tail[..4096].iter().all(|&b| b == 0),
            "the re-prime must precede the data"
        );
        assert_eq!(&tail[4096..], &data[..1024]);
        unsafe { libc::close(r) };
    }

    // the skip-buffer hold: until the driver clock passes the event (and the
    // host isn't in its own recovery window), buffers are consumed unwritten
    // and the hold stays armed
    #[test]
    fn recovery_holds_buffers_until_the_clock_passes_the_event() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);
        port.dsp.write_zeroes(0);
        port.ext.xrun_timestamp = 5_000;

        let data = pattern(2048, 2);

        // same-cycle clock: not past the event yet
        let n = unsafe { recover_or_hold(&mut port, 5_000, 0, data.as_ptr().cast(), 2048) };
        assert_eq!(n, 2048);
        assert_eq!(port.ext.xrun_timestamp, 5_000);
        assert!(
            drain(r).is_empty(),
            "a held buffer must not reach the device"
        );

        // past the event, but the host flags its own xrun recovery: still held
        let n = unsafe {
            recover_or_hold(
                &mut port,
                6_000,
                SPA_IO_CLOCK_FLAG_XRUN_RECOVER,
                data.as_ptr().cast(),
                2048,
            )
        };
        assert_eq!(n, 2048);
        assert_eq!(port.ext.xrun_timestamp, 5_000);
        assert!(drain(r).is_empty());

        // past the event with no host recovery: re-primes and writes
        let n = unsafe { recover_or_hold(&mut port, 6_000, 0, data.as_ptr().cast(), 2048) };
        assert_eq!(n, 2048);
        assert_eq!(port.ext.xrun_timestamp, 0);
        let out = drain(r);
        assert_eq!(out.len(), 4096 + 2048);
        assert!(out[..4096].iter().all(|&b| b == 0));
        assert_eq!(&out[4096..], &data[..]);
        unsafe { libc::close(r) };
    }

    // the underrun gate arms the recovery hold once per event: the driver
    // clock is snapshotted on the first detection and held cycles must not
    // re-stamp it (odelay reads 0 on a pipe - a truly empty ring)
    #[test]
    fn underrun_detection_arms_the_hold_once() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);
        port.dsp.write_zeroes(0); // the gate runs on a running channel
        port.config = Some(super::PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48000,
            channels: 4,
            positions: vec![],
            flags: 0,
        });
        let callbacks = spa_callbacks {
            funcs: std::ptr::null(),
            data: std::ptr::null_mut(),
        };
        let log = crate::spa::Log::test_null();

        detect_underrun(&mut port, 2048, 3, 1_000_000, 500_000, &callbacks, &log);
        assert_eq!(port.ext.xrun_timestamp, 500_000);

        // a later cycle's count must not move the armed snapshot
        detect_underrun(&mut port, 2048, 5, 2_000_000, 700_000, &callbacks, &log);
        assert_eq!(port.ext.xrun_timestamp, 500_000);
        unsafe { libc::close(r) };
    }

    // the follower fill snap: a level error past one period refills to target
    // on underfill and skips the cycle's buffer on overfill; in-band errors go
    // to the DLL instead
    #[test]
    fn fill_snap_refills_underfill_and_skips_overfill() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);

        // underfill past one period: refill to target, don't skip
        let (corr, skip) = follower_servo(&mut port, 1024, 8, 0);
        assert_eq!(corr, 1.0);
        assert!(!skip);
        let out = drain(r);
        assert_eq!(out.len(), 4096 - 1024);
        assert!(out.iter().all(|&b| b == 0));

        // overfill past one period: skip the buffer, write nothing (the device
        // drains toward target meanwhile)
        // target + one period + one frame: just past the snap threshold
        let (corr, skip) = follower_servo(&mut port, 4096 + 2048 + 8, 8, 0);
        assert_eq!(corr, 1.0);
        assert!(skip);
        assert!(drain(r).is_empty());

        // in-band error: no snap, the DLL absorbs it. With a negotiated
        // config the geometry latches and the DLL engages: the first update
        // cold-starts the gains, the second produces a real correction
        port.config = Some(super::PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48000,
            channels: 4,
            positions: vec![],
            flags: 0,
        });
        super::commit_geometry(&mut port, 65536, 2048, 1024, 2048, 4096);
        port.setup_period = 2048;
        port.ext.target_delay = 4096;
        follower_servo(&mut port, 4096 + 512, 8, 1);
        port.was_matching = true; // the caller latches this after each cycle
        let (corr, skip) = follower_servo(&mut port, 4096 + 512, 8, 2);
        assert!(!skip);
        assert!((0.9..=1.1).contains(&corr));
        assert!(corr != 1.0, "the DLL never engaged");
        assert!(drain(r).is_empty());
        unsafe { libc::close(r) };
    }

    // the same-device follower's level correction snaps the same way, without
    // a DLL to relock
    #[test]
    fn same_device_level_correct_snaps_the_fill() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);

        assert!(!level_correct(&mut port, 4096)); // on target: nothing to do
        assert!(drain(r).is_empty());
        assert!(level_correct(&mut port, 4096 + 2049)); // overfill: drain a cycle
        assert!(drain(r).is_empty());
        assert!(!level_correct(&mut port, 1024)); // underfill: refill to target
        let out = drain(r);
        assert_eq!(out.len(), 4096 - 1024);
        assert!(out.iter().all(|&b| b == 0));
        unsafe { libc::close(r) };
    }

    // the in-place retune: a sustained period change with enough ring
    // headroom recommits the geometry and snaps the fill to the new target
    #[test]
    fn retune_recommits_in_place_and_snaps_the_fill() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);
        port.dsp.write_zeroes(0); // a retuning channel is running
        port.ext.buffer_size = 16384;
        let log = crate::spa::Log::test_null();

        // one flip is debounced: write at the old geometry for a cycle
        assert!(!retune_period(&mut port, 4096, 8, 0, &log));
        assert_eq!(port.setup_period, 2048);
        assert!(drain(r).is_empty());

        // sustained: retune in place and fill to the new target (odelay
        // reads 0 on a pipe, so the snap writes the whole target)
        assert!(!retune_period(&mut port, 4096, 8, 0, &log));
        assert_eq!(port.setup_period, 4096);
        assert_eq!(port.ext.target_delay, 5120); // fill_floor(4096, 1024) binds
        assert_eq!(port.ext.period_mismatch, 0);
        let out = drain(r);
        assert_eq!(out.len(), 5120);
        assert!(out.iter().all(|&b| b == 0));
        unsafe { libc::close(r) };
    }

    // a ring too small for the new period wants a trigger suspend; the pipe
    // refuses the ioctl (the dying-fd model), so retune asks for a rebuild
    // and keeps the debounce counter armed for an immediate retry
    #[test]
    fn retune_requests_rebuild_when_the_suspend_is_refused() {
        let (r, w) = pipe_pair(true, true);
        let mut port = test_port(w, 4096, 2048);
        port.dsp.write_zeroes(0);
        let log = crate::spa::Log::test_null();

        assert!(!retune_period(&mut port, 4096, 8, 0, &log));
        assert!(retune_period(&mut port, 4096, 8, 0, &log));
        assert_eq!(port.setup_period, 2048); // untouched; the rebuild replaces the device
        assert!(port.ext.period_mismatch >= 2);
        assert!(drain(r).is_empty());
        unsafe { libc::close(r) };
    }

    #[test]
    fn request_covers_the_largest_negotiable_quantum() {
        // the prime-time request holds the stable floor so later period changes
        // retune in place; the kernel cap always wins
        let cap = crate::sound::ring_byte_cap(8, 48000);
        let req = buffer_request(4096, 16384, cap, 0, 2048, 4096, 4);
        assert!(req >= buffer_required(16384, desired_delay(16384, 4), 2048, 16384));
        assert!(req >= crate::sound::MIN_RING_BYTES.min(cap));
        assert!(req <= cap);
    }
}
