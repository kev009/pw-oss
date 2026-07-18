use std::os::raw::c_int;

use libspa::sys::*;

use crate::node::{Direction, ParamBuild, State, MAX_PORTS};

// several State fields are per-port in disguise (rate_match, active_buffers,
// the single PortInfo); fix those before raising this
const _: () = assert!(MAX_PORTS == 1);
const EMPTY_CYCLE: isize = -1; // no data queued this cycle (scheduling jitter)

pub(crate) enum SourceDir {}

// direction-specific State fields (State.ext)
#[derive(Default)]
pub(crate) struct SourceExt {
    pub active_buffers: usize,
}

// consecutive overrun-ticking cycles with the ring pinned at the ceiling
// before recovery re-primes; gives the catch-up read a chance to drain a
// transient first (it clears the pin in one cycle when the buffer allows)
const PINNED_CYCLE_LIMIT: u32 = 3;

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SourcePortExt {
    pub primed: bool,
    pub target_fill: u32,     // servo fill target; a period plus half an arrival
    pub read_peak: u32,       // catch-up threshold, capped by the granted ring
    pub ring_size: u32,       // granted soft ring in bytes (GETISPACE totals; 0 = unknown)
    pub pinned_cycles: u32,   // consecutive overrun ticks with the ring pinned full
    pub period_mismatch: u32, // consecutive cycles at a different period (debounce)
    pub was_freewheeling: bool, // freewheel active last cycle (re-prime on exit)
}

// The servo fill target and the catch-up threshold, shared by the prime and
// re-tune paths.
//
// Target: one period plus HALF AN ARRIVAL of bottom margin. Queued readings
// move in whole arrivals (the granted fragment, or the parent channel's
// hardware drain quantum on vchans - 4 ms lumps on a RODECaster), so a bare
// one-period target bottoms the sawtooth at exactly zero and any negative
// wakeup jitter finds an empty ring: a silent one-period hole in the
// recording (the EMPTY_CYCLE path). The added latency is physics - data
// arriving every N ms can't be delivered with a sub-N/2 margin.
//
// Peak: half an arrival plus half a period of slack above target, but capped
// one arrival under the granted ring. The kernel silently clamps the ring at
// CHN_2NDBUFMAXSIZE; on fat strides the uncapped peak lands past the ring
// end and the catch-up read goes dead - any excess then parks at the ceiling
// (the bounded read only ever takes a period, and the +/-1% rate match needs
// ~a second to bleed one period), where every late cycle overruns again. The
// floor keeps routine arrival wander (one lump) from triggering catch-up
// reads that fight the servo.
fn fill_targets(period: u32, blocksize: u32, ring: u32) -> (u32, u32) {
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

// The prime-time ring request: four periods of overrun slack, floored at
// four periods of the LARGEST negotiable quantum (max_period comes from
// sound::max_ring_period_bytes - the shared policy behind the sink's stable
// floor and the advertised node.max-latency), never below MIN_RING_BYTES,
// never above the kernel cap (which always wins). Capacity is not latency:
// target_fill still controls capture latency, while a ring sized for every
// negotiable quantum lets period changes retune in place instead of
// stopping the channel to resize.
fn ring_request(period: u32, max_period: u32, cap: u32) -> u32 {
    period
        .saturating_mul(4)
        .max(max_period.saturating_mul(4))
        .max(crate::sound::MIN_RING_BYTES)
        .min(cap)
}

// the ring a non-degenerate capture geometry needs: the fill target plus a
// catch-up band (peak >= target + one arrival) plus one arrival of top
// headroom, floored at the two-quanta structural bound. The prime warning
// and the in-place retune gate both key on this; below it fill_targets
// pins the peak at (or under) the target and the catch-up read fights the
// servo on every arrival.
fn ring_required(period: u32, blocksize: u32) -> u32 {
    let (target, _) = fill_targets(period, blocksize, 0);
    target
        .saturating_add(blocksize.saturating_mul(2))
        .max(period.saturating_mul(2))
}

// the shared geometry-commit tail of the prime and in-place retune paths:
// fill targets for `period` against the granted ring, and a servo relock
fn commit_geometry(port: &mut crate::node::Port<SourceDir>, period: u32, blocksize: u32) {
    let (target, peak) = fill_targets(period, blocksize, port.ext.ring_size);
    port.setup_period = period;
    port.setup_blocksize = blocksize;
    port.ext.target_fill = target;
    port.ext.read_peak = peak;
    port.dll.init();
    port.bw_adapt.reset(); // cold-starts at the granularity cap next servo cycle
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
fn retune_period(
    port: &mut crate::node::Port<SourceDir>,
    period_in_bytes: u32,
    log: &crate::spa::Log,
) -> bool {
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
    port.ext.period_mismatch = 0;
    // cached blocksize: the triggered channel refuses SETFRAGMENT and the
    // hw cadence is per-session, so the grant can't have changed since
    // prime (the sink reuses its cache for the same reason)
    let blocksize = port.setup_blocksize;
    if port.ext.ring_size >= ring_required(period_in_bytes, blocksize) {
        commit_geometry(port, period_in_bytes, blocksize);
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
        port.ext.primed = false;
        false
    } else {
        true
    }
}

// The prime phase - the capture analogue of the sink's zero priming: trigger
// the device, discard any backlog so the fill level starts out known, and
// hand the graph one period of silence while the ring fills. Don't wait for
// real data: an empty first cycle reads as a missed deadline to the graph.
// Re-apply the fragment layout while the channel is in setup (legal after a
// trigger suspend too, so live oss.fragment changes reach a suspended
// source). Returns the cycle's byte count (the period of silence), or
// EMPTY_CYCLE before a format is negotiated (unreachable past the caller's
// gate).
unsafe fn prime_capture(
    port: &mut crate::node::Port<SourceDir>,
    period_in_bytes: u32,
    graph_rate: u32,
    oss_fragment: u32,
    data: *mut std::os::raw::c_void,
    maxsize: u32,
    log: &crate::spa::Log,
) -> isize {
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
        let frag = if oss_fragment == 0 {
            1024
        } else {
            oss_fragment.min(cap)
        };
        port.dsp.set_small_fragments(
            frag,
            ring_request(
                period_in_bytes,
                crate::sound::max_ring_period_bytes(stride, rate, graph_rate),
                crate::sound::ring_byte_cap(stride, rate),
            ),
        );
    }
    let ready = port.dsp.ready_for_reading(0);
    // one GETISPACE serves the backlog, the granted fragment and the ring
    // total: they come from the same struct, and the layout fields are
    // final now that ready_for_reading has triggered the channel. The
    // ACTUAL grant, not the request - the kernel clamps the ring silently,
    // and the fill geometry must fit reality.
    let (backlog, fragsize, ring) = port.dsp.ispace_layout();
    if ready {
        let mut backlog = backlog;
        while backlog > 0 {
            // whole frames only: GETISPACE is byte-granular and can sit
            // mid-frame; an unaligned read would tear every later sample
            let chunk = (backlog.min(maxsize) / stride) * stride;
            if chunk == 0 {
                break; // a sub-frame tail; it completes into a frame later
            }
            let n = port.dsp.read(data, chunk as usize);
            if n <= 0 {
                break;
            }
            backlog -= n as u32;
        }
    }
    port.ext.primed = true;
    port.ext.ring_size = ring;
    // the measurement/arrival quantum is the granted fragment or the
    // hardware cadence sndstat reports, whichever is coarser (see the
    // sink); data arrives in these lumps regardless of the soft fragment
    let chunk = crate::utils::ns_to_frame_bytes(port.dsp.hw_quantum_ns, rate, stride);
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
    std::ptr::write_bytes(data.cast::<u8>(), 0, len as usize);
    len as isize
}

#[derive(Debug, Clone)]
pub struct PortConfig {
    pub format: libspa::param::audio::AudioFormat,
    pub rate: u32,
    pub channels: u32,
    pub positions: Vec<u32>, // the negotiated channel positions, replayed in the Format readback
    pub flags: u32,
    pub stride: u32,
}

impl PortConfig {
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
        self.stride
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

// The follower-servo phase, matching a foreign clock: the DLL serves rate
// matching only (when driving, the servo runs in on_timeout where the clock
// is published; a same-device follower has nothing to correct). `queued` is
// the pre-read fill the caller measured this cycle. Returns the rate
// correction.
fn follower_servo(
    port: &mut crate::node::Port<SourceDir>,
    queued: u32,
    now: u64,
    stride: u32,
    rate: u32,
) -> f64 {
    let mut corr: f64 = 1.0;
    if !port.was_matching {
        // matching just engaged; relock rather than apply stale state
        port.dll.init();
        port.bw_adapt.reset();
    }
    // capture error is inverted vs the sink: a slow device queues less
    let err_raw = port.ext.target_fill as f64 - queued as f64;
    // the healthy swing is half an arrival either side of target; a
    // snap threshold below that resets the DLL on every arrival
    let snap = port
        .setup_period
        .max(port.setup_blocksize / 2 + port.setup_period / 2);
    if err_raw.abs() > snap as f64 {
        // fill snap (see the sink): a level error past one period would
        // wind the integrator against the +/-1% clamp; the bounded read
        // drains genuine backlog, so just relock here
        port.dll.init();
        port.bw_adapt.reset();
    } else {
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = err_raw.clamp(-max_err, max_err);
        corr = port.dll.update(err);
        port.bw_adapt.update(
            &mut port.dll,
            err,
            stride,
            port.setup_blocksize,
            now,
            port.setup_period,
            rate * stride,
        );
    }

    #[cfg(debug_assertions)]
    eprintln!("capture: corr = {}, err = {}", corr, err_raw);

    corr
}

// The read-tail phase. Bounded read: one period, plus only the backlog
// beyond the healthy peak (genuine catch-up). Draining everything each
// cycle turns consumer backpressure into a permanent extra period of
// latency (an oversized chunk holds io.status HAVE_DATA, we skip the device
// next cycle, it queues 2 periods, repeat) and pollutes the servo error. If
// the device is late, keep the graph timeline stable: read only queued
// bytes from the blocking fd and zero-pad the rest of the period instead of
// returning an empty or short cycle.
unsafe fn bounded_read(
    port: &mut crate::node::Port<SourceDir>,
    queued: u32,
    data: *mut std::os::raw::c_void,
    maxsize: u32,
    stride: u32,
) -> isize {
    let want = if port.setup_period != 0 {
        // catch-up beyond the healthy peak (fill_targets: target plus slack,
        // capped by the granted ring so a fill at the ceiling is drainable);
        // the servo handles the rest without a pegged error, and a threshold
        // under the arrival peak would drag the fill below target on every
        // arrival
        port.setup_period
            .saturating_add(queued.saturating_sub(port.ext.read_peak))
    } else {
        queued
    };
    // floored to whole frames: `queued` is byte-granular and can sit
    // mid-frame; an unaligned read would start the next buffer mid-sample
    let ispace = (want.min(queued).min(maxsize) / stride) * stride;
    let nread = if ispace > 0 {
        port.dsp.read(data, ispace as usize).max(0) as u32
    } else {
        0
    };
    let period = port.setup_period.min(maxsize);
    let out = if period > 0 { nread.max(period) } else { nread };
    if out > nread {
        std::ptr::write_bytes(
            data.cast::<u8>().add(nread as usize),
            0,
            (out - nread) as usize,
        );
    }
    out as isize
}

// The overrun phase. A rec overrun means chn_rdfeed found the soft ring
// full at interrupt time and DISPOSED the hardware lump UPSTREAM of us -
// our queued fill is intact and already bounded by the ring, so the counter
// alone is not corrupted state (the sink learned the same about vchan
// underrun accounting). Re-priming on every tick amplified a 4 ms kernel
// drop into a 20+ ms skip (backlog discard, a period of silence, a DLL
// relock whose overshoot re-tripped the ceiling at a ~1.3 s cadence).
// Recovery is only warranted when the ring stays PINNED at the ceiling
// across consecutive cycles - i.e. the catch-up read can't drain it
// (consumer stall, graph buffer smaller than the backlog, wedged reads).
// The freewheel branch never triggers the device (it may still be in
// setup), and while freewheeling the ring overruns by design - the exit
// edge re-primes, so don't flood the counter meanwhile.
unsafe fn recover_overrun(
    port: &mut crate::node::Port<SourceDir>,
    freewheel: bool,
    pre_read_fill: Option<u32>,
    data_system: &crate::spa::System,
    callbacks: &spa_callbacks,
    log: &crate::spa::Log,
) {
    let overrun_count = if port.dsp.is_running() && !freewheel {
        port.dsp.overruns()
    } else {
        0
    };
    if overrun_count == 0 {
        port.ext.pinned_cycles = 0;
        return;
    }
    // pinned = pre-read fill within one arrival of the ring end; with an
    // unknown ring treat every tick as pinned (can't gate what we can't
    // measure). A cycle without a pre-read sample (prime/freewheel) just
    // cleared the ring, so the state is fresh by construction.
    let pinned = match (pre_read_fill, port.ext.ring_size) {
        (Some(fill), ring) if ring > 0 => fill > ring.saturating_sub(port.setup_blocksize),
        (Some(_), _) => true,
        (None, _) => false,
    };
    port.ext.pinned_cycles = if pinned {
        port.ext.pinned_cycles + 1
    } else {
        0
    };
    if port.ext.pinned_cycles >= PINNED_CYCLE_LIMIT {
        port.ext.pinned_cycles = 0;
        let now = crate::utils::now_ns(data_system);
        if let Some(suppressed) = port.warn_limit.check(now) {
            crate::warn!(log, "OSS reported {:3} overruns @ {} with the ring pinned; re-priming (+{} warnings suppressed)",
        overrun_count, now, suppressed);
        }
        // only for real recovery, not per ignored tick
        crate::node::emit_xrun(callbacks, now / 1000);

        // recover like the sink's underrun path: re-enter priming next cycle,
        // which drains the backlog and relocks the DLL - otherwise the
        // un-drained backlog becomes permanent capture latency while the
        // integrator winds up against an error the reads can't remove.
        // Trigger-suspend first so the re-prime's SETFRAGMENT can also
        // RESIZE the ring: a pinned ring may be one the current quantum
        // outgrew, and a Running channel silently skips the layout
        // re-application (a refused suspend just re-primes at the old size).
        let _ = port.dsp.suspend();
        port.ext.primed = false;
        port.bw_adapt.reset();
        port.dll.init();
    } else {
        // suppressed counts stay diagnosable (see the sink's underrun gate)
        crate::debug!(
            log,
            "{} overrun counts ignored (kernel disposed upstream; fill {:?} of ring {})",
            overrun_count,
            pre_read_fill,
            port.ext.ring_size
        );
    }
}

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time). The pre-read fill level
// here and process()'s post-drain accounting see the same signal: we drain
// the ring every cycle, so what's queued is one period's accumulation.
unsafe fn timeout_servo(state: &mut State<SourceDir>, nsec: u64, rate: u32) -> (f64, i64) {
    let mut corr: f64 = 1.0;
    let mut delay: i64 = 0;
    for port in &mut state.ports {
        let Some(cfg) = port.config.as_ref() else {
            continue;
        };
        let stride = cfg.stride.max(1);
        let device_rate = cfg.rate.max(1);
        if !port.dsp.is_running()
            || !port.ext.primed
            || port.setup_period == 0
            || port.resetup_pending
        {
            continue;
        }

        let queued = port.dsp.ispace_in_bytes().max(0) as u32;
        // device frames scale to the graph rate; the resampler queue is already
        // graph-side (matching the sink's publication)
        let resamp = if state.rate_match.is_null() {
            0
        } else {
            (*state.rate_match).delay as i64
        };
        delay = (queued / stride) as i64 * rate as i64 / device_rate as i64 + resamp;

        // capture error is inverted vs the sink: a slow device queues less than a
        // period; clamp so wakeup jitter can't wind up the integrator
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = (port.ext.target_fill as f64 - queued as f64).clamp(-max_err, max_err);
        corr = port.dll.update(err);
        port.bw_adapt.update(
            &mut port.dll,
            err,
            stride,
            port.setup_blocksize,
            nsec,
            port.setup_period,
            device_rate * stride,
        );

        // a diverged servo must not wedge the graph clock
        if !(0.5..=2.0).contains(&corr) {
            crate::warn!(state.log, "capture DLL diverged (corr {}); relocking", corr);
            port.dll.init();
            port.bw_adapt.reset();
            corr = 1.0;
        }

        #[cfg(debug_assertions)]
        eprintln!("capture: corr = {}, queued = {}", corr, queued);
    }
    (corr, delay)
}

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(
    dsp: &mut crate::sound::Dsp,
    config: &PortConfig,
    fragment: u32,
    log: &crate::spa::Log,
) -> c_int {
    // a busy or vanished device must fail negotiation, not abort
    if let Err(err) = dsp.open() {
        crate::warn!(log, "dsp open: {}", err);
        return -(err as c_int);
    }
    // ditto for a device that won't take the format exactly
    if let Err(err) = dsp.configure(config.oss_format(), config.channels, config.rate) {
        crate::warn!(log, "device rejected {:?}: {}", config, err);
        dsp.close();
        return -(err as c_int);
    }
    // on direct opens the hardware blocksize is per-session state; re-read it
    // now that THIS configuration is in effect (vchan/uaudio values are stable)
    dsp.refresh_hw_quantum();
    dsp.set_small_fragments(fragment, crate::sound::MIN_RING_BYTES); // normalized oss.fragment (0 = 1 KiB default)
    0
}

unsafe fn process_ports(state: &mut State<SourceDir>) -> c_int {
    let mut result = SPA_STATUS_OK as i32;
    let state_ptr: *mut State<SourceDir> = state;

    for (port_idx, port) in state.ports.iter_mut().enumerate() {
        let Some((stride, rate)) = port.stride_rate() else {
            continue; // no format negotiated yet
        };

        if port.buffers.is_empty() || port.io.is_null() {
            continue; // not (fully) negotiated yet
        }

        if port.resetup_pending {
            continue; // the main thread is rebuilding the device
        }

        if port.dsp.is_closed() {
            // Suspend closed the device but the host restarted without a fresh
            // format; rebuild off-loop instead of tripping the dsp state asserts
            port.resetup_pending = crate::node::queue_resetup(state_ptr, port_idx);
            continue;
        }

        if (*port.io).status == SPA_STATUS_HAVE_DATA as i32 {
            // a pending buffer the peer hasn't consumed yet: report HAVE_DATA, or
            // the adapter treats the cycle as empty (alsa-pcm-source.c does this)
            result |= SPA_STATUS_HAVE_DATA as i32;
            continue;
        }
        if (*port.io).status != SPA_STATUS_OK as i32
            && (*port.io).status != SPA_STATUS_NEED_DATA as i32
        {
            continue;
        }

        let buffer_id = if (*port.io).buffer_id == -1i32 as u32 {
            // hand out the next never-used buffer; the host returns ids after that
            let idx = state.ext.active_buffers;
            state.ext.active_buffers += 1;
            idx as u32
        } else {
            (*port.io).buffer_id
        };

        // buffer_id (or our fallback index) and n_datas come from outside. Validate
        // them instead of asserting; a panic here aborts the process across extern "C".
        let buffer = match port
            .buffers
            .get(buffer_id as usize)
            .copied()
            .and_then(|b| b.as_ref())
        {
            Some(b) if b.n_datas == 1 => b, // we fill the block directly, so need exactly one
            _ => {
                crate::warn!(state.log, "unusable buffer (id {}); skipping", buffer_id);
                continue;
            }
        };

        // we read straight into the block, so require a MemPtr with data, chunk and
        // maxsize all valid. as_ref() (not offset(0)) handles a null datas pointer.
        let data_0 = match buffer.datas.as_ref() {
            Some(d)
                if d.type_ == SPA_DATA_MemPtr
                    && !d.data.is_null()
                    && !d.chunk.is_null()
                    && d.maxsize > 0 =>
            {
                d
            }
            _ => {
                crate::warn!(
                    state.log,
                    "buffer data is not a usable MemPtr block; skipping"
                );
                continue;
            }
        };

        let matching =
            state.following && !crate::utils::same_clock(state.position, &state.clock_name);

        let mut corr: f64 = 1.0; // DLL rate correction for the follower rate match

        // one period in device bytes (0 while position is absent)
        let mut period_in_bytes = 0u32;
        let mut graph_rate = 0u32;
        if !state.position.is_null() {
            let driver_clock = (*state.position).clock;
            if driver_clock.target_rate.denom > 0 {
                graph_rate = driver_clock.target_rate.denom;
                period_in_bytes = crate::utils::device_period_bytes(
                    driver_clock.target_duration,
                    rate,
                    graph_rate,
                    stride,
                );
            }
        }

        if retune_period(port, period_in_bytes, &state.log) {
            // the driver refused the trigger stop (dying fd): rebuild off-loop
            // rather than commit a fill target the current ring can't hold; if
            // even that fails (no main loop), keep running at the stale
            // geometry - degraded, but nothing stalls
            port.resetup_pending = crate::node::queue_resetup(state_ptr, port_idx);
            if port.resetup_pending {
                continue;
            }
        }

        let freewheel = !state.position.is_null()
            && (*state.position).clock.flags & SPA_IO_CLOCK_FLAG_FREEWHEEL != 0;

        // realtime resumed after freewheeling: the ring overflowed by design
        // while reads were skipped, so re-prime explicitly for a known fill.
        // (This used to lean on the overrun counter, which the gate below now
        // deliberately ignores while the ring state is sane.)
        if port.ext.was_freewheeling && !freewheel {
            port.ext.primed = false;
        }
        port.ext.was_freewheeling = freewheel;

        // pre-read fill this cycle, where the read path sampled it; the overrun
        // gate below needs the level BEFORE the read (a post-read reading is
        // near-empty on every healthy cycle and would gate out real wedges)
        let mut pre_read_fill: Option<u32> = None;

        let nbytes = if freewheel && period_in_bytes > 0 {
            // freewheeling: hand out silence without touching the device (ALSA
            // skips its reads); the ring overflows meanwhile and the exit edge
            // above re-primes when realtime resumes
            let len = period_in_bytes.min(data_0.maxsize);
            std::ptr::write_bytes(data_0.data.cast::<u8>(), 0, len as usize);
            len as isize
        } else if !port.ext.primed && period_in_bytes > 0 {
            prime_capture(
                port,
                period_in_bytes,
                graph_rate,
                state.oss_fragment,
                data_0.data,
                data_0.maxsize,
                &state.log,
            )
        } else if !port.dsp.is_running() {
            // un-primed and no usable position yet (the prime branch needs a
            // period): the device is still in setup, where the space ioctls assert
            EMPTY_CYCLE
        } else {
            // Gate on the queued byte count, not poll: the kernel's poll trigger
            // is one full fragment, which can exceed a small graph period - every
            // read (and the servo error) would then be biased by a fragment. The
            // priming pass already triggered the channel; GETISPACE doesn't need
            // the trigger.
            let queued = port.dsp.ispace_in_bytes().max(0) as u32;
            pre_read_fill = Some(queued);
            if queued == 0 {
                crate::debug!(state.log, "capture: empty cycle (no data queued at wakeup)");
            }

            // A debounce hold cycle runs at stale geometry - don't feed its
            // transitional error to the DLL (the sink gates the same way).
            if matching
                && period_in_bytes > 0
                && port.setup_period != 0
                && port.ext.period_mismatch == 0
            {
                corr = follower_servo(
                    port,
                    queued,
                    crate::utils::now_ns(&state.data_system),
                    stride,
                    rate,
                );
            }

            bounded_read(port, queued, data_0.data, data_0.maxsize, stride)
        };

        // Rate-match only as a follower on a foreign clock: when driving, the
        // timer steering applies the correction, and a same-device follower ticks
        // from our clock so there is nothing to match (ALSA gates on the clock
        // name the same way).
        port.was_matching = matching;
        // Realtime capture cycles are period-padded if the device is late; keep
        // rate matching coherent with the buffer we handed to the graph.
        if nbytes >= 0 && !state.rate_match.is_null() {
            if matching {
                (*state.rate_match).flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                (*state.rate_match).rate = (1.0 / corr).clamp(0.99, 1.01);
            } else {
                (*state.rate_match).flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                (*state.rate_match).rate = 1.0;
            }
        }

        recover_overrun(
            port,
            freewheel,
            pre_read_fill,
            &state.data_system,
            &state.callbacks,
            &state.log,
        );

        if nbytes != -1 {
            #[cfg(debug_assertions)]
            if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
                crate::trace!(state.log, "nbytes: {}", nbytes);
                spa_debug_mem(0, data_0.data, 16.min(nbytes) as usize);
            }

            (*data_0.chunk).offset = 0;
            (*data_0.chunk).size = nbytes as u32;
            (*data_0.chunk).stride = stride as i32;
            (*data_0.chunk).flags = 0;

            (*port.io).buffer_id = buffer_id;
            (*port.io).status = SPA_STATUS_HAVE_DATA as i32;

            result |= SPA_STATUS_HAVE_DATA as i32;
        } else {
            (*port.io).buffer_id = buffer_id; // -1i32 as u32;
            (*port.io).status = SPA_STATUS_OK as i32;
        }
    }

    result
}

impl Direction for SourceDir {
    const DIRECTION: spa_direction = SPA_DIRECTION_OUTPUT;
    const PLAYBACK: bool = false;
    const MEDIA_CLASS: &'static str = "Audio/Source";
    // a capture driver signals HAVE_DATA (alsa-pcm.c capture_ready); the
    // NEED_DATA form is for playback drivers
    const READY_STATUS: i32 = SPA_STATUS_HAVE_DATA as i32;
    const CMD_WARN_PREFIX: &'static str = "oss-source: ";

    type Device = crate::sound::Dsp;
    type Config = PortConfig;
    type Ext = SourceExt;
    type PortExt = SourcePortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        std::ptr::NonNull::new(&raw mut OSS_SOURCE_TOPIC).expect("a static's address is never null")
    }

    fn info_item(_ext: &mut SourceExt, _key: &str, _value: &str) {}

    fn ext_ready(_ext: &mut SourceExt) {}

    unsafe fn build_node_param(
        state: &mut State<SourceDir>,
        b: &mut libspa::pod::builder::Builder,
        id: u32,
        index: u32,
    ) -> ParamBuild {
        #[allow(non_upper_case_globals)]
        match (id, index) {
            (SPA_PARAM_PropInfo, 0) => crate::utils::build_latency_offset_prop_info(b).unwrap(),
            (SPA_PARAM_PropInfo, 1) => crate::utils::build_params_prop_info(
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
                &[(crate::keys::OSS_FRAGMENT, state.oss_fragment)],
            )
            .unwrap(),
            (SPA_PARAM_Props, _) => return ParamBuild::Exhausted,
            (SPA_PARAM_ProcessLatency, 0) => {
                crate::utils::build_process_latency_info(b, &state.process_latency).unwrap()
            }
            (SPA_PARAM_ProcessLatency, _) => return ParamBuild::Exhausted,
            _ => return ParamBuild::Unknown,
        };
        ParamBuild::Built
    }

    // a NULL Props pod resets the props to their defaults and re-applies them
    unsafe fn reset_props(state: &mut State<SourceDir>) -> c_int {
        let res = crate::node::store_and_rebuild(state, |state| {
            state.oss_fragment = state.oss_fragment_default;
        });
        if res != 0 {
            return res;
        }
        crate::node::handle_process_latency(state, crate::utils::process_latency_default());
        0
    }

    unsafe fn set_props_params(state: &mut State<SourceDir>, value: &libspa::pod::Value) -> c_int {
        use libspa::pod::Value;
        match value {
            Value::Struct(values) if values.len() % 2 == 0 => {
                for kv in values.chunks(2) {
                    match (&kv[0], &kv[1]) {
                        // pw-cli set-param <object-id> Props '{ "params": ["oss.fragment", 4096]}'
                        (Value::String(s), Value::Int(x))
                            if s == crate::keys::OSS_FRAGMENT && *x >= 0 =>
                        {
                            // stored normalized, so the Props readback reports the
                            // effective (rounded/clamped) value, not the raw request
                            let new_fragment = crate::node::normalize_fragment(*x as u32);
                            if new_fragment != state.oss_fragment {
                                // unchanged echoes must not rebuild a running device
                                let res = crate::node::apply_props_param(state, move |state| {
                                    state.oss_fragment = new_fragment
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
        state: &mut State<SourceDir>,
        raw: &spa_audio_info_raw,
    ) -> Result<PortConfig, c_int> {
        let format = libspa::param::audio::AudioFormat(raw.format);

        // only formats from our EnumFormat are expected; reject the rest
        let (oss_format, bytes_per_sample) = match format {
            libspa::param::audio::AudioFormat::S32LE => (crate::sound::AFMT_S32_LE, 4),
            libspa::param::audio::AudioFormat::S32BE => (crate::sound::AFMT_S32_BE, 4),
            libspa::param::audio::AudioFormat::S16LE => (crate::sound::AFMT_S16_LE, 2),
            libspa::param::audio::AudioFormat::S16BE => (crate::sound::AFMT_S16_BE, 2),
            _ => {
                crate::warn!(state.log, "rejecting unsupported format {:?}", format);
                return Err(-libc::ENOTSUP);
            }
        };

        let config = PortConfig {
            format,
            rate: raw.rate,
            channels: raw.channels,
            positions: raw.position[..raw.channels as usize].to_vec(),
            flags: raw.flags,
            stride: bytes_per_sample * raw.channels, // bytes per interleaved frame
        };

        crate::debug!(state.log, "reconfiguring with {:?}", config);

        let _ = oss_format;
        Ok(config)
    }

    fn try_open_configure(
        dsp: &mut crate::sound::Dsp,
        config: &PortConfig,
        fragment: u32,
        log: &crate::spa::Log,
    ) -> c_int {
        try_open_configure(dsp, config, fragment, log)
    }

    unsafe fn on_device_swapped(state: &mut State<SourceDir>, port_idx: usize) {
        let port = &mut state.ports[port_idx];
        port.dll.init(); // fresh device, fresh servo
        port.ext.primed = false;
        state.ext.active_buffers = 0;
    }

    unsafe fn on_buffers_swapped(state: &mut State<SourceDir>) {
        state.ext.active_buffers = 0;
    }

    unsafe fn on_start_loop(state: &mut State<SourceDir>) {
        // the device kept capturing across a Pause; re-prime so the first
        // cycles deliver fresh audio at a known fill, not the paused backlog
        for port in &mut state.ports {
            port.ext.primed = false;
            port.dll.init();
            port.bw_adapt.reset();
        }
    }

    unsafe fn on_suspend_loop(state: &mut State<SourceDir>) {
        for port in &mut state.ports {
            port.ext.primed = false; // resume re-primes for a known fill
        }
    }

    unsafe fn on_role_flip(_state: &mut State<SourceDir>) {}

    // data loop only
    unsafe fn update_timers(state: &mut State<SourceDir>) {
        #[cfg(debug_assertions)]
        crate::trace!(state.log, "update_timers");

        let mut now = timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        let err = state
            .data_system
            .clock_gettime(libc::CLOCK_MONOTONIC, &mut now);
        assert!(err >= 0);

        state.next_time = (now.tv_sec * SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64;

        if state.started && !state.following && !state.position.is_null() {
            #[cfg(debug_assertions)]
            crate::trace!(state.log, "next time {}", state.next_time);
            crate::node::set_timeout(state, state.next_time);
        } else {
            #[cfg(debug_assertions)]
            crate::trace!(state.log, "next time {}", 0);
            crate::node::set_timeout(state, 0);
        }
    }

    unsafe fn debug_cycle(_state: &State<SourceDir>, _now: u64, _nsec: u64) {}

    unsafe fn timeout_servo(state: &mut State<SourceDir>, nsec: u64, rate: u32) -> (f64, i64) {
        timeout_servo(state, nsec, rate)
    }

    unsafe fn process_ports(state: &mut State<SourceDir>) -> c_int {
        process_ports(state)
    }
}

const OSS_SOURCE_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub const OSS_SOURCE_FACTORY: spa_handle_factory = spa_handle_factory {
    version: SPA_VERSION_HANDLE_FACTORY,
    name: c"freebsd-oss.source".as_ptr(),
    info: &OSS_SOURCE_FACTORY_INFO,
    get_size: Some(crate::node::get_size::<SourceDir>),
    init: Some(crate::node::init::<SourceDir>),
    enum_interface_info: Some(crate::node::enum_interface_info),
};

// mut: the host logger writes level/has_custom_level back after registration
pub(crate) static mut OSS_SOURCE_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: c"spa.oss.source".as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};

#[cfg(test)]
mod tests {
    use super::{
        bounded_read, fill_targets, follower_servo, retune_period, ring_request, ring_required,
        SourceDir, SourcePortExt,
    };
    use crate::sound::test_util::{pattern, pipe_pair};

    // a Port on a pipe-backed device: the pipe plays the capture ring
    // (byte-exact accounting), GETISPACE fails on a pipe, so the phase
    // functions get the queued fill passed explicitly (as the callers do)
    fn test_port(
        read_fd: libc::c_int,
        period: u32,
        read_peak: u32,
    ) -> crate::node::Port<SourceDir> {
        crate::node::Port {
            config: None,
            buffers: vec![],
            io: std::ptr::null_mut(),
            dsp: crate::sound::Dsp::test_on_fd(read_fd, 8),
            dll: Default::default(),
            setup_period: period,
            bw_adapt: Default::default(),
            setup_blocksize: 1024,
            resetup_pending: false,
            was_matching: false,
            warn_limit: crate::utils::RateLimit::new(),
            ext: SourcePortExt {
                read_peak,
                ..Default::default()
            },
        }
    }

    // the read tail on a pipe-backed device: catch-up reads only the backlog
    // beyond the healthy peak, and a late cycle (nothing queued) still hands
    // the graph a whole period of silence
    #[test]
    fn bounded_read_caps_catchup_and_pads_late_cycles() {
        let (r, w) = pipe_pair(false, false);
        let mut port = test_port(r, 1024, 2048);
        let mut buf = vec![0xaau8; 8192];

        // backlog past the peak: one period plus the excess is drained, no more
        let s = pattern(4096, 4);
        assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 4096) }, 4096);
        let n = unsafe { bounded_read(&mut port, 4096, buf.as_mut_ptr().cast(), 8192, 8) };
        assert_eq!(n, 1024 + (4096 - 2048));
        assert_eq!(&buf[..n as usize], &s[..n as usize]);

        // late cycle: nothing queued, so nothing is read from the blocking fd
        // and the graph still gets a whole period of silence
        let n = unsafe { bounded_read(&mut port, 0, buf.as_mut_ptr().cast(), 8192, 8) };
        assert_eq!(n, 1024);
        assert!(buf[..1024].iter().all(|&b| b == 0));
        unsafe { libc::close(w) };
    }

    #[test]
    fn fill_targets_track_arrival_granularity() {
        for period in [1024u32, 4096, 16384, 65536] {
            for blocksize in [512u32, 1024, 2047, 2048, 16384, 65536] {
                // unbounded: target sits half an arrival over one period, peak half
                // an arrival plus half a period over that
                let (target, peak) = fill_targets(period, blocksize, 0);
                assert_eq!(target, period + blocksize / 2);
                assert_eq!(peak, target + blocksize / 2 + period / 2);

                // an adequate ring keeps a catch-up band (one arrival above target)
                // and one arrival of top headroom - the read never goes dead
                let ring = ring_required(period, blocksize);
                let (target2, peak2) = fill_targets(period, blocksize, ring);
                assert_eq!(target2, target);
                assert!(
                    peak2 >= target + blocksize,
                    "catch-up band lost: peak {} < target {} + arrival {} (ring {})",
                    peak2,
                    target,
                    blocksize,
                    ring
                );
                assert!(
                    peak2 <= ring - blocksize,
                    "undrainable: peak {} past ring {} - arrival {}",
                    peak2,
                    ring,
                    blocksize
                );

                // a degenerate ring still pins the peak inside it
                let (_, peak3) = fill_targets(period, blocksize, period);
                assert!(peak3 <= period.saturating_sub(blocksize));
            }
        }
    }

    // the in-place retune: enough ring for the new period recommits the
    // fill geometry without touching the device
    #[test]
    fn retune_recommits_in_place() {
        let (r, w) = pipe_pair(false, false);
        let mut port = test_port(r, 1024, 0);
        port.ext.primed = true;
        port.ext.ring_size = 8192;
        let log = crate::spa::Log::test_null();

        assert!(!retune_period(&mut port, 2048, &log)); // debounced
        assert_eq!(port.setup_period, 1024);
        assert!(!retune_period(&mut port, 2048, &log)); // sustained: retune
        assert_eq!(port.setup_period, 2048);
        assert_eq!(port.ext.target_fill, 2048 + 512); // period + half an arrival
        assert_eq!(port.ext.read_peak, 4096);
        assert!(port.ext.primed);
        unsafe { libc::close(w) };
    }

    // a ring the new period outgrew wants a trigger suspend; the pipe
    // refuses the ioctl (the dying-fd model), so retune asks for a rebuild
    #[test]
    fn retune_requests_rebuild_when_the_suspend_is_refused() {
        let (r, w) = pipe_pair(false, false);
        let mut port = test_port(r, 1024, 0);
        port.ext.primed = true;
        port.ext.ring_size = 1024;
        // a read transitions the device to running, so suspend really issues
        // the (failing) SETTRIGGER instead of short-circuiting from setup
        let s = pattern(8, 5);
        assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 8) }, 8);
        let mut buf = [0u8; 8];
        assert_eq!(unsafe { port.dsp.read(buf.as_mut_ptr().cast(), 8) }, 8);
        let log = crate::spa::Log::test_null();

        assert!(!retune_period(&mut port, 2048, &log));
        assert!(retune_period(&mut port, 2048, &log));
        assert!(port.ext.primed); // not re-primed; the rebuild replaces the device
        assert_eq!(port.setup_period, 1024);
        unsafe { libc::close(w) };
    }

    // the follower servo: in-band errors feed the DLL, a level error past
    // the snap threshold (period.max(blocksize/2 + period/2)) relocks
    // instead of winding the integrator
    #[test]
    fn follower_servo_locks_in_band_and_relocks_on_snap() {
        let (r, w) = pipe_pair(false, false);
        let mut port = test_port(r, 1024, 0);
        port.ext.target_fill = 2560;

        let corr = follower_servo(&mut port, 2560 + 1500, 0, 8, 48000);
        assert_eq!(corr, 1.0); // snapped: relock only

        let corr = follower_servo(&mut port, 2560 - 512, 0, 8, 48000);
        assert!((0.9..=1.1).contains(&corr)); // in-band: the DLL absorbs it
        unsafe { libc::close(w) };
    }

    #[test]
    fn ring_request_floors_and_caps() {
        // four periods of the largest negotiable quantum, floored at the byte
        // budget, and the kernel cap always wins
        assert_eq!(ring_request(1024, 16384, 1 << 20), 16384 * 4);
        assert_eq!(ring_request(32768, 16384, 1 << 20), 32768 * 4);
        assert!(ring_request(64, 64, 1 << 20) >= crate::sound::MIN_RING_BYTES);
        assert_eq!(ring_request(65536, 65536, 4096), 4096);
    }
}
