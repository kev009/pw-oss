use std::ffi::c_int;

use libspa::sys::*;

use crate::node::{
    DataControl, DataState, Direction, MAX_PORTS, MainState, ParamBuild, PortConfig,
};

mod buffer;

use buffer::*;

// PortInfo currently supports one capture port.
const _: () = assert!(MAX_PORTS == 1);
const EMPTY_CYCLE: isize = -1; // no data queued this cycle (scheduling jitter)

pub(crate) enum SourceDir {}

// direction-specific main/data fields are empty for capture
#[derive(Default)]
pub(crate) struct SourceMainExt {}

#[derive(Default)]
pub(crate) struct SourceDataExt {}

// consecutive overrun-ticking cycles with the ring pinned at the ceiling
// before recovery re-primes; gives the catch-up read a chance to drain a
// transient first (it clears the pin in one cycle when the buffer allows)
const PINNED_CYCLE_LIMIT: u32 = 3;

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SourcePortExt {
    pub primed: bool,
    pub active_buffers: usize,  // next never-used buffer id to hand out
    pub target_fill: u32,       // servo fill target; a period plus half an arrival
    pub read_peak: u32,         // catch-up threshold, capped by the granted ring
    pub ring_size: u32,         // granted soft ring in bytes (GETISPACE totals; 0 = unknown)
    pub pinned_cycles: u32,     // consecutive overrun ticks with the ring pinned full
    pub period_mismatch: u32,   // consecutive cycles at a different period (debounce)
    pub was_freewheeling: bool, // freewheel active last cycle (re-prime on exit)
}

fn measured_fill(port: &crate::node::Port<SourceDir>) -> u32 {
    crate::node::device_event_fill(port).unwrap_or_else(|| port.dsp.ispace_in_bytes().max(0) as u32)
}

fn measured_overruns(port: &mut crate::node::Port<SourceDir>) -> u32 {
    if let Some(count) = crate::node::take_device_event_xruns(port) {
        count
    } else {
        let total = port.dsp.overruns();
        crate::node::take_fallback_xruns(port, total)
    }
}

pub(super) fn silence_byte(port: &crate::node::Port<SourceDir>) -> u8 {
    port.config
        .as_ref()
        .map(PortConfig::silence_byte)
        .unwrap_or(0)
}

// The follower-servo phase, matching a foreign clock: the DLL serves rate
// matching only (when driving, the servo runs at the device wake where the clock
// is published; a same-device follower has nothing to correct). `queued` is
// the pre-read fill the caller measured this cycle. Returns the rate
// correction.
fn follower_servo(
    port: &mut crate::node::Port<SourceDir>,
    queued: u32,
    now: u64,
    stride: u32,
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
        port.bw_adapt.update_fill(&mut port.dll, err, now);
    }

    #[cfg(debug_assertions)]
    eprintln!("capture: corr = {corr}, err = {err_raw}");

    corr
}

// The read-tail phase. Bounded read: one period, plus only the backlog
// beyond the healthy peak (genuine catch-up). Draining everything each
// cycle turns consumer backpressure into a permanent extra period of
// latency (an oversized chunk holds io.status HAVE_DATA, we skip the device
// next cycle, it queues 2 periods, repeat) and pollutes the servo error. If
// the device is late, keep the graph timeline stable: read only queued
// bytes from the blocking fd and silence-pad the rest of the period instead of
// returning an empty or short cycle.
fn bounded_read(
    port: &mut crate::node::Port<SourceDir>,
    queued: u32,
    data: &mut [u8],
    stride: u32,
) -> isize {
    let maxsize = data.len() as u32;
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
        port.dsp.read(&mut data[..ispace as usize]).max(0) as u32
    } else {
        0
    };
    let period = port.setup_period.min(maxsize);
    let out = if period > 0 { nread.max(period) } else { nread };
    if out > nread {
        data[nread as usize..out as usize].fill(silence_byte(port));
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
// `overrun_count` is the counter the caller read this cycle (nonzero, or
// this isn't called) and `now` the caller's timestamp; measured outside so
// tests can drive the pin gate.
fn recover_overrun(
    port: &mut crate::node::Port<SourceDir>,
    overrun_count: u32,
    pre_read_fill: Option<u32>,
    now: u64,
    log: &crate::spa::Log,
) {
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
        if let Some(suppressed) = port.warn_limit.check(now) {
            crate::warn!(
                log,
                "OSS reported {:3} overruns @ {} with the ring pinned; re-priming (+{} warnings suppressed)",
                overrun_count,
                now,
                suppressed
            );
        }
        // only for real recovery, not per ignored tick; deposited, not
        // called - process() notifies the host after the State borrows end
        // (collect-then-notify, see node::process)
        port.pending_xrun = Some(now / 1000);

        // recover like the sink's underrun path: re-enter priming next cycle,
        // which drains the backlog and relocks the DLL - otherwise the
        // un-drained backlog becomes permanent capture latency while the
        // integrator winds up against an error the reads can't remove.
        // Trigger-suspend first so the re-prime's SETFRAGMENT can also
        // RESIZE the ring: a pinned ring may be one the current quantum
        // outgrew, and a Running channel silently skips the layout
        // re-application (a refused suspend just re-primes at the old size).
        if port.dsp.suspend() {
            // The successful trigger reset starts a new kernel xrun epoch;
            // the re-prime also reapplies SETFRAGMENT and LOW_WATER.
            crate::node::reset_device_event(port);
        }
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

// used from the main thread only; returns 0 or -errno with the device closed
fn try_open_configure(
    dsp: &mut crate::oss::Dsp,
    config: &PortConfig,
    fragment: u32,
    log: &crate::spa::Log,
) -> c_int {
    let Ok(channel_order) = config.oss_channel_order() else {
        crate::warn!(
            log,
            "rejecting unsupported channel map: {:?}",
            config.positions
        );
        return -libc::EINVAL;
    };
    // a busy or vanished device must fail negotiation, not abort
    if let Err(err) = dsp.open() {
        crate::warn!(log, "dsp open: {}", err);
        return -(err as c_int);
    }
    // ditto for a device that won't take the format exactly
    if let Err(err) = dsp.configure(
        config.oss_format(),
        config.channels,
        config.rate,
        channel_order,
    ) {
        crate::warn!(log, "device rejected {:?}: {}", config, err);
        dsp.close();
        return -(err as c_int);
    }
    // on direct opens the hardware blocksize is per-session state; re-read it
    // now that THIS configuration is in effect (vchan/uaudio values are stable)
    dsp.refresh_hw_quantum();
    dsp.set_small_fragments(fragment, crate::oss::MIN_RING_BYTES); // normalized oss.fragment (0 = 1 KiB default)
    0
}

fn process_ports(state: &mut DataState<SourceDir>) -> c_int {
    let mut result = SPA_STATUS_OK as i32;

    // indexed (not iter_mut) so the rebuild arms below can end the &mut port
    // borrow, borrow the whole State, and re-borrow the port
    for port_idx in 0..state.ports.len() {
        // Consume any completed background rebuild before the cycle reads the
        // port (it may swap in a fresh device or clear the config); a rebuild
        // still in flight skips the cycle.
        if crate::node::poll_rebuild(state, port_idx) {
            continue;
        }
        let port = &mut state.ports[port_idx];
        let Some((stride, rate)) = port.stride_rate() else {
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
            continue;
        }

        // io is non-null here (checked above); the reads stay behind with()
        let io_status = port.io.with(|io| io.status);
        if io_status == Some(SPA_STATUS_HAVE_DATA as i32) {
            // a pending buffer the peer hasn't consumed yet: report HAVE_DATA, or
            // the adapter treats the cycle as empty (alsa-pcm-source.c does this)
            result |= SPA_STATUS_HAVE_DATA as i32;
            continue;
        }
        if io_status != Some(SPA_STATUS_OK as i32) && io_status != Some(SPA_STATUS_NEED_DATA as i32)
        {
            continue;
        }

        let io_buffer_id = port.io.with(|io| io.buffer_id).unwrap_or(-1i32 as u32);
        let buffer_id = if io_buffer_id == -1i32 as u32 {
            // hand out the next never-used buffer; the host returns ids after that
            let idx = port.ext.active_buffers;
            port.ext.active_buffers += 1;
            idx as u32
        } else {
            io_buffer_id
        };

        // buffer_id may be our fallback index; the validation is the shared
        // per-cycle gate (a source cycle just skips, no status to publish).
        // SAFETY: the host keeps the registered buffer pointers valid until
        // the next port_use_buffers (its contract), and the returned block is
        // used within this cycle only
        let Some(mut data_0) =
            (unsafe { crate::node::valid_data_block(port, buffer_id, &state.log) })
        else {
            continue;
        };

        // the whole block as this cycle's writable view (valid until the io
        // publication below)
        let cycle_data = data_0.output_slice();

        let matching = state.following
            && !state
                .position
                .with(|p| crate::node::same_clock(p, &state.clock_name))
                .unwrap_or(false);

        let mut corr: f64 = 1.0; // DLL rate correction for the follower rate match

        // one period in device bytes (0 while position is absent)
        let mut period_in_bytes = 0u32;
        let mut graph_rate = 0u32;
        if let Some(driver_clock) = state.position.with_ref(|p| p.clock)
            && driver_clock.target_rate.denom > 0
        {
            graph_rate = driver_clock.target_rate.denom;
            period_in_bytes = crate::node::device_period_bytes(
                driver_clock.target_duration,
                rate,
                graph_rate,
                stride,
            );
        }

        if retune_period(port, period_in_bytes, &state.log) {
            // the driver refused the trigger stop (dying fd): rebuild off-loop
            // rather than commit a fill target the current ring can't hold; if
            // even that fails (no main loop), keep running at the stale
            // geometry - degraded, but nothing stalls
            // (the &mut port borrow ends here: queue_rebuild snapshots an
            // owned request and owns the pending claim)
            let pending = crate::node::queue_rebuild(state, port_idx);
            if pending {
                continue;
            }
        }
        // re-borrow: the retune arm above may have borrowed the whole State
        let port = &mut state.ports[port_idx];

        let freewheel = state.position.with_ref(|p| p.clock.flags).unwrap_or(0)
            & SPA_IO_CLOCK_FLAG_FREEWHEEL
            != 0;

        // realtime resumed after freewheeling: the ring overflowed by design
        // while reads were skipped, so re-prime explicitly for a known fill
        // (the overrun gate below deliberately ignores the counter while the
        // ring state is sane, so it cannot cover this)
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
            let len = period_in_bytes.min(cycle_data.len() as u32);
            cycle_data[..len as usize].fill(silence_byte(port));
            len as isize
        } else if !port.ext.primed && period_in_bytes > 0 {
            prime_capture(
                port,
                period_in_bytes,
                graph_rate,
                state.oss_fragment,
                cycle_data,
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
            let queued = measured_fill(port);
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
                // 0 on a failed clock read: the bandwidth adaptation loses
                // this cycle's window, nothing aborts
                corr = follower_servo(
                    port,
                    queued,
                    crate::node::try_now_ns(&state.data_system).unwrap_or(0),
                    stride,
                );
            }

            bounded_read(port, queued, cycle_data, stride)
        };

        // Rate-match only as a follower on a foreign clock: when driving, the
        // timer steering applies the correction, and a same-device follower ticks
        // from our clock so there is nothing to match (ALSA gates on the clock
        // name the same way).
        port.was_matching = matching;
        // Realtime capture cycles are period-padded if the device is late; keep
        // rate matching coherent with the buffer we handed to the graph.
        if nbytes >= 0 {
            port.rate_match.with(|rm| {
                if matching {
                    rm.flags |= SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                    rm.rate = (1.0 / corr).clamp(0.99, 1.01);
                } else {
                    rm.flags &= !SPA_IO_RATE_MATCH_FLAG_ACTIVE;
                    rm.rate = 1.0;
                }
            });
        }

        let overruns = if port.dsp.is_running() && !freewheel {
            measured_overruns(port)
        } else {
            0
        };
        if overruns > 0 {
            // 0 on a failed clock read only mis-stamps the warn rate limit
            recover_overrun(
                port,
                overruns,
                pre_read_fill,
                crate::node::try_now_ns(&state.data_system).unwrap_or(0),
                &state.log,
            );
        } else {
            port.ext.pinned_cycles = 0;
        }

        if nbytes != -1 {
            #[cfg(debug_assertions)]
            if state.log.log_level() >= SPA_LOG_LEVEL_TRACE {
                crate::trace!(state.log, "nbytes: {}", nbytes);
                unsafe { spa_debug_mem(0, data_0.data_ptr(), 16.min(nbytes) as usize) };
            }

            data_0.publish(nbytes as u32, stride);
            port.io.with(|io| {
                io.buffer_id = buffer_id;
                io.status = SPA_STATUS_HAVE_DATA as i32;
            });

            result |= SPA_STATUS_HAVE_DATA as i32;
        } else {
            port.io.with(|io| {
                io.buffer_id = buffer_id; // -1i32 as u32;
                io.status = SPA_STATUS_OK as i32;
            });
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

    type Device = crate::oss::Dsp;
    type MainExt = SourceMainExt;
    type DataExt = SourceDataExt;
    type PortExt = SourcePortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        std::ptr::NonNull::new(&raw mut OSS_SOURCE_TOPIC).expect("a static's address is never null")
    }

    fn info_item(_ext: &mut SourceMainExt, _key: &str, _value: &str) {}

    fn ext_ready(_ext: &mut SourceMainExt) {}

    fn data_ext(_ext: &SourceMainExt) -> SourceDataExt {
        SourceDataExt {}
    }

    fn build_node_param(state: &mut MainState<SourceDir>, id: u32, index: u32) -> ParamBuild {
        #[expect(non_upper_case_globals)]
        let pod = match (id, index) {
            (SPA_PARAM_PropInfo, 0) => crate::spa::build_latency_offset_prop_info(),
            (SPA_PARAM_PropInfo, 1) => crate::spa::build_params_prop_info(
                crate::keys::OSS_FRAGMENT,
                "OSS fragment size (bytes, power of two, 0 = automatic)",
                state.oss_fragment,
                16384,
            ),
            (SPA_PARAM_Props, 0) => crate::spa::build_latency_offset_props(
                state.process_latency.ns,
                &[(crate::keys::OSS_FRAGMENT, state.oss_fragment)],
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
    fn reset_props(state: &mut MainState<SourceDir>, data: &DataControl<SourceDir>) -> c_int {
        let fragment = state.oss_fragment_default;
        let old_fragment = state.oss_fragment;
        state.oss_fragment = fragment;
        let res = crate::node::store_and_rebuild(state, data, move |state| {
            state.oss_fragment = fragment;
        });
        if res != 0 {
            state.oss_fragment = old_fragment;
            return res;
        }
        crate::node::handle_process_latency(state, crate::spa::process_latency_default());
        0
    }

    fn apply_oss_delay(
        _state: &mut MainState<SourceDir>,
        _data: &DataControl<SourceDir>,
        _delay: u32,
    ) -> c_int {
        0 // a playback-only knob; the capture side ignores it (as before)
    }

    fn try_open_configure(
        dsp: &mut crate::oss::Dsp,
        config: &PortConfig,
        fragment: u32,
        log: &crate::spa::Log,
    ) -> c_int {
        try_open_configure(dsp, config, fragment, log)
    }

    fn on_device_swapped(state: &mut DataState<SourceDir>, port_idx: usize) {
        let port = &mut state.ports[port_idx];
        crate::node::reset_device_event(port);
        port.dll.init(); // fresh device, fresh servo
        port.ext.primed = false;
        port.ext.active_buffers = 0;
    }

    fn on_buffers_swapped(state: &mut DataState<SourceDir>, port_idx: usize) {
        state.ports[port_idx].ext.active_buffers = 0;
    }

    fn on_start_loop(state: &mut DataState<SourceDir>) {
        // the device kept capturing across a Pause; re-prime so the first
        // cycles deliver fresh audio at a known fill, not the paused backlog
        for port in &mut state.ports {
            port.ext.primed = false;
            port.dll.init();
            port.bw_adapt.reset();
        }
    }

    fn on_suspend_loop(state: &mut DataState<SourceDir>) {
        for port in &mut state.ports {
            port.ext.primed = false; // resume re-primes for a known fill
        }
    }

    fn on_role_flip(state: &mut DataState<SourceDir>) {
        // as the sink: a role flip shifts the servo's measurement phase, and
        // stale integrator state would briefly steer the published clock on a
        // follower -> driver transition; relock instead
        for port in &mut state.ports {
            port.dll.init();
            port.bw_adapt.reset();
            port.was_matching = false;
        }
    }

    fn debug_cycle(_state: &DataState<SourceDir>, _now: u64, _nsec: u64) {}

    fn servo_ready(port: &crate::node::Port<SourceDir>) -> bool {
        port.ext.primed
    }

    // the pre-read fill here and process()'s post-drain accounting see the
    // same signal: we drain the ring every cycle, so what's queued is one
    // period's accumulation
    fn servo_fill(port: &mut crate::node::Port<SourceDir>) -> u32 {
        measured_fill(port)
    }

    fn servo_hold(_port: &crate::node::Port<SourceDir>) -> bool {
        false // the primed gate already covers recovery
    }

    // capture error is inverted vs the sink: a slow device queues less than
    // a period
    fn servo_err(port: &crate::node::Port<SourceDir>, fill: u32) -> f64 {
        port.ext.target_fill as f64 - fill as f64
    }

    fn wake_threshold(port: &crate::node::Port<SourceDir>) -> u32 {
        port.ext.target_fill.max(port.setup_period).max(1)
    }

    fn process_ports(state: &mut DataState<SourceDir>) -> c_int {
        process_ports(state)
    }
}

const OSS_SOURCE_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub(crate) const OSS_SOURCE_FACTORY: spa_handle_factory = spa_handle_factory {
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
mod tests;
