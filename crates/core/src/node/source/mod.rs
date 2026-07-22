use std::ffi::{c_char, c_int};

use libspa::sys::*;

use crate::backend::{self, CaptureOperations as _, StreamLifecycle as _};
use crate::spa::Log;

use super::{
    BackendPropertiesOf, DataControl, DataState, Direction, MAX_PORTS, MainState, ParamBuild, Port,
    PortConfig, build_backend_node_param, device_period_bytes, enum_interface_info, get_size, init,
    latch_rebuild_required, pending_xrun, poll_rebuild, queue_rebuild, reset_backend_props,
    reset_stream_epoch, same_clock, take_polled_xruns, take_wake_xruns, try_now_ns,
    valid_data_block, wake_queue_fill,
};

mod buffer;

use buffer::*;

// PortInfo currently supports one capture port.
const _: () = assert!(MAX_PORTS == 1);
const EMPTY_CYCLE: isize = -1; // no data queued this cycle (scheduling jitter)

pub(crate) struct SourceDir<B>(std::marker::PhantomData<B>);

// direction-specific main/data fields are empty for capture
#[derive(Default)]
pub(crate) struct SourceDataExt {}

// direction-specific Port fields (Port.ext)
#[derive(Default)]
pub(crate) struct SourcePortExt {
    pub primed: bool,
    pub active_buffers: usize,  // next never-used buffer id to hand out
    pub target_fill: u32,       // servo fill target; a period plus half an arrival
    pub read_peak: u32,         // catch-up threshold, capped by the granted ring
    pub ring_size: u32,         // backend-reported capture capacity; 0 = unknown
    pub retune_pending: bool,   // backend is debouncing a changed period
    pub was_freewheeling: bool, // freewheel active last cycle (re-prime on exit)
}

fn measured_fill<B: backend::Backend>(port: &Port<SourceDir<B>>) -> u32 {
    wake_queue_fill(port).unwrap_or_else(|| port.dsp.queued_bytes())
}

fn measured_overruns<B: backend::Backend>(port: &mut Port<SourceDir<B>>) -> backend::XrunDelta {
    if let Some(count) = take_wake_xruns(port) {
        count
    } else {
        let observation = port.dsp.overruns();
        take_polled_xruns(port, observation)
    }
}

pub(super) fn fill_silence<B: backend::Backend>(port: &Port<SourceDir<B>>, output: &mut [u8]) {
    port.config
        .as_ref()
        .map(PortConfig::silence_pattern)
        .unwrap_or_else(|| backend::SilencePattern::zero(1))
        .fill(output);
}

// The follower-servo phase, matching a foreign clock: the DLL serves rate
// matching only (when driving, the servo runs at the device wake where the clock
// is published; a same-device follower has nothing to correct). `queued` is
// the pre-read fill the caller measured this cycle. Returns the rate
// correction.
fn follower_servo<B: backend::Backend>(
    port: &mut Port<SourceDir<B>>,
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
        .max(port.delivery_quantum_bytes / 2 + port.setup_period / 2);
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
fn bounded_read<B: backend::Backend>(
    port: &mut Port<SourceDir<B>>,
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
        let outcome = port.dsp.read(&mut data[..ispace as usize]);
        latch_rebuild_required(port, outcome.status);
        outcome.bytes as u32
    } else {
        0
    };
    let period = port.setup_period.min(maxsize);
    let out = if period > 0 { nread.max(period) } else { nread };
    if out > nread {
        fill_silence(port, &mut data[nread as usize..out as usize]);
    }
    out as isize
}

// Let the selected backend classify and perform native overrun recovery, then
// translate that decision into the shared graph/xrun lifecycle.
fn recover_overrun<B: backend::Backend>(
    port: &mut Port<SourceDir<B>>,
    overrun: backend::XrunDelta,
    pre_read_fill: Option<u32>,
    now: u64,
    log: &Log,
) {
    let overrun_count = overrun.events;
    if let Some(reset_epoch) = port.dsp.recover_overrun(overrun_count, pre_read_fill, log) {
        if let Some(suppressed) = port.warn_limit.check(now) {
            port.dsp
                .log_overrun_recovery(overrun_count, now, suppressed, log);
        }
        // only for real recovery, not per ignored tick; deposited, not
        // called - process() notifies the host after the State borrows end
        // (collect-then-notify, see node::process)
        port.pending_xrun = Some(pending_xrun(now / 1000, overrun, port.config.as_ref()));

        if reset_epoch {
            // A native reset establishes a fresh measurement epoch, but it
            // must not forgive a fatal I/O outcome already latched for this
            // descriptor during the same cycle.
            let rebuild_required = port.rebuild_required;
            reset_stream_epoch(port);
            port.rebuild_required |= rebuild_required;
        }
        port.ext.primed = false;
        port.bw_adapt.reset();
        port.dll.init();
    }
}

fn process_ports<B: backend::Backend>(state: &mut DataState<SourceDir<B>>) -> c_int {
    let mut result = SPA_STATUS_OK as i32;

    // indexed (not iter_mut) so the rebuild arms below can end the &mut port
    // borrow, borrow the whole State, and re-borrow the port
    for port_idx in 0..state.ports.len() {
        // Consume any completed background rebuild before the cycle reads the
        // port (it may swap in a fresh device or clear the config); a rebuild
        // still in flight skips the cycle.
        if poll_rebuild(state, port_idx) {
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
            queue_rebuild(state, port_idx);
            continue;
        }

        // io is non-null here (checked above); the reads stay behind with()
        let io_status = port.io.with(|io| io.status);
        if io_status == Some(SPA_STATUS_HAVE_DATA as i32) {
            // A pending buffer the peer has not consumed still represents
            // output; report HAVE_DATA so the adapter does not treat it as an
            // empty cycle.
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
        let Some(mut data_0) = (unsafe { valid_data_block(port, buffer_id, &state.log) }) else {
            continue;
        };

        // the whole block as this cycle's writable view (valid until the io
        // publication below)
        let cycle_data = data_0.output_slice();

        let matching = state.following
            && !state
                .position
                .with(|p| same_clock(p, &state.clock_name))
                .unwrap_or(false);

        let mut corr: f64 = 1.0; // DLL rate correction for the follower rate match

        // one period in device bytes (0 while position is absent)
        let mut period_in_bytes = 0u32;
        let mut graph_rate = 0u32;
        if let Some(driver_clock) = state.position.with_ref(|p| p.clock)
            && driver_clock.target_rate.denom > 0
        {
            graph_rate = driver_clock.target_rate.denom;
            period_in_bytes =
                device_period_bytes(driver_clock.target_duration, rate, graph_rate, stride);
        }

        if retune_period(port, period_in_bytes, &state.log) {
            // the driver refused the trigger stop (dying fd): rebuild off-loop
            // rather than commit a fill target the current ring can't hold; if
            // even that fails (no main loop), keep running at the stale
            // geometry - degraded, but nothing stalls
            // (the &mut port borrow ends here: queue_rebuild snapshots an
            // owned request and owns the pending claim)
            let pending = queue_rebuild(state, port_idx);
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
            // Freewheeling hands out silence without touching the stream; its
            // queue may fill meanwhile, so the exit edge above re-primes when
            // realtime resumes.
            let len = period_in_bytes.min(cycle_data.len() as u32);
            fill_silence(port, &mut cycle_data[..len as usize]);
            len as isize
        } else if !port.ext.primed && period_in_bytes > 0 {
            prime_capture(
                port,
                period_in_bytes,
                graph_rate,
                &state.backend_properties,
                cycle_data,
                &state.log,
            )
        } else if !port.dsp.is_running() {
            // Unprimed and without a usable position yet (the prime branch
            // needs a period): the stream is still in setup, where queue
            // measurements are not valid.
            EMPTY_CYCLE
        } else {
            // Gate on semantic queued bytes rather than a readiness edge, whose
            // delivery quantum can exceed a small graph period and bias every
            // read and servo sample. Priming already started the stream.
            let queued = measured_fill(port);
            pre_read_fill = Some(queued);
            if queued == 0 {
                crate::debug!(state.log, "capture: empty cycle (no data queued at wakeup)");
            }

            // A debounce hold cycle runs at stale geometry - don't feed its
            // transitional error to the DLL (the sink gates the same way).
            if matching && period_in_bytes > 0 && port.setup_period != 0 && !port.ext.retune_pending
            {
                // 0 on a failed clock read: the bandwidth adaptation loses
                // this cycle's window, nothing aborts
                corr = follower_servo(
                    port,
                    queued,
                    try_now_ns(&state.data_system).unwrap_or(0),
                    stride,
                );
            }

            bounded_read(port, queued, cycle_data, stride)
        };

        // Rate-match only as a follower on a foreign clock: when driving, the
        // timer steering applies the correction, and a same-device follower
        // ticks from our clock so there is nothing to match.
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
            backend::XrunDelta::default()
        };
        if overruns.events > 0 {
            // 0 on a failed clock read only mis-stamps the warn rate limit
            recover_overrun(
                port,
                overruns,
                pre_read_fill,
                try_now_ns(&state.data_system).unwrap_or(0),
                &state.log,
            );
        } else {
            port.dsp.clear_overrun_observation();
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

impl<B: backend::Backend> Direction for SourceDir<B> {
    const DIRECTION: spa_direction = SPA_DIRECTION_OUTPUT;
    const PLAYBACK: bool = false;
    const MEDIA_CLASS: &'static str = "Audio/Source";
    // Capture publishes a completed output buffer; NEED_DATA is the playback
    // driver's input request.
    const READY_STATUS: i32 = SPA_STATUS_HAVE_DATA as i32;
    const CMD_WARN_PREFIX: &'static str = B::SOURCE_COMMAND_PREFIX;

    type Backend = B;
    type Device = B::Capture;
    type DataExt = SourceDataExt;
    type PortExt = SourcePortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        B::source_log_topic()
    }

    fn data_ext(_properties: &BackendPropertiesOf<SourceDir<B>>) -> SourceDataExt {
        SourceDataExt {}
    }

    fn sync_backend_properties(
        _ext: &mut SourceDataExt,
        _properties: &BackendPropertiesOf<SourceDir<B>>,
    ) {
    }

    fn build_node_param(state: &mut MainState<SourceDir<B>>, id: u32, index: u32) -> ParamBuild {
        build_backend_node_param(state, id, index)
    }

    // a NULL Props pod resets the props to their defaults and re-applies them
    fn reset_props(state: &mut MainState<SourceDir<B>>, data: &DataControl<SourceDir<B>>) -> c_int {
        reset_backend_props(state, data)
    }

    fn try_open_configure(
        stream: &mut B::Capture,
        config: &PortConfig,
        properties: &BackendPropertiesOf<SourceDir<B>>,
        log: &Log,
    ) -> Result<backend::ConfigureOutcome, c_int> {
        stream.configure(config, properties, log)
    }

    fn on_device_swapped(state: &mut DataState<SourceDir<B>>, port_idx: usize) {
        let port = &mut state.ports[port_idx];
        reset_stream_epoch(port);
        port.dll.init(); // fresh device, fresh servo
        port.ext.primed = false;
        port.ext.active_buffers = 0;
    }

    fn on_buffers_swapped(state: &mut DataState<SourceDir<B>>, port_idx: usize) {
        state.ports[port_idx].ext.active_buffers = 0;
    }

    fn on_start_loop(state: &mut DataState<SourceDir<B>>) {
        // the device kept capturing across a Pause; re-prime so the first
        // cycles deliver fresh audio at a known fill, not the paused backlog
        for port in &mut state.ports {
            port.ext.primed = false;
            port.dll.init();
            port.bw_adapt.reset();
        }
    }

    fn on_suspend_loop(state: &mut DataState<SourceDir<B>>) {
        for port in &mut state.ports {
            port.ext.primed = false; // resume re-primes for a known fill
        }
    }

    fn on_role_flip(state: &mut DataState<SourceDir<B>>) {
        // as the sink: a role flip shifts the servo's measurement phase, and
        // stale integrator state would briefly steer the published clock on a
        // follower -> driver transition; relock instead
        for port in &mut state.ports {
            port.dll.init();
            port.bw_adapt.reset();
            port.was_matching = false;
        }
    }

    fn debug_cycle(_state: &DataState<SourceDir<B>>, _now: u64, _nsec: u64) {}

    fn servo_ready(port: &Port<SourceDir<B>>) -> bool {
        port.ext.primed
    }

    // the pre-read fill here and process()'s post-drain accounting see the
    // same signal: we drain the ring every cycle, so what's queued is one
    // period's accumulation
    fn servo_fill(port: &mut Port<SourceDir<B>>) -> u32 {
        measured_fill(port)
    }

    fn servo_hold(_port: &Port<SourceDir<B>>) -> bool {
        false // the primed gate already covers recovery
    }

    // capture error is inverted vs the sink: a slow device queues less than
    // a period
    fn servo_err(port: &Port<SourceDir<B>>, fill: u32) -> f64 {
        port.ext.target_fill as f64 - fill as f64
    }

    fn wake_buffer_state(port: &Port<SourceDir<B>>) -> backend::WakeBufferState {
        backend::WakeBufferState {
            frame_stride: port.stride_rate().map_or(1, |(stride, _)| stride),
            period_bytes: port.setup_period,
            quantum_bytes: port.delivery_quantum_bytes,
            capacity_bytes: port.ext.ring_size,
            target_fill_bytes: port.ext.target_fill,
        }
    }

    fn process_ports(state: &mut DataState<SourceDir<B>>) -> c_int {
        process_ports(state)
    }
}

const SOURCE_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub const fn factory<B: backend::Backend>(name: *const c_char) -> spa_handle_factory {
    spa_handle_factory {
        version: SPA_VERSION_HANDLE_FACTORY,
        name,
        info: &SOURCE_FACTORY_INFO,
        get_size: Some(get_size::<SourceDir<B>>),
        init: Some(init::<SourceDir<B>>),
        enum_interface_info: Some(enum_interface_info),
    }
}

#[cfg(test)]
mod tests;
