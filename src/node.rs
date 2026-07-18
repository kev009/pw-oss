// The shared node core. sink.rs and source.rs are the same SPA node modulo
// direction: everything direction-agnostic lives here once, generic over
// `Direction`, and the genuinely direction-specific logic (the process() data
// path, the servo error sign, priming/recovery semantics, the oss.delay prop)
// is supplied through the `Direction` hooks each module implements. The
// extern "C" vtable entries are generic and monomorphized per direction.

use std::mem::MaybeUninit;
use std::os::raw::{c_char, c_int, c_void};

use libspa::sys::*;

pub(crate) const MAX_PORTS: usize = 1;

// the shared surface of sound::Dsp/sound::DspWriter used by the generic core;
// the direction-specific ops (write/odelay vs read/ispace) stay on the
// concrete types and are used from the Direction hooks
pub(crate) trait DeviceOps {
    fn new(path: &str) -> Self;
    fn path(&self) -> &str;
    fn is_closed(&self) -> bool;
    fn is_running(&self) -> bool;
    fn close(&mut self);
    fn suspend(&mut self) -> bool;
}

impl DeviceOps for crate::sound::Dsp {
    fn new(path: &str) -> Self {
        crate::sound::Dsp::new(path)
    }
    fn path(&self) -> &str {
        crate::sound::Dsp::path(self)
    }
    fn is_closed(&self) -> bool {
        crate::sound::Dsp::is_closed(self)
    }
    fn is_running(&self) -> bool {
        crate::sound::Dsp::is_running(self)
    }
    fn close(&mut self) {
        crate::sound::Dsp::close(self);
    }
    fn suspend(&mut self) -> bool {
        crate::sound::Dsp::suspend(self)
    }
}

impl DeviceOps for crate::sound::DspWriter {
    fn new(path: &str) -> Self {
        crate::sound::DspWriter::new(path)
    }
    fn path(&self) -> &str {
        &self.path
    }
    fn is_closed(&self) -> bool {
        crate::sound::DspWriter::is_closed(self)
    }
    fn is_running(&self) -> bool {
        crate::sound::DspWriter::is_running(self)
    }
    fn close(&mut self) {
        crate::sound::DspWriter::close(self);
    }
    fn suspend(&mut self) -> bool {
        crate::sound::DspWriter::suspend(self)
    }
}

// the negotiated format, shared by both directions (the stride is derived
// from the format map at parse time and stored)
#[derive(Debug, Clone)]
pub(crate) struct PortConfig {
    pub format: libspa::param::audio::AudioFormat,
    pub rate: u32,
    pub channels: u32,
    pub positions: Vec<u32>, // the negotiated channel positions, replayed in the Format readback
    pub flags: u32,
    pub stride: u32, // bytes per interleaved frame
}

impl PortConfig {
    pub(crate) fn oss_format(&self) -> u32 {
        // parse_config admits only formats from the map, so the lookup can't
        // miss; 0 (matching no AFMT) beats a panic across extern "C"
        crate::utils::oss_format_info(self.format.0)
            .map(|(m, _)| m)
            .unwrap_or(0)
    }
}

// outcome of a per-(id, index) node param build (the enum_params hook)
pub(crate) enum ParamBuild {
    Built,     // a pod was written into the builder
    Exhausted, // no more values for this param id
    Unknown,   // unknown param id
}

pub(crate) trait Direction: Sized + 'static {
    /// the port direction from the graph's perspective
    const DIRECTION: spa_direction;
    /// probe_caps()/install direction flag
    const PLAYBACK: bool;
    const MEDIA_CLASS: &'static str;
    /// status a driving node passes to ready(): a playback driver signals
    /// NEED_DATA; a capture driver signals HAVE_DATA (alsa-pcm.c capture_ready)
    const READY_STATUS: i32;
    /// historical prefix of the unknown-command warning ("oss-source: " there)
    const CMD_WARN_PREFIX: &'static str;

    // Send: crosses onto the data loop through install_device's swap
    type Device: DeviceOps + Send;
    type Ext: Default; // direction-specific State fields
    type PortExt: Default; // direction-specific Port fields

    // init: the module's registered log topic (see the lib.rs section
    // entries); a raw pointer because the host mutates the pointee
    fn log_topic() -> std::ptr::NonNull<spa_log_topic>;

    // init: direction-specific node property keys (e.g. the sink's oss.delay)
    fn info_item(ext: &mut Self::Ext, key: &str, value: &str);
    // init: after the info dict is parsed (e.g. latching the oss.delay default)
    fn ext_ready(ext: &mut Self::Ext);

    // enum_params: build one node param pod for (id, index)
    fn build_node_param(
        state: &mut State<Self>,
        b: &mut libspa::pod::builder::Builder,
        id: u32,
        index: u32,
    ) -> ParamBuild;
    // set_param(Props) with a NULL pod: reset the props to their defaults
    unsafe fn reset_props(state: &mut State<Self>) -> c_int;
    // set_param(Props): the SPA_PROP_params property (the sink's oss.delay)
    unsafe fn set_props_params(state: &mut State<Self>, value: &libspa::pod::Value) -> c_int;

    // used from the main thread only; returns 0 or -errno with the device
    // closed. `fragment` is the normalized oss.fragment (0 = automatic); the
    // source applies it at open time, the sink at prime time (the period is
    // only known then)
    fn try_open_configure(
        dsp: &mut Self::Device,
        config: &PortConfig,
        fragment: u32,
        log: &crate::spa::Log,
    ) -> c_int;
    // install_device: direction-specific resets inside the loop-side swap
    fn on_device_swapped(state: &mut State<Self>, port_idx: usize);
    // port_use_buffers: direction-specific resets inside the loop-side swap
    fn on_buffers_swapped(state: &mut State<Self>, port_idx: usize);

    // send_command(Start): direction-specific resets, on the data loop
    fn on_start_loop(state: &mut State<Self>);
    // send_command(Suspend): direction-specific resets, on the data loop
    fn on_suspend_loop(state: &mut State<Self>);
    // set_io: the driver/follower role flipped on a live node
    fn on_role_flip(state: &mut State<Self>);

    // on_timeout: debug-build cycle tracing (the sink prints one line)
    unsafe fn debug_cycle(state: &State<Self>, now: u64, nsec: u64);
    // on_timeout servo hooks (see node::timeout_servo): the extra readiness
    // gate (the source's primed flag), the fill measurement, the recovery
    // hold (the sink's xrun window) and the signed servo error for a fill
    fn servo_ready(port: &Port<Self>) -> bool;
    fn servo_fill(port: &mut Port<Self>) -> u32;
    fn servo_hold(port: &Port<Self>) -> bool;
    fn servo_err(port: &Port<Self>, fill: u32) -> f64;

    // process(): the direction-specific data path over the ports
    unsafe fn process_ports(state: &mut State<Self>) -> c_int;

    const NODE_METHODS: spa_node_methods = spa_node_methods {
        version: SPA_VERSION_NODE_METHODS,
        add_listener: Some(add_listener::<Self>),
        set_callbacks: Some(set_callbacks::<Self>),
        sync: Some(sync::<Self>),
        enum_params: Some(enum_params::<Self>),
        set_param: Some(set_param::<Self>),
        set_io: Some(set_io::<Self>),
        send_command: Some(send_command::<Self>),
        add_port: Some(add_port),
        remove_port: Some(remove_port),
        port_enum_params: Some(port_enum_params::<Self>),
        port_set_param: Some(port_set_param::<Self>),
        port_use_buffers: Some(port_use_buffers::<Self>),
        port_set_io: Some(port_set_io::<Self>),
        port_reuse_buffer: Some(port_reuse_buffer),
        process: Some(process::<Self>),
    };
}

// repr(C): the host casts spa_handle* to State*, so `handle` must stay
// the first field at offset 0
#[repr(C)]
pub(crate) struct State<D: Direction> {
    pub handle: spa_handle,
    pub node: spa_node,
    pub node_info: crate::spa::NodeInfo,
    pub port_info: crate::spa::PortInfo,
    pub data_loop: crate::spa::Loop,
    pub data_system: crate::spa::System,
    pub log: crate::spa::Log,
    pub clock: crate::spa::IoArea<spa_io_clock>,
    pub position: crate::spa::IoArea<spa_io_position>,
    pub clock_name: std::ffi::CString, // stamped into spa_io_clock.name
    pub main_loop: Option<crate::spa::Loop>, // for deferring device rebuilds off the data loop
    pub dsp_path: String,
    pub timer_source: spa_source,
    pub next_time: u64,
    pub hooks: spa_hook_list,
    pub callbacks: spa_callbacks,
    pub ports: [Port<D>; MAX_PORTS],
    pub caps: crate::sound::DspCaps,
    pub caps_fallback: bool, // init-time probe failed (busy device); re-probe lazily
    pub oss_fragment: u32, // normalized fragment size in bytes (0 = automatic); read by the prime paths
    pub oss_fragment_default: u32, // init-dict value, restored by a NULL Props reset
    pub loop_thread: std::sync::atomic::AtomicUsize, // thread process()/on_timeout run on (0 = unseen)
    pub latency: [spa_latency_info; 2], // indexed by direction; written by the host, replayed on read
    pub process_latency: spa_process_latency_info,
    pub started: bool,
    pub clearing: bool, // teardown in progress; queued tasks must no-op
    pub following: bool,
    pub ring_cap_published: bool, // node.max-latency emitted (props dict is append-only)
    pub ext: D::Ext,              // direction-specific fields (see sink.rs/source.rs)
}

impl<D: Direction> State<D> {
    fn node_is_follower(&self) -> bool {
        let driver = self.position.with(|p| p.clock.id);
        let ours = self.clock.with(|c| c.id);
        matches!((driver, ours), (Some(d), Some(o)) if d != o)
    }
}

pub(crate) struct Port<D: Direction> {
    pub config: Option<PortConfig>,
    pub buffers: Vec<*mut spa_buffer>,
    pub io: crate::spa::IoArea<spa_io_buffers>,
    pub rate_match: crate::spa::IoArea<spa_io_rate_match>, // per-port io area (port_set_io)
    pub dsp: D::Device,
    pub dll: crate::dll::SpaDLL,
    pub setup_period: u32, // device bytes per graph cycle the stream/servo was set up for
    pub bw_adapt: crate::dll::BwAdapt, // variance-adaptive bandwidth (ALSA scheme)
    pub setup_blocksize: u32, // device fragment size (measurement quantization)
    pub resetup_pending: bool, // a main-thread device rebuild is queued; skip cycles
    pub was_matching: bool, // rate matching active last cycle (relock on transition)
    pub warn_limit: crate::utils::RateLimit,
    pub ext: D::PortExt, // direction-specific fields (see sink.rs/source.rs)
}

// The validated fields of the buffer's single data block, copied out BY
// VALUE: the block lives in host-owned buffer memory, so a returned
// reference could only carry a fabricated lifetime. NonNull records what
// valid_data_block checked; callers deref within the cycle, before any
// buffer swap can run on this loop.
#[derive(Clone, Copy)]
pub(crate) struct DataBlock {
    pub data: std::ptr::NonNull<c_void>, // the mapped MemPtr block
    pub chunk: std::ptr::NonNull<spa_chunk>,
    pub maxsize: u32, // > 0; offsets/sizes must be clamped against it
}

// The per-cycle buffer validation shared by both process paths. buffer_id
// and n_datas come from the peer, and the cycle maps/fills the block
// directly, so require exactly one MemPtr data block with data, chunk and
// maxsize all valid; validate instead of asserting - a panic here aborts
// the process across extern "C". as_ref() (not offset(0)) handles a null
// datas pointer without UB. None = unusable (logged); the caller decides
// the cycle's status.
pub(crate) unsafe fn valid_data_block<D: Direction>(
    port: &Port<D>,
    buffer_id: u32,
    log: &crate::spa::Log,
) -> Option<DataBlock> {
    let buffer: &spa_buffer = match port
        .buffers
        .get(buffer_id as usize)
        .copied()
        // as_ref (not a deref) handles a null host buffer pointer without UB
        .and_then(|b| unsafe { b.as_ref() })
    {
        Some(b) if b.n_datas == 1 => b,
        _ => {
            crate::warn!(
                log,
                "{}: unusable buffer (id {}); skipping",
                port.dsp.path(),
                buffer_id
            );
            return None;
        }
    };
    match unsafe { buffer.datas.as_ref() } {
        Some(d)
            if d.type_ == SPA_DATA_MemPtr
                && !d.data.is_null()
                && !d.chunk.is_null()
                && d.maxsize > 0 =>
        {
            Some(DataBlock {
                // non-null: both were checked in the guard above
                data: std::ptr::NonNull::new(d.data).expect("data checked in the guard"),
                chunk: std::ptr::NonNull::new(d.chunk).expect("chunk checked in the guard"),
                maxsize: d.maxsize,
            })
        }
        _ => {
            crate::warn!(
                log,
                "{}: buffer data is not a usable MemPtr block; skipping",
                port.dsp.path()
            );
            None
        }
    }
}

impl<D: Direction> Port<D> {
    // the negotiated (stride, rate) as copies, not a borrow: the process phases
    // commit geometry through &mut Port. None until a format is negotiated -
    // callers skip the cycle then rather than panic across extern "C"
    // (stride >= 1 post-negotiation, so the .max(1) is pure defense).
    pub(crate) fn stride_rate(&self) -> Option<(u32, u32)> {
        let config = self.config.as_ref()?;
        Some((config.stride.max(1), config.rate))
    }
}

unsafe extern "C" fn add_listener<D: Direction>(
    object: *mut c_void,
    listener: *mut spa_hook,
    events: *const spa_node_events,
    data: *mut c_void,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    let mut save = MaybeUninit::<spa_hook_list>::uninit();
    unsafe {
        spa_hook_list_isolate(
            &mut state.hooks,
            save.as_mut_ptr(),
            listener,
            events.cast(),
            data,
        );
    }

    // The initial emissions only reach the newly added listener (the list is
    // isolated). One method per traversal, mirroring C's spa_hook_list_call:
    // a listener that removes and frees its hook from inside a callback must
    // not be called (or have its hook read) again for the next method.
    unsafe {
        let old_mask = state
            .node_info
            .replace_change_mask(crate::spa::SPA_NODE_CHANGE_MASK_ALL as u64);
        crate::spa::emit_events(&mut state.hooks, |f: &spa_node_events, data| {
            if let Some(node_info_fun) = f.info {
                node_info_fun(data, state.node_info.raw());
            }
        });
        let _ = state.node_info.replace_change_mask(old_mask);

        let old_mask = state
            .port_info
            .replace_change_mask(crate::spa::SPA_PORT_CHANGE_MASK_ALL as u64);
        crate::spa::emit_events(&mut state.hooks, |f: &spa_node_events, data| {
            if let Some(port_info_fun) = f.port_info {
                port_info_fun(data, D::DIRECTION, 0, state.port_info.raw());
            }
        });
        let _ = state.port_info.replace_change_mask(old_mask);
    }

    // isolate above initialized `save`
    unsafe { spa_hook_list_join(&mut state.hooks, save.assume_init_mut()) };

    0
}

// re-emit node_info to every listener (carrying whatever change_mask the caller
// set, e.g. PARAMS), then clear the mask
pub(crate) unsafe fn emit_node_info<D: Direction>(state: &mut State<D>) {
    // one emission through the C listener vtables end to end
    unsafe {
        crate::spa::emit_events(&mut state.hooks, |f: &spa_node_events, data| {
            if let Some(node_info_fun) = f.info {
                node_info_fun(data, state.node_info.raw());
            }
        });
    }
    let _ = state.node_info.replace_change_mask(0);
}

// the process latency (user-set latency offset) shifts the node's reported
// latency, so a change re-emits the Props/ProcessLatency node params and the
// port Latency param
pub(crate) unsafe fn handle_process_latency<D: Direction>(
    state: &mut State<D>,
    info: spa_process_latency_info,
) {
    let ns_changed = state.process_latency.ns != info.ns;
    if state.process_latency.quantum == info.quantum
        && state.process_latency.rate == info.rate
        && !ns_changed
    {
        return;
    }

    state.process_latency = info;

    let _ = state.node_info.replace_change_mask(0);
    if ns_changed {
        state.node_info.bump_param(SPA_PARAM_Props);
    }
    state.node_info.bump_param(SPA_PARAM_ProcessLatency);
    unsafe { emit_node_info(state) };

    let _ = state.port_info.replace_change_mask(0);
    state.port_info.bump_param(SPA_PARAM_Latency);
    unsafe { emit_port_info(state) };
}

// re-emit port_info to every listener (carrying whatever change_mask the caller
// set, e.g. RATE/PARAMS), then clear the mask
pub(crate) unsafe fn emit_port_info<D: Direction>(state: &mut State<D>) {
    // one emission through the C listener vtables end to end
    unsafe {
        crate::spa::emit_events(&mut state.hooks, |f: &spa_node_events, data| {
            if let Some(port_info_fun) = f.port_info {
                port_info_fun(data, D::DIRECTION, 0, state.port_info.raw());
            }
        });
    }
    let _ = state.port_info.replace_change_mask(0);
}

unsafe extern "C" fn set_callbacks<D: Direction>(
    object: *mut c_void,
    callbacks: *const spa_node_callbacks,
    data: *mut c_void,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null());
    // SAFETY: the host keeps the callback table and its data pointer valid
    // while set (the set_callbacks contract), so on_timeout/process may use
    // them from the data loop
    let callbacks = unsafe { crate::utils::SendWrap::new(callbacks) };
    let data = unsafe { crate::utils::SendWrap::new(data) };
    if !unsafe {
        crate::utils::block_on_loop(&(*state).data_loop, state, move |state| {
            let callbacks = callbacks.into_inner();
            let data = data.into_inner();
            state.callbacks.funcs = callbacks as *const c_void;
            state.callbacks.data = data;
        })
    } {
        return -libc::EIO;
    }
    0
}

unsafe extern "C" fn sync<D: Direction>(object: *mut c_void, seq: c_int) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");
    unsafe { crate::spa::node_emit_done(&mut state.hooks, seq) };
    0
}

unsafe extern "C" fn enum_params<D: Direction>(
    object: *mut c_void,
    seq: c_int,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    if max == 0 {
        return 0;
    }

    let mut buffer = vec![];
    let mut fbuffer = vec![]; // spa_pod_filter output; kept apart from the source pod (see spa::filter_pod)

    let mut index = start;
    let mut count = 0;

    while count < max {
        use libspa::pod::builder::Builder;

        let mut builder = Builder::new(&mut buffer);

        match D::build_node_param(state, &mut builder, id, index) {
            ParamBuild::Built => (),
            ParamBuild::Exhausted => return 0,
            ParamBuild::Unknown => return -libc::ENOENT, // unknown param id (ALSA convention)
        }

        drop(builder); // its borrow of `buffer` must end before we take the source pointer

        let mut result = spa_result_node_params {
            id,
            index,
            next: index + 1,
            param: std::ptr::null_mut(),
        };

        // the built pod lives in `buffer`, distinct from the filter output
        if let Some(param) = unsafe {
            crate::spa::filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter)
        } {
            result.param = param;
            unsafe {
                crate::spa::node_emit_result(
                    &mut state.hooks,
                    seq,
                    0,
                    SPA_RESULT_TYPE_NODE_PARAMS,
                    &result,
                );
            }
            count += 1;
        }

        index += 1;
    }

    0
}

unsafe extern "C" fn set_param<D: Direction>(
    object: *mut c_void,
    id: u32,
    _flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    use libspa::pod::{Object, Value};

    #[allow(non_upper_case_globals)]
    match id {
        SPA_PARAM_Props => {
            if param.is_null() {
                // a NULL pod resets the props to their defaults
                let res = unsafe { D::reset_props(state) };
                if res == 0 {
                    let _ = state.node_info.replace_change_mask(0);
                    state.node_info.bump_param(SPA_PARAM_Props);
                    unsafe { emit_node_info(state) };
                }
                return res;
            }
            match unsafe { crate::utils::deserialize_pod(param) } {
                Some(Value::Object(Object {
                    type_, properties, ..
                })) if type_ == SPA_TYPE_OBJECT_Props => {
                    for property in properties {
                        match property.key {
                            // there is no way adapter is actually supposed to pass all those properties (or parameters?) to us,
                            // it's probably a bug
                            // softvol-handled by the adapter
                            SPA_PROP_volume
                            | SPA_PROP_mute
                            | SPA_PROP_channelVolumes
                            | SPA_PROP_channelMap
                            | SPA_PROP_monitorMute
                            | SPA_PROP_monitorVolumes
                            | SPA_PROP_softMute
                            | SPA_PROP_softVolumes => (),
                            SPA_PROP_latencyOffsetNsec => {
                                if let Value::Long(ns) = property.value {
                                    let mut info = state.process_latency;
                                    info.ns = ns;
                                    unsafe { handle_process_latency(state, info) };
                                }
                            }
                            SPA_PROP_params => {
                                let res = unsafe { D::set_props_params(state, &property.value) };
                                if res != 0 {
                                    return res;
                                }
                            }
                            key => {
                                crate::debug!(state.log, "ignoring unknown prop {}", key);
                            }
                        }
                    }
                }
                _ => return -libc::EINVAL,
            }
            0
        }
        SPA_PARAM_ProcessLatency => {
            if param.is_null() {
                unsafe { handle_process_latency(state, crate::utils::process_latency_default()) };
                return 0;
            }
            match unsafe { crate::utils::parse_process_latency_info(param) } {
                Some(info) => {
                    unsafe { handle_process_latency(state, info) };
                    0
                }
                None => -libc::EINVAL,
            }
        }
        id => {
            crate::warn!(state.log, "set_param: unknown param {}", id);
            -libc::ENOENT
        }
    }
}

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time); both directions share
// the skeleton, with the fill measurement and error sign supplied through
// the Direction servo_* hooks. Returns (corr, delay) for the clock.
unsafe fn timeout_servo<D: Direction>(state: &mut State<D>, nsec: u64, rate: u32) -> (f64, i64) {
    let mut corr: f64 = 1.0;
    let mut delay: i64 = 0;
    for port in &mut state.ports {
        let Some((stride, device_rate)) = port.stride_rate() else {
            continue;
        };
        let device_rate = device_rate.max(1);
        if !port.dsp.is_running()
            || port.setup_period == 0
            || port.resetup_pending
            || !D::servo_ready(port)
        {
            continue;
        }

        let fill = D::servo_fill(port);
        // device frames scale to the graph rate; the resampler queue is already
        // graph-side (audioconvert reports it unscaled, like ALSA adds it)
        let resamp = port.rate_match.with(|rm| rm.delay as i64).unwrap_or(0);
        delay = (fill as i64 / stride as i64) * rate as i64 / device_rate as i64 + resamp;

        if D::servo_hold(port) {
            continue; // recovering; process() is discarding buffers, hold the servo
        }

        // clamp the error so a wakeup-jitter spike can't wind up the integrator
        // against an actuator that moves slowly (ALSA clamps to max_error too)
        let err_raw = D::servo_err(port, fill);
        let max_err = (port.setup_period as f64 / 2.0).max(256.0 * stride as f64);
        let err = err_raw.clamp(-max_err, max_err);
        corr = port.dll.update(err);
        port.bw_adapt.update(&mut port.dll, err, nsec);

        // a diverged servo must not wedge the graph clock
        if !(0.5..=2.0).contains(&corr) {
            crate::warn!(
                state.log,
                "{}: DLL diverged (corr {}); relocking",
                port.dsp.path(),
                corr
            );
            port.dll.init();
            port.bw_adapt.reset();
            corr = 1.0;
        }

        #[cfg(debug_assertions)]
        eprintln!("{}: corr = {}, err = {}", port.dsp.path(), corr, err_raw);
    }
    (corr, delay)
}

// ALSA adapts the DLL bandwidth continuously from the error variance
// (alsa-pcm.c, BW_PERIOD); we approximate with two stages: a fast lock at
// BW_MAX after (re)start, then the low steady-state bandwidth
unsafe extern "C" fn on_timeout<D: Direction>(source: *mut spa_source) {
    // the timer source we registered in init; its data points at our State
    let state = unsafe { (*source).data.cast::<State<D>>().as_mut() }
        .expect("(*source).data is not supposed to be null");

    #[cfg(debug_assertions)]
    crate::trace!(state.log, "on_timeout");

    let mut expirations = 0;
    if unsafe {
        state
            .data_system
            .timerfd_read(state.timer_source.fd, &mut expirations)
    } < 0
    {
        // disarmed (Pause/Suspend) in this same wakeup; nothing to read
        return;
    }

    // after the drain: the source is level-triggered, so bailing with the fd
    // readable would busy-spin the loop; the one-shot timer is only re-armed
    // by set_timeout below, so returning here really does park it
    if !check_loop_identity(state) {
        return;
    }

    // stopped between the timer firing and this callback; don't signal ready()
    // into a node being reconfigured, and don't re-arm
    if !state.started || state.following {
        return;
    }

    if state.position.is_null() || state.clock.is_null() {
        return; // ios cleared while the timer was armed; skip the cycle
    }

    // a failed clock read parks the timer (one-shot; only set_timeout re-arms)
    // instead of aborting the data loop
    let Some(now) = crate::utils::try_now_ns(&state.data_system) else {
        return;
    };

    // resync after a long stall instead of replaying a burst of stale cycles
    // (ALSA snaps when more than a second behind)
    if now.saturating_sub(state.next_time) > SPA_NSEC_PER_SEC as u64 {
        crate::warn!(
            state.log,
            "timer stalled ({} ns behind); resyncing",
            now - state.next_time
        );
        state.next_time = now;
    }

    let nsec = state.next_time;

    unsafe { D::debug_cycle(state, now, nsec) };

    // position and clock were null-checked above and stay set for the cycle
    let (duration, rate) = state
        .position
        .with(|p| (p.clock.target_duration, p.clock.target_rate.denom))
        .unwrap_or((0, 0));
    if duration == 0 || rate == 0 {
        // malformed position: idle-tick, and advance next_time so the deadline
        // isn't stale when the position recovers
        state.next_time = nsec + SPA_NSEC_PER_SEC as u64 / 100;
        unsafe { set_timeout(state, state.next_time) };
        return;
    }

    let (corr, delay) = unsafe { timeout_servo(state, nsec, rate) };

    // steer the timer by the correction so the published clock genuinely follows
    // the device (ALSA warps next_time the same way); this also closes the loop
    // in passthrough setups where no resampler consumes a rate_match
    state.next_time =
        nsec + (duration as f64 * SPA_NSEC_PER_SEC as f64 / (rate as f64 * corr)) as u64;

    let next_time = state.next_time;
    state.clock.with(|c| {
        c.nsec = nsec;
        c.rate = c.target_rate;
        c.position += c.duration;
        c.duration = duration;
        c.delay = delay;
        c.rate_diff = corr;
        c.next_nsec = next_time;
    });

    let Some(callbacks) = (unsafe { node_callbacks(&state.callbacks) }) else {
        unsafe { set_timeout(state, state.next_time) };
        return; // no callbacks (yet, or cleared); keep the clock ticking
    };
    if let Some(ready_fun) = callbacks.ready {
        let err = unsafe { ready_fun(state.callbacks.data, D::READY_STATUS) };
        #[cfg(debug_assertions)]
        crate::trace!(state.log, "ready -> {}", err);
        #[cfg(not(debug_assertions))]
        let _ = err;
    }

    unsafe { set_timeout(state, state.next_time) };
}

// Data loop only. Arm the wakeup timer from now when this node drives the
// graph (started, not following, position present); park it otherwise. A
// failed clock read parks too - process()/on_timeout degrade the same way -
// rather than aborting the data loop (the sink's former copy assert!()ed).
pub(crate) unsafe fn update_timers<D: Direction>(state: &mut State<D>) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "update_timers");

    let next = if state.started && !state.following && !state.position.is_null() {
        crate::utils::try_now_ns(&state.data_system).unwrap_or(0)
    } else {
        0
    };
    if next != 0 {
        state.next_time = next;
    }
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "next time {}", next);
    unsafe { set_timeout(state, next) };
}

pub(crate) unsafe fn set_timeout<D: Direction>(state: &mut State<D>, next_time: u64) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "set_timeout {}", next_time);

    let timerspec = itimerspec {
        it_value: timespec {
            tv_sec: (next_time / SPA_NSEC_PER_SEC as u64) as i64,
            tv_nsec: (next_time % SPA_NSEC_PER_SEC as u64) as i64,
        },
        it_interval: timespec {
            tv_sec: 0,
            tv_nsec: 0,
        },
    };

    unsafe {
        state.data_system.timerfd_settime(
            state.timer_source.fd,
            SPA_FD_TIMER_ABSTIME as i32,
            &timerspec,
            std::ptr::null_mut(),
        );
    }
}

// the io areas set_io accepts, with the geometry a full deref needs
const NODE_IO_AREAS: [(u32, usize, usize); 2] = [
    (
        SPA_IO_Clock,
        std::mem::size_of::<spa_io_clock>(),
        std::mem::align_of::<spa_io_clock>(),
    ),
    (
        SPA_IO_Position,
        std::mem::size_of::<spa_io_position>(),
        std::mem::align_of::<spa_io_position>(),
    ),
];

// ditto for port_set_io
const PORT_IO_AREAS: [(u32, usize, usize); 2] = [
    (
        SPA_IO_Buffers,
        std::mem::size_of::<spa_io_buffers>(),
        std::mem::align_of::<spa_io_buffers>(),
    ),
    (
        SPA_IO_RateMatch,
        std::mem::size_of::<spa_io_rate_match>(),
        std::mem::align_of::<spa_io_rate_match>(),
    ),
];

// The io-area admission shared by set_io and port_set_io: an id outside the
// caller's table is -ENOENT; NULL/0 clears the area; a non-empty area must
// admit a full deref of the struct. A short one is -ENOSPC - the installed
// header specifies it for both entry points ("-ENOSPC when \a size is too
// small", spa/node/node.h set_io and port_set_io) - while a misaligned one
// stays the generic invalid-argument -EINVAL (no closer errno is specified).
fn io_area_ok(table: &[(u32, usize, usize)], id: u32, data: *const c_void, size: usize) -> c_int {
    let Some(&(_, min_size, align)) = table.iter().find(|(t, _, _)| *t == id) else {
        return -libc::ENOENT;
    };
    if !data.is_null() {
        if size < min_size {
            return -libc::ENOSPC;
        }
        if data as usize % align != 0 {
            return -libc::EINVAL;
        }
    }
    0
}

unsafe extern "C" fn set_io<D: Direction>(
    object: *mut c_void,
    id: u32,
    data: *mut c_void,
    size: usize,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null());

    let res = io_area_ok(&NODE_IO_AREAS, id, data, size);
    if res != 0 {
        return res;
    }

    // clock/position are read on the data loop; apply the change there.
    // SAFETY: the host keeps the io areas valid while set (set_io contract)
    let data = unsafe { crate::utils::SendWrap::new(data) };
    let applied = unsafe {
        crate::utils::block_on_loop(&(*state).data_loop, state, move |state| {
            let data = data.into_inner();
            let was_armed = !state.clock.is_null() && !state.position.is_null();

            #[allow(non_upper_case_globals)]
            match id {
                SPA_IO_Clock => {
                    // SAFETY: size/alignment validated above; the host keeps
                    // the area valid while set (the set_io contract)
                    state.clock.set(data); // null clears

                    // identify our clock so same-device followers can skip rate matching
                    state
                        .clock
                        .with(|c| crate::utils::set_clock_name(c, &state.clock_name));
                }
                // SAFETY: as above
                SPA_IO_Position => state.position.set(data), // null clears
                _ => (),                                     // filtered above
            };

            if state.started {
                let armed = !state.clock.is_null() && !state.position.is_null();
                let following = state.node_is_follower();
                let flipped = state.following != following;
                if flipped {
                    state.following = following;
                    D::on_role_flip(state);
                }
                // rearm/park only on a real transition (io presence or role); resetting
                // the timer phase on every re-point causes cycle bunching
                if flipped || was_armed != armed {
                    update_timers(state);
                }
            }
        })
    };
    if !applied {
        return -libc::EIO;
    }

    0
}

unsafe extern "C" fn send_command<D: Direction>(
    object: *mut c_void,
    command: *const spa_command,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    assert!(!command.is_null());
    let body = unsafe { (*command).body.body };

    crate::debug!(
        state.log,
        "received command: {}",
        crate::utils::spa_command_to_str(&body)
    );

    #[allow(non_upper_case_globals)]
    match (body.type_, body.id) {
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start) => {
            if state
                .ports
                .iter()
                .any(|p| p.config.is_none() || p.buffers.is_empty())
            {
                return -libc::EIO; // not negotiated yet (ALSA rejects this too)
            }
            let state: *mut State<D> = state;
            if !unsafe {
                crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
                    // sane clock delay/rate_diff until process() publishes measured values
                    state.clock.with(|c| {
                        c.delay = 0;
                        c.rate_diff = 1.0;
                    });

                    D::on_start_loop(state);

                    state.started = true;
                    state.following = state.node_is_follower();

                    update_timers(state);
                })
            } {
                return -libc::EIO;
            }
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => {
            let state: *mut State<D> = state;
            if !unsafe {
                crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
                    state.started = false;
                    update_timers(state);
                })
            } {
                return -libc::EIO;
            }
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => {
            // Quiesce the loop first (blocking), then stop the channels from this
            // (main) thread - SETTRIGGER's chn_abort can sleep. The fd stays open,
            // so resume is a re-prime instead of a device rebuild; a driver that
            // refuses the trigger falls back to the close/rebuild path.
            let state: *mut State<D> = state;
            if !unsafe {
                crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
                    state.started = false;
                    update_timers(state);
                    D::on_suspend_loop(state);
                })
            } {
                return -libc::EIO;
            }
            // dsp is loop-owned, but the blocking started=false above quiesced the
            // loop: no process/on_timeout touches it again before a later blocking
            // invoke re-establishes ordering, so the main thread owns it here
            for port in unsafe { &mut (*state).ports } {
                if !port.dsp.is_closed() && !port.dsp.suspend() {
                    port.dsp.close();
                }
            }
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin | SPA_NODE_COMMAND_ParamEnd) => 0, // we don't care
        (cmd_type, cmd_id) => {
            crate::warn!(
                state.log,
                "{}unknown command: {}, {}",
                D::CMD_WARN_PREFIX,
                cmd_type,
                cmd_id
            );
            -libc::ENOTSUP
        }
    }
}

unsafe extern "C" fn add_port(
    _object: *mut c_void,
    _direction: spa_direction,
    _port_id: u32,
    _props: *const spa_dict,
) -> c_int {
    -libc::ENOTSUP // the ports are static
}

unsafe extern "C" fn remove_port(
    _object: *mut c_void,
    _direction: spa_direction,
    _port_id: u32,
) -> c_int {
    -libc::ENOTSUP // the ports are static
}

// No EnumPortConfig/PortConfig params here, on purpose: a follower's
// PortConfig surface is dead code under the adapter. audioadapter answers
// both params itself in passthrough and from its convert node otherwise
// (audioadapter.c:221) and only mirrors PropInfo/Props/ProcessLatency from
// the follower's node info (follower_info, audioadapter.c:1312); WirePlumber
// never reads them either - it probes EnumFormat and writes PortConfig on
// the adapter (module-si-audio-adapter.c si_audio_adapter_find_format /
// set_ports_format). Passthrough mode is carried entirely by the port
// params below: reconfigure_mode sets our Format with the NEAREST flag
// (audioadapter.c:758) and the graph link then negotiates buffers against
// the port directly (negotiate_buffers/negotiate_format short-circuit when
// follower == target, audioadapter.c:445, :995).

// replays the negotiated format exactly, for port_enum_params(Format)
unsafe fn build_port_format_info(
    builder: &mut libspa::pod::builder::Builder,
    config: &PortConfig,
    id: u32,
) {
    let mut position = [0u32; 64];
    for (slot, &p) in position.iter_mut().zip(config.positions.iter()) {
        *slot = p;
    }

    let raw = spa_audio_info_raw {
        format: config.format.0,
        flags: config.flags,
        rate: config.rate,
        channels: config.channels,
        position,
    };

    // the raw struct is fully initialized above; output goes into the builder
    unsafe { spa_format_audio_raw_build(builder.as_raw_ptr(), id, &raw) };
}

unsafe extern "C" fn port_enum_params<D: Direction>(
    object: *mut c_void,
    seq: c_int,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }
    if max == 0 {
        return 0;
    }

    let mut buffer = vec![];
    let mut fbuffer = vec![]; // spa_pod_filter output; kept apart from the source pod (see spa::filter_pod)

    let mut index = start;
    let mut count = 0;

    while count < max {
        use libspa::pod::builder::Builder;

        let mut builder = Builder::new(&mut buffer);

        #[allow(non_upper_case_globals)]
        match (id, index) {
            (SPA_PARAM_EnumFormat, i) => {
                if state.caps_fallback {
                    // the init-time probe hit a busy device and baked in fallback
                    // caps; retry now (main thread, transient open)
                    if let Some(caps) = crate::sound::probe_caps(&state.dsp_path, D::PLAYBACK) {
                        crate::info!(state.log, "re-probed caps: {:?}", caps);
                        state.caps = caps;
                        state.caps_fallback = false;
                    }
                }
                if !crate::utils::build_enum_format_info(&mut builder, &state.caps, i).unwrap() {
                    return 0;
                }
            }
            (SPA_PARAM_Format, 0) => {
                match state.ports[port_id as usize].config.as_ref() {
                    Some(cfg) => {
                        unsafe { build_port_format_info(&mut builder, cfg, SPA_PARAM_Format) };
                    }
                    None => return -libc::ENOENT, // no format negotiated yet
                }
            }
            (SPA_PARAM_Buffers, 0) => {
                match state.ports[port_id as usize].config.as_ref() {
                    Some(cfg) => {
                        crate::utils::build_buffers_info(&mut builder, cfg.stride).unwrap();
                    }
                    None => return -libc::ENOENT, // format not negotiated yet
                }
            }
            (SPA_PARAM_Latency, 0 | 1) => {
                let mut info = state.latency[index as usize];
                // the process latency shifts what we report toward the peer (upstream
                // for the sink, downstream for the source)
                if info.direction == D::DIRECTION {
                    crate::utils::process_latency_info_add(&state.process_latency, &mut info);
                }
                crate::utils::build_latency_info(&mut builder, &info).unwrap();
            }
            // a known id whose indices are exhausted ends the enumeration
            (SPA_PARAM_Format | SPA_PARAM_Buffers | SPA_PARAM_Latency, _) => return 0,
            _ => return -libc::ENOENT, // unknown param id (ALSA convention)
        };

        drop(builder); // its borrow of `buffer` must end before we take the source pointer

        let mut result = spa_result_node_params {
            id,
            index,
            next: index + 1,
            param: std::ptr::null_mut(),
        };

        // the built pod lives in `buffer`, distinct from the filter output
        if let Some(param) = unsafe {
            crate::spa::filter_pod(&mut fbuffer, buffer.as_mut_ptr() as *mut spa_pod, filter)
        } {
            result.param = param;
            unsafe {
                crate::spa::node_emit_result(
                    &mut state.hooks,
                    seq,
                    0,
                    SPA_RESULT_TYPE_NODE_PARAMS,
                    &result,
                );
            }
            count += 1;
        }

        index += 1;
    }

    0
}

// port_set_param(Format): validate the raw format against the format map and
// build the shared config (the stride falls out of the map's bytes/sample)
fn parse_config<D: Direction>(
    state: &State<D>,
    raw: &spa_audio_info_raw,
) -> Result<PortConfig, c_int> {
    let format = libspa::param::audio::AudioFormat(raw.format);

    // only formats from our EnumFormat are expected; reject the rest
    let Some((_, bytes_per_sample)) = crate::utils::oss_format_info(raw.format) else {
        crate::warn!(state.log, "rejecting unsupported format {:?}", format);
        return Err(-libc::ENOTSUP);
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

    Ok(config)
}

// port_set_param(Format) with a pod: parse and validate the requested raw
// format, snap it onto the caps under the NEAREST flag, and install the
// device. Ok(res) falls through to the port-info re-emit (even on a failed
// install - the flags derive from the then-cleared config); Err returns to
// the host without emitting, matching the ALSA plugin's early rejects.
// Ok(1) = the format deviates from the request (the adapter then re-reads
// our Format param, alsa-pcm.c:2548 / audioadapter.c:596).
unsafe fn set_format_param<D: Direction>(
    state: &mut State<D>,
    port_idx: usize,
    flags: u32,
    param: *const spa_pod,
) -> Result<c_int, c_int> {
    use libspa::param::format::{MediaSubtype, MediaType};
    use libspa::param::format_utils::parse_format;

    match parse_format(unsafe { libspa::pod::Pod::from_raw(param) }) {
        Ok((MediaType::Audio, MediaSubtype::Raw)) => (),
        Ok((t, st)) => {
            crate::warn!(
                state.log,
                "unknown media type combination: {:?}, {:?}",
                t,
                st
            );
            return Err(-libc::ENOENT);
        }
        Err(err) => {
            crate::warn!(state.log, "parse_format failed: {}", err);
            return Err(-libc::EINVAL);
        }
    }

    let mut raw = MaybeUninit::<spa_audio_info_raw>::uninit();
    if unsafe { spa_format_audio_raw_parse(param, raw.as_mut_ptr()) } < 0 {
        crate::warn!(state.log, "spa_format_audio_raw_parse failed");
        return Err(-libc::EINVAL);
    }

    // a non-negative parse filled the struct
    let mut raw = unsafe { raw.assume_init() };

    // reject bad values rather than assert (an FFI panic aborts pipewire);
    // format flags are stored but unused, OSS writes interleaved frames
    if raw.rate == 0 || raw.channels == 0 || raw.channels > SPA_AUDIO_MAX_CHANNELS {
        crate::warn!(
            state.log,
            "rejecting format: rate={} channels={}",
            raw.rate,
            raw.channels
        );
        return Err(-libc::EINVAL);
    }

    // audioadapter always sets the follower format with NEAREST
    // (audioadapter.c:758, :1059); snap only what the exact path
    // below would reject, so in-caps requests stay untouched
    let admitted = |caps: &crate::sound::DspCaps, raw: &spa_audio_info_raw| {
        crate::utils::oss_format_info(raw.format)
            .is_some_and(|(m, _)| caps.admits(m, raw.channels, raw.rate))
    };
    let mut snapped = false;
    if flags & crate::spa::SPA_NODE_PARAM_FLAG_NEAREST != 0 && !admitted(&state.caps, &raw) {
        snapped = crate::utils::snap_raw_to_caps(&state.caps, &mut raw);
        if snapped {
            crate::info!(
                state.log,
                "snapped requested format to caps: format={} rate={} channels={}",
                raw.format,
                raw.rate,
                raw.channels
            );
        }
    }

    let config = parse_config(state, &raw)?;

    // Validate against the advertised caps first: an out-of-caps
    // request on an exclusive device would EBUSY-retire the WORKING
    // fd and then fail configure, killing the stream for nothing.
    // configure() stays as the backstop for stale caps (a rejection
    // there re-probes and re-announces).
    if !state
        .caps
        .admits(config.oss_format(), raw.channels, raw.rate)
    {
        crate::warn!(
            state.log,
            "rejecting out-of-caps format: rate={} channels={}",
            raw.rate,
            raw.channels
        );
        return Err(-libc::EINVAL);
    }

    let mut res = unsafe { install_device(state, port_idx, config) };
    if res == 0 && snapped {
        res = 1;
    }
    if res == -libc::EINVAL || res == -libc::ENOTSUP {
        // the device rejected caps-derived values: the snapshot may be
        // stale (vchans/bitperfect toggled at runtime); re-probe and
        // re-announce EnumFormat so the host renegotiates from reality
        if let Some(caps) = crate::sound::probe_caps(&state.dsp_path, D::PLAYBACK) {
            state.caps_fallback = false;
            // bump only on a real change: the serial flip re-triggers the
            // adapter's negotiation, and an unchanged snapshot would loop
            // it against the same rejection
            if caps != state.caps {
                crate::info!(state.log, "re-probed caps after rejection: {:?}", caps);
                state.caps = caps;
                state.port_info.bump_param(SPA_PARAM_EnumFormat);
            }
        }
    }
    Ok(res)
}

// port_set_param(Format) with a NULL pod: release the format. Close the
// device and drop the buffers (the refused trigger-suspend may have closed
// the dsp, hence the guard); all three are read by process(), so do it from
// the data loop.
unsafe fn release_format<D: Direction>(state: &mut State<D>, port_idx: usize) -> c_int {
    let state_ptr: *mut State<D> = state;
    if !unsafe {
        crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
            let port = &mut state.ports[port_idx];
            if !port.dsp.is_closed() {
                port.dsp.close();
            }
            port.buffers.clear();
            port.config = None;
        })
    } {
        return -libc::EIO; // the loop still holds the buffers; freeing them would dangle
    }
    0
}

// update the port rate and flip Format/Buffers flags to reflect whether a
// format is negotiated, then re-emit so the host re-reads them (PipeWire
// ALSA sink/source pattern)
unsafe fn publish_format_state<D: Direction>(state: &mut State<D>, port_idx: usize) {
    let _ = state.port_info.replace_change_mask(0);
    if let Some(cfg) = state.ports[port_idx].config.as_ref() {
        state.port_info.set_rate(spa_fraction {
            num: 1,
            denom: cfg.rate,
        });
        state
            .port_info
            .set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
        state
            .port_info
            .set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
    } else {
        state
            .port_info
            .set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
        state.port_info.set_param_flags(SPA_PARAM_Buffers, 0);
    }
    unsafe { emit_port_info(state) };
}

// the host writes the reverse-direction latency (downstream for the sink,
// upstream for the source); store it and re-emit so it propagates through
// the graph
unsafe fn set_latency_param<D: Direction>(
    state: &mut State<D>,
    direction: spa_direction,
    param: *const spa_pod,
) -> c_int {
    let other = direction ^ 1;
    let info = if param.is_null() {
        crate::utils::latency_info_default(other)
    } else {
        match unsafe { crate::utils::parse_latency_info(param) } {
            Some(info) if info.direction == other => info,
            _ => return -libc::EINVAL,
        }
    };
    state.latency[info.direction as usize] = info;

    let _ = state.port_info.replace_change_mask(0);
    state.port_info.bump_param(SPA_PARAM_Latency);
    unsafe { emit_port_info(state) };

    0
}

unsafe extern "C" fn port_set_param<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }

    #[allow(non_upper_case_globals)]
    match id {
        SPA_PARAM_Format => {
            let res = if !param.is_null() {
                match unsafe { set_format_param(state, port_id as usize, flags, param) } {
                    Ok(res) => res,
                    Err(err) => return err,
                }
            } else {
                match unsafe { release_format(state, port_id as usize) } {
                    0 => 0,
                    err => return err,
                }
            };
            // emit even on failure: the flags derive from the (now cleared) config
            unsafe { publish_format_state(state, port_id as usize) };
            res
        }
        SPA_PARAM_Latency => unsafe { set_latency_param(state, direction, param) },
        SPA_PARAM_Tag => 0,
        id => {
            crate::warn!(state.log, "port_set_param: unknown param {}", id);
            -libc::ENOENT
        }
    }
}

// oss.fragment: 0 = automatic; otherwise round DOWN to a power of two and
// clamp to [64, 16384] bytes. The kernel would take 16..65536 (dsp.c:1251
// RANGE(fragln, 4, 16)); staying well inside keeps the request grantable
// verbatim and the buffer budget sane (CHN_2NDBUFMAXSIZE, channel.h:442).
pub(crate) fn normalize_fragment(v: u32) -> u32 {
    if v == 0 {
        0
    } else {
        (1u32 << (31 - v.leading_zeros())).clamp(64, 16384)
    }
}

// The oss.* tunable live re-apply path: store the new loop-owned value on the
// data loop (the prime paths read it there), then rebuild any running port
// from this (main) thread so the next cycle re-primes with the new layout.
pub(crate) unsafe fn store_and_rebuild<D: Direction>(
    state: &mut State<D>,
    store: impl FnOnce(&mut State<D>) + Send,
) -> c_int {
    let mut running = [false; MAX_PORTS];
    let applied = {
        let running_ref = &mut running;
        let state_ptr: *mut State<D> = state;
        unsafe {
            crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
                store(state);
                // dsp state is loop-owned; snapshot it here
                for (i, port) in state.ports.iter().enumerate() {
                    running_ref[i] = port.dsp.is_running();
                }
            })
        }
    };
    if !applied {
        return -libc::EIO;
    }
    for (port_idx, &was_running) in running.iter().enumerate() {
        if !was_running {
            continue; // not streaming; picked up at the next start/prime
        }
        if let Some(config) = state.ports[port_idx].config.clone() {
            if unsafe { install_device(state, port_idx, config) } != 0 {
                // the host didn't initiate this rebuild; without a re-announce it
                // keeps believing a format is set on a dead port
                unsafe { emit_format_lost(state) };
            }
        }
    }
    0
}

// announce a Props change (so readback stays fresh), then apply it through
// store_and_rebuild; shared by the sink's and source's set_props_params
pub(crate) unsafe fn apply_props_param<D: Direction>(
    state: &mut State<D>,
    store: impl FnOnce(&mut State<D>) + Send,
) -> c_int {
    let _ = state.node_info.replace_change_mask(0);
    state.node_info.bump_param(SPA_PARAM_Props);
    unsafe { emit_node_info(state) };
    unsafe { store_and_rebuild(state, store) }
}

// Open and configure on the calling (main) thread - device opens can sleep for
// tens of ms and must stay off the shared data loop - then swap only the
// pointers there and close the old device back here. Exclusive devices
// (bitperfect, vchans off) allow a single open per direction, so EBUSY retires
// the old device first and retries, accepting a brief gap. On failure the
// port is left cleared.
pub(crate) unsafe fn install_device<D: Direction>(
    state: &mut State<D>,
    port_idx: usize,
    config: PortConfig,
) -> c_int {
    let mut new_dsp = D::Device::new(&state.dsp_path);
    // oss_fragment only mutates from main-thread calls, serialized with us
    let mut res = D::try_open_configure(&mut new_dsp, &config, state.oss_fragment, &state.log);

    if res == -libc::EBUSY {
        let mut retired = None;
        {
            let retired_ref = &mut retired;
            let closed = D::Device::new(&state.dsp_path);
            let state_ptr: *mut State<D> = state;
            if !unsafe {
                crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
                    *retired_ref = Some(std::mem::replace(&mut state.ports[port_idx].dsp, closed));
                    // a cycle landing in this window must skip, not queue a rebuild of
                    // the device we are about to install (cleared by the final swap)
                    state.ports[port_idx].resetup_pending = true;
                })
            } {
                return -libc::EIO;
            }
        }
        drop(retired); // closes the old fd here, off the RT path
        res = D::try_open_configure(&mut new_dsp, &config, state.oss_fragment, &state.log);
    }

    let ok = res == 0;
    let mut old_dsp = None;
    let swapped = {
        let old_ref = &mut old_dsp;
        let state_ptr: *mut State<D> = state;
        unsafe {
            crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
                let port = &mut state.ports[port_idx];
                // new_dsp is a closed writer/reader when negotiation failed above
                *old_ref = Some(std::mem::replace(&mut port.dsp, new_dsp));
                port.config = if ok { Some(config) } else { None };
                port.resetup_pending = false;
                port.was_matching = false; // force a relock when matching resumes
                D::on_device_swapped(state, port_idx);
            })
        }
    };
    drop(old_dsp); // ditto

    if !swapped {
        return -libc::EIO; // the swap never ran; the port keeps its old state
    }
    if res == 0 {
        unsafe { publish_ring_quantum_cap(state, port_idx) }; // stride is known now
    }
    res
}

// FreeBSD caps every soft ring at CHN_2NDBUFMAXSIZE (131 KiB); at fat strides
// (a 20-channel S32 interface is 80 bytes/frame) the ring holds only ~1.6
// periods at quantum 1024 and both directions glitch structurally - the
// capture side has no room for arrival jitter, the playback side can't hold
// two quanta plus the delay target. Publish node.max-latency once the stride
// is known so the graph never negotiates a quantum the kernel ring can't hold
// four of (pw_impl_node parses the fraction into max_latency, which caps the
// driver quantum). Emitted only when the cap bites below the common
// 2048-frame default in TIME, at a conservative 44.1 kHz reference -
// clock.rate is unknown here and an over-published cap is inert (sound.rs
// advertised_quantum_cap_frames); published once -
// the props dict is append-only, and a stride change without a node rebuild
// is not worth a duplicate entry.
unsafe fn publish_ring_quantum_cap<D: Direction>(state: &mut State<D>, port_idx: usize) {
    let Some(config) = state.ports[port_idx].config.as_ref() else {
        return;
    };
    let stride = config.stride.max(1);
    let rate = config.rate;
    // the shared ring policy (sound.rs); the published fraction is time-based
    // (frames/device rate), so it needs no graph-rate scaling
    let Some(frames) = crate::sound::advertised_quantum_cap_frames(stride, rate) else {
        return;
    };
    if state.ring_cap_published {
        return;
    }
    state.ring_cap_published = true;
    crate::info!(
        state.log,
        "kernel ring ({} bytes) at stride {} holds 4 periods only up to \
    quantum {}; publishing node.max-latency",
        crate::sound::ring_byte_cap(stride, rate),
        stride,
        frames
    );
    let _ = state.node_info.replace_change_mask(0);
    state
        .node_info
        .add_prop("node.max-latency", format!("{frames}/{rate}"));
    unsafe { emit_node_info(state) };
}

// A device rebuild the HOST didn't initiate just failed and cleared the
// config: flip the param flags and re-emit port info so the session manager
// renegotiates, instead of stranding a silently dead node (port_set_param
// does the same for host-initiated failures).
unsafe fn emit_format_lost<D: Direction>(state: &mut State<D>) {
    let _ = state.port_info.replace_change_mask(0);
    state
        .port_info
        .set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
    state.port_info.set_param_flags(SPA_PARAM_Buffers, 0);
    // the EnumFormat serial flip is what audioadapter actually reacts to: only
    // an EnumFormat flags change sets its recheck_format (audioadapter.c
    // follower_port_info), so without it the adapter keeps have_format=true
    // and never re-issues set_param(Format)
    state.port_info.bump_param(SPA_PARAM_EnumFormat);
    unsafe { emit_port_info(state) };
}

// queue a main-thread rebuild of `port_idx`'s device (resetup_task); false =
// no main loop or the invoke failed, and the caller keeps running degraded.
// Takes the raw state pointer because callers hold &mut Port from the ports
// iteration; only the disjoint main_loop field is read here.
pub(crate) unsafe fn queue_resetup<D: Direction>(
    state_ptr: *mut State<D>,
    port_idx: usize,
) -> bool {
    unsafe {
        (*state_ptr).main_loop.as_ref().is_some_and(|main_loop| {
            crate::utils::invoke_on_loop(main_loop, state_ptr, move |state| {
                resetup_task(state, port_idx);
            })
        })
    }
}

// spa_node_callbacks leads with `version: u32` (the SPA vtable convention,
// spa/node/node.h); node_callbacks' prefix read below depends on it
const _: () = assert!(std::mem::offset_of!(spa_node_callbacks, version) == 0);

// The host's callback table behind its version gate. Read ONLY the version
// prefix (offset 0, asserted above) first: a host built against an older,
// shorter table must be rejected before a full spa_node_callbacks - possibly
// larger in this build - is read out of its allocation. None = no table set
// (yet, or cleared) or one predating the fields we call.
//
// # Safety
// `callbacks.funcs`, when non-null, must point at a live node-callbacks
// table (the set_callbacks contract; the host keeps it alive while set).
pub(crate) unsafe fn node_callbacks(callbacks: &spa_callbacks) -> Option<&spa_node_callbacks> {
    if callbacks.funcs.is_null() {
        return None;
    }
    // only the version prefix until the gate passes
    let version = unsafe { callbacks.funcs.cast::<u32>().read() };
    if !crate::spa::version_ok(version, SPA_VERSION_NODE_CALLBACKS) {
        return None;
    }
    // version >= ours: the table spans our whole struct
    unsafe { callbacks.funcs.cast::<spa_node_callbacks>().as_ref() }
}

// report an xrun EVENT to the host (pw-top's xrun counter); the length
// isn't known at detection, so 0 delay
pub(crate) unsafe fn emit_xrun(callbacks: &spa_callbacks, trigger_us: u64) {
    if let Some(xrun_fun) = unsafe { node_callbacks(callbacks) }.and_then(|c| c.xrun) {
        unsafe { xrun_fun(callbacks.data, trigger_us, 0, std::ptr::null_mut()) };
    }
}

// runs on the main thread (queued from the data loop via invoke_on_loop)
pub(crate) unsafe fn resetup_task<D: Direction>(state: &mut State<D>, port_idx: usize) {
    if state.clearing {
        return; // teardown is flushing us out; don't touch the device
    }
    // a Suspend that landed after this task was queued must win: reopening
    // here would leave a suspended node holding an exclusive device
    // (started only mutates through blocking loop invokes, so this
    // main-thread read is serialized against them)
    if !state.started {
        let state_ptr: *mut State<D> = state;
        unsafe {
            crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
                for port in &mut state.ports {
                    port.resetup_pending = false;
                }
            });
        }
        return;
    }
    // consume-or-bail: an intervening install_device (renegotiation) already
    // cleared the flag, making this task stale
    let mut still_pending = false;
    {
        let pending_ref = &mut still_pending;
        let state_ptr: *mut State<D> = state;
        unsafe {
            crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
                *pending_ref = state.ports[port_idx].resetup_pending;
            });
        }
    }
    if !still_pending {
        return;
    }
    // config only mutates from main-thread calls, which are serialized with us
    match state.ports[port_idx].config.clone() {
        Some(config) => {
            if unsafe { install_device(state, port_idx, config) } != 0 {
                unsafe { emit_format_lost(state) };
            }
        }
        None => {
            let state_ptr: *mut State<D> = state;
            unsafe {
                crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
                    state.ports[port_idx].resetup_pending = false;
                });
            }
        }
    }
}

// PipeWire doesn't contract that the DataLoop in spa_support (where our timer
// lives and every marshaled mutation runs) is the loop process() is driven
// from; under multi-data-loop configs (context.num-data-loops > 1) the two
// are independent acquires and can diverge, breaking every serialization
// invariant here. Detect it and refuse to process rather than corrupt; the
// remedy is pinning node.loop.name for this node.
fn check_loop_identity<D: Direction>(state: &State<D>) -> bool {
    use std::sync::atomic::Ordering;
    let tid = unsafe { libc::pthread_self() } as usize;
    // The expected id is SEEDED from a closure run on the data loop at init,
    // not claimed by whoever calls first: a pure follower never runs
    // on_timeout, so a process() arriving on a divergent host loop would
    // otherwise install itself as the expected thread and undo the
    // block_on_loop serialization.
    let seen =
        match state
            .loop_thread
            .compare_exchange(0, tid, Ordering::Relaxed, Ordering::Relaxed)
        {
            Ok(_) => tid, // seeding failed at init; degrade to first-caller-wins
            Err(seen) => seen,
        };
    if seen == tid {
        return true;
    }
    if seen != usize::MAX && state.loop_thread.swap(usize::MAX, Ordering::Relaxed) != usize::MAX {
        crate::warn!(
            state.log,
            "process() and our data loop run on different threads \
      (multi-data-loop config?); pin node.loop.name for this node. Disabling processing."
        );
    }
    false
}

unsafe extern "C" fn process<D: Direction>(object: *mut c_void) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    // a cycle that was already signaled when we paused can still land here; drop
    // it instead of assert!()ing, which aborts the daemon across extern "C"
    if !state.started || state.position.is_null() {
        return SPA_STATUS_OK as i32;
    }

    if !check_loop_identity(state) {
        return SPA_STATUS_OK as i32;
    }

    unsafe { D::process_ports(state) }
}

unsafe extern "C" fn port_use_buffers<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    flags: u32,
    buffers: *mut *mut spa_buffer,
    n_buffers: u32,
) -> c_int {
    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }
    let _ = flags;

    let new_buffers = if !buffers.is_null() && n_buffers > 0 {
        // the host passes n_buffers valid pointers; copied before the loop swap
        unsafe { std::slice::from_raw_parts(buffers, n_buffers as usize) }.to_vec()
    } else {
        vec![]
    };

    // process() walks this vec on the data loop; swap it there.
    // SAFETY: the host keeps the buffer pointers valid until the next
    // use_buffers call (the port_use_buffers contract)
    let port_idx = port_id as usize;
    let new_buffers = unsafe { crate::utils::SendWrap::new(new_buffers) };
    let state_ptr: *mut State<D> = state;
    if !unsafe {
        crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, move |state| {
            state.ports[port_idx].buffers = new_buffers.into_inner();
            D::on_buffers_swapped(state, port_idx);
        })
    } {
        return -libc::EIO; // keeping stale host buffer pointers would be a UAF
    }

    0
}

unsafe extern "C" fn port_set_io<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    data: *mut c_void,
    size: usize,
) -> c_int {
    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }

    let res = io_area_ok(&PORT_IO_AREAS, id, data, size);
    if res != 0 {
        return res;
    }

    let state =
        unsafe { object.cast::<State<D>>().as_mut() }.expect("object is not supposed to be null");

    // these pointers are dereferenced by process() on the data loop.
    // SAFETY: the host keeps the io areas valid while set (port_set_io
    // contract)
    let data = unsafe { crate::utils::SendWrap::new(data) };
    let state: *mut State<D> = state;
    let applied = unsafe {
        crate::utils::block_on_loop(&(*state).data_loop, state, move |state| {
            let data = data.into_inner();
            // SAFETY (both arms): size/alignment validated above; the host
            // keeps the area valid while set (the port_set_io contract)
            #[allow(non_upper_case_globals)]
            match id {
                SPA_IO_Buffers => state.ports[port_id as usize].io.set(data), // null clears
                // you'd think RateMatch would be a node parameter instead; ACTIVE is
                // managed per cycle in process(), only set while matching
                SPA_IO_RateMatch => state.ports[port_id as usize].rate_match.set(data),
                _ => (),
            }
        })
    };
    if !applied {
        return -libc::EIO;
    }

    0
}

unsafe extern "C" fn port_reuse_buffer(
    _object: *mut c_void,
    _port_id: u32,
    _buffer_id: u32,
) -> c_int {
    -libc::ENOTSUP // buffers are recycled through io.buffer_id
}

unsafe extern "C" fn get_interface<D: Direction>(
    handle: *mut spa_handle,
    type_: *const c_char,
    interface: *mut *mut c_void,
) -> c_int {
    let state =
        unsafe { handle.cast::<State<D>>().as_mut() }.expect("handle is not supposed to be null");
    assert!(!interface.is_null());
    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Node.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe { *interface = &mut state.node as *mut _ as *mut c_void };
    } else {
        return -libc::ENOENT;
    }
    0
}

unsafe extern "C" fn clear<D: Direction>(handle: *mut spa_handle) -> c_int {
    let state: *mut State<D> = handle.cast();
    assert!(!state.is_null());

    // A queued resetup_task holds this state pointer; a blocking self-invoke
    // on the main loop flushes all pending queue items (in submission order)
    // before we free anything, and `clearing` makes the flushed tasks no-op.
    unsafe { (*state).clearing = true };
    if let Some(main_loop) = unsafe { (*state).main_loop.as_ref() } {
        unsafe { crate::utils::block_on_loop(main_loop, state, |_| {}) };
    }

    // the data loop still holds the timer source; detach it there before the
    // state is freed, then close the timerfd
    if !unsafe {
        crate::utils::block_on_loop(&(*state).data_loop, state, |state| {
            state.data_loop.remove_source(&mut state.timer_source);
        })
    } {
        // freeing the state now would leave the loop a dangling source; a clean
        // abort beats a use-after-free on the next timer tick
        eprintln!("freebsd-oss: can't detach the timer source; aborting");
        std::process::abort();
    }
    unsafe { (*state).data_system.close((*state).timer_source.fd) };

    // the host frees the memory after clear; drop the fields exactly once here
    unsafe { std::ptr::drop_in_place(state) };
    0
}

pub(crate) extern "C" fn get_size<D: Direction>(
    _factory: *const spa_handle_factory,
    _params: *const spa_dict,
) -> usize {
    std::mem::size_of::<State<D>>()
}

// the init-dict node properties: the device path, the shared oss.fragment
// default and whatever direction-specific keys D::info_item consumes
unsafe fn parse_init_dict<D: Direction>(info: *const spa_dict) -> (Option<String>, u32, D::Ext) {
    let mut dsp_path = None;
    let mut oss_fragment = 0u32; // automatic (today's layout) unless the dict says otherwise
    let mut ext = D::Ext::default();

    if let Some(info) = unsafe { info.as_ref() } {
        #[cfg(debug_assertions)]
        unsafe {
            crate::spa::dump_spa_dict(info);
        }

        unsafe {
            crate::spa::for_each_dict_item(info, |key, value| {
                if key == crate::keys::OSS_DSP_PATH {
                    dsp_path = Some(value.to_string());
                } else if key == crate::keys::OSS_FRAGMENT {
                    // direction-shared per-device default, e.g. from a wireplumber node
                    // rule; stored normalized so readback reports the effective value
                    if let Ok(v) = value.parse::<u32>() {
                        oss_fragment = normalize_fragment(v);
                    }
                } else {
                    D::info_item(&mut ext, key, value);
                }
            });
        }
    }
    D::ext_ready(&mut ext);

    (dsp_path, oss_fragment, ext)
}

// the static node/port info published at init: flags, props and the param
// directory (the readable/writable flags flip later in port_set_param)
fn publish_static_info<D: Direction>(state: &mut State<D>) {
    state.node_info.fix_pointers();

    if D::DIRECTION == SPA_DIRECTION_INPUT {
        state.node_info.set_max_input_ports(1);
    } else {
        state.node_info.set_max_output_ports(1);
    }
    // the RT flag declares process() safe to call from the realtime data
    // loop (the ALSA plugin nodes set the same flag), and pw-top/clients
    // read it back from the node info
    state.node_info.set_flags(SPA_NODE_FLAG_RT as u64);

    state
        .node_info
        .add_prop(crate::spa::key(SPA_KEY_MEDIA_CLASS), D::MEDIA_CLASS);
    state
        .node_info
        .add_prop(crate::spa::key(SPA_KEY_NODE_DRIVER), "true");

    // no EnumPortConfig/PortConfig (or node-level IO/EnumFormat): dead
    // surface on a follower, see the note above build_port_format_info
    state
        .node_info
        .add_param(SPA_PARAM_PropInfo, SPA_PARAM_INFO_READ);
    state
        .node_info
        .add_param(SPA_PARAM_Props, SPA_PARAM_INFO_READWRITE);
    state
        .node_info
        .add_param(SPA_PARAM_ProcessLatency, SPA_PARAM_INFO_READWRITE);

    state.port_info.fix_pointers();

    state
        .port_info
        .set_flags((SPA_PORT_FLAG_PHYSICAL | SPA_PORT_FLAG_TERMINAL) as u64);
    // 1/48000 is the pre-negotiation placeholder; publish_format_state
    // replaces it with the negotiated rate on every port_set_param(Format)
    state.port_info.set_rate(spa_fraction {
        num: 1,
        denom: 48000,
    });

    // advertise the format as writable so the host (re)negotiates it; Buffers is
    // unreadable until a format is set (it needs the stride). Flags flip in
    // port_set_param.
    state
        .port_info
        .add_param(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
    state
        .port_info
        .add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
    state.port_info.add_param(SPA_PARAM_Buffers, 0);
    state
        .port_info
        .add_param(SPA_PARAM_Latency, SPA_PARAM_INFO_READWRITE);
}

pub(crate) unsafe extern "C" fn init<D: Direction>(
    _factory: *const spa_handle_factory,
    handle: *mut spa_handle,
    info: *const spa_dict,
    support: *const spa_support,
    n_support: u32,
) -> c_int {
    // the support array is the host's init contract: n_support valid entries
    let log =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast()) }
            as *mut spa_log;
    let log = unsafe { crate::spa::Log::wrap(log, Some(D::log_topic())) };

    let data_loop = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_DataLoop.as_ptr().cast(),
        )
    } as *mut spa_loop;
    let data_system = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_DataSystem.as_ptr().cast(),
        )
    } as *mut spa_system;
    let main_loop =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast()) }
            as *mut spa_loop;

    if data_loop.is_null() || data_system.is_null() {
        return -libc::EINVAL;
    }

    let data_loop = unsafe { crate::spa::Loop::wrap(data_loop) };
    let data_system = unsafe { crate::spa::System::wrap(data_system) };

    let timer_fd = unsafe {
        data_system.timerfd_create(
            libc::CLOCK_MONOTONIC,
            (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as i32,
        )
    };
    if timer_fd < 0 {
        return timer_fd; // fd exhaustion fails node creation, not the daemon
    }

    let (dsp_path, oss_fragment, ext) = unsafe { parse_init_dict::<D>(info) };

    let Some(dsp_path) = dsp_path else {
        unsafe { data_system.close(timer_fd) };
        crate::error!(
            log,
            "{} missing from the node properties",
            crate::keys::OSS_DSP_PATH
        );
        return -libc::EINVAL;
    };

    let mut caps_fallback = false;
    let caps = crate::sound::probe_caps(&dsp_path, D::PLAYBACK).unwrap_or_else(|| {
        crate::warn!(log, "{}: can't probe device caps; using fallback", dsp_path);
        caps_fallback = true;
        crate::sound::DspCaps::fallback()
    });
    crate::debug!(log, "{}: {:?}", dsp_path, caps);

    let state =
        unsafe { handle.cast::<State<D>>().as_mut() }.expect("handle is not supposed to be null");

    let node_methods: &'static spa_node_methods = &D::NODE_METHODS;

    // the host hands us uninitialized memory of get_size() bytes; write the
    // whole State without dropping the garbage "old" value
    unsafe {
        std::ptr::write(
            state,
            State {
                handle: spa_handle {
                    version: SPA_VERSION_HANDLE,
                    get_interface: Some(get_interface::<D>),
                    clear: Some(clear::<D>),
                },

                node: spa_node {
                    iface: spa_interface {
                        type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
                        version: SPA_VERSION_NODE,
                        cb: spa_callbacks {
                            funcs: node_methods as *const _ as *const c_void,
                            data: state as *mut _ as *mut c_void,
                        },
                    },
                },

                node_info: crate::spa::NodeInfo::new(),
                port_info: crate::spa::PortInfo::new(),

                data_loop,
                data_system,
                log,

                clock: crate::spa::IoArea::null(),
                position: crate::spa::IoArea::null(),
                clock_name: std::ffi::CString::new(format!(
                    "freebsd-oss.{}",
                    dsp_path.trim_start_matches("/dev/")
                ))
                .unwrap_or_default(),
                main_loop: if main_loop.is_null() {
                    None
                } else {
                    Some(crate::spa::Loop::wrap(main_loop))
                },
                dsp_path: dsp_path.clone(),

                timer_source: spa_source {
                    loop_: std::ptr::null_mut(),
                    func: Some(on_timeout::<D>),
                    data: state as *mut _ as *mut c_void,
                    fd: timer_fd,
                    mask: SPA_IO_IN,
                    rmask: 0,
                    priv_: std::ptr::null_mut(),
                },

                next_time: 0,

                hooks: spa_hook_list {
                    list: spa_list {
                        next: std::ptr::null_mut(),
                        prev: std::ptr::null_mut(),
                    },
                },

                callbacks: spa_callbacks {
                    funcs: std::ptr::null(),
                    data: std::ptr::null_mut(),
                },

                ports: [Port {
                    config: None,
                    buffers: vec![],
                    io: crate::spa::IoArea::null(),
                    rate_match: crate::spa::IoArea::null(),
                    dsp: D::Device::new(&dsp_path),
                    dll: std::default::Default::default(),
                    setup_period: 0,
                    bw_adapt: std::default::Default::default(),
                    setup_blocksize: 0,
                    resetup_pending: false,
                    was_matching: false,
                    warn_limit: crate::utils::RateLimit::new(),
                    ext: std::default::Default::default(),
                }; MAX_PORTS],

                caps,
                caps_fallback,
                oss_fragment,
                oss_fragment_default: oss_fragment,
                loop_thread: std::sync::atomic::AtomicUsize::new(0),

                latency: [
                    crate::utils::latency_info_default(SPA_DIRECTION_INPUT),
                    crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT),
                ],

                process_latency: crate::utils::process_latency_default(),

                started: false,
                clearing: false,
                following: false,
                ring_cap_published: false,

                ext,
            },
        );
    }

    publish_static_info(state);

    unsafe { spa_hook_list_init(&mut state.hooks) };

    let err = unsafe { state.data_loop.add_source(&mut state.timer_source) };
    if err < 0 {
        unsafe {
            state.data_system.close(state.timer_source.fd);
            // the host won't call clear() after a failed init; free what we built
            std::ptr::drop_in_place(state);
        }
        return err;
    }

    // learn the data loop's thread identity from the loop itself (see
    // check_loop_identity); pw's data loops run before any node loads, so
    // this executes on the loop thread, not inline
    let state_ptr: *mut State<D> = state;
    unsafe {
        crate::utils::block_on_loop(&(*state_ptr).data_loop, state_ptr, |state| {
            state.loop_thread.store(
                libc::pthread_self() as usize,
                std::sync::atomic::Ordering::Relaxed,
            );
        });
    }

    0
}

const INTERFACE_INFO: [spa_interface_info; 1] = [spa_interface_info {
    type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
}];

pub(crate) unsafe extern "C" fn enum_interface_info(
    _factory: *const spa_handle_factory,
    info: *mut *const spa_interface_info,
    index: *mut u32,
) -> c_int {
    assert!(!info.is_null());
    assert!(!index.is_null());
    // non-null asserted above; the caller contract makes both valid and writable
    unsafe {
        match *index {
            0 => {
                *info = &INTERFACE_INFO[0];
                *index += 1;
                1
            }
            _ => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sink::SinkDir;

    // the oss.fragment normalization contract: 0 stays automatic, everything
    // else rounds DOWN to a power of two and clamps into [64, 16384]
    #[test]
    fn normalize_fragment_rounds_down_and_clamps() {
        assert_eq!(normalize_fragment(0), 0); // automatic
        assert_eq!(normalize_fragment(1), 64); // clamps up to the floor
        assert_eq!(normalize_fragment(63), 64); // rounds to 32, clamps to 64
        assert_eq!(normalize_fragment(64), 64);
        assert_eq!(normalize_fragment(65), 64); // round-down, then in range
        assert_eq!(normalize_fragment(1000), 512); // non-pow2 rounds down
        assert_eq!(normalize_fragment(4096), 4096); // pow2 passes through
        assert_eq!(normalize_fragment(16384), 16384);
        assert_eq!(normalize_fragment(30000), 16384); // clamps to the ceiling
        assert_eq!(normalize_fragment(1 << 31), 16384);
        assert_eq!(normalize_fragment(u32::MAX), 16384);
    }

    // an aligned backing store for the admission tests (every io struct's
    // alignment divides 16)
    #[repr(align(16))]
    struct Aligned([u8; 4096]);

    #[test]
    fn io_area_admission_null_short_exact_misaligned() {
        let mut area = Aligned([0; 4096]);
        let p = area.0.as_mut_ptr().cast::<c_void>();
        let full = std::mem::size_of::<spa_io_clock>();

        // NULL/0 clears whatever the size says
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, std::ptr::null(), 0),
            0
        );
        // exact and oversized areas are admitted
        assert_eq!(io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full), 0);
        assert_eq!(io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full + 8), 0);
        // a non-empty-but-short area is -ENOSPC (the header's "size is too
        // small" errno for set_io/port_set_io)
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full - 1),
            -libc::ENOSPC
        );
        // a misaligned one is the generic -EINVAL
        let off = unsafe { p.cast::<u8>().add(1) }.cast::<c_void>();
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, off, full),
            -libc::EINVAL
        );
        // ids outside the caller's table are -ENOENT (set_io does not take
        // the port areas and vice versa)
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Buffers, p, full),
            -libc::ENOENT
        );
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_Clock, p, full),
            -libc::ENOENT
        );
        // the port table's own areas admit the same policy
        let bsize = std::mem::size_of::<spa_io_buffers>();
        assert_eq!(io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, p, bsize), 0);
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, p, bsize - 1),
            -libc::ENOSPC
        );
        // a short AND misaligned area reports the size problem first: the
        // host's remedy (grow the area) subsumes re-placing it
        let off = unsafe { p.cast::<u8>().add(1) }.cast::<c_void>();
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, off, bsize - 1),
            -libc::ENOSPC
        );
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_RateMatch, std::ptr::null(), 0),
            0
        );
    }

    fn test_port(fd: std::os::raw::c_int) -> Port<SinkDir> {
        Port {
            config: None,
            buffers: vec![],
            io: crate::spa::IoArea::null(),
            rate_match: crate::spa::IoArea::null(),
            dsp: crate::sound::DspWriter::test_on_fd(fd, 8),
            dll: Default::default(),
            setup_period: 0,
            bw_adapt: Default::default(),
            setup_blocksize: 0,
            resetup_pending: false,
            was_matching: false,
            warn_limit: crate::utils::RateLimit::new(),
            ext: Default::default(),
        }
    }

    // a stack fixture: one spa_buffer with one MemPtr data block; the tests
    // then break one field at a time
    fn fixture(payload: &mut [u8], chunk: *mut spa_chunk) -> (spa_buffer, Box<spa_data>) {
        let mut data: spa_data = unsafe { std::mem::zeroed() };
        data.type_ = SPA_DATA_MemPtr;
        data.maxsize = payload.len() as u32;
        data.data = payload.as_mut_ptr().cast();
        data.chunk = chunk;
        let mut data = Box::new(data);
        let mut buffer: spa_buffer = unsafe { std::mem::zeroed() };
        buffer.n_datas = 1;
        buffer.datas = &mut *data;
        (buffer, data)
    }

    // the per-cycle buffer gate: exactly one MemPtr block with data, chunk
    // and maxsize all valid is admitted; everything else skips (None), never
    // faults - buffer_id and the block layout come from the peer
    #[test]
    fn valid_data_block_admits_only_a_usable_memptr_block() {
        let (r, w) = crate::sound::test_util::pipe_pair(true, true);
        let mut port = test_port(w);
        let log = crate::spa::Log::test_null();
        let mut payload = [0u8; 64];
        let mut chunk: spa_chunk = unsafe { std::mem::zeroed() };

        // happy path: the descriptor carries the validated pointers by value
        let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
        port.buffers = vec![&mut buffer];
        let block = unsafe { valid_data_block(&port, 0, &log) };
        assert!(block.is_some_and(|d| {
            std::ptr::eq(d.data.as_ptr(), payload.as_ptr().cast())
                && std::ptr::eq(d.chunk.as_ptr(), &raw const chunk)
                && d.maxsize == payload.len() as u32
        }));

        // out-of-range buffer_id
        assert!(unsafe { valid_data_block(&port, 1, &log) }.is_none());

        // a null host buffer pointer
        port.buffers = vec![std::ptr::null_mut()];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        // n_datas != 1
        let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
        buffer.n_datas = 2;
        port.buffers = vec![&mut buffer];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        // null datas array
        let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
        buffer.datas = std::ptr::null_mut();
        port.buffers = vec![&mut buffer];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        // null data pointer
        let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
        data.data = std::ptr::null_mut();
        port.buffers = vec![&mut buffer];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        // null chunk
        let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
        data.chunk = std::ptr::null_mut();
        port.buffers = vec![&mut buffer];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        // zero maxsize
        let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
        data.maxsize = 0;
        port.buffers = vec![&mut buffer];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        // a non-MemPtr block
        let (mut buffer, mut data) = fixture(&mut payload, &mut chunk);
        data.type_ = SPA_DATA_MemFd;
        port.buffers = vec![&mut buffer];
        assert!(unsafe { valid_data_block(&port, 0, &log) }.is_none());

        unsafe { libc::close(r) };
    }
}
