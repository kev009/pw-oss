// The shared node core. sink.rs and source.rs are the same SPA node modulo
// direction: everything direction-agnostic lives here once, generic over
// `Direction`, and the genuinely direction-specific logic (the process() data
// path, the servo error sign, priming/recovery semantics, the oss.delay prop)
// is supplied through the `Direction` hooks each module implements. The
// extern "C" vtable entries are generic and monomorphized per direction.

use std::os::raw::{c_char, c_int, c_void};

use libspa::sys::*;

mod events;
mod rebuild;

use events::NodeEvents;
use rebuild::{
    MainEvent, NodeShared, RebuildWork, RebuildWorkSlot, RebuildWorker, install_device,
    queue_main_event, release_rebuild_takeover,
};
pub(crate) use rebuild::{
    apply_props_param, normalize_fragment, poll_rebuild, queue_rebuild, store_and_rebuild,
};

#[cfg(test)]
use rebuild::{
    DepositOnUnwind, DeviceSwap, RebuildRequest, SwapOutcome, WORK_BUSY, WORK_CLOSED,
    WorkSubmission, rebuild_request_is_current, rebuild_task,
};

pub(crate) const MAX_PORTS: usize = 1;

trait MutexExt<T> {
    fn lock_unpoisoned(&self) -> std::sync::MutexGuard<'_, T>;
}

impl<T> MutexExt<T> for std::sync::Mutex<T> {
    fn lock_unpoisoned(&self) -> std::sync::MutexGuard<'_, T> {
        self.lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

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
    Built(Vec<u8>), // the serialized pod for this (id, index)
    Exhausted,      // no more values for this param id
    Unknown,        // unknown param id
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
    /// Direction-specific prefix for unknown-command warnings.
    const CMD_WARN_PREFIX: &'static str;

    // Send: crosses onto the data loop through install_device's swap
    type Device: DeviceOps + Send;
    type MainExt: Default; // direction-specific main-loop/readback fields
    type DataExt: Default; // direction-specific data-loop fields
    type PortExt: Default; // direction-specific Port fields

    // Registered module log topic (see the lib.rs section entries). The host
    // mutates the pointee, so keep it as a raw pointer.
    fn log_topic() -> std::ptr::NonNull<spa_log_topic>;

    // Parse direction-specific node properties such as the sink's oss.delay.
    fn info_item(ext: &mut Self::MainExt, key: &str, value: &str);
    // Finalize direction-specific state after parsing the info dictionary.
    fn ext_ready(ext: &mut Self::MainExt);
    // Seed data-loop fields from the parsed control model.
    fn data_ext(ext: &Self::MainExt) -> Self::DataExt;

    // Serialize one node parameter pod for (id, index).
    fn build_node_param(state: &mut MainState<Self>, id: u32, index: u32) -> ParamBuild;
    // Reset Props to their defaults.
    fn reset_props(state: &mut MainState<Self>, data: &DataControl<Self>) -> c_int;
    // Apply oss.delay. The sink caps, stores, and rebuilds; the source ignores it.
    fn apply_oss_delay(state: &mut MainState<Self>, data: &DataControl<Self>, delay: u32) -> c_int;

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
    // Reset direction-specific state during a device swap.
    fn on_device_swapped(state: &mut DataState<Self>, port_idx: usize);
    // port_use_buffers: direction-specific resets inside the loop-side swap
    fn on_buffers_swapped(state: &mut DataState<Self>, port_idx: usize);

    // send_command(Start): direction-specific resets, on the data loop
    fn on_start_loop(state: &mut DataState<Self>);
    // send_command(Suspend): direction-specific resets, on the data loop
    fn on_suspend_loop(state: &mut DataState<Self>);
    // set_io: the driver/follower role flipped on a live node
    fn on_role_flip(state: &mut DataState<Self>);

    // on_timeout: debug-build cycle tracing (the sink prints one line)
    fn debug_cycle(state: &DataState<Self>, now: u64, nsec: u64);
    // on_timeout servo hooks (see node::timeout_servo): the extra readiness
    // gate (the source's primed flag), the fill measurement, the recovery
    // hold (the sink's xrun window) and the signed servo error for a fill
    fn servo_ready(port: &Port<Self>) -> bool;
    fn servo_fill(port: &mut Port<Self>) -> u32;
    fn servo_hold(port: &Port<Self>) -> bool;
    fn servo_err(port: &Port<Self>, fill: u32) -> f64;

    // process(): the direction-specific data path over the ports
    fn process_ports(state: &mut DataState<Self>) -> c_int;

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

#[repr(C)]
// The pinned FFI shell. Runtime entry points project only one disjoint field
// from its raw pointer; they never create a reference to this whole object.
// `handle` stays first because the host casts spa_handle* back to State*.
pub(crate) struct State<D: Direction> {
    pub handle: spa_handle,
    pub node: spa_node,
    // Checked through its own atomic before process() projects `data`.
    gate: DataThreadGate,
    main: MainState<D>,
    data: DataState<D>,
}

struct DataThreadGate {
    thread: std::sync::atomic::AtomicUsize,
    log: crate::spa::Log,
}

pub(crate) struct MainState<D: Direction> {
    events: std::sync::Arc<NodeEvents<D>>,
    // A copyable host-loop endpoint plus the stable address of State::data are
    // combined into DataControl at each control entry point.
    pub data_loop: crate::spa::Loop,
    pub log: crate::spa::Log,
    pub dsp_path: String,
    pub caps: crate::sound::DspCaps,
    pub caps_fallback: bool,
    pub oss_fragment: u32,
    pub oss_fragment_default: u32,
    pub latency: [spa_latency_info; 2],
    pub process_latency: spa_process_latency_info,
    pub shared: std::sync::Arc<NodeShared<D>>,
    // Owns the only thread that may execute an asynchronous device
    // open/configure/close. DataState holds only its bounded submission
    // endpoint; clear stops and joins this worker before State is dropped.
    rebuild_worker: RebuildWorker<D>,
    pub ring_cap_published: bool,
    pub ext: D::MainExt,
}

pub(crate) struct DataState<D: Direction> {
    pub data_loop: crate::spa::Loop,
    pub data_system: crate::spa::System,
    pub log: crate::spa::Log,
    pub clock: crate::spa::IoArea<spa_io_clock>,
    pub position: crate::spa::IoArea<spa_io_position>,
    pub clock_name: std::ffi::CString, // stamped into spa_io_clock.name
    pub main_loop: Option<crate::spa::Loop>, // for endpoint-only notifications
    pub dsp_path: String,
    pub timer_source: spa_source,
    pub next_time: u64,
    pub callbacks: NodeCallbacks,
    pub ports: [Port<D>; MAX_PORTS],
    pub oss_fragment: u32, // normalized fragment size in bytes (0 = automatic); read by the prime paths
    // the Arc'd rendezvous with the owned rebuild worker and
    // clear(); outlives the FFI shell by construction (see NodeShared)
    pub shared: std::sync::Arc<NodeShared<D>>,
    // The data loop is the sole producer. A device-bearing command that
    // finds the worker slot occupied stays here and is retried before any
    // further completion is consumed; it is never dropped on the RT path.
    rebuild_work: std::sync::Arc<RebuildWorkSlot<D>>,
    deferred_work: Option<RebuildWork<D>>,
    // Main-thread synchronous installs take this data-loop lease before
    // waiting for the worker. While set, process neither consumes a
    // completion nor submits new work.
    rebuild_takeover: bool,
    events: std::sync::Arc<NodeEvents<D>>,
    // Data-loop-owned: process_ports records endpoint work here, and generic
    // process() extracts it before ending its DataState phase. Delivery happens
    // only afterward, so an inline loop invoke cannot overlap the data borrow.
    pending_main_event: Option<MainEvent>,
    pub started: bool,
    pub following: bool,
    pub ext: D::DataExt, // direction-specific fields (see sink.rs/source.rs)
}

impl<D: Direction> DataState<D> {
    fn node_is_follower(&self) -> bool {
        let driver = self.position.with_ref(|p| p.clock.id);
        let ours = self.clock.with_ref(|c| c.id);
        matches!((driver, ours), (Some(d), Some(o)) if d != o)
    }
}

// A main-loop capability for exactly one operation: synchronously run a
// closure against the disjoint data-loop state. It is constructed from raw
// projections of the pinned FFI shell, so no reference to the shell exists
// while the data loop borrows its field.
pub(crate) struct DataControl<D: Direction> {
    loop_: crate::spa::Loop,
    data: *mut DataState<D>,
}

impl<D: Direction> Copy for DataControl<D> {}
impl<D: Direction> Clone for DataControl<D> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<D: Direction> DataControl<D> {
    unsafe fn from_raw(state: *mut State<D>) -> Self {
        Self {
            loop_: unsafe { std::ptr::addr_of!((*state).main.data_loop).read() },
            data: unsafe { std::ptr::addr_of_mut!((*state).data) },
        }
    }

    fn invoke(&self, f: impl FnOnce(&mut DataState<D>) + Send) -> bool {
        unsafe { crate::utils::block_on_loop(&self.loop_, self.data, f) }
    }

    fn query<R: Send>(&self, f: impl FnOnce(&mut DataState<D>) -> R + Send) -> Option<R> {
        let mut result = None;
        let result_ref = &mut result;
        if self.invoke(move |state| *result_ref = Some(f(state))) {
            result
        } else {
            None
        }
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
    // A main-loop device rebuild is in flight; skip cycles until poll_rebuild
    // consumes its completion. Data-loop-owned: set when the order is queued,
    // cleared when the completion swap is consumed (or by the install/suspend
    // swap closures, which also run on this loop) - no other thread touches it.
    pub rebuild_pending: bool,
    // Data-loop-owned rebuild fence. Increment it whenever the port's device
    // or configuration changes. A completion applies only when its snapshot
    // still matches; wrapping is safe because the fence uses equality only.
    pub generation: u64,
    pub was_matching: bool, // rate matching active last cycle (relock on transition)
    pub warn_limit: crate::utils::RateLimit,
    // Data-loop-owned xrun detected this cycle (trigger time in
    // µs). detect_underrun/recover_overrun deposit it instead of calling the
    // host back mid-cycle; process() drains it and invokes the copied xrun
    // hook only after the DataState/port borrows end (collect-then-notify).
    pub pending_xrun: Option<u64>,
    pub ext: D::PortExt, // direction-specific fields (see sink.rs/source.rs)
}

// Validated view of one host-owned buffer block. valid_data_block is the only
// constructor, and callers keep the view within the current data-loop cycle.
//
// The accessors rely on these invariants:
// - `data` points at a block readable and writable for `maxsize` bytes and
//   `chunk` at a live spa_chunk for the whole cycle;
// - `maxsize` is nonzero and no larger than isize::MAX;
// - accessors clamp peer-provided offsets and sizes to the block;
// - each host block has at most one live DataBlock per cycle. DataBlock is
//   deliberately not Copy or Clone so mutable views cannot alias.
pub(crate) struct DataBlock {
    data: std::ptr::NonNull<c_void>, // the mapped MemPtr block
    chunk: std::ptr::NonNull<spa_chunk>,
    maxsize: u32, // > 0, <= isize::MAX; offsets/sizes are clamped against it
}

impl DataBlock {
    // The peer's chunk viewed as a readable slice (the sink's input): the
    // offset wraps at maxsize and the size clamps to what remains past it,
    // so the range is in bounds for the block whatever the peer wrote.
    pub(crate) fn input_slice(&self) -> &[u8] {
        // chunk is valid for the cycle (valid_data_block's contract)
        let chunk = unsafe { self.chunk.as_ref() };
        let offset = chunk.offset % self.maxsize;
        let size = chunk.size.min(self.maxsize - offset);
        // in bounds: offset < maxsize and size <= maxsize - offset above
        unsafe {
            std::slice::from_raw_parts(
                self.data.as_ptr().cast::<u8>().add(offset as usize),
                size as usize,
            )
        }
    }

    // the whole block as a writable slice (the source fills it, then
    // publishes the chunk); &mut self keys the borrow so the block can't be
    // read through input_slice/data_ptr while the view is live
    pub(crate) fn output_slice(&mut self) -> &mut [u8] {
        // data is valid for maxsize bytes for the cycle (valid_data_block)
        unsafe {
            std::slice::from_raw_parts_mut(self.data.as_ptr().cast::<u8>(), self.maxsize as usize)
        }
    }

    // publish the cycle's output: the chunk describes `size` bytes at offset 0
    pub(crate) fn publish(&mut self, size: u32, stride: u32) {
        assert!(size <= self.maxsize, "published size exceeds the block");
        // chunk is valid for the cycle (valid_data_block's contract)
        let chunk = unsafe { self.chunk.as_mut() };
        chunk.offset = 0;
        chunk.size = size;
        chunk.stride = stride as i32;
        chunk.flags = 0;
    }

    // the peer-declared stride (the sink's debug cross-check)
    pub(crate) fn chunk_stride(&self) -> i32 {
        // chunk is valid for the cycle (valid_data_block's contract)
        unsafe { self.chunk.as_ref() }.stride
    }

    // debug sites only (spa_debug_mem) plus the unit tests; compiled out of
    // release builds - `test` keeps `cargo test --release` building
    #[cfg(any(debug_assertions, test))]
    pub(crate) fn data_ptr(&self) -> *mut c_void {
        self.data.as_ptr()
    }
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
                && d.maxsize > 0
                // slice::from_raw_parts caps lengths at isize::MAX; only
                // reachable on 32-bit targets, where a u32 maxsize can exceed it
                && d.maxsize as u64 <= isize::MAX as u64 =>
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
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let node_events = unsafe { (*std::ptr::addr_of!((*state).main)).events.clone() };

    let initial = |hooks: &crate::spa::ListenerList<spa_node_events>| {
        // The initial emissions only reach the newly added listener (the list
        // is isolated). One method per traversal, mirroring C's
        // spa_hook_list_call: a listener that removes and frees its hook
        // inside a callback must not be read for the next method.
        let (node_info, port_info) = node_events.initial_snapshots();
        // Hold the endpoint's dispatch turn across the whole initial
        // transaction. Reentrant mutations queue behind both snapshots
        // instead of publishing newer state between them.
        let dispatch_claim = node_events.begin_dispatch();
        hooks.emit(|f, data| {
            if let Some(node_info_fun) = f.info {
                // through the C listener vtable (add_listener contract)
                unsafe { node_info_fun(data, node_info.raw()) };
            }
        });
        hooks.emit(|f, data| {
            if let Some(port_info_fun) = f.port_info {
                // through the C listener vtable (add_listener contract)
                unsafe { port_info_fun(data, D::DIRECTION, 0, port_info.raw()) };
            }
        });
        dispatch_claim
    };
    let dispatch_claim = unsafe { node_events.with_new_listener(listener, events, data, initial) };
    if let Some(claim) = dispatch_claim.as_ref() {
        // SAFETY: the State snapshot borrow ended before isolation, and the
        // scoped helper restored the complete list before nested work drains.
        unsafe { node_events.drain(claim) };
    }

    0
}

// re-emit node_info to every listener (carrying whatever change_mask the caller
// set, e.g. PARAMS), then clear the mask
pub(crate) fn emit_node_info<D: Direction>(state: &MainState<D>) {
    let events = state.events.clone();
    events.queue_node_info();
}

// the process latency (user-set latency offset) shifts the node's reported
// latency, so a change re-emits the Props/ProcessLatency node params and the
// port Latency param
pub(crate) fn handle_process_latency<D: Direction>(
    state: &mut MainState<D>,
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

    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        if ns_changed {
            info.bump_param(SPA_PARAM_Props);
        }
        info.bump_param(SPA_PARAM_ProcessLatency);
    });
    emit_node_info(state);

    state.events.with_port_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Latency);
    });
    emit_port_info(state);
}

// re-emit port_info to every listener (carrying whatever change_mask the caller
// set, e.g. RATE/PARAMS), then clear the mask
pub(crate) fn emit_port_info<D: Direction>(state: &MainState<D>) {
    let events = state.events.clone();
    events.queue_port_info();
}

unsafe extern "C" fn set_callbacks<D: Direction>(
    object: *mut c_void,
    callbacks: *const spa_node_callbacks,
    data: *mut c_void,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null());

    // SAFETY: `callbacks`, when non-null, points at a live table whose
    // version prefix describes its true length, and the host keeps `data`
    // valid while the table is set (the set_callbacks contract)
    let mut new_callbacks = NodeCallbacks::none();
    unsafe { new_callbacks.set(callbacks, data) };

    // on_timeout/process call the table from the data loop; store it there.
    // SAFETY: a by-value table copy plus the host data pointer, which stays
    // valid while set (the same contract)
    let new_callbacks = unsafe { crate::utils::SendWrap::new(new_callbacks) };
    let control = unsafe { DataControl::from_raw(state) };
    if !control.invoke(move |state| state.callbacks = new_callbacks.into_inner()) {
        return -libc::EIO;
    }
    0
}

unsafe extern "C" fn sync<D: Direction>(object: *mut c_void, seq: c_int) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { (*std::ptr::addr_of!((*state).main)).events.clone() };
    // SAFETY: only the independently owned endpoint is borrowed.
    unsafe { events.emit_done(seq) };
    0
}

// emit one filtered enum_params result to every listener (node and port
// enumeration share this shape)
unsafe fn emit_param_result<D: Direction>(
    events: &NodeEvents<D>,
    seq: c_int,
    id: u32,
    index: u32,
    param: *mut spa_pod,
) {
    let result = spa_result_node_params {
        id,
        index,
        next: index + 1,
        param,
    };
    unsafe { events.emit_result(seq, &result) };
}

unsafe extern "C" fn enum_params<D: Direction>(
    object: *mut c_void,
    seq: c_int,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    // Clone the independently allocated endpoint before enumeration. Each
    // build step below gets a fresh State borrow; dispatch uses only events.
    let events = unsafe { (*std::ptr::addr_of!((*state).main)).events.clone() };
    let main = unsafe { std::ptr::addr_of_mut!((*state).main) };

    unsafe {
        crate::spa::enum_params_loop(
            main,
            (start, max),
            filter,
            |state, index| match D::build_node_param(state, id, index) {
                ParamBuild::Built(pod) => crate::spa::ParamStep::Built(pod),
                ParamBuild::Exhausted => crate::spa::ParamStep::Stop(0),
                // unknown param id (ALSA convention)
                ParamBuild::Unknown => crate::spa::ParamStep::Stop(-libc::ENOENT),
            },
            |index, param| emit_param_result(&events, seq, id, index, param),
        )
    }
}

// Updates accepted from a Props pod. None means the property was absent.
// The sink consumes oss_delay and the source ignores it. Capping oss_delay
// and normalizing oss.fragment happen when the update is applied so readback
// reports the effective value.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct PropsUpdate {
    pub latency_offset_ns: Option<i64>,
    pub oss_delay: Option<u32>,
    pub oss_fragment: Option<u32>,
}

// Validated node parameter requests. Raw pods do not cross this boundary.
pub(crate) enum NodeParamRequest {
    ResetProps, // set_param(Props, NULL)
    Props(PropsUpdate),
    ResetProcessLatency, // set_param(ProcessLatency, NULL)
    ProcessLatency(spa_process_latency_info),
}

// Parse a deserialized Props object. The adapter owns soft-volume properties,
// unknown keys are logged and skipped, and invalid oss.* values are ignored.
fn parse_props_update(
    properties: Vec<libspa::pod::Property>,
    log: &crate::spa::Log,
) -> PropsUpdate {
    use libspa::pod::Value;

    let mut update = PropsUpdate::default();
    for property in properties {
        #[allow(non_upper_case_globals)]
        match property.key {
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
                    update.latency_offset_ns = Some(ns);
                }
            }
            // pw-cli set-param <object-id> Props '{ "params": ["oss.delay", 8]}'
            SPA_PROP_params => parse_oss_params(&property.value, &mut update),
            key => {
                crate::debug!(log, "ignoring unknown prop {}", key);
            }
        }
    }
    update
}

// the SPA_PROP_params payload: a Struct of ("key", value) pairs
fn parse_oss_params(value: &libspa::pod::Value, update: &mut PropsUpdate) {
    use libspa::pod::Value;
    let Value::Struct(values) = value else {
        return;
    };
    if values.len() % 2 != 0 {
        return;
    }
    for kv in values.chunks(2) {
        match (&kv[0], &kv[1]) {
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_DELAY && *x >= 0 => {
                update.oss_delay = Some(*x as u32);
            }
            (Value::String(s), Value::Int(x)) if s == crate::keys::OSS_FRAGMENT && *x >= 0 => {
                update.oss_fragment = Some(*x as u32);
            }
            _ => (),
        }
    }
}

// Apply a validated request to the main-loop model. Data-loop effects cross
// only through DataControl. Props apply in this order: latency offset,
// oss.delay, then oss.fragment. The first failing oss.* update returns its
// errno.
pub(crate) fn apply_node_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    request: NodeParamRequest,
) -> c_int {
    match request {
        NodeParamRequest::ResetProps => {
            let res = D::reset_props(state, data);
            if res == 0 {
                state.events.with_node_info(|info| {
                    let _ = info.replace_change_mask(0);
                    info.bump_param(SPA_PARAM_Props);
                });
                emit_node_info(state);
            }
            res
        }
        NodeParamRequest::Props(update) => {
            if let Some(ns) = update.latency_offset_ns {
                let mut info = state.process_latency;
                info.ns = ns;
                handle_process_latency(state, info);
            }
            if let Some(delay) = update.oss_delay {
                let res = D::apply_oss_delay(state, data, delay);
                if res != 0 {
                    return res;
                }
            }
            if let Some(fragment) = update.oss_fragment {
                // stored normalized, so the Props readback reports the
                // effective (rounded/clamped) value, not the raw request
                let new_fragment = normalize_fragment(fragment);
                if new_fragment != state.oss_fragment {
                    // unchanged echoes must not rebuild a running device
                    let old_fragment = state.oss_fragment;
                    // install_device consumes the main-loop copy while the
                    // data-loop store/rebuild is in progress.
                    state.oss_fragment = new_fragment;
                    let res = apply_props_param(state, data, move |state| {
                        state.oss_fragment = new_fragment;
                    });
                    if res != 0 {
                        state.oss_fragment = old_fragment;
                        return res;
                    }
                }
            }
            0
        }
        NodeParamRequest::ResetProcessLatency => {
            handle_process_latency(state, crate::utils::process_latency_default());
            0
        }
        NodeParamRequest::ProcessLatency(info) => {
            handle_process_latency(state, info);
            0
        }
    }
}

unsafe extern "C" fn set_param<D: Direction>(
    object: *mut c_void,
    id: u32,
    _flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let log = unsafe { (*std::ptr::addr_of!((*state).main)).log.clone() };

    use libspa::pod::{Object, Value};

    // Reject unknown ids before reading the pod. NULL resets a known
    // parameter; malformed or mistyped pods return -EINVAL.
    #[allow(non_upper_case_globals)]
    let request = match id {
        SPA_PARAM_Props => {
            if param.is_null() {
                // a NULL pod resets the props to their defaults
                NodeParamRequest::ResetProps
            } else {
                // Deserialize before borrowing State.
                match unsafe { crate::utils::deserialize_pod(param) } {
                    Some(Value::Object(Object {
                        type_, properties, ..
                    })) if type_ == SPA_TYPE_OBJECT_Props => {
                        NodeParamRequest::Props(parse_props_update(properties, &log))
                    }
                    _ => return -libc::EINVAL,
                }
            }
        }
        SPA_PARAM_ProcessLatency => {
            if param.is_null() {
                NodeParamRequest::ResetProcessLatency
            } else {
                // Deserialize before borrowing State.
                let value = unsafe { crate::utils::deserialize_pod(param) };
                match crate::utils::parse_process_latency_info(value.as_ref()) {
                    Some(info) => NodeParamRequest::ProcessLatency(info),
                    None => return -libc::EINVAL,
                }
            }
        }
        id => {
            crate::warn!(log, "set_param: unknown param {}", id);
            return -libc::ENOENT;
        }
    };
    let control = unsafe { DataControl::from_raw(state) };
    let (events, result) = {
        // All info emissions produced by the safe phase are queued as owned
        // snapshots. End this State borrow before invoking any listener.
        let state = unsafe { &mut *std::ptr::addr_of_mut!((*state).main) };
        let events = state.events.clone();
        let result = apply_node_param(state, &control, request);
        (events, result)
    };
    // SAFETY: the scoped State borrow above ended before this dispatch.
    unsafe { events.flush() };
    result
}

// Run the servo before the clock is published so every field below belongs
// to this cycle (the shape of ALSA's update_time); both directions share
// the skeleton, with the fill measurement and error sign supplied through
// the Direction servo_* hooks. Returns (corr, delay) for the clock.
fn timeout_servo<D: Direction>(state: &mut DataState<D>, nsec: u64, rate: u32) -> (f64, i64) {
    let mut corr: f64 = 1.0;
    let mut delay: i64 = 0;
    for port in &mut state.ports {
        let Some((stride, device_rate)) = port.stride_rate() else {
            continue;
        };
        let device_rate = device_rate.max(1);
        if !port.dsp.is_running()
            || port.setup_period == 0
            || port.rebuild_pending
            || !D::servo_ready(port)
        {
            continue;
        }

        let fill = D::servo_fill(port);
        // device frames scale to the graph rate; the resampler queue is already
        // graph-side (audioconvert reports it unscaled, like ALSA adds it)
        let resamp = port.rate_match.with_ref(|rm| rm.delay as i64).unwrap_or(0);
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
    let root: *mut State<D> = unsafe { (*source).data.cast() };
    assert!(!root.is_null(), "(*source).data is not supposed to be null");
    let state = unsafe { std::ptr::addr_of_mut!((*root).data) };

    // Phase 1, under a scoped borrow: drain the timer, run the servo and
    // publish the clock (every early exit arms or parks the timer itself).
    // Collect the ready notification here as a copied hook: pw
    // runs process() inline from ready() on this same thread, conjuring a
    // fresh &mut DataState, so the callback must not run under this borrow.
    // SAFETY: the registered source data points at our live State (the
    // add_source contract); the borrow ends before the notify call below.
    let notify = timeout_cycle(unsafe { &mut *state });

    let Some(hook) = notify else {
        return; // early exit; the timer was armed or parked inside
    };
    if let Some((cb, data)) = hook {
        if let Some(ready_fun) = cb.ready {
            // no State borrow is live here; sound per NodeCallbacks::hook
            let err = unsafe { ready_fun(data, D::READY_STATUS) };
            #[cfg(debug_assertions)]
            crate::trace!(unsafe { &(*state).log }, "ready -> {}", err);
            #[cfg(not(debug_assertions))]
            let _ = err;
        }
    }

    // Phase 2: re-borrow to arm the timer for the deadline the cycle
    // computed. SAFETY: the callback returned, so no reentrant borrow is
    // live; the source stays registered while the node lives. The callback
    // may have synchronously paused the node or cleared its IO, so do not
    // undo the timer park that transition just installed.
    let state = unsafe { &mut *state };
    if state.started && !state.following && !state.position.is_null() && !state.clock.is_null() {
        set_timeout(state, state.next_time);
    } else {
        set_timeout(state, 0);
    }
}

// the on_timeout cycle body, run under one scoped &mut DataState borrow. None =
// early exit (the timer was armed/parked as needed); Some(hook) = the full
// cycle ran, the clock is published, and the caller must invoke the ready
// hook (when present) and then arm the timer for state.next_time.
#[allow(clippy::type_complexity)] // the copied C (table, data) pair
fn timeout_cycle<D: Direction>(
    state: &mut DataState<D>,
) -> Option<Option<(spa_node_callbacks, *mut c_void)>> {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "on_timeout");

    let mut expirations = 0;
    if state
        .data_system
        .timerfd_read(state.timer_source.fd, &mut expirations)
        < 0
    {
        // disarmed (Pause/Suspend) in this same wakeup; nothing to read
        return None;
    }

    // after the drain: the source is level-triggered, so bailing with the fd
    // readable would busy-spin the loop; the one-shot timer is only re-armed
    // by set_timeout below, so returning here really does park it
    // stopped between the timer firing and this callback; don't signal ready()
    // into a node being reconfigured, and don't re-arm
    if !state.started || state.following {
        return None;
    }

    if state.position.is_null() || state.clock.is_null() {
        return None; // ios cleared while the timer was armed; skip the cycle
    }

    // A failed clock read must not abort the data loop, but a bare return
    // would park the one-shot timer until the next external transition
    // (only set_timeout re-arms it): retry on a RELATIVE ~10 ms one-shot.
    // next_time deliberately does not advance - it re-anchors only from a
    // successful read (the stall resync below); an absolute re-arm computed
    // from a stale deadline would fire immediately and busy-spin the loop
    // until the synthetic deadline caught up with wall time.
    let Some(now) = crate::utils::try_now_ns(&state.data_system) else {
        set_timeout_rel(state, SPA_NSEC_PER_SEC as u64 / 100);
        return None;
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

    D::debug_cycle(state, now, nsec);

    // position and clock were null-checked above and stay set for the cycle
    let (duration, rate) = state
        .position
        .with_ref(|p| (p.clock.target_duration, p.clock.target_rate.denom))
        .unwrap_or((0, 0));
    if duration == 0 || rate == 0 {
        // malformed position: idle-tick, and advance next_time so the deadline
        // isn't stale when the position recovers
        state.next_time = nsec + SPA_NSEC_PER_SEC as u64 / 100;
        set_timeout(state, state.next_time);
        return None;
    }

    let (corr, delay) = timeout_servo(state, nsec, rate);

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

    // hand the copied hook out (None inside = no callbacks yet, or cleared;
    // the caller keeps the clock ticking either way)
    Some(state.callbacks.hook())
}

// Data loop only. Arm the wakeup timer from now when this node drives the
// graph (started, not following, position present); park it otherwise. A
// failed clock read must not park a node that wants to run (nothing but
// another external transition would ever re-arm it): retry on a relative
// ~10 ms one-shot without touching next_time - it re-anchors only from a
// successful read (here or on_timeout's stall resync; an absolute arm from
// a stale next_time would busy-spin) - and let on_timeout take over from
// there; nothing aborts the data loop (the sink's former copy assert!()ed).
pub(crate) fn update_timers<D: Direction>(state: &mut DataState<D>) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "update_timers");

    if !(state.started && !state.following && !state.position.is_null()) {
        set_timeout(state, 0); // park
        return;
    }
    match crate::utils::try_now_ns(&state.data_system) {
        Some(now) => {
            state.next_time = now;
            #[cfg(debug_assertions)]
            crate::trace!(state.log, "next time {}", now);
            set_timeout(state, now); // immediate fire from a fresh anchor
        }
        None => set_timeout_rel(state, SPA_NSEC_PER_SEC as u64 / 100),
    }
}

pub(crate) fn set_timeout<D: Direction>(state: &DataState<D>, next_time: u64) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "set_timeout {}", next_time);

    // absolute one-shot on the loop clock; 0 disarms (parks)
    arm_timer(state, next_time, SPA_FD_TIMER_ABSTIME as i32);
}

// Relative one-shot: the clock-read failure paths' retry. They have no
// trustworthy "now" to anchor an absolute deadline on, and an absolute arm
// from a stale next_time fires immediately - a busy-spin for as long as the
// clock keeps failing. `delay_ns` must be nonzero (zero would disarm).
pub(crate) fn set_timeout_rel<D: Direction>(state: &DataState<D>, delay_ns: u64) {
    #[cfg(debug_assertions)]
    crate::trace!(state.log, "set_timeout_rel {}", delay_ns);

    debug_assert!(delay_ns > 0);
    arm_timer(state, delay_ns, 0);
}

fn arm_timer<D: Direction>(state: &DataState<D>, value_ns: u64, flags: i32) {
    let timerspec = itimerspec {
        it_value: timespec {
            tv_sec: (value_ns / SPA_NSEC_PER_SEC as u64) as i64,
            tv_nsec: (value_ns % SPA_NSEC_PER_SEC as u64) as i64,
        },
        it_interval: timespec {
            tv_sec: 0,
            tv_nsec: 0,
        },
    };

    state
        .data_system
        .timerfd_settime(state.timer_source.fd, flags, &timerspec);
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
    let control = unsafe { DataControl::from_raw(state) };
    let applied = control.invoke(move |state| {
        let data = data.into_inner();
        let was_armed = !state.clock.is_null() && !state.position.is_null();

        #[allow(non_upper_case_globals)]
        match id {
            SPA_IO_Clock => {
                // SAFETY: size/alignment validated above; the host keeps
                // the area valid while set (the set_io contract)
                unsafe { state.clock.set(data) }; // null clears

                // identify our clock so same-device followers can skip rate matching
                state
                    .clock
                    .with(|c| crate::utils::set_clock_name(c, &state.clock_name));
            }
            // SAFETY: as above
            SPA_IO_Position => unsafe { state.position.set(data) }, // null clears
            _ => (),                                                // filtered above
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
    });
    if !applied {
        return -libc::EIO;
    }

    0
}

type ExtractedDevices<D> = [(usize, <D as Direction>::Device); MAX_PORTS];

fn replace_port_devices<D: Direction>(
    ports: &mut [Port<D>; MAX_PORTS],
    devices: ExtractedDevices<D>,
) -> [D::Device; MAX_PORTS] {
    devices.map(|(index, device)| {
        ports[index].rebuild_pending = false;
        std::mem::replace(&mut ports[index].dsp, device)
    })
}

// Return devices extracted by Suspend without transferring their ownership
// into the loop closure. If the invoke cannot run, the caller still owns them
// and can retry or release them only after the loop is known unavailable.
fn restore_extracted_devices<D: Direction>(
    control: &DataControl<D>,
    devices: &mut Option<ExtractedDevices<D>>,
) -> Option<[D::Device; MAX_PORTS]> {
    devices.as_ref()?;
    control.query(|state| {
        let devices = devices
            .take()
            .expect("the caller retains extracted devices until this invoke runs");
        let placeholders = replace_port_devices(&mut state.ports, devices);
        state.rebuild_takeover = false;
        placeholders
    })
}

fn restore_started_if_stop_unobserved(
    started: &std::sync::atomic::AtomicBool,
    was_started: bool,
    data_stopped: &std::sync::atomic::AtomicBool,
) {
    if !data_stopped.load(std::sync::atomic::Ordering::Acquire) {
        started.store(was_started, std::sync::atomic::Ordering::Release);
    }
}

unsafe extern "C" fn send_command<D: Direction>(
    object: *mut c_void,
    command: *const spa_command,
) -> c_int {
    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");
    let control = unsafe { DataControl::from_raw(state) };
    let (log, shared, rebuild_work) = unsafe {
        let main = std::ptr::addr_of!((*state).main);
        (
            (*main).log.clone(),
            (*main).shared.clone(),
            (*main).rebuild_worker.endpoint(),
        )
    };

    assert!(!command.is_null());
    let body = unsafe { (*command).body.body };

    crate::debug!(
        log,
        "received command: {}",
        crate::utils::spa_command_to_str(&body)
    );

    #[allow(non_upper_case_globals)]
    match (body.type_, body.id) {
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start) => {
            let started = control.query(|state| {
                if state
                    .ports
                    .iter()
                    .any(|p| p.config.is_none() || p.buffers.is_empty())
                {
                    return false;
                }
                // sane clock delay/rate_diff until process() publishes measured values
                state.clock.with(|c| {
                    c.delay = 0;
                    c.rate_diff = 1.0;
                });
                D::on_start_loop(state);
                state.started = true;
                state.following = state.node_is_follower();
                update_timers(state);
                true
            });
            match started {
                Some(true) => (),
                Some(false) => {
                    crate::warn!(log, "can't start: ports are not negotiated");
                    return -libc::EIO;
                }
                None => {
                    crate::warn!(log, "can't start: data loop did not accept the command");
                    return -libc::EIO;
                }
            };
            // Publish only after DataState is fully started. The worker
            // pairs this Release with its pre/post-open Acquire checks.
            shared
                .started
                .store(true, std::sync::atomic::Ordering::Release);
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => {
            // Publish the stop before the blocking data-loop handoff. A
            // worker open finishing in that window must retire its result,
            // not hand an already-paused node a fresh exclusive fd.
            let was_started = shared
                .started
                .swap(false, std::sync::atomic::Ordering::AcqRel);
            let data_stopped = std::sync::atomic::AtomicBool::new(false);
            let Some(deferred) = control.query(|state| {
                state.started = false;
                data_stopped.store(true, std::sync::atomic::Ordering::Release);
                update_timers(state);
                state.rebuild_takeover = true;
                let deferred = state.deferred_work.take();
                for port in &mut state.ports {
                    port.rebuild_pending = true;
                    port.generation = port.generation.wrapping_add(1);
                    state
                        .shared
                        .generation
                        .store(port.generation, std::sync::atomic::Ordering::Release);
                }
                deferred
            }) else {
                restore_started_if_stop_unobserved(&shared.started, was_started, &data_stopped);
                return -libc::EIO;
            };
            drop(deferred);
            // Catch both a completion deposited before the fence and one
            // from a worker that passed its final check just before it.
            shared.discard_swap();
            if !rebuild_work.wait_idle() {
                release_rebuild_takeover(&control, 0);
                return -libc::EIO;
            }
            shared.discard_swap();
            if !release_rebuild_takeover(&control, 0) {
                return -libc::EIO;
            }
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => {
            // As with Pause, stop wins before waiting for the data loop.
            let was_started = shared
                .started
                .swap(false, std::sync::atomic::Ordering::AcqRel);
            let data_stopped = std::sync::atomic::AtomicBool::new(false);
            let data_stopped_ref = &data_stopped;
            // Device::new may probe sndstat (the sink does), so build every
            // closed placeholder here on main rather than inside DataControl.
            let dsp_path = unsafe { &*std::ptr::addr_of!((*state).main.dsp_path) };
            let placeholders: [D::Device; MAX_PORTS] =
                std::array::from_fn(|_| D::Device::new(dsp_path));
            // Quiesce and transfer device ownership out of DataState. Potentially
            // sleeping SETTRIGGER/close operations then run on this thread while
            // the data loop sees only closed placeholders.
            let Some((devices, deferred)) = control.query(move |state| {
                state.started = false;
                data_stopped_ref.store(true, std::sync::atomic::Ordering::Release);
                update_timers(state);
                D::on_suspend_loop(state);
                state.rebuild_takeover = true;
                let deferred = state.deferred_work.take();
                let mut placeholders = placeholders.into_iter();
                let devices: [(usize, D::Device); MAX_PORTS] = std::array::from_fn(|index| {
                    let port = &mut state.ports[index];
                    port.rebuild_pending = true;
                    port.generation = port.generation.wrapping_add(1);
                    state
                        .shared
                        .generation
                        .store(port.generation, std::sync::atomic::Ordering::Release);
                    let placeholder = placeholders
                        .next()
                        .expect("one prebuilt placeholder per port");
                    (index, std::mem::replace(&mut port.dsp, placeholder))
                });
                (devices, deferred)
            }) else {
                restore_started_if_stop_unobserved(&shared.started, was_started, &data_stopped);
                return -libc::EIO;
            };
            let mut devices = Some(devices);
            drop(deferred);
            // a deposited-but-unconsumed rebuild would hold an open
            // (possibly exclusive) device across the whole suspended stretch
            // (nothing polls while stopped); close it now, off the RT path.
            shared.discard_swap();
            if !rebuild_work.wait_idle() {
                let mut placeholders = restore_extracted_devices(&control, &mut devices);
                if placeholders.is_none() && devices.is_some() {
                    placeholders = restore_extracted_devices(&control, &mut devices);
                }
                if placeholders.is_none() && devices.is_some() {
                    crate::warn!(
                        log,
                        "can't restore devices after rebuild worker shutdown: data loop is unavailable"
                    );
                }
                drop(placeholders);
                return -libc::EIO;
            }
            shared.discard_swap();
            for (_, dsp) in devices
                .as_mut()
                .expect("Suspend retains the extracted devices until restoration")
            {
                if !dsp.is_closed() && !dsp.suspend() {
                    dsp.close();
                }
            }
            let placeholders = restore_extracted_devices(&control, &mut devices);
            let Some(placeholders) = placeholders else {
                // The first invoke retained ownership on failure. Retry once
                // so a transient handoff error does not release live or
                // suspended descriptors while placeholders remain installed.
                let restored = devices.is_none()
                    || restore_extracted_devices(&control, &mut devices).is_some();
                if !restored {
                    crate::warn!(
                        log,
                        "can't restore suspended devices: data loop is unavailable"
                    );
                }
                return -libc::EIO;
            };
            // Closed placeholders still own heap fields; destroy them on main.
            drop(placeholders);
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin | SPA_NODE_COMMAND_ParamEnd) => 0, // we don't care
        (cmd_type, cmd_id) => {
            crate::warn!(
                log,
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

// replays the negotiated format exactly, for port_enum_params(Format);
// kept on the C spa_format_audio_raw_build FFI (unlike the Value-tree
// builders in utils.rs) so the pod stays byte-identical to the C helper
fn build_port_format_info(config: &PortConfig, id: u32) -> Vec<u8> {
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

    let mut buffer = vec![];
    let builder = libspa::pod::builder::Builder::new(&mut buffer);
    // the raw struct is fully initialized above; output goes into the builder
    unsafe { spa_format_audio_raw_build(builder.as_raw_ptr(), id, &raw) };
    drop(builder);
    buffer
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
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");

    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }
    let events = unsafe { (*std::ptr::addr_of!((*state).main)).events.clone() };
    let main = unsafe { std::ptr::addr_of_mut!((*state).main) };
    let control = unsafe { DataControl::from_raw(state) };

    unsafe {
        crate::spa::enum_params_loop(
            main,
            (start, max),
            filter,
            |state, index| {
                use crate::spa::ParamStep;
                #[allow(non_upper_case_globals)]
                match (id, index) {
                    (SPA_PARAM_EnumFormat, i) => {
                        if state.caps_fallback {
                            // the init-time probe hit a busy device and baked in fallback
                            // caps; retry now (main thread, transient open)
                            if let Some(caps) =
                                crate::sound::probe_caps(&state.dsp_path, D::PLAYBACK)
                            {
                                crate::info!(state.log, "re-probed caps: {:?}", caps);
                                state.caps = caps;
                                state.caps_fallback = false;
                            }
                        }
                        match crate::utils::build_enum_format_info(&state.caps, i) {
                            Some(pod) => ParamStep::Built(pod),
                            None => ParamStep::Stop(0),
                        }
                    }
                    (SPA_PARAM_Format, 0) => {
                        match control.query(move |data| data.ports[port_id as usize].config.clone())
                        {
                            Some(Some(cfg)) => {
                                ParamStep::Built(build_port_format_info(&cfg, SPA_PARAM_Format))
                            }
                            Some(None) => ParamStep::Stop(-libc::ENOENT),
                            None => ParamStep::Stop(-libc::EIO),
                        }
                    }
                    (SPA_PARAM_Buffers, 0) => {
                        match control.query(move |data| data.ports[port_id as usize].config.clone())
                        {
                            Some(Some(cfg)) => {
                                ParamStep::Built(crate::utils::build_buffers_info(cfg.stride))
                            }
                            Some(None) => ParamStep::Stop(-libc::ENOENT),
                            None => ParamStep::Stop(-libc::EIO),
                        }
                    }
                    (SPA_PARAM_Latency, 0 | 1) => {
                        let mut info = state.latency[index as usize];
                        // the process latency shifts what we report toward the peer (upstream
                        // for the sink, downstream for the source)
                        if info.direction == D::DIRECTION {
                            crate::utils::process_latency_info_add(
                                &state.process_latency,
                                &mut info,
                            );
                        }
                        ParamStep::Built(crate::utils::build_latency_info(&info))
                    }
                    // a known id whose indices are exhausted ends the enumeration
                    (SPA_PARAM_Format | SPA_PARAM_Buffers | SPA_PARAM_Latency, _) => {
                        ParamStep::Stop(0)
                    }
                    _ => ParamStep::Stop(-libc::ENOENT), // unknown param id (ALSA convention)
                }
            },
            |index, param| emit_param_result(&events, seq, id, index, param),
        )
    }
}

// port_set_param(Format): validate the raw format against the format map and
// build the shared config (the stride falls out of the map's bytes/sample)
fn parse_config<D: Direction>(
    state: &MainState<D>,
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

// A validated Format request. The channel map occupies
// raw.position[..raw.channels]; no pod data is retained.
pub(crate) struct RequestedFormat {
    pub raw: spa_audio_info_raw,
}

// Decode and validate a raw-audio Format pod. Non-raw media returns -ENOENT;
// malformed or degenerate formats return -EINVAL.
//
// # Safety
// `param` must point at a valid, complete spa_pod (the port_set_param
// contract). This is the only raw-pod consumer on the Format path.
unsafe fn decode_format(
    param: *const spa_pod,
    log: &crate::spa::Log,
) -> Result<RequestedFormat, c_int> {
    use libspa::param::format::{MediaSubtype, MediaType};
    use libspa::param::format_utils::parse_format;

    match parse_format(unsafe { libspa::pod::Pod::from_raw(param) }) {
        Ok((MediaType::Audio, MediaSubtype::Raw)) => (),
        Ok((t, st)) => {
            crate::warn!(log, "unknown media type combination: {:?}, {:?}", t, st);
            return Err(-libc::ENOENT);
        }
        Err(err) => {
            crate::warn!(log, "parse_format failed: {}", err);
            return Err(-libc::EINVAL);
        }
    }

    // zeroed, not MaybeUninit: the C parse treats every key as optional and
    // leaves absent ones untouched, so a hostile pod omitting rate/channels
    // would otherwise graduate stack garbage into "parsed" values
    let mut raw: spa_audio_info_raw = unsafe { std::mem::zeroed() };
    if unsafe { spa_format_audio_raw_parse(param, &mut raw) } < 0 {
        crate::warn!(log, "spa_format_audio_raw_parse failed");
        return Err(-libc::EINVAL);
    }

    // format flags are stored but unused, OSS writes interleaved frames
    if raw.rate == 0 || raw.channels == 0 || raw.channels > SPA_AUDIO_MAX_CHANNELS {
        crate::warn!(
            log,
            "rejecting format: rate={} channels={}",
            raw.rate,
            raw.channels
        );
        return Err(-libc::EINVAL);
    }

    Ok(RequestedFormat { raw })
}

// Apply a validated Format request. NEAREST may snap unsupported values to
// the advertised capabilities. Ok(1) tells the adapter to read back the
// adjusted format; validation errors return without emitting port info.
fn set_format_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
    flags: u32,
    requested: RequestedFormat,
) -> Result<c_int, c_int> {
    let mut raw = requested.raw;

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

    let mut res = install_device(state, data, port_idx, config);
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
                state
                    .events
                    .with_port_info(|info| info.bump_param(SPA_PARAM_EnumFormat));
            }
        }
    }
    Ok(res)
}

// port_set_param(Format) with a NULL pod: release the format. Swap a closed
// placeholder and drop the buffers on the data loop, then destroy the old
// device back on the calling main thread (close can sleep).
fn release_format<D: Direction>(
    state: &MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
) -> c_int {
    let placeholder = D::Device::new(&state.dsp_path);
    let Some((retired, deferred)) = data.query(move |state| {
        debug_assert!(!state.rebuild_takeover, "format releases serialize");
        state.rebuild_takeover = true;
        let deferred = state.deferred_work.take();
        let port = &mut state.ports[port_idx];
        let retired = std::mem::replace(&mut port.dsp, placeholder);
        port.buffers.clear();
        port.config = None;
        // retire any in-flight background rebuild, and drop its pending
        // claim with it - a released port must not keep skipping cycles
        // for a completion the bump just retired
        port.generation = port.generation.wrapping_add(1);
        state
            .shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        port.rebuild_pending = true;
        (retired, deferred)
    }) else {
        return -libc::EIO; // the loop still holds the buffers; freeing them would dangle
    };
    drop(retired);
    drop(deferred);
    // Nothing polls a released port. Quiesce the invalidated worker command
    // and drain both sides of the wait so a late Installed deposit cannot
    // retain an exclusive fd indefinitely.
    state.shared.discard_swap();
    if !state.rebuild_worker.wait_idle() {
        release_rebuild_takeover(data, port_idx);
        return -libc::EIO;
    }
    state.shared.discard_swap();
    if release_rebuild_takeover(data, port_idx) {
        0
    } else {
        -libc::EIO
    }
}

// update the port rate and flip Format/Buffers flags to reflect whether a
// format is negotiated, then re-emit so the host re-reads them (PipeWire
// ALSA sink/source pattern)
fn publish_format_state<D: Direction>(state: &MainState<D>, rate: Option<u32>) {
    state.events.with_port_info(|info| {
        let _ = info.replace_change_mask(0);
        if let Some(rate) = rate {
            info.set_rate(spa_fraction {
                num: 1,
                denom: rate,
            });
            info.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
            info.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
        } else {
            info.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
            info.set_param_flags(SPA_PARAM_Buffers, 0);
        }
    });
    emit_port_info(state);
    // This is the ordering token for deferred FormatLost delivery. Advance
    // after the matching owned snapshot has been queued but before callbacks
    // can run at the extern wrapper's flush boundary.
    state.events.advance_format_publication_epoch();
}

// A validated Latency request. The host supplies the opposite direction
// (downstream for a sink, upstream for a source); NULL resets that direction
// to its default. Invalid or same-direction values return -EINVAL.
pub(crate) struct LatencyRequest {
    info: spa_latency_info,
}

fn decode_latency_request(
    direction: spa_direction,
    value: Option<&libspa::pod::Value>,
) -> Result<LatencyRequest, c_int> {
    let other = direction ^ 1;
    let info = match value {
        None => crate::utils::latency_info_default(other),
        Some(v) => match crate::utils::parse_latency_info(Some(v)) {
            Some(info) if info.direction == other => info,
            _ => return Err(-libc::EINVAL),
        },
    };
    Ok(LatencyRequest { info })
}

// Store the latency and re-emit it through the graph.
fn set_latency_param<D: Direction>(state: &mut MainState<D>, request: LatencyRequest) -> c_int {
    let info = request.info;
    state.latency[info.direction as usize] = info;

    state.events.with_port_info(|port| {
        let _ = port.replace_change_mask(0);
        port.bump_param(SPA_PARAM_Latency);
    });
    emit_port_info(state);

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
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let control = unsafe { DataControl::from_raw(state) };
    let main = unsafe { std::ptr::addr_of_mut!((*state).main) };
    let events = unsafe { (*main).events.clone() };
    // SAFETY: the host keeps param valid for this method call. The inner
    // phase queues owned snapshots and invokes no listeners.
    let result =
        unsafe { port_set_param_inner(&mut *main, &control, direction, port_id, id, flags, param) };
    // SAFETY: port_set_param_inner returned, ending its State borrow.
    unsafe { events.flush() };
    result
}

unsafe fn port_set_param_inner<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    flags: u32,
    param: *const spa_pod,
) -> c_int {
    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }

    #[allow(non_upper_case_globals)]
    match id {
        SPA_PARAM_Format => {
            let res = if !param.is_null() {
                // decode to owned data at the boundary; the set is safe code
                let requested = match unsafe { decode_format(param, &state.log) } {
                    Ok(requested) => requested,
                    Err(err) => return err,
                };
                match set_format_param(state, data, port_id as usize, flags, requested) {
                    Ok(res) => res,
                    Err(err) => return err,
                }
            } else {
                match release_format(state, data, port_id as usize) {
                    0 => 0,
                    err => return err,
                }
            };
            // emit even on failure: the flags derive from the (now cleared) config
            let rate = match data.query(move |data| {
                data.ports[port_id as usize]
                    .config
                    .as_ref()
                    .map(|config| config.rate)
            }) {
                Some(rate) => rate,
                None => return -libc::EIO,
            };
            publish_format_state(state, rate);
            res
        }
        SPA_PARAM_Latency => {
            // deserialize at the FFI boundary (None = NULL pod, the reset),
            // decode to the owned request there too; the apply is safe code
            let value = if param.is_null() {
                None
            } else {
                match unsafe { crate::utils::deserialize_pod(param) } {
                    Some(value) => Some(value),
                    None => return -libc::EINVAL,
                }
            };
            match decode_latency_request(direction, value.as_ref()) {
                Ok(request) => set_latency_param(state, request),
                Err(err) => err,
            }
        }
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
pub(crate) struct NodeCallbacks {
    // None means no compatible table is set. The host data pointer accompanies
    // every callback.
    cb: Option<(spa_node_callbacks, *mut c_void)>,
}

impl NodeCallbacks {
    pub(crate) const fn none() -> Self {
        Self { cb: None }
    }

    /// Copy the host's table behind the version gate; NULL clears. Only the
    /// version prefix (offset 0, asserted above) is read until the gate
    /// passes: a host built against an older, shorter table must be rejected
    /// before the full spa_node_callbacks - possibly larger in this build -
    /// is read out of its allocation.
    ///
    /// # Safety
    /// `funcs`, when non-null, must point at a live node-callbacks table
    /// whose version prefix describes its true length, and the host must
    /// keep `data` valid while the table is set (the set_callbacks
    /// contract) - that contract is what makes ready()/xrun() safe calls.
    pub(crate) unsafe fn set(&mut self, funcs: *const spa_node_callbacks, data: *mut c_void) {
        if funcs.is_null() {
            self.cb = None;
            return;
        }
        let version = unsafe { funcs.cast::<u32>().read() };
        if !crate::spa::version_ok(version, SPA_VERSION_NODE_CALLBACKS) {
            self.cb = None;
            return;
        }
        // The version gate guarantees the allocation spans this table.
        self.cb = Some((unsafe { funcs.read() }, data));
    }

    // The copied (table, data) pair for the collect-then-notify call sites:
    // a ready/xrun callback may re-enter node methods (pw runs process()
    // inline from ready() on this same thread) and conjure a fresh
    // &mut DataState, so the caller must end every data/port borrow before
    // invoking a slot from this copy. The invocation is sound per set()'s
    // contract (validated table copy; `data` valid while the table is set).
    pub(crate) fn hook(&self) -> Option<(spa_node_callbacks, *mut c_void)> {
        self.cb
    }
}

// PipeWire may drive process() from a different loop than the DataLoop that
// owns the timer and marshaled state. Refuse processing when their thread
// identities differ; users can pin node.loop.name to keep them together.
fn check_loop_identity(gate: &DataThreadGate) -> bool {
    use std::sync::atomic::Ordering;
    let tid = unsafe { libc::pthread_self() } as usize;
    // Seed the expected id from a closure run on the data loop at init,
    // not claimed by whoever calls first: a pure follower never runs
    // on_timeout, so a process() arriving on a divergent host loop would
    // otherwise install itself as the expected thread and undo the
    // block_on_loop serialization.
    let seen = gate.thread.load(Ordering::Acquire);
    if seen == tid {
        return true;
    }
    if seen != usize::MAX && gate.thread.swap(usize::MAX, Ordering::Relaxed) != usize::MAX {
        crate::warn!(
            gate.log,
            "process() and our data loop run on different threads \
      (multi-data-loop config?); pin node.loop.name for this node. Disabling processing."
        );
    }
    false
}

unsafe extern "C" fn process<D: Direction>(object: *mut c_void) -> c_int {
    let root: *mut State<D> = object.cast();
    assert!(!root.is_null(), "object is not supposed to be null");
    // Reject a divergent process loop before projecting or borrowing DataState.
    let gate = unsafe { &*std::ptr::addr_of!((*root).gate) };
    if !check_loop_identity(gate) {
        return SPA_STATUS_OK as i32;
    }
    let state = unsafe { std::ptr::addr_of_mut!((*root).data) };

    // Phase 1, under a scoped borrow: the data path. Xrun notifications are
    // collected only (detect_underrun/recover_overrun deposit them on the
    // port) so the C callback below runs with no DataState borrow live.
    // SAFETY: object is our State shell (the spa_interface data contract); the
    // borrow ends before any callback is invoked.
    let (result, xrun, main_event) = {
        let state = unsafe { &mut *state };

        // a cycle that was already signaled when we paused can still land here;
        // drop it instead of assert!()ing, which aborts the daemon across
        // extern "C"
        if !state.started || state.position.is_null() {
            return SPA_STATUS_OK as i32;
        }

        let result = D::process_ports(state);
        // collect-then-notify: drain the deposited xrun stamp with the hook copy
        let pending = state.ports.iter_mut().find_map(|p| p.pending_xrun.take());
        let main_event = state.pending_main_event.take().map(|event| {
            (
                state.main_loop,
                state.shared.clone(),
                state.log.clone(),
                event,
            )
        });
        (
            result,
            pending.map(|t| (t, state.callbacks.hook())),
            main_event,
        )
    };

    if let Some((trigger_us, Some((cb, data)))) = xrun {
        if let Some(xrun_fun) = cb.xrun {
            // the xrun event for pw-top's counter; the length isn't known at
            // detection, so 0 delay. No State borrow is live here; sound per
            // NodeCallbacks::hook (validated copy, data valid while set).
            unsafe { xrun_fun(data, trigger_us, 0, std::ptr::null_mut()) };
        }
    }

    if let Some((main_loop, shared, log, event)) = main_event {
        // queue_task may execute inline. No DataState reference is live here, so
        // listener reentry through the endpoint is sound. Deliver after the
        // copied xrun hook: a listener may replace callbacks and invalidate
        // the old callback data pointer.
        queue_main_event(main_loop, shared, log, event);
    }

    result
}

unsafe extern "C" fn port_use_buffers<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    flags: u32,
    buffers: *mut *mut spa_buffer,
    n_buffers: u32,
) -> c_int {
    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");

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
    let control = unsafe { DataControl::from_raw(state) };
    if !control.invoke(move |state| {
        state.ports[port_idx].buffers = new_buffers.into_inner();
        D::on_buffers_swapped(state, port_idx);
    }) {
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

    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");

    // these pointers are dereferenced by process() on the data loop.
    // SAFETY: the host keeps the io areas valid while set (port_set_io
    // contract)
    let data = unsafe { crate::utils::SendWrap::new(data) };
    let control = unsafe { DataControl::from_raw(state) };
    let applied = control.invoke(move |state| {
        let data = data.into_inner();
        // SAFETY (both arms): size/alignment validated above; the host
        // keeps the area valid while set (the port_set_io contract)
        #[allow(non_upper_case_globals)]
        match id {
            SPA_IO_Buffers => unsafe { state.ports[port_id as usize].io.set(data) }, // null clears
            // ACTIVE is managed per cycle in process() and set only while
            // rate matching.
            SPA_IO_RateMatch => unsafe { state.ports[port_id as usize].rate_match.set(data) },
            _ => (),
        }
    });
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
    let state = handle.cast::<State<D>>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    assert!(!interface.is_null());
    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Node.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe {
            *interface = std::ptr::addr_of_mut!((*state).node).cast::<c_void>();
        }
    } else {
        return -libc::ENOENT;
    }
    0
}

unsafe extern "C" fn clear<D: Direction>(handle: *mut spa_handle) -> c_int {
    let state: *mut State<D> = handle.cast();
    assert!(!state.is_null());

    // Queued tasks own only messages and a Weak event endpoint, so no task
    // can dereference State after this function drops it. What clear() must
    // still guarantee: the host has stopped driving the node before clear
    // (Suspend/Pause and io teardown precede it in the SPA lifecycle). A
    // host that still calls process()/on_timeout() afterward frees the
    // ground under the data loop; timer detachment below is our side of the
    // contract.
    {
        let main = unsafe { &mut *std::ptr::addr_of_mut!((*state).main) };
        // Win every open/configure race before asking the worker to stop.
        // stop() drains device-bearing commands on that thread and joins it,
        // so no blocking device destructor remains concurrent with teardown.
        main.shared
            .started
            .store(false, std::sync::atomic::Ordering::Release);
        main.rebuild_worker.stop();
        // A final worker completion may own a device; destroy it here on the
        // main thread, after the worker can no longer deposit another one.
        main.shared.discard_swap();
    }

    // the data loop still holds the timer source; detach it there before the
    // state is freed, then close the timerfd
    let control = unsafe { DataControl::from_raw(state) };
    if !control.invoke(|state| {
        unsafe { state.data_loop.remove_source(&mut state.timer_source) };
        state.data_system.close(state.timer_source.fd);
    }) {
        // freeing the state now would leave the loop a dangling source; a clean
        // abort beats a use-after-free on the next timer tick
        eprintln!("freebsd-oss: can't detach the timer source; aborting");
        std::process::abort();
    }
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
unsafe fn parse_init_dict<D: Direction>(
    info: *const spa_dict,
) -> (Option<String>, u32, D::MainExt) {
    let mut dsp_path = None;
    let mut oss_fragment = 0u32; // automatic (today's layout) unless the dict says otherwise
    let mut ext = D::MainExt::default();

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
fn publish_static_info<D: Direction>(state: &MainState<D>) {
    state.events.with_info(|node, port| {
        // NodeEvents is now at its final Arc allocation, so weave the inline
        // params arrays' self-pointers only after State construction.
        node.fix_pointers();
        port.fix_pointers();

        if D::DIRECTION == SPA_DIRECTION_INPUT {
            node.set_max_input_ports(1);
        } else {
            node.set_max_output_ports(1);
        }
        // The RT flag declares process() safe on the realtime data loop.
        node.set_flags(SPA_NODE_FLAG_RT as u64);
        node.add_prop(crate::spa::key(SPA_KEY_MEDIA_CLASS), D::MEDIA_CLASS);
        node.add_prop(crate::spa::key(SPA_KEY_NODE_DRIVER), "true");

        // No EnumPortConfig/PortConfig (or node-level IO/EnumFormat): dead
        // surface on a follower, see build_port_format_info.
        node.add_param(SPA_PARAM_PropInfo, SPA_PARAM_INFO_READ);
        node.add_param(SPA_PARAM_Props, SPA_PARAM_INFO_READWRITE);
        node.add_param(SPA_PARAM_ProcessLatency, SPA_PARAM_INFO_READWRITE);

        port.set_flags((SPA_PORT_FLAG_PHYSICAL | SPA_PORT_FLAG_TERMINAL) as u64);
        // 1/48000 is the pre-negotiation placeholder.
        port.set_rate(spa_fraction {
            num: 1,
            denom: 48000,
        });
        port.add_param(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
        port.add_param(SPA_PARAM_Buffers, 0);
        port.add_param(SPA_PARAM_Latency, SPA_PARAM_INFO_READWRITE);
    });
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

    let timer_fd = data_system.timerfd_create(
        libc::CLOCK_MONOTONIC,
        (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as i32,
    );
    if timer_fd < 0 {
        return timer_fd; // fd exhaustion fails node creation, not the daemon
    }

    let (dsp_path, oss_fragment, ext) = unsafe { parse_init_dict::<D>(info) };

    let Some(dsp_path) = dsp_path else {
        data_system.close(timer_fd);
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

    let state = handle.cast::<State<D>>();
    assert!(!state.is_null(), "handle is not supposed to be null");

    let node_methods: &'static spa_node_methods = &D::NODE_METHODS;
    let events = std::sync::Arc::new(NodeEvents::<D>::new());
    let shared = std::sync::Arc::new(NodeShared::new(std::sync::Arc::downgrade(&events)));
    let rebuild_worker = match RebuildWorker::<D>::start() {
        Ok(worker) => worker,
        Err(err) => {
            data_system.close(timer_fd);
            crate::error!(log, "can't start the device rebuild worker: {}", err);
            return -libc::EIO;
        }
    };
    let rebuild_work = rebuild_worker.endpoint();
    let data_ext = D::data_ext(&ext);
    let main_loop = if main_loop.is_null() {
        None
    } else {
        Some(unsafe { crate::spa::Loop::wrap(main_loop) })
    };

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

                gate: DataThreadGate {
                    thread: std::sync::atomic::AtomicUsize::new(0),
                    log: log.clone(),
                },
                main: MainState {
                    events: events.clone(),
                    data_loop,
                    log: log.clone(),
                    dsp_path: dsp_path.clone(),
                    caps,
                    caps_fallback,
                    oss_fragment,
                    oss_fragment_default: oss_fragment,
                    latency: [
                        crate::utils::latency_info_default(SPA_DIRECTION_INPUT),
                        crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT),
                    ],
                    process_latency: crate::utils::process_latency_default(),
                    shared: shared.clone(),
                    rebuild_worker,
                    ring_cap_published: false,
                    ext,
                },
                data: DataState {
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
                    main_loop,
                    dsp_path: dsp_path.clone(),
                    timer_source: spa_source {
                        loop_: std::ptr::null_mut(),
                        func: Some(on_timeout::<D>),
                        data: state.cast::<c_void>(),
                        fd: timer_fd,
                        mask: SPA_IO_IN,
                        rmask: 0,
                        priv_: std::ptr::null_mut(),
                    },
                    next_time: 0,
                    callbacks: NodeCallbacks::none(),
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
                        rebuild_pending: false,
                        generation: 0,
                        was_matching: false,
                        warn_limit: crate::utils::RateLimit::new(),
                        pending_xrun: None,
                        ext: std::default::Default::default(),
                    }; MAX_PORTS],
                    oss_fragment,
                    shared,
                    rebuild_work,
                    deferred_work: None,
                    rebuild_takeover: false,
                    events,
                    pending_main_event: None,
                    started: false,
                    following: false,
                    ext: data_ext,
                },
            },
        );
    }

    let main = unsafe { &*std::ptr::addr_of!((*state).main) };
    publish_static_info(main);

    let data = unsafe { &mut *std::ptr::addr_of_mut!((*state).data) };
    let err = unsafe { data.data_loop.add_source(&mut data.timer_source) };
    if err < 0 {
        unsafe {
            data.data_system.close(data.timer_source.fd);
            // the host won't call clear() after a failed init; free what we built
            std::ptr::drop_in_place(state);
        }
        return err;
    }

    // learn the data loop's thread identity from the loop itself (see
    // check_loop_identity); pw's data loops run before any node loads, so
    // this executes on the loop thread, not inline
    let control = unsafe { DataControl::from_raw(state) };
    let thread = unsafe { std::ptr::addr_of!((*state).gate.thread) };
    let loop_thread = unsafe { crate::utils::SendWrap::new(thread.cast_mut()) };
    let seeded = control.invoke(move |_data| {
        let thread = loop_thread.into_inner();
        let tid = unsafe { libc::pthread_self() } as usize;
        // A process call cannot legitimately precede successful init, but
        // preserve a gate that was already disabled rather than reviving it.
        let _ = unsafe { &*thread }.compare_exchange(
            0,
            tid,
            std::sync::atomic::Ordering::Release,
            std::sync::atomic::Ordering::Relaxed,
        );
    });
    if !seeded {
        unsafe { &*thread }.store(usize::MAX, std::sync::atomic::Ordering::Release);
        crate::warn!(
            unsafe { &*std::ptr::addr_of!((*state).gate.log) },
            "can't seed the data-loop thread identity; disabling processing"
        );
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
mod tests;
