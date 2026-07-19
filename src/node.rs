// The shared node core. sink.rs and source.rs are the same SPA node modulo
// direction: everything direction-agnostic lives here once, generic over
// `Direction`, and the genuinely direction-specific logic (the process() data
// path, the servo error sign, priming/recovery semantics, the oss.delay prop)
// is supplied through the `Direction` hooks each module implements. The
// extern "C" vtable entries are generic and monomorphized per direction.

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

struct EventInfo {
    node: crate::spa::NodeInfo,
    port: crate::spa::PortInfo,
}

enum NodeNotification {
    Node(Box<crate::spa::NodeInfo>),
    Port(Box<crate::spa::PortInfo>),
    Done(c_int),
    ActivateListeners(std::sync::Arc<crate::spa::ListenerList<spa_node_events>>),
}

struct PendingNodeNotifications {
    queue: std::collections::VecDeque<NodeNotification>,
    dispatching: bool,
}

// Main-loop-owned listener and info state. The Arc gives queued messages a
// lifetime-safe endpoint without exposing State across loops. Info mutation
// is locked only on the main loop; callbacks run after the lock is released,
// against owned snapshots, so listener reentry cannot alias or invalidate the
// live payload. The hook list itself follows SPA's main-loop serialization.
struct NodeEvents<D: Direction> {
    hooks: crate::spa::ListenerList<spa_node_events>,
    info: std::sync::Mutex<EventInfo>,
    pending: std::sync::Mutex<PendingNodeNotifications>,
    // Changes only when the advertised Format/Buffers state is published.
    // Deferred FormatLost messages carry the value they observed so a newer
    // successful format publication cannot be overwritten by a stale task.
    format_publication_epoch: std::sync::atomic::AtomicU64,
    _direction: std::marker::PhantomData<fn() -> D>,
}

struct NodeDispatchClaim<'a, D: Direction>(&'a NodeEvents<D>);

impl<D: Direction> Drop for NodeDispatchClaim<'_, D> {
    fn drop(&mut self) {
        self.0
            .pending
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .dispatching = false;
    }
}

// SAFETY: NodeEvents' safe methods serialize info through the mutex and never
// expose it. Listener-list access is confined to those methods and the SPA
// add-listener entry point, all of which the host calls on the main loop.
// Cross-loop users hold only Weak/Arc handles and queue an owned MainEvent
// back to that loop; they never traverse the list themselves.
unsafe impl<D: Direction> Send for NodeEvents<D> {}
unsafe impl<D: Direction> Sync for NodeEvents<D> {}

impl<D: Direction> NodeEvents<D> {
    fn new() -> Self {
        Self {
            hooks: crate::spa::ListenerList::new(),
            info: std::sync::Mutex::new(EventInfo {
                node: crate::spa::NodeInfo::new(),
                port: crate::spa::PortInfo::new(),
            }),
            pending: std::sync::Mutex::new(PendingNodeNotifications {
                queue: std::collections::VecDeque::new(),
                dispatching: false,
            }),
            format_publication_epoch: std::sync::atomic::AtomicU64::new(0),
            _direction: std::marker::PhantomData,
        }
    }

    fn with_info<R>(
        &self,
        apply: impl FnOnce(&mut crate::spa::NodeInfo, &mut crate::spa::PortInfo) -> R,
    ) -> R {
        let mut info = self
            .info
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let EventInfo { node, port } = &mut *info;
        apply(node, port)
    }

    fn with_node_info<R>(&self, apply: impl FnOnce(&mut crate::spa::NodeInfo) -> R) -> R {
        self.with_info(|node, _port| apply(node))
    }

    fn with_port_info<R>(&self, apply: impl FnOnce(&mut crate::spa::PortInfo) -> R) -> R {
        self.with_info(|_node, port| apply(port))
    }

    fn initial_snapshots(&self) -> (Box<crate::spa::NodeInfo>, Box<crate::spa::PortInfo>) {
        self.with_info(|node, port| {
            let mut node = node.snapshot();
            let mut port = port.snapshot();
            let _ = node.replace_change_mask(crate::spa::SPA_NODE_CHANGE_MASK_ALL as u64);
            let _ = port.replace_change_mask(crate::spa::SPA_PORT_CHANGE_MASK_ALL as u64);
            (node, port)
        })
    }

    fn queue_node_info(&self) {
        let snapshot = self.with_node_info(|info| {
            let snapshot = info.snapshot();
            let _ = info.replace_change_mask(0);
            snapshot
        });
        self.pending
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .queue
            .push_back(NodeNotification::Node(snapshot));
    }

    fn queue_port_info(&self) {
        let snapshot = self.with_port_info(|info| {
            let snapshot = info.snapshot();
            let _ = info.replace_change_mask(0);
            snapshot
        });
        self.pending
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .queue
            .push_back(NodeNotification::Port(snapshot));
    }

    fn format_publication_epoch(&self) -> u64 {
        self.format_publication_epoch
            .load(std::sync::atomic::Ordering::Acquire)
    }

    fn advance_format_publication_epoch(&self) {
        self.format_publication_epoch
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
    }

    // Register immediately on the active list when no older FIFO work exists.
    // Otherwise enqueue an activation barrier before the synchronous initial
    // callbacks: older notifications stay ahead of the new listener, while
    // anything those callbacks queue lands after it.
    unsafe fn with_new_listener<R>(
        &self,
        listener: *mut spa_hook,
        events: *const spa_node_events,
        data: *mut c_void,
        initial: impl FnOnce(&crate::spa::ListenerList<spa_node_events>) -> R,
    ) -> R {
        let deferred = {
            let mut pending = self
                .pending
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            if pending.dispatching || !pending.queue.is_empty() {
                // Arc is intentional even though ListenerList itself remains
                // main-loop-only: NodeEvents has cross-loop Arc handles, while
                // this atomic owner keeps a reentrantly drained cohort alive
                // through its synchronous initial callback.
                #[allow(clippy::arc_with_non_send_sync)]
                let hooks = std::sync::Arc::new(crate::spa::ListenerList::new());
                pending
                    .queue
                    .push_back(NodeNotification::ActivateListeners(hooks.clone()));
                Some(hooks)
            } else {
                None
            }
        };
        let hooks = deferred.as_deref().unwrap_or(&self.hooks);
        unsafe { hooks.with_isolated_listener(listener, events, data, || initial(hooks)) }
    }

    // SAFETY: no reference into the associated State may be live. Listener
    // code may re-enter any node method and create a new mutable State borrow.
    unsafe fn dispatch(&self, notification: &NodeNotification) {
        match notification {
            NodeNotification::Node(snapshot) => self.hooks.emit(|f, data| {
                if let Some(info) = f.info {
                    // through the C listener vtable (add_listener contract)
                    unsafe { info(data, snapshot.raw()) };
                }
            }),
            NodeNotification::Port(snapshot) => self.hooks.emit(|f, data| {
                if let Some(info) = f.port_info {
                    // through the C listener vtable (add_listener contract)
                    unsafe { info(data, D::DIRECTION, 0, snapshot.raw()) };
                }
            }),
            NodeNotification::Done(seq) => self.hooks.emit(|f, data| {
                if let Some(result) = f.result {
                    unsafe { result(data, *seq, 0, 0, std::ptr::null()) };
                }
            }),
            NodeNotification::ActivateListeners(hooks) => {
                // SAFETY: drain processes barriers between listener
                // traversals; the isolated batch finished its initial
                // callbacks before it was eligible to reach this point.
                unsafe { self.hooks.append_from(hooks) };
            }
        }
    }

    // Claim the endpoint's dispatch turn. A reentrant producer appends to the
    // same FIFO and returns; the outer owner completes its current transaction
    // before draining the nested one.
    fn begin_dispatch(&self) -> Option<NodeDispatchClaim<'_, D>> {
        let mut pending = self
            .pending
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if pending.dispatching {
            None
        } else {
            pending.dispatching = true;
            Some(NodeDispatchClaim(self))
        }
    }

    // Called only by the owner returned from begin_dispatch(), after the
    // surrounding State borrow has ended. Pop one notification at a time so
    // the mutex is never held across arbitrary listener code.
    // SAFETY: as dispatch(); callers must end their State phase first.
    unsafe fn drain(&self, _claim: &NodeDispatchClaim<'_, D>) {
        loop {
            let notification = {
                let mut pending = self
                    .pending
                    .lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner);
                match pending.queue.pop_front() {
                    Some(notification) => notification,
                    None => return,
                }
            };
            unsafe { self.dispatch(&notification) };
        }
    }

    // SAFETY: no associated State reference may be live. Reentrant flushes
    // only enqueue; the outer drain preserves FIFO transaction ordering.
    unsafe fn flush(&self) {
        if let Some(claim) = self.begin_dispatch() {
            unsafe { self.drain(&claim) };
        }
    }

    fn record_format_lost(&self) {
        self.with_port_info(|info| {
            let _ = info.replace_change_mask(0);
            info.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
            info.set_param_flags(SPA_PARAM_Buffers, 0);
            // This serial flip is what audioadapter reacts to: an EnumFormat
            // flags change sets recheck_format and starts renegotiation.
            info.bump_param(SPA_PARAM_EnumFormat);
        });
        self.queue_port_info();
    }

    fn record_current_format_lost(&self) {
        self.record_format_lost();
        // Retire duplicate deferred losses before the queued snapshot is
        // flushed and listeners can re-enter.
        self.advance_format_publication_epoch();
    }

    // SAFETY: no State reference may be live during listener dispatch.
    unsafe fn emit_format_lost_now(&self, expected_epoch: u64) {
        if self.format_publication_epoch() != expected_epoch {
            return;
        }
        self.record_current_format_lost();
        unsafe { self.flush() };
    }

    // SAFETY: no associated State reference may be live.
    unsafe fn emit_done(&self, seq: c_int) {
        self.pending
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .queue
            .push_back(NodeNotification::Done(seq));
        unsafe { self.flush() };
    }

    // SAFETY: no associated State reference may be live.
    unsafe fn emit_result(&self, seq: c_int, result: &spa_result_node_params) {
        crate::spa::node_emit_result(&self.hooks, seq, 0, SPA_RESULT_TYPE_NODE_PARAMS, result);
    }
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
    resetup_worker: ResetupWorker<D>,
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
    // the Arc'd rendezvous with the owned resetup worker and
    // clear(); outlives the FFI shell by construction (see NodeShared)
    pub shared: std::sync::Arc<NodeShared<D>>,
    // The data loop is the sole producer. A device-bearing command that
    // finds the worker slot occupied stays here and is retried before any
    // further completion is consumed; it is never dropped on the RT path.
    resetup_work: std::sync::Arc<ResetupWorkSlot<D>>,
    deferred_work: Option<ResetupWork<D>>,
    // Main-thread synchronous installs take this data-loop lease before
    // waiting for the worker. While set, process neither consumes a
    // completion nor submits new work.
    resetup_takeover: bool,
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
    // A main-loop device rebuild is in flight; skip cycles until poll_resetup
    // consumes its completion. Data-loop-owned: set when the order is queued,
    // cleared when the completion swap is consumed (or by the install/suspend
    // swap closures, which also run on this loop) - no other thread touches it.
    pub resetup_pending: bool,
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
            || port.resetup_pending
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

unsafe extern "C" fn send_command<D: Direction>(
    object: *mut c_void,
    command: *const spa_command,
) -> c_int {
    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");
    let control = unsafe { DataControl::from_raw(state) };
    let (log, shared, resetup_work) = unsafe {
        let main = std::ptr::addr_of!((*state).main);
        (
            (*main).log.clone(),
            (*main).shared.clone(),
            (*main).resetup_worker.endpoint(),
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
            let Some(true) = started else {
                return -libc::EIO; // not negotiated yet (ALSA rejects this too)
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
            shared
                .started
                .store(false, std::sync::atomic::Ordering::Release);
            let Some(deferred) = control.query(|state| {
                state.started = false;
                update_timers(state);
                state.resetup_takeover = true;
                let deferred = state.deferred_work.take();
                for port in &mut state.ports {
                    port.resetup_pending = true;
                    port.generation = port.generation.wrapping_add(1);
                    state
                        .shared
                        .generation
                        .store(port.generation, std::sync::atomic::Ordering::Release);
                }
                deferred
            }) else {
                return -libc::EIO;
            };
            drop(deferred);
            // Catch both a completion deposited before the fence and one
            // from a worker that passed its final check just before it.
            shared.discard_swap();
            if !resetup_work.wait_idle() {
                release_resetup_takeover(&control, 0);
                return -libc::EIO;
            }
            shared.discard_swap();
            if !release_resetup_takeover(&control, 0) {
                return -libc::EIO;
            }
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => {
            // As with Pause, stop wins before waiting for the data loop.
            shared
                .started
                .store(false, std::sync::atomic::Ordering::Release);
            // Device::new may probe sndstat (the sink does), so build every
            // closed placeholder here on main rather than inside DataControl.
            let dsp_path = unsafe { &*std::ptr::addr_of!((*state).main.dsp_path) };
            let placeholders: [D::Device; MAX_PORTS] =
                std::array::from_fn(|_| D::Device::new(dsp_path));
            // Quiesce and transfer device ownership out of DataState. Potentially
            // sleeping SETTRIGGER/close operations then run on this thread while
            // the data loop sees only closed placeholders.
            let Some((mut devices, deferred)) = control.query(move |state| {
                state.started = false;
                update_timers(state);
                D::on_suspend_loop(state);
                state.resetup_takeover = true;
                let deferred = state.deferred_work.take();
                let mut placeholders = placeholders.into_iter();
                let devices: [(usize, D::Device); MAX_PORTS] = std::array::from_fn(|index| {
                    let port = &mut state.ports[index];
                    port.resetup_pending = true;
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
                return -libc::EIO;
            };
            drop(deferred);
            // a deposited-but-unconsumed rebuild would hold an open
            // (possibly exclusive) device across the whole suspended stretch
            // (nothing polls while stopped); close it now, off the RT path.
            shared.discard_swap();
            if !resetup_work.wait_idle() {
                release_resetup_takeover(&control, 0);
                return -libc::EIO;
            }
            shared.discard_swap();
            for (_, dsp) in &mut devices {
                if !dsp.is_closed() && !dsp.suspend() {
                    dsp.close();
                }
            }
            let placeholders = control.query(move |state| {
                let mut devices = devices.into_iter();
                let placeholders: [D::Device; MAX_PORTS] = std::array::from_fn(|index| {
                    let (_, dsp) = devices.next().expect("one suspended device per port");
                    state.ports[index].resetup_pending = false;
                    std::mem::replace(&mut state.ports[index].dsp, dsp)
                });
                state.resetup_takeover = false;
                placeholders
            });
            let Some(placeholders) = placeholders else {
                release_resetup_takeover(&control, 0);
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
        debug_assert!(!state.resetup_takeover, "format releases serialize");
        state.resetup_takeover = true;
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
        port.resetup_pending = true;
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
    if !state.resetup_worker.wait_idle() {
        release_resetup_takeover(data, port_idx);
        return -libc::EIO;
    }
    state.shared.discard_swap();
    if release_resetup_takeover(data, port_idx) {
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
// Synchronous contract (see install_device): main-thread entry, blocking
// frame-bounded invokes only.
pub(crate) fn store_and_rebuild<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    store: impl FnOnce(&mut DataState<D>) + Send,
) -> c_int {
    let configs: Option<[Option<PortConfig>; MAX_PORTS]> = data.query(move |data| {
        store(data);
        std::array::from_fn(|i| {
            data.ports[i]
                .dsp
                .is_running()
                .then(|| data.ports[i].config.clone())
                .flatten()
        })
    });
    let Some(configs) = configs else {
        return -libc::EIO;
    };
    for (port_idx, config) in configs.into_iter().enumerate() {
        if let Some(config) = config {
            if install_device(state, data, port_idx, config) != 0 {
                // the host didn't initiate this rebuild; without a re-announce it
                // keeps believing a format is set on a dead port
                emit_format_lost(state);
            }
        }
    }
    0
}

// announce a Props change (so readback stays fresh), then apply it through
// store_and_rebuild; shared by the oss.* prop appliers of both directions
pub(crate) fn apply_props_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    store: impl FnOnce(&mut DataState<D>) + Send,
) -> c_int {
    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Props);
    });
    emit_node_info(state);
    store_and_rebuild(state, data, store)
}

// Release the synchronous rebuild lease after an error. A dead loop cannot
// observe the retained flag, so failure remains best-effort.
fn release_resetup_takeover<D: Direction>(data: &DataControl<D>, port_idx: usize) -> bool {
    data.invoke(move |state| {
        state.resetup_takeover = false;
        state.ports[port_idx].resetup_pending = false;
    })
}

// Open and configure on the main thread because device operations may block,
// then swap the device on the data loop. The takeover fence invalidates
// asynchronous rebuilds while the synchronous install is active. On EBUSY,
// retire the current exclusive device and retry; other failures leave the
// port cleared.
pub(crate) fn install_device<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
    config: PortConfig,
) -> c_int {
    // Acquire the sole-producer takeover on the data loop before waiting.
    // The generation bump makes both queued and active resetups stale;
    // resetup_takeover makes later cycles skip without consuming or
    // submitting until the final swap releases the lease.
    let Some(deferred) = data.query(move |data| {
        debug_assert!(!data.resetup_takeover, "synchronous installs serialize");
        data.resetup_takeover = true;
        let deferred = data.deferred_work.take();
        let port = &mut data.ports[port_idx];
        port.resetup_pending = true;
        port.generation = port.generation.wrapping_add(1);
        data.shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        deferred
    }) else {
        return -libc::EIO;
    };
    // Any retained RetireAndRetry/device ownership now dies here, never on
    // the data loop.
    drop(deferred);

    // Close a completion that predates the fence, then wait until an active
    // command observes the generation change and finishes. The second drain
    // catches the completion it may have deposited before becoming idle.
    state.shared.discard_swap();
    if !state.resetup_worker.wait_idle() {
        release_resetup_takeover(data, port_idx);
        return -libc::EIO;
    }
    state.shared.discard_swap();

    let mut new_dsp = D::Device::new(&state.dsp_path);
    // oss_fragment only mutates from main-thread calls, serialized with us
    let mut res = D::try_open_configure(&mut new_dsp, &config, state.oss_fragment, &state.log);

    if res == -libc::EBUSY {
        let closed = D::Device::new(&state.dsp_path);
        let Some(retired) =
            data.query(move |state| std::mem::replace(&mut state.ports[port_idx].dsp, closed))
        else {
            release_resetup_takeover(data, port_idx);
            return -libc::EIO;
        };
        drop(retired); // closes the old fd here, off the RT path
        res = D::try_open_configure(&mut new_dsp, &config, state.oss_fragment, &state.log);
    }

    let ok = res == 0;
    let cap_config = config.clone();
    let old_dsp = data.query(move |state| {
        let port = &mut state.ports[port_idx];
        // new_dsp is a closed writer/reader when negotiation failed above
        let old = std::mem::replace(&mut port.dsp, new_dsp);
        port.config = if ok { Some(config) } else { None };
        // Retire any in-flight background rebuild.
        port.generation = port.generation.wrapping_add(1);
        state
            .shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        port.resetup_pending = false;
        port.was_matching = false; // force a relock when matching resumes
        D::on_device_swapped(state, port_idx);
        state.resetup_takeover = false;
        old
    });
    let swapped = old_dsp.is_some();
    drop(old_dsp); // ditto

    if !swapped {
        release_resetup_takeover(data, port_idx);
        return -libc::EIO; // the swap never ran; the port keeps its old state
    }
    if res == 0 {
        publish_ring_quantum_cap(state, &cap_config); // stride is known now
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
fn publish_ring_quantum_cap<D: Direction>(state: &mut MainState<D>, config: &PortConfig) {
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
    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        info.add_prop("node.max-latency", format!("{frames}/{rate}"));
    });
    emit_node_info(state);
}

// Announce a failed background rebuild so the session manager renegotiates
// the cleared format.
fn emit_format_lost<D: Direction>(state: &MainState<D>) {
    state.events.record_current_format_lost();
}

// Asynchronous rebuilds carry owned requests from the data loop to the
// blocking-I/O worker. The worker never accesses State: it returns an owned
// DeviceSwap through NodeShared, and the data loop accepts it only when the
// port generation still matches. Retired devices also move to the worker so
// potentially blocking closes stay off the real-time path.

// The completion mailbox has one slot; multi-port support requires one slot
// per port.
const _: () = assert!(
    MAX_PORTS == 1,
    "NodeShared's completion mailbox assumes a single port"
);

// Shared state for the data loop, rebuild worker, and clear(). Worker guards
// keep it alive independently of State.
pub(crate) struct NodeShared<D: Direction> {
    // A lifetime-safe main-loop event endpoint. State owns the sole strong
    // handle; queued work can test/upgrade this Weak without ever obtaining
    // a State pointer. Once clear drops State, later deliveries are no-ops.
    events: std::sync::Weak<NodeEvents<D>>,
    // mirror of State.started, written by send_command on the main thread
    // (Start/Pause/Suspend), read by resetup_task on the worker: a
    // stop that lands after a task was queued must win, or the task would
    // hand a stopped node an open (possibly exclusive) device
    started: std::sync::atomic::AtomicBool,
    // Mirror of the single data-loop port's generation. Worker resetup
    // work checks it before and after an open so a released/superseded
    // request cannot leave an exclusive stale fd in the completion slot.
    generation: std::sync::atomic::AtomicU64,
    // The completion mailbox: a preallocated single-slot cell. The worker
    // deposits (replacing an unconsumed predecessor); the main loop
    // may discard during synchronous changes and teardown, while the
    // data loop consumes at cycle start. The RT side never locks or
    // allocates: take_swap is one CAS plus the in-place move, and when it
    // loses the race against a mid-deposit writer it returns None and polls
    // again next cycle. Only the non-RT writer may spin, and only while
    // the reader is inside its few-instruction move. The value lives in the
    // UnsafeCell and is touched exclusively by whoever holds SLOT_BUSY -
    // the protocol behind the manual Sync impl below.
    slot_state: std::sync::atomic::AtomicU8,
    slot: std::cell::UnsafeCell<Option<DeviceSwap<D>>>,
}

const SLOT_EMPTY: u8 = 0; // no message; the cell is None
const SLOT_FULL: u8 = 1; // one message; the cell is Some
const SLOT_BUSY: u8 = 2; // one side is moving the value; the cell is theirs

// SAFETY: the slot cell is only read or written by the thread that CASed
// slot_state to SLOT_BUSY (exchange/take_swap below), and the FULL store
// after a deposit is Release, paired with take_swap's Acquire CAS - so the
// message payload is published before the consumer can move it out. The
// remaining fields are atomics or thread-safe Weak handles.
unsafe impl<D: Direction> Sync for NodeShared<D> {}

// Owned event sent from the data loop to the main-loop endpoint.
enum MainEvent {
    // Re-announce a format cleared by a failed background rebuild.
    FormatLost { expected_publication_epoch: u64 },
}

impl<D: Direction> NodeShared<D> {
    fn new(events: std::sync::Weak<NodeEvents<D>>) -> Self {
        Self {
            events,
            started: std::sync::atomic::AtomicBool::new(false),
            generation: std::sync::atomic::AtomicU64::new(0),
            slot_state: std::sync::atomic::AtomicU8::new(SLOT_EMPTY),
            slot: std::cell::UnsafeCell::new(None),
        }
    }

    // Queued tasks may deliver until clear() drops the event endpoint.
    fn is_alive(&self) -> bool {
        self.events.strong_count() != 0
    }

    // Deliver queued messages on the main-loop thread. A dead Weak drops them.
    fn main_event(&self, event: MainEvent) {
        let Some(events) = self.events.upgrade() else {
            return;
        };
        match event {
            // SAFETY: this endpoint message contains no State reference and
            // its caller ended the process DataState phase before queueing it.
            MainEvent::FormatLost {
                expected_publication_epoch,
            } => unsafe { events.emit_format_lost_now(expected_publication_epoch) },
        }
    }

    // The non-RT writer side of the slot protocol: acquire SLOT_BUSY from
    // EMPTY or FULL, swap the new value in, publish the resulting state, and
    // hand the predecessor back to the caller to drop off the RT path.
    fn exchange(&self, new: Option<DeviceSwap<D>>) -> Option<DeviceSwap<D>> {
        use std::sync::atomic::Ordering;
        loop {
            let cur = self.slot_state.load(Ordering::Relaxed);
            if cur == SLOT_BUSY {
                // Writers are worker/main-loop only, never RT. Yield instead
                // of burning a core if the few-instruction slot owner was
                // preempted while BUSY.
                std::thread::yield_now();
                continue;
            }
            debug_assert!(cur == SLOT_EMPTY || cur == SLOT_FULL);
            if self
                .slot_state
                .compare_exchange_weak(cur, SLOT_BUSY, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
            {
                let full = new.is_some();
                // SAFETY: SLOT_BUSY is held; the cell is exclusively ours
                let prev = unsafe { std::mem::replace(&mut *self.slot.get(), new) };
                self.slot_state
                    .store(if full { SLOT_FULL } else { SLOT_EMPTY }, Ordering::Release);
                return prev;
            }
        }
    }

    // worker: leave the completion for the data loop (replacing an
    // unconsumed predecessor, whose device closes here, off the RT path)
    fn deposit(&self, swap: DeviceSwap<D>) {
        let prev = self.exchange(Some(swap));
        drop(prev);
    }

    // Data loop (single consumer): the completion, if one arrived. Never
    // waits: a writer mid-deposit just reads as "nothing yet".
    fn take_swap(&self) -> Option<DeviceSwap<D>> {
        use std::sync::atomic::Ordering;
        if self
            .slot_state
            .compare_exchange(SLOT_FULL, SLOT_BUSY, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None; // empty, or a writer holds the slot
        }
        // SAFETY: SLOT_BUSY is held; the cell is exclusively ours
        let swap = unsafe { (*self.slot.get()).take() };
        debug_assert!(swap.is_some(), "SLOT_FULL always covers a Some cell");
        self.slot_state.store(SLOT_EMPTY, Ordering::Release);
        swap
    }

    // main thread (install_device, Suspend, clear): void any undelivered
    // completion; its device closes here, off the RT path
    fn discard_swap(&self) {
        let dropped = self.exchange(None);
        drop(dropped);
    }
}

// Rebuild request sent to the worker. It contains everything needed to open
// and configure a device without accessing State.
pub(crate) struct ResetupRequest<D: Direction> {
    port_idx: usize,
    generation: u64,
    config: PortConfig,
    path: String,
    oss_fragment: u32,
    retried: bool, // the EBUSY retire round trip already happened
    // RetireAndRetry only: the port's dying fd, closed by the worker under
    // its unwind guard before the retry opens
    retire_first: Option<D::Device>,
    log: crate::spa::Log,
    // Weak avoids a NodeShared -> mailbox -> retry request -> NodeShared
    // cycle while a RetireAndRetry completion waits for the data loop.
    shared: std::sync::Weak<NodeShared<D>>,
}

// Worker result. The data loop applies it only while the port generation
// still matches.
struct DeviceSwap<D: Direction> {
    port_idx: usize,
    generation: u64,
    outcome: SwapOutcome<D>,
}

enum SwapOutcome<D: Direction> {
    // open+configure succeeded: install and resume
    Installed {
        dsp: D::Device,
        config: PortConfig,
    },
    // the node was stopped when the task ran: drop the pending claim; the
    // next started cycle re-queues if the port still needs a rebuild
    Aborted,
    // open failed with EBUSY and the port's own (dying) fd is the likely
    // blocker on an exclusive device (bitperfect, vchans off): retire it,
    // then re-run the request - the retire needs the data loop, so it is
    // another message round trip
    RetireAndRetry {
        request: ResetupRequest<D>,
        placeholder: D::Device,
    },
    // open/configure failed (even after the retire, for EBUSY): the port
    // loses its format; poll_resetup clears the config and queues the
    // format-lost re-announce
    Failed {
        placeholder: D::Device,
    },
}

// Owned commands for the per-node blocking-I/O worker. No variant contains
// State or a pointer into it. In particular, retirement transfers device
// ownership all the way to this worker so a Device destructor can never run
// on the data loop.
enum ResetupWork<D: Direction> {
    Resetup(ResetupRequest<D>),
    RetireDevice(D::Device),
    RetireSwap(DeviceSwap<D>),
    #[cfg(test)]
    Test(Box<dyn FnOnce() + Send>),
}

enum WorkSubmission<D: Direction> {
    Submitted,
    Returned(ResetupWork<D>),
}

const WORK_EMPTY: u8 = 0;
const WORK_FULL: u8 = 1;
const WORK_BUSY: u8 = 2;
const WORK_CLOSED: u8 = 3;

// A preallocated, single-producer/single-consumer work slot. MAX_PORTS == 1
// and resetup_pending permit only one rebuild order at a time. DataState's
// additional deferred_work cell retains the one retirement/retry that can
// collide with an occupied slot. Submission never waits and never allocates.
struct ResetupWorkSlot<D: Direction> {
    stopping: std::sync::atomic::AtomicBool,
    state: std::sync::atomic::AtomicU8,
    value: std::cell::UnsafeCell<Option<ResetupWork<D>>>,
    thread: std::sync::OnceLock<std::thread::Thread>,
    // The worker sets active while holding this mutex before it takes a
    // published command. That closes the otherwise-racy gap between an
    // empty slot and execution for main-thread takeover waits.
    active: std::sync::Mutex<bool>,
    idle: std::sync::Condvar,
}

// SAFETY: the data loop is the sole producer and the worker is the sole
// consumer. Either side may access value only after changing state from
// EMPTY/FULL to BUSY; the Release publication of FULL is paired with the
// worker's Acquire CAS. A failed producer CAS returns its still-owned value.
unsafe impl<D: Direction> Sync for ResetupWorkSlot<D> {}

impl<D: Direction> ResetupWorkSlot<D> {
    fn new() -> Self {
        Self {
            stopping: std::sync::atomic::AtomicBool::new(false),
            state: std::sync::atomic::AtomicU8::new(WORK_EMPTY),
            value: std::cell::UnsafeCell::new(None),
            thread: std::sync::OnceLock::new(),
            active: std::sync::Mutex::new(false),
            idle: std::sync::Condvar::new(),
        }
    }

    // Data-loop producer. Ownership is returned on every failure so a
    // device-bearing command cannot be destroyed in this call.
    fn try_submit(&self, work: ResetupWork<D>) -> WorkSubmission<D> {
        self.try_submit_after_claim(work, || {})
    }

    // The callback is normally empty and optimizes away. Tests use it to
    // pause a producer after EMPTY->BUSY and deterministically exercise
    // takeover/shutdown against an in-progress publication.
    fn try_submit_after_claim(
        &self,
        work: ResetupWork<D>,
        after_claim: impl FnOnce(),
    ) -> WorkSubmission<D> {
        use std::sync::atomic::Ordering;
        if self
            .state
            .compare_exchange(WORK_EMPTY, WORK_BUSY, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return WorkSubmission::Returned(work);
        }
        after_claim();
        // SAFETY: this producer changed EMPTY to BUSY and owns the cell.
        unsafe { *self.value.get() = Some(work) };
        self.state.store(WORK_FULL, Ordering::Release);
        if let Some(thread) = self.thread.get() {
            thread.unpark();
        }
        WorkSubmission::Submitted
    }

    // Worker consumer. BUSY is reported like empty; the publishing producer
    // will unpark us after its short in-place move.
    fn take(&self) -> Option<ResetupWork<D>> {
        use std::sync::atomic::Ordering;
        if self
            .state
            .compare_exchange(WORK_FULL, WORK_BUSY, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None;
        }
        // SAFETY: this consumer changed FULL to BUSY and owns the cell.
        let work = unsafe { (*self.value.get()).take() };
        debug_assert!(work.is_some(), "WORK_FULL always covers a Some cell");
        self.state.store(WORK_EMPTY, Ordering::Release);
        work
    }

    fn wake(&self) {
        if let Some(thread) = self.thread.get() {
            thread.unpark();
        }
    }

    // Atomically close the EMPTY claim point. A producer that already owns
    // BUSY is allowed to publish; the worker drains it before this loop can
    // win EMPTY->CLOSED. No stale boolean load can reopen the slot afterward.
    fn close(&self) {
        use std::sync::atomic::Ordering;
        loop {
            match self.state.compare_exchange(
                WORK_EMPTY,
                WORK_CLOSED,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) | Err(WORK_CLOSED) => return,
                Err(WORK_FULL | WORK_BUSY) => {
                    self.wake();
                    std::thread::yield_now();
                }
                Err(state) => unreachable!("invalid resetup work state {state}"),
            }
        }
    }

    // Main thread only, after DataState::resetup_takeover has excluded the
    // ordinary producer. Wait until every command published before the lease
    // has been taken and completely processed.
    fn wait_idle(&self) -> bool {
        use std::sync::atomic::Ordering;
        self.wake();
        let mut active = self
            .active
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        loop {
            if !*active && self.state.load(Ordering::Acquire) == WORK_EMPTY {
                return true;
            }
            if self.stopping.load(Ordering::Acquire) {
                return false;
            }
            active = self
                .idle
                .wait(active)
                .unwrap_or_else(std::sync::PoisonError::into_inner);
        }
    }
}

// MainState's owned worker handle. Drop is deliberately idempotent: init can
// fail after State is written but before init returns, and drop_in_place must
// not detach a thread parked on an Arc that otherwise has no shutdown owner.
struct ResetupWorker<D: Direction> {
    work: std::sync::Arc<ResetupWorkSlot<D>>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl<D: Direction> ResetupWorker<D> {
    fn start() -> std::io::Result<Self> {
        let work = std::sync::Arc::new(ResetupWorkSlot::new());
        let worker_work = work.clone();
        let join = std::thread::Builder::new()
            .name(format!("pw-oss-{}-resetup", D::MEDIA_CLASS))
            .spawn(move || resetup_worker_loop(worker_work))?;
        // OnceLock cannot already be set: this endpoint was just created.
        let _ = work.thread.set(join.thread().clone());
        // Cover the worker parking before the Thread handle was published.
        work.wake();
        Ok(Self {
            work,
            join: Some(join),
        })
    }

    fn endpoint(&self) -> std::sync::Arc<ResetupWorkSlot<D>> {
        self.work.clone()
    }

    fn wait_idle(&self) -> bool {
        self.work.wait_idle()
    }

    fn stop(&mut self) {
        use std::sync::atomic::Ordering;
        let Some(join) = self.join.take() else {
            return;
        };
        self.work.stopping.store(true, Ordering::Release);
        self.work.wake();
        self.work.close();
        self.work.wake();
        // Per-command panics are contained by the loop. A remaining panic is
        // still joined here so no thread can outlive its node.
        let _ = join.join();
    }
}

impl<D: Direction> Drop for ResetupWorker<D> {
    fn drop(&mut self) {
        self.stop();
    }
}

fn resetup_worker_loop<D: Direction>(work: std::sync::Arc<ResetupWorkSlot<D>>) {
    use std::sync::atomic::Ordering;
    loop {
        // Set active under the same mutex takeover waiters use, before
        // taking the slot. They can therefore never observe EMPTY/idle in
        // the taken-but-not-yet-executing gap.
        let command = {
            let mut active = work
                .active
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let command = work.take();
            if command.is_some() {
                *active = true;
            }
            command
        };
        if let Some(command) = command {
            if work.stopping.load(Ordering::Acquire) {
                // Device-bearing commands are destroyed here, on the worker,
                // even during shutdown. Resetup orders are simply cancelled.
                drop(command);
            } else {
                // DepositOnUnwind turns a panicking resetup into Aborted; the
                // outer catch keeps this worker alive for later commands.
                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    run_resetup_work(command);
                }));
            }
            let mut active = work
                .active
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            *active = false;
            work.idle.notify_all();
            continue;
        }
        if work.state.load(Ordering::Acquire) == WORK_CLOSED {
            break;
        }
        if work.stopping.load(Ordering::Acquire) {
            // stop() is waiting to win EMPTY->CLOSED. Yield until a producer
            // publishes or the close becomes visible.
            std::thread::yield_now();
            continue;
        }
        std::thread::park();
    }
}

fn run_resetup_work<D: Direction>(work: ResetupWork<D>) {
    match work {
        ResetupWork::Resetup(request) => resetup_task(request),
        ResetupWork::RetireDevice(device) => drop(device),
        ResetupWork::RetireSwap(swap) => drop(swap),
        #[cfg(test)]
        ResetupWork::Test(test) => test(),
    }
}

// Retry one retained worker command before consuming another completion.
// false tells the cycle to skip: the retained value may own the currently
// retiring device and must remain the next operation observed by the worker.
fn flush_deferred_work<D: Direction>(state: &mut DataState<D>) -> bool {
    let Some(work) = state.deferred_work.take() else {
        return true;
    };
    match state.resetup_work.try_submit(work) {
        WorkSubmission::Submitted => true,
        WorkSubmission::Returned(work) => {
            state.deferred_work = Some(work);
            false
        }
    }
}

// Submit without ever dropping on failure. The single deferred cell is free
// whenever this is called: poll_resetup flushes it before taking a completion,
// and queue_resetup refuses a second order while one is retained.
fn submit_or_defer<D: Direction>(state: &mut DataState<D>, work: ResetupWork<D>) {
    debug_assert!(
        state.deferred_work.is_none(),
        "worker work must preserve its single-producer order"
    );
    if let WorkSubmission::Returned(work) = state.resetup_work.try_submit(work) {
        state.deferred_work = Some(work);
    }
}

/// Queue an owned worker rebuild order for `port_idx`'s device and mark
/// the port pending (cycles skip until poll_resetup consumes the
/// completion). Data loop only. Returns whether an order is now in flight;
/// false = no config, or an earlier worker command still needs submission.
/// Callers must not write resetup_pending themselves.
pub(crate) fn queue_resetup<D: Direction>(state: &mut DataState<D>, port_idx: usize) -> bool {
    if state.resetup_takeover {
        return false;
    }
    if !flush_deferred_work(state) {
        return false;
    }
    let port = &state.ports[port_idx];
    let Some(config) = port.config.clone() else {
        return false; // no negotiated format; nothing to rebuild
    };
    let request = ResetupRequest {
        port_idx,
        generation: port.generation,
        config,
        path: state.dsp_path.clone(),
        // loop-owned (the prime paths read it here), so this data-loop read
        // is the serialization-correct snapshot
        oss_fragment: state.oss_fragment,
        retried: false,
        retire_first: None,
        log: state.log.clone(),
        shared: std::sync::Arc::downgrade(&state.shared),
    };
    submit_or_defer(state, ResetupWork::Resetup(request));
    // The request is either in the worker slot or retained in DataState.
    state.ports[port_idx].resetup_pending = true;
    true
}

// The unwind guard behind every worker resetup path: a task that dies
// without depositing strands resetup_pending forever (nothing but a
// consumed completion clears it while the node runs). Dropped while still
// armed - i.e. during the unwind - it deposits Aborted for the request's
// generation: the next running cycle drops the claim and may re-queue.
struct DepositOnUnwind<D: Direction> {
    shared: std::sync::Arc<NodeShared<D>>,
    port_idx: usize,
    generation: u64,
    armed: bool,
}

impl<D: Direction> DepositOnUnwind<D> {
    // the normal completion: deposit the computed outcome and disarm
    fn complete(mut self, outcome: SwapOutcome<D>) {
        self.armed = false;
        self.shared.deposit(DeviceSwap {
            port_idx: self.port_idx,
            generation: self.generation,
            outcome,
        });
    }
}

impl<D: Direction> Drop for DepositOnUnwind<D> {
    fn drop(&mut self) {
        if self.armed {
            self.shared.deposit(DeviceSwap {
                port_idx: self.port_idx,
                generation: self.generation,
                outcome: SwapOutcome::Aborted,
            });
        }
    }
}

// Runs on the per-node worker with an owned request: opens and configures
// the replacement device off the RT path and deposits the outcome into the
// shared mailbox for poll_resetup. Atomics synchronize endpoint lifetime,
// started changes, and data-loop generation transitions around the
// potentially blocking open.
fn resetup_request_is_current<D: Direction>(shared: &NodeShared<D>, generation: u64) -> bool {
    use std::sync::atomic::Ordering;
    shared.is_alive()
        && shared.started.load(Ordering::Acquire)
        && shared.generation.load(Ordering::Acquire) == generation
}

fn resetup_task<D: Direction>(mut request: ResetupRequest<D>) {
    let Some(shared) = request.shared.upgrade() else {
        // clear() dropped the rendezvous before this task ran
        return;
    };
    if !shared.is_alive() {
        // clear() ran; nobody is left to consume a deposit (a retire_first
        // payload still closes when `request` drops here, on this thread)
        return;
    }
    // armed from here on: even a panicking open/close below deposits
    let guard = DepositOnUnwind {
        shared,
        port_idx: request.port_idx,
        generation: request.generation,
        armed: true,
    };
    // RetireAndRetry: the dying fd must close before the retry opens (an
    // exclusive device would EBUSY otherwise); under the guard, so a
    // panicking close still unclaims the pending flag
    if let Some(old) = request.retire_first.take() {
        drop(old);
    }
    let outcome = if !resetup_request_is_current(&guard.shared, request.generation) {
        // A release, replacement, or Suspend/Pause landed after the queue.
        // Do not reopen, but still deliver a completion so the ordinary
        // generation fence can account for the task.
        SwapOutcome::Aborted
    } else {
        let mut dsp = D::Device::new(&request.path);
        let res = D::try_open_configure(
            &mut dsp,
            &request.config,
            request.oss_fragment,
            &request.log,
        );
        if !resetup_request_is_current(&guard.shared, request.generation) {
            // Clear, Pause, or a concurrent data-loop transition superseded
            // the request during the potentially blocking open/configure.
            // Close the stale fd here, never in the mailbox.
            drop(dsp);
            SwapOutcome::Aborted
        } else if res == 0 {
            SwapOutcome::Installed {
                dsp,
                config: request.config.clone(),
            }
        } else if res == -libc::EBUSY && !request.retried {
            // retire_first is None again here (taken above); poll_resetup
            // fills it with the dying fd for the retry round trip
            SwapOutcome::RetireAndRetry {
                request: ResetupRequest {
                    retried: true,
                    ..request
                },
                // try_open_configure leaves the device closed on failure;
                // reuse it on the RT side instead of constructing there.
                placeholder: dsp,
            }
        } else {
            crate::warn!(
                request.log,
                "{}: background rebuild failed ({}); the port loses its format",
                request.path,
                res
            );
            // As above, failure leaves a ready closed placeholder.
            SwapOutcome::Failed { placeholder: dsp }
        }
    };
    guard.complete(outcome);
}

// Deliver endpoint-only work after process() has ended its State phase. The
// queued closure carries only NodeShared; no State pointer crosses loops. A
// non-blocking invoke may execute inline on a single-loop host, which is why
// callers must collect the event first and call here only after dropping every
// State reference.
fn queue_main_event<D: Direction>(
    main_loop: Option<crate::spa::Loop>,
    shared: std::sync::Arc<NodeShared<D>>,
    log: crate::spa::Log,
    event: MainEvent,
) {
    let Some(main_loop) = main_loop else {
        return;
    };
    // SAFETY: host loops outlive the queued item (queue_task's contract)
    let queued = unsafe {
        crate::utils::queue_task(&main_loop, move || {
            shared.main_event(event);
        })
    };
    if !queued {
        // emission lost: the node stays format-less until the host
        // renegotiates on its own; nothing dangles
        crate::warn!(
            log,
            "can't deliver a deferred node event (main loop unavailable)"
        );
    }
}

// At the start of each data-loop cycle, apply
// a deposited rebuild completion. A matching generation applies it; a stale
// one (superseded by install/release/Suspend) is retired to the worker for
// closing. Returns whether the cycle must skip the port (rebuild still
// in flight, or this cycle consumed a non-install outcome).
pub(crate) fn poll_resetup<D: Direction>(state: &mut DataState<D>, port_idx: usize) -> bool {
    if state.resetup_takeover {
        return true;
    }
    if !flush_deferred_work(state) {
        return true;
    }
    let Some(swap) = state.shared.take_swap() else {
        return state.ports[port_idx].resetup_pending;
    };
    debug_assert_eq!(
        swap.port_idx, port_idx,
        "single mailbox slot: MAX_PORTS == 1"
    );
    if swap.generation != state.ports[port_idx].generation {
        // superseded; the payload may hold an open device - transfer the
        // whole owned message to the blocking-I/O worker.
        submit_or_defer(state, ResetupWork::RetireSwap(swap));
        return state.ports[port_idx].resetup_pending;
    }
    match swap.outcome {
        SwapOutcome::Installed { dsp, config } => {
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, dsp);
            port.config = Some(config);
            port.generation = port.generation.wrapping_add(1);
            state
                .shared
                .generation
                .store(port.generation, std::sync::atomic::Ordering::Release);
            port.resetup_pending = false;
            port.was_matching = false; // force a relock when matching resumes
            D::on_device_swapped(state, port_idx);
            crate::info!(
                state.log,
                "{}: background device rebuild applied",
                state.dsp_path
            );
            submit_or_defer(state, ResetupWork::RetireDevice(old));
            false // the cycle continues on the fresh device (prime re-runs)
        }
        SwapOutcome::Aborted => {
            // stopped when the task ran; drop the claim so the next cycle
            // (running again, or it wouldn't poll) can re-queue
            state.ports[port_idx].resetup_pending = false;
            true
        }
        SwapOutcome::RetireAndRetry {
            mut request,
            placeholder,
        } => {
            // swap the dying fd out behind a closed placeholder so the
            // retry's open can succeed on an exclusive device; it rides the
            // request as retire_first, so close-then-retry runs as one
            // worker command (ordering holds) under the task's unwind guard
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, placeholder);
            request.retire_first = Some(old);
            submit_or_defer(state, ResetupWork::Resetup(request));
            true
        }
        SwapOutcome::Failed { placeholder } => {
            // mirror install_device's failure shape: closed device, cleared
            // config, and a re-announce so the host renegotiates instead of
            // believing a format is set on a dead port
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, placeholder);
            port.config = None;
            port.generation = port.generation.wrapping_add(1);
            state
                .shared
                .generation
                .store(port.generation, std::sync::atomic::Ordering::Release);
            port.resetup_pending = false;
            port.was_matching = false;
            D::on_device_swapped(state, port_idx);
            submit_or_defer(state, ResetupWork::RetireDevice(old));
            // process() extracts and queues this only after its &mut DataState
            // phase ends. The endpoint epoch prevents an old loss from
            // overwriting a newer successful format publication.
            state.pending_main_event = Some(MainEvent::FormatLost {
                expected_publication_epoch: state.events.format_publication_epoch(),
            });
            true
        }
    }
}

// spa_node_callbacks leads with `version: u32` (the SPA vtable convention,
// spa/node/node.h); NodeCallbacks::set's prefix read below depends on it
const _: () = assert!(std::mem::offset_of!(spa_node_callbacks, version) == 0);

// A version-checked copy of the host callback table. Hosts must call
// set_callbacks again to publish changes; in-place table mutations are not
// observed.
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
        main.resetup_worker.stop();
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
    let resetup_worker = match ResetupWorker::<D>::start() {
        Ok(worker) => worker,
        Err(err) => {
            data_system.close(timer_fd);
            crate::error!(log, "can't start the device resetup worker: {}", err);
            return -libc::EIO;
        }
    };
    let resetup_work = resetup_worker.endpoint();
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
                    resetup_worker,
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
                        resetup_pending: false,
                        generation: 0,
                        was_matching: false,
                        warn_limit: crate::utils::RateLimit::new(),
                        pending_xrun: None,
                        ext: std::default::Default::default(),
                    }; MAX_PORTS],
                    oss_fragment,
                    shared,
                    resetup_work,
                    deferred_work: None,
                    resetup_takeover: false,
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
mod tests {
    use super::*;
    use crate::sink::SinkDir;

    struct ReentrantInfoContext {
        events: *const NodeEvents<SinkDir>,
        seen: Vec<u32>,
    }

    unsafe extern "C" fn reentrant_info(data: *mut c_void, info: *const spa_node_info) {
        let context = unsafe { &mut *data.cast::<ReentrantInfoContext>() };
        context.seen.push(unsafe { (*info).max_input_ports });
        if context.seen.len() == 1 {
            let events = unsafe { &*context.events };
            events.with_node_info(|info| info.set_max_input_ports(3));
            events.queue_node_info();
            // SAFETY: this callback holds no State reference; it simulates a
            // reentrant node method flushing its independently owned event.
            unsafe { events.flush() };
        }
    }

    #[test]
    fn node_notifications_preserve_fifo_order_under_reentrant_flush() {
        let events = NodeEvents::<SinkDir>::new();
        events.with_info(|node, port| {
            node.fix_pointers();
            node.add_param(SPA_PARAM_Props, SPA_PARAM_INFO_READ);
            port.fix_pointers();
        });
        let mut context = ReentrantInfoContext {
            events: &events,
            seen: Vec::new(),
        };
        let mut table: spa_node_events = unsafe { std::mem::zeroed() };
        table.version = SPA_VERSION_NODE_EVENTS;
        table.info = Some(reentrant_info);
        let mut hook: spa_hook = unsafe { std::mem::zeroed() };
        let initial = || {
            events.with_node_info(|info| info.set_max_input_ports(1));
            events.queue_node_info();
            events.with_node_info(|info| info.set_max_input_ports(2));
            events.queue_node_info();
            // SAFETY: the test owns no State; only the independent endpoint
            // is live during outer and nested callbacks.
            unsafe { events.flush() };
        };
        unsafe {
            events.hooks.with_isolated_listener(
                &mut hook,
                &raw const table,
                (&raw mut context).cast(),
                initial,
            );
        }
        assert_eq!(context.seen, [1, 2, 3]);
    }

    struct LateNodeListener {
        seen: Vec<u32>,
    }

    unsafe extern "C" fn record_late_node_info(data: *mut c_void, info: *const spa_node_info) {
        unsafe { &mut *data.cast::<LateNodeListener>() }
            .seen
            .push(unsafe { (*info).max_input_ports });
    }

    struct AddNodeListenerContext {
        events: *const NodeEvents<SinkDir>,
        late_hook: *mut spa_hook,
        late_table: *const spa_node_events,
        late_data: *mut c_void,
        seen: Vec<u32>,
    }

    unsafe extern "C" fn add_node_listener_during_dispatch(
        data: *mut c_void,
        info: *const spa_node_info,
    ) {
        let context = unsafe { &mut *data.cast::<AddNodeListenerContext>() };
        context.seen.push(unsafe { (*info).max_input_ports });
        if context.seen.len() != 1 {
            return;
        }
        let events = unsafe { &*context.events };
        events.with_node_info(|info| info.set_max_input_ports(3));
        let initial = |hooks: &crate::spa::ListenerList<spa_node_events>| {
            let (node, _port) = events.initial_snapshots();
            hooks.emit(|f, data| {
                if let Some(info) = f.info {
                    unsafe { info(data, node.raw()) };
                }
            });
        };
        unsafe {
            events.with_new_listener(
                context.late_hook,
                context.late_table,
                context.late_data,
                initial,
            );
        }
        // This notification was created after the activation barrier and must
        // reach the new listener; the already-queued value 2 must not.
        events.with_node_info(|info| info.set_max_input_ports(4));
        events.queue_node_info();
    }

    #[test]
    fn node_listener_added_during_dispatch_starts_at_its_barrier() {
        let events = NodeEvents::<SinkDir>::new();
        events.with_info(|node, port| {
            node.fix_pointers();
            port.fix_pointers();
        });
        let mut late = LateNodeListener { seen: Vec::new() };
        let mut late_table: spa_node_events = unsafe { std::mem::zeroed() };
        late_table.version = SPA_VERSION_NODE_EVENTS;
        late_table.info = Some(record_late_node_info);
        let mut late_hook: spa_hook = unsafe { std::mem::zeroed() };
        let mut context = AddNodeListenerContext {
            events: &events,
            late_hook: &mut late_hook,
            late_table: &late_table,
            late_data: (&raw mut late).cast(),
            seen: Vec::new(),
        };
        let mut table: spa_node_events = unsafe { std::mem::zeroed() };
        table.version = SPA_VERSION_NODE_EVENTS;
        table.info = Some(add_node_listener_during_dispatch);
        let mut hook: spa_hook = unsafe { std::mem::zeroed() };
        unsafe {
            events.with_new_listener(
                &mut hook,
                &raw const table,
                (&raw mut context).cast(),
                |_hooks| {},
            );
        }

        events.with_node_info(|info| info.set_max_input_ports(1));
        events.queue_node_info();
        events.with_node_info(|info| info.set_max_input_ports(2));
        events.queue_node_info();
        unsafe { events.flush() };
        events.with_node_info(|info| info.set_max_input_ports(5));
        events.queue_node_info();
        unsafe { events.flush() };

        assert_eq!(context.seen, [1, 2, 4, 5]);
        assert_eq!(
            late.seen,
            [3, 4, 5],
            "initial state, post-barrier change, then later change"
        );
    }

    struct ReentrantDoneContext {
        events: *const NodeEvents<SinkDir>,
        order: Vec<i32>,
    }

    unsafe extern "C" fn info_queues_done(data: *mut c_void, info: *const spa_node_info) {
        let context = unsafe { &mut *data.cast::<ReentrantDoneContext>() };
        context
            .order
            .push(unsafe { (*info).max_input_ports } as i32);
        if context.order.len() == 1 {
            let events = unsafe { &*context.events };
            // Reentrant sync: it must append behind the already queued second
            // info snapshot, not overtake the active transaction.
            unsafe { events.emit_done(7) };
        }
    }

    unsafe extern "C" fn record_done(
        data: *mut c_void,
        seq: c_int,
        _res: c_int,
        _type: u32,
        _result: *const c_void,
    ) {
        unsafe { &mut *data.cast::<ReentrantDoneContext>() }
            .order
            .push(-seq);
    }

    #[test]
    fn node_done_does_not_overtake_an_active_transaction() {
        let events = NodeEvents::<SinkDir>::new();
        events.with_node_info(|node| node.fix_pointers());
        let mut context = ReentrantDoneContext {
            events: &events,
            order: Vec::new(),
        };
        let mut table: spa_node_events = unsafe { std::mem::zeroed() };
        table.version = SPA_VERSION_NODE_EVENTS;
        table.info = Some(info_queues_done);
        table.result = Some(record_done);
        let mut hook: spa_hook = unsafe { std::mem::zeroed() };
        let initial = || {
            events.with_node_info(|info| info.set_max_input_ports(1));
            events.queue_node_info();
            events.with_node_info(|info| info.set_max_input_ports(2));
            events.queue_node_info();
            unsafe { events.flush() };
        };
        unsafe {
            events.hooks.with_isolated_listener(
                &mut hook,
                &raw const table,
                (&raw mut context).cast(),
                initial,
            );
        }
        assert_eq!(context.order, [1, 2, -7]);
    }

    #[test]
    fn node_dispatch_claim_releases_on_unwind() {
        let events = NodeEvents::<SinkDir>::new();
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _claim = events.begin_dispatch().expect("first claim");
            panic!("injected dispatch panic");
        }));
        assert!(panicked.is_err());
        assert!(
            events.begin_dispatch().is_some(),
            "the unwind guard must release dispatch ownership"
        );
    }

    fn published_port_param_flags(events: &NodeEvents<SinkDir>, id: u32) -> u32 {
        let (_node, port) = events.initial_snapshots();
        let raw = unsafe { &*port.raw() };
        let params = unsafe { std::slice::from_raw_parts(raw.params, raw.n_params as usize) };
        params
            .iter()
            .find(|param| param.id == id)
            .map_or(0, |param| param.flags)
    }

    #[test]
    fn format_loss_epoch_rejects_stale_delivery_but_survives_suspend() {
        let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
        events.with_port_info(|port| {
            port.fix_pointers();
            port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
            port.add_param(SPA_PARAM_Buffers, 0);
            port.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
            port.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
        });
        events.advance_format_publication_epoch();
        let failed_epoch = events.format_publication_epoch();
        let shared = NodeShared::new(std::sync::Arc::downgrade(&events));

        // A newer successful format publication retires the delayed loss.
        events.advance_format_publication_epoch();
        shared.main_event(MainEvent::FormatLost {
            expected_publication_epoch: failed_epoch,
        });
        assert_eq!(
            published_port_param_flags(&events, SPA_PARAM_Format),
            SPA_PARAM_INFO_READWRITE
        );
        assert_eq!(
            published_port_param_flags(&events, SPA_PARAM_Buffers),
            SPA_PARAM_INFO_READ
        );

        // Suspend changes the device generation but not this publication
        // epoch. A current loss must therefore still be applied.
        let current_epoch = events.format_publication_epoch();
        shared.main_event(MainEvent::FormatLost {
            expected_publication_epoch: current_epoch,
        });
        assert_eq!(
            published_port_param_flags(&events, SPA_PARAM_Format),
            SPA_PARAM_INFO_WRITE
        );
        assert_eq!(published_port_param_flags(&events, SPA_PARAM_Buffers), 0);
    }

    #[test]
    fn synchronous_format_loss_retires_a_same_epoch_deferred_loss() {
        let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
        events.with_port_info(|port| {
            port.fix_pointers();
            port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
            port.add_param(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
        });
        events.advance_format_publication_epoch();
        let old_epoch = events.format_publication_epoch();

        // This is the synchronous props-rebuild failure path: it queues the
        // loss snapshot before its caller flushes notifications.
        events.record_current_format_lost();
        let loss_epoch = events.format_publication_epoch();
        assert_eq!(loss_epoch, old_epoch.wrapping_add(1));

        // A data-path loss already queued against the old format must now be
        // inert, rather than toggling the EnumFormat serial a second time.
        let shared = NodeShared::new(std::sync::Arc::downgrade(&events));
        shared.main_event(MainEvent::FormatLost {
            expected_publication_epoch: old_epoch,
        });
        assert_eq!(events.format_publication_epoch(), loss_epoch);
    }

    #[test]
    fn resetup_worker_runs_off_caller_and_survives_a_panicking_job() {
        let mut worker = ResetupWorker::<SinkDir>::start().expect("worker starts");
        let endpoint = worker.endpoint();
        let caller = std::thread::current().id();
        let (started_tx, started_rx) = std::sync::mpsc::channel();
        assert!(matches!(
            endpoint.try_submit(ResetupWork::Test(Box::new(move || {
                started_tx
                    .send(std::thread::current().id())
                    .expect("test receiver lives");
                panic!("injected worker panic");
            }))),
            WorkSubmission::Submitted
        ));
        let first_thread = started_rx.recv().expect("the first job ran");
        assert_ne!(first_thread, caller, "blocking work must not run inline");

        let (next_tx, next_rx) = std::sync::mpsc::channel();
        assert!(matches!(
            endpoint.try_submit(ResetupWork::Test(Box::new(move || {
                next_tx
                    .send(std::thread::current().id())
                    .expect("test receiver lives");
            }))),
            WorkSubmission::Submitted
        ));
        assert_eq!(
            next_rx.recv().expect("the worker survived"),
            first_thread,
            "later work stays on the same owned worker"
        );
        worker.stop();
    }

    #[test]
    fn resetup_worker_shutdown_destroys_queued_ownership_off_caller() {
        struct DropThread(std::sync::mpsc::Sender<std::thread::ThreadId>);
        impl Drop for DropThread {
            fn drop(&mut self) {
                let _ = self.0.send(std::thread::current().id());
            }
        }

        let mut worker = ResetupWorker::<SinkDir>::start().expect("worker starts");
        let caller = std::thread::current().id();
        let (drop_tx, drop_rx) = std::sync::mpsc::channel();
        let probe = DropThread(drop_tx);
        assert!(matches!(
            worker
                .endpoint()
                .try_submit(ResetupWork::Test(Box::new(move || drop(probe)))),
            WorkSubmission::Submitted
        ));
        worker.stop();
        assert_ne!(
            drop_rx.recv().expect("shutdown drained the probe"),
            caller,
            "queued ownership is destroyed before join, on the worker"
        );
    }

    #[test]
    fn resetup_takeover_waits_for_an_inflight_worker_operation() {
        let mut worker = ResetupWorker::<SinkDir>::start().expect("worker starts");
        let endpoint = worker.endpoint();
        let (entered_tx, entered_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        assert!(matches!(
            endpoint.try_submit(ResetupWork::Test(Box::new(move || {
                entered_tx.send(()).expect("test receiver lives");
                release_rx.recv().expect("the fake open is released");
            }))),
            WorkSubmission::Submitted
        ));
        entered_rx.recv().expect("the fake open is in flight");

        let (idle_tx, idle_rx) = std::sync::mpsc::channel();
        let waiter = std::thread::spawn(move || {
            idle_tx
                .send(endpoint.wait_idle())
                .expect("test receiver lives");
        });
        assert!(
            idle_rx
                .recv_timeout(std::time::Duration::from_millis(20))
                .is_err(),
            "takeover must not cross a taken but unfinished worker command"
        );
        release_tx.send(()).expect("release the fake open");
        assert!(
            idle_rx.recv().expect("takeover completed"),
            "a live worker reaches idle"
        );
        waiter.join().expect("takeover waiter stays sound");
        worker.stop();
    }

    #[test]
    fn resetup_wait_idle_covers_a_producer_mid_publication() {
        let mut worker = ResetupWorker::<SinkDir>::start().expect("worker starts");
        let endpoint = worker.endpoint();
        let producer_endpoint = endpoint.clone();
        let (claimed_tx, claimed_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let (ran_tx, ran_rx) = std::sync::mpsc::channel();
        let producer = std::thread::spawn(move || {
            matches!(
                producer_endpoint.try_submit_after_claim(
                    ResetupWork::Test(Box::new(move || {
                        ran_tx.send(()).expect("test receiver lives");
                    })),
                    || {
                        claimed_tx.send(()).expect("test receiver lives");
                        release_rx.recv().expect("publisher is released");
                    },
                ),
                WorkSubmission::Submitted
            )
        });
        claimed_rx.recv().expect("producer owns WORK_BUSY");

        let waiter_endpoint = endpoint.clone();
        let (idle_tx, idle_rx) = std::sync::mpsc::channel();
        let waiter = std::thread::spawn(move || {
            idle_tx
                .send(waiter_endpoint.wait_idle())
                .expect("test receiver lives");
        });
        assert!(
            idle_rx
                .recv_timeout(std::time::Duration::from_millis(20))
                .is_err(),
            "BUSY publication is not idle"
        );
        release_tx.send(()).expect("release the publisher");
        assert!(producer.join().expect("producer stays sound"));
        ran_rx.recv().expect("published command ran");
        assert!(idle_rx.recv().expect("waiter completed"));
        waiter.join().expect("waiter stays sound");
        worker.stop();
    }

    #[test]
    fn resetup_stop_closes_a_concurrently_claimed_submission() {
        struct DropThread(std::sync::mpsc::Sender<std::thread::ThreadId>);
        impl Drop for DropThread {
            fn drop(&mut self) {
                let _ = self.0.send(std::thread::current().id());
            }
        }

        let worker = ResetupWorker::<SinkDir>::start().expect("worker starts");
        let endpoint = worker.endpoint();
        let producer_endpoint = endpoint.clone();
        let (claimed_tx, claimed_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let (drop_tx, drop_rx) = std::sync::mpsc::channel();
        let producer = std::thread::spawn(move || {
            let probe = DropThread(drop_tx);
            matches!(
                producer_endpoint.try_submit_after_claim(
                    ResetupWork::Test(Box::new(move || drop(probe))),
                    || {
                        claimed_tx.send(()).expect("test receiver lives");
                        release_rx.recv().expect("publisher is released");
                    },
                ),
                WorkSubmission::Submitted
            )
        });
        claimed_rx.recv().expect("producer owns WORK_BUSY");

        let stopper = std::thread::spawn(move || {
            let mut worker = worker;
            worker.stop();
        });
        while !endpoint.stopping.load(std::sync::atomic::Ordering::Acquire) {
            std::thread::yield_now();
        }
        assert_eq!(
            endpoint.state.load(std::sync::atomic::Ordering::Acquire),
            WORK_BUSY
        );
        release_tx.send(()).expect("release the publisher");
        assert!(producer.join().expect("producer stays sound"));
        stopper.join().expect("stopper stays sound");
        assert_eq!(
            endpoint.state.load(std::sync::atomic::Ordering::Acquire),
            WORK_CLOSED
        );
        assert_ne!(
            drop_rx.recv().expect("worker drained ownership"),
            std::thread::current().id(),
            "shutdown destruction stays off the caller"
        );
        assert!(matches!(
            endpoint.try_submit(ResetupWork::Test(Box::new(|| {}))),
            WorkSubmission::Returned(ResetupWork::Test(_))
        ));
    }

    #[test]
    fn unseeded_data_loop_gate_never_falls_back_to_first_process_caller() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let unseeded = DataThreadGate {
            thread: AtomicUsize::new(0),
            log: crate::spa::Log::test_null(),
        };
        assert!(!check_loop_identity(&unseeded));
        assert_eq!(
            unseeded.thread.load(Ordering::Acquire),
            usize::MAX,
            "an unseeded gate is permanently disabled"
        );

        let current = unsafe { libc::pthread_self() } as usize;
        let seeded = DataThreadGate {
            thread: AtomicUsize::new(current),
            log: crate::spa::Log::test_null(),
        };
        assert!(check_loop_identity(&seeded));
    }

    #[test]
    fn resetup_work_slot_returns_ownership_when_full_or_stopped() {
        let slot = ResetupWorkSlot::<SinkDir>::new();
        assert!(matches!(
            slot.try_submit(ResetupWork::Test(Box::new(|| {}))),
            WorkSubmission::Submitted
        ));
        let second = ResetupWork::Test(Box::new(|| {}));
        let second = match slot.try_submit(second) {
            WorkSubmission::Submitted => panic!("a full slot must reject the second command"),
            WorkSubmission::Returned(second) => second,
        };
        assert!(matches!(second, ResetupWork::Test(_)));

        let _first = slot.take().expect("drain the first command");
        slot.stopping
            .store(true, std::sync::atomic::Ordering::Release);
        slot.close();
        assert_eq!(
            slot.state.load(std::sync::atomic::Ordering::Acquire),
            WORK_CLOSED
        );
        let stopped = ResetupWork::Test(Box::new(|| {}));
        assert!(
            matches!(
                slot.try_submit(stopped),
                WorkSubmission::Returned(ResetupWork::Test(_))
            ),
            "a stopped endpoint returns ownership"
        );
    }

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

    // Latency requests reset on NULL and accept only the opposite direction.
    #[test]
    fn latency_requests_decode_direction_gated() {
        let dir = |d, v: Option<&libspa::pod::Value>| {
            decode_latency_request(d, v).map(|r| r.info.direction)
        };
        assert_eq!(dir(SPA_DIRECTION_INPUT, None), Ok(SPA_DIRECTION_OUTPUT));
        assert_eq!(dir(SPA_DIRECTION_OUTPUT, None), Ok(SPA_DIRECTION_INPUT));

        let info = crate::utils::latency_info_default(SPA_DIRECTION_OUTPUT);
        let value = crate::utils::parse_back(&crate::utils::build_latency_info(&info));
        assert_eq!(
            dir(SPA_DIRECTION_INPUT, Some(&value)),
            Ok(SPA_DIRECTION_OUTPUT)
        );
        // same-direction info and non-latency pods are rejected
        assert_eq!(dir(SPA_DIRECTION_OUTPUT, Some(&value)), Err(-libc::EINVAL));
        assert_eq!(
            dir(SPA_DIRECTION_INPUT, Some(&libspa::pod::Value::Int(1))),
            Err(-libc::EINVAL)
        );
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

    // Known Props populate the update; adapter-owned, unknown, and invalid
    // values are ignored.
    #[test]
    fn props_update_parses_known_keys_and_drops_the_rest() {
        use crate::utils::pod_prop;
        use libspa::pod::Value;
        let log = crate::spa::Log::test_null();

        let params = Value::Struct(vec![
            Value::String(crate::keys::OSS_DELAY.into()),
            Value::Int(8),
            Value::String(crate::keys::OSS_FRAGMENT.into()),
            Value::Int(4096),
            Value::String("bogus.key".into()),
            Value::Int(1),
        ]);
        let update = parse_props_update(
            vec![
                pod_prop(SPA_PROP_volume, Value::Float(1.0)), // softvol: adapter's
                pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(250_000)),
                pod_prop(SPA_PROP_params, params),
                pod_prop(0x77777, Value::Int(3)), // unknown key: logged, skipped
            ],
            &log,
        );
        assert_eq!(
            update,
            PropsUpdate {
                latency_offset_ns: Some(250_000),
                oss_delay: Some(8),
                oss_fragment: Some(4096),
            }
        );

        // negative values are ignored, an odd-length struct is ignored whole,
        // and a mistyped latency offset stays None
        let update = parse_props_update(
            vec![
                pod_prop(SPA_PROP_latencyOffsetNsec, Value::Int(250_000)),
                pod_prop(
                    SPA_PROP_params,
                    Value::Struct(vec![
                        Value::String(crate::keys::OSS_DELAY.into()),
                        Value::Int(-1),
                    ]),
                ),
            ],
            &log,
        );
        assert_eq!(update, PropsUpdate::default());
        let update = parse_props_update(
            vec![pod_prop(
                SPA_PROP_params,
                Value::Struct(vec![Value::String(crate::keys::OSS_DELAY.into())]),
            )],
            &log,
        );
        assert_eq!(update, PropsUpdate::default());
    }

    // Format decoding accepts readback pods and rejects degenerate values.
    #[test]
    fn decode_format_roundtrips_and_rejects_degenerate_values() {
        let log = crate::spa::Log::test_null();
        let config = PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48000,
            channels: 2,
            positions: vec![SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR],
            flags: 0,
            stride: 4,
        };
        // the builder returns bytes; the C parser needs a pod-aligned buffer
        let aligned = |bytes: &[u8]| {
            let mut buf = vec![0u64; bytes.len().div_ceil(8)];
            unsafe {
                std::ptr::copy_nonoverlapping(
                    bytes.as_ptr(),
                    buf.as_mut_ptr().cast::<u8>(),
                    bytes.len(),
                );
            };
            buf
        };

        let pod = build_port_format_info(&config, SPA_PARAM_Format);
        let buf = aligned(&pod);
        let requested = unsafe { decode_format(buf.as_ptr().cast(), &log) }
            .expect("our own Format pod must decode");
        assert_eq!(requested.raw.format, SPA_AUDIO_FORMAT_S16_LE);
        assert_eq!(requested.raw.rate, 48000);
        assert_eq!(requested.raw.channels, 2);
        assert_eq!(
            &requested.raw.position[..2],
            &[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR]
        );

        // A zero rate is structurally valid but semantically invalid.
        let zero_rate = PortConfig { rate: 0, ..config };
        let pod = build_port_format_info(&zero_rate, SPA_PARAM_Format);
        let buf = aligned(&pod);
        assert_eq!(
            unsafe { decode_format(buf.as_ptr().cast(), &log) }
                .err()
                .expect("rate 0 must be rejected"),
            -libc::EINVAL
        );
    }

    // the completion mailbox: deposits are visible to the consumer, a fresh
    // deposit replaces an unconsumed one (whose payload drops in the
    // depositor's thread), take is one-shot, and discard voids the slot
    #[test]
    fn resetup_mailbox_delivers_replaces_and_discards() {
        let shared: NodeShared<SinkDir> = NodeShared::new(std::sync::Weak::new());
        assert!(shared.take_swap().is_none());

        shared.deposit(DeviceSwap {
            port_idx: 0,
            generation: 3,
            outcome: SwapOutcome::Aborted,
        });
        shared.deposit(DeviceSwap {
            port_idx: 0,
            generation: 4,
            outcome: SwapOutcome::Failed {
                placeholder: crate::sound::DspWriter::new("/nonexistent/dsp"),
            },
        });
        let swap = shared.take_swap().expect("a deposited swap");
        assert_eq!(swap.generation, 4, "the newer deposit wins");
        assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));
        assert!(shared.take_swap().is_none(), "take is one-shot");

        shared.deposit(DeviceSwap {
            port_idx: 0,
            generation: 5,
            outcome: SwapOutcome::Aborted,
        });
        shared.discard_swap();
        assert!(shared.take_swap().is_none(), "discard voids the slot");
    }

    // The worker never accesses State: stopped requests abort before opening,
    // failed opens preserve the request generation, and cleared nodes receive
    // no completion.
    #[test]
    fn resetup_task_deposits_and_respects_the_gates() {
        use std::sync::atomic::Ordering;
        let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
        let shared = std::sync::Arc::new(NodeShared::new(std::sync::Arc::downgrade(&events)));
        shared.generation.store(7, Ordering::Release);
        let request = |shared: &std::sync::Arc<NodeShared<SinkDir>>| ResetupRequest {
            port_idx: 0,
            generation: 7,
            config: PortConfig {
                format: libspa::param::audio::AudioFormat::S16LE,
                rate: 48000,
                channels: 2,
                positions: vec![],
                flags: 0,
                stride: 4,
            },
            path: "/nonexistent/dsp".into(),
            oss_fragment: 0,
            retried: false,
            retire_first: None,
            log: crate::spa::Log::test_null(),
            shared: std::sync::Arc::downgrade(shared),
        };

        // not started: aborted without touching the device
        resetup_task(request(&shared));
        let swap = shared.take_swap().expect("a deposit even when stopped");
        assert_eq!(swap.generation, 7);
        assert!(matches!(swap.outcome, SwapOutcome::Aborted));

        // started, open fails (no such device): Failed, generation echoed
        shared.started.store(true, Ordering::Release);
        resetup_task(request(&shared));
        let swap = shared.take_swap().expect("a deposit on failure");
        assert_eq!(swap.generation, 7);
        assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));

        // superseded while queued: abort before opening or publishing a
        // device for the stale generation
        shared.generation.store(8, Ordering::Release);
        resetup_task(request(&shared));
        let swap = shared.take_swap().expect("a stale task still completes");
        assert_eq!(swap.generation, 7);
        assert!(matches!(swap.outcome, SwapOutcome::Aborted));

        // cleared node (the endpoint is gone): no deposit at all
        drop(events);
        resetup_task(request(&shared));
        assert!(shared.take_swap().is_none());
    }

    #[test]
    fn resetup_gate_rechecks_started_and_generation_after_blocking_work() {
        use std::sync::atomic::Ordering;

        let events = std::sync::Arc::new(NodeEvents::<SinkDir>::new());
        let shared = std::sync::Arc::new(NodeShared::new(std::sync::Arc::downgrade(&events)));
        shared.started.store(true, Ordering::Release);
        shared.generation.store(7, Ordering::Release);
        let worker_shared = shared.clone();
        let (entered_tx, entered_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let worker = std::thread::spawn(move || {
            assert!(resetup_request_is_current(&worker_shared, 7));
            entered_tx.send(()).expect("test receiver lives");
            release_rx.recv().expect("mock open is released");
            resetup_request_is_current(&worker_shared, 7)
        });
        entered_rx.recv().expect("mock open passed its first gate");

        // Model a data-loop replacement and a concurrent Pause while the
        // blocking open is in progress. Either change must reject its result.
        shared.generation.store(8, Ordering::Release);
        shared.started.store(false, Ordering::Release);
        release_tx.send(()).expect("release the mock open");
        assert!(
            !worker.join().expect("gate worker stays sound"),
            "the post-open gate must reject superseded work"
        );
    }

    // the unwind guard: a panicking task body still deposits Aborted for
    // its generation (queue_task swallows the panic, and without the
    // deposit the port's pending claim would be stranded forever); a
    // completed task's guard deposits nothing extra
    #[test]
    fn a_panicking_resetup_path_still_deposits_aborted() {
        let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new(std::sync::Weak::new()));

        let guard = DepositOnUnwind {
            shared: shared.clone(),
            port_idx: 0,
            generation: 9,
            armed: true,
        };
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
            let _guard = guard; // dropped mid-unwind, like a panicking open
            panic!("injected task panic");
        }));
        assert!(panicked.is_err());
        let swap = shared
            .take_swap()
            .expect("the guard deposited during unwind");
        assert_eq!(swap.generation, 9, "the stale-rebuild fence stays sane");
        assert!(matches!(swap.outcome, SwapOutcome::Aborted));

        // the normal path deposits its outcome exactly once
        let guard = DepositOnUnwind {
            shared: shared.clone(),
            port_idx: 0,
            generation: 10,
            armed: true,
        };
        guard.complete(SwapOutcome::Failed {
            placeholder: crate::sound::DspWriter::new("/nonexistent/dsp"),
        });
        let swap = shared.take_swap().expect("the completed outcome");
        assert_eq!(swap.generation, 10);
        assert!(matches!(swap.outcome, SwapOutcome::Failed { .. }));
        assert!(shared.take_swap().is_none(), "no second deposit");
    }

    // The slot protocol under three-way contention: worker deposits,
    // data-loop takes, and main-loop discard/replacement may all overlap.
    // The reader never waits, and observed writer generations never regress.
    #[test]
    fn resetup_mailbox_is_safe_under_contention() {
        let shared = std::sync::Arc::new(NodeShared::<SinkDir>::new(std::sync::Weak::new()));
        let writer = {
            let shared = shared.clone();
            std::thread::spawn(move || {
                for generation in 0..10_000u64 {
                    shared.deposit(DeviceSwap {
                        port_idx: 0,
                        generation,
                        outcome: SwapOutcome::Aborted,
                    });
                }
            })
        };
        let discarder = {
            let shared = shared.clone();
            std::thread::spawn(move || {
                for _ in 0..10_000 {
                    shared.discard_swap();
                }
            })
        };
        let mut last = None;
        loop {
            let done = writer.is_finished() && discarder.is_finished();
            if let Some(swap) = shared.take_swap() {
                if let Some(prev) = last {
                    assert!(swap.generation > prev, "a replaced deposit reappeared");
                }
                last = Some(swap.generation);
            } else if done {
                break;
            }
        }
        writer.join().expect("the writer must not panic");
        discarder.join().expect("the discarder must not panic");

        // A post-contention sentinel proves no actor left BUSY behind and the
        // final value still transfers exactly once.
        shared.deposit(DeviceSwap {
            port_idx: 0,
            generation: 10_000,
            outcome: SwapOutcome::Aborted,
        });
        assert_eq!(shared.take_swap().map(|swap| swap.generation), Some(10_000));
        assert!(shared.take_swap().is_none());
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
            generation: 0,
            was_matching: false,
            warn_limit: crate::utils::RateLimit::new(),
            pending_xrun: None,
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

        // Happy path: the descriptor carries the validated pointers by value
        // and the accessors stay inside the block. The chunk says 32 bytes at
        // offset 16, so input_slice views exactly that window; output_slice
        // spans the whole block.
        chunk.offset = 16;
        chunk.size = 32;
        chunk.stride = 8;
        let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
        port.buffers = vec![&mut buffer];
        let mut block = unsafe { valid_data_block(&port, 0, &log) }.expect("a usable MemPtr block");
        assert!(std::ptr::eq(
            block.data_ptr().cast::<u8>(),
            payload.as_ptr()
        ));
        assert_eq!(block.chunk_stride(), 8);
        assert!(std::ptr::eq(
            block.input_slice().as_ptr(),
            payload[16..].as_ptr()
        ));
        assert_eq!(block.input_slice().len(), 32);
        assert_eq!(block.output_slice().len(), payload.len());

        // a peer offset past the block wraps and the size clamps to what
        // remains (the input clamp the sink write path depends on)
        block.publish(60, 8);
        let mut block = unsafe { valid_data_block(&port, 0, &log) }.expect("a usable MemPtr block");
        // publish rewrote the chunk: 60 bytes at offset 0
        assert_eq!(block.input_slice().len(), 60);
        // and re-reading through a chunk pointing past the end stays bounded
        block.output_slice()[0] = 0xaa;
        let (mut buffer, _data) = fixture(&mut payload, &mut chunk);
        chunk.offset = 60;
        chunk.size = 32;
        port.buffers = vec![&mut buffer];
        let block = unsafe { valid_data_block(&port, 0, &log) }.expect("a usable MemPtr block");
        assert_eq!(block.input_slice().len(), 4);
        assert!(std::ptr::eq(
            block.input_slice().as_ptr(),
            payload[60..].as_ptr()
        ));

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
