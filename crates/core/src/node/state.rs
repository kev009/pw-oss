use std::mem::offset_of;

use super::{BwAdapt, RateLimit, SpaDLL, *};
use crate::backend;
use crate::backend::StreamLifecycle as _;
use crate::spa::{self, IoArea, Log, Loop, LoopSource, System, TimerFd, block_on_loop};

#[repr(C)]
// The pinned FFI shell. Runtime entry points project only one disjoint field
// from its raw pointer; they never create a reference to this whole object.
// `handle` stays first because the host casts spa_handle* back to State*.
pub(crate) struct State<D: Direction> {
    pub handle: spa_handle,
    pub node: spa_node,
    // Checked through its own atomic before process() projects `data`.
    pub(super) gate: DataThreadGate,
    pub(super) main: MainState<D>,
    pub(super) data: DataState<D>,
}

// Raw State pointers enter through the SPA interface. These helpers project
// one field at a time so callers cannot accidentally borrow the whole shell.
// A MainState borrow may coexist with DataControl; a DataState borrow may not.
pub(super) unsafe fn main_ref<'a, D: Direction>(state: *const State<D>) -> &'a MainState<D> {
    unsafe { (&raw const (*state).main).as_ref_unchecked() }
}

pub(super) unsafe fn main_mut<'a, D: Direction>(state: *mut State<D>) -> &'a mut MainState<D> {
    unsafe { (&raw mut (*state).main).as_mut_unchecked() }
}

// Keep direct data-loop borrows inside a lexical callback. Unlike a helper
// returning &mut with a caller-chosen lifetime, this cannot leak the borrow
// into a later DataControl handoff.
pub(super) unsafe fn with_data_mut<D: Direction, R>(
    state: *mut State<D>,
    apply: impl for<'a> FnOnce(&'a mut DataState<D>) -> R,
) -> R {
    let data = unsafe { (&raw mut (*state).data).as_mut_unchecked() };
    apply(data)
}

pub(super) unsafe fn gate_ref<'a, D: Direction>(state: *const State<D>) -> &'a DataThreadGate {
    unsafe { (&raw const (*state).gate).as_ref_unchecked() }
}

pub(super) unsafe fn main_ptr<D: Direction>(state: *mut State<D>) -> *mut MainState<D> {
    unsafe { &raw mut (*state).main }
}

pub(super) struct DataThreadGate {
    pub(super) thread: std::sync::atomic::AtomicUsize,
    pub(super) log: Log,
}

pub(crate) struct MainState<D: Direction> {
    pub(super) events: std::rc::Rc<NodeEvents<D>>,
    // A copyable host-loop endpoint plus the stable address of State::data are
    // combined into DataControl at each control entry point.
    pub data_loop: Loop,
    pub log: Log,
    pub stream_path: String,
    pub caps: backend::StreamCaps,
    pub caps_fallback: bool,
    pub backend_properties: BackendPropertiesOf<D>,
    pub latency: [spa_latency_info; 2],
    pub process_latency: spa_process_latency_info,
    pub shared: std::sync::Arc<NodeShared<D>>,
    // Owns the only thread that may execute an asynchronous device
    // open/configure/close. DataState holds only its bounded submission
    // endpoint; clear stops and joins this worker before State is dropped.
    pub(super) rebuild_worker: RebuildWorker<D>,
    pub ring_cap_published: bool,
}

pub(crate) struct DataState<D: Direction> {
    pub data_loop: Loop,
    pub data_system: System,
    pub log: Log,
    pub clock: IoArea<spa_io_clock>,
    pub position: IoArea<spa_io_position>,
    pub clock_name: std::ffi::CString, // stamped into spa_io_clock.name
    pub main_loop: Option<Loop>,       // for endpoint-only notifications
    pub stream_path: String,
    // Exactly one notification descriptor owns wake_source.fd: the portable
    // SPA timer or the selected backend wake driver.
    pub(super) timer_fd: Option<TimerFd>,
    pub(super) wake_driver: Option<WakeDriverOf<D>>,
    // Native wake registration or threshold setup failed for this stream
    // generation. An explicit driver update gives it one bounded retry without
    // repeating the warning until one succeeds.
    pub(super) wake_failed_stream: Option<backend::StreamIdentity>,
    pub(super) wake_source: LoopSource,
    // True only while on_wake is inside the host's ready callback. An inline
    // process() leaves next-wake selection to the callback epilogue; a
    // deferred process() performs it itself.
    pub(super) ready_dispatching: bool,
    pub next_time: u64,
    pub callbacks: NodeCallbacks,
    pub ports: [Port<D>; MAX_PORTS],
    pub backend_properties: BackendPropertiesOf<D>,
    // the Arc'd rendezvous with the owned rebuild worker and
    // clear(); outlives the FFI shell by construction (see NodeShared)
    pub shared: std::sync::Arc<NodeShared<D>>,
    // The data loop is the sole producer. A device-bearing command that
    // finds the worker slot occupied stays here and is retried before any
    // further completion is consumed; it is never dropped on the RT path.
    pub(super) rebuild_work: std::sync::Arc<RebuildWorkSlot<D>>,
    pub(super) deferred_work: Option<RebuildWork<D>>,
    // Main-thread synchronous installs take this data-loop lease before
    // waiting for the worker. While set, process neither consumes a
    // completion nor submits new work.
    pub(super) rebuild_takeover: bool,
    pub(super) format_publication: FormatPublication,
    pub(super) main_events: MainEventTarget<D>,
    // Data-loop-owned: process_ports records endpoint work here, and generic
    // process() extracts it before ending its DataState phase. Delivery happens
    // only afterward, so an inline loop invoke cannot overlap the data borrow.
    pub(super) pending_main_event: Option<MainEvent>,
    pub started: bool,
    pub following: bool,
    pub ext: D::DataExt, // direction-specific fields (see sink/source)
}

impl<D: Direction> DataState<D> {
    pub(super) fn node_is_follower(&self) -> bool {
        let driver = self.position.with_ref(|p| p.clock.id);
        let ours = self.clock.with_ref(|c| c.id);
        matches!((driver, ours), (Some(d), Some(o)) if d != o)
    }
}

// A short-lived, non-Copy main-loop capability for synchronously borrowing
// the disjoint data-loop state. The host serializes control methods on the
// main loop; callers must not retain this past State teardown.
pub(crate) struct DataControl<D: Direction> {
    loop_: Loop,
    data: *mut DataState<D>,
}

impl<D: Direction> DataControl<D> {
    pub(super) unsafe fn from_raw(state: *mut State<D>) -> Self {
        Self {
            loop_: unsafe { main_ref(state).data_loop },
            data: unsafe { &raw mut (*state).data },
        }
    }

    pub(super) fn invoke(&self, f: impl FnOnce(&mut DataState<D>) + Send) -> bool {
        unsafe { block_on_loop(&self.loop_, self.data, f) }
    }

    pub(super) fn query<R: Send>(
        &self,
        f: impl FnOnce(&mut DataState<D>) -> R + Send,
    ) -> Option<R> {
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
    pub io: IoArea<spa_io_buffers>,
    pub rate_match: IoArea<spa_io_rate_match>, // per-port io area (port_set_io)
    pub dsp: D::Device,
    pub dll: SpaDLL,
    pub setup_period: u32, // device bytes per graph cycle the stream/servo was set up for
    pub bw_adapt: BwAdapt, // variance-adaptive servo bandwidth
    pub delivery_quantum_bytes: u32, // applied delivery granularity
    // A main-loop device rebuild is in flight; skip cycles until poll_rebuild
    // consumes its completion. Data-loop-owned: set when the order is queued,
    // cleared when the completion swap is consumed (or by the install/suspend
    // swap closures, which also run on this loop) - no other thread touches it.
    pub rebuild_pending: bool,
    // Data-loop-owned rebuild fence. Increment it whenever the port's device
    // or configuration changes. A completion applies only when its snapshot
    // still matches; wrapping is safe because the fence uses equality only.
    pub generation: u64,
    // Stable for the lifetime of this logical port; native descriptor changes
    // are fenced by pairing it with `generation` in every wake registration.
    pub stream_token: backend::StreamToken,
    pub was_matching: bool, // rate matching active last cycle (relock on transition)
    pub warn_limit: RateLimit,
    // Data-loop-owned xrun detected this cycle (trigger time in
    // µs). detect_underrun/recover_overrun deposit it instead of calling the
    // host back mid-cycle; process() drains it and invokes the copied xrun
    // hook only after the DataState/port borrows end (collect-then-notify).
    pub pending_xrun: Option<PendingXrun>,
    // Latest backend wake snapshot, valid until this cycle's device I/O.
    // Timer-driven/follower paths leave it empty and query the stream.
    pub stream_wake: Option<backend::StreamWake>,
    // Disconnect is ownership state, not a cycle measurement. Keep it latched
    // across timer wakes and failed rebuild submissions until the stream or
    // measurement epoch is reset.
    pub rebuild_required: bool,
    pub xrun_tracker: backend::XrunTracker,
    pub ext: D::PortExt, // direction-specific fields (see sink/source)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PendingXrun {
    pub(crate) trigger_us: u64,
    pub(crate) delay_us: u64,
    pub(crate) quality: Option<backend::ObservationQuality>,
}

pub(crate) fn pending_xrun(
    trigger_us: u64,
    delta: backend::XrunDelta,
    config: Option<&PortConfig>,
) -> PendingXrun {
    let delay_us = config.map_or(0, |config| {
        let rate = u128::from(config.rate.max(1));
        if let Some(frames) = delta.lost_frames {
            (u128::from(frames).saturating_mul(1_000_000) / rate).min(u128::from(u64::MAX)) as u64
        } else if let Some(bytes) = delta.lost_bytes {
            (u128::from(bytes).saturating_mul(1_000_000)
                / (rate * u128::from(config.stride.max(1))))
            .min(u128::from(u64::MAX)) as u64
        } else {
            0
        }
    });
    PendingXrun {
        trigger_us,
        delay_us,
        quality: delta.quality,
    }
}

impl<D: Direction> Port<D> {
    pub(crate) const fn stream_identity(&self) -> backend::StreamIdentity {
        backend::StreamIdentity::new(self.stream_token, self.generation)
    }
}

pub(crate) fn wake_queue_fill<D: Direction>(port: &Port<D>) -> Option<u32> {
    let event = port.stream_wake?;
    if event.stream != port.stream_identity() {
        return None;
    }
    event
        .queue
        .map(|queue| queue.fill_bytes.min(u64::from(u32::MAX)) as u32)
}

pub(crate) fn take_wake_xruns<D: Direction>(port: &mut Port<D>) -> Option<backend::XrunDelta> {
    let event = port.stream_wake?;
    if event.stream != port.stream_identity() {
        return None;
    }
    event
        .xruns
        .map(|observation| port.xrun_tracker.observe(observation))
}

pub(crate) fn take_polled_xruns<D: Direction>(
    port: &mut Port<D>,
    observation: backend::XrunObservation,
) -> backend::XrunDelta {
    port.xrun_tracker.observe(observation)
}

pub(crate) fn reset_stream_epoch<D: Direction>(port: &mut Port<D>) {
    port.stream_wake = None;
    port.rebuild_required = false;
    port.xrun_tracker.reset();
}

pub(crate) fn latch_rebuild_required<D: Direction>(port: &mut Port<D>, status: backend::IoStatus) {
    if status.requires_rebuild() {
        port.rebuild_required = true;
    }
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
    log: &Log,
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

// A version-checked copy of the host callback table. Hosts must call
// set_callbacks again to publish changes; in-place table mutations are not
// observed.
pub(crate) struct NodeCallbacks {
    // None means no compatible table is set. The host data pointer accompanies
    // every callback.
    cb: Option<(spa_node_callbacks, *mut c_void)>,
}

// spa_node_callbacks leads with `version: u32` (the SPA vtable convention,
// spa/node/node.h); NodeCallbacks::set's prefix read below depends on it.
const _: () = assert!(offset_of!(spa_node_callbacks, version) == 0);

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
        if !spa::version_ok(version, SPA_VERSION_NODE_CALLBACKS) {
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

#[cfg(test)]
mod tests {
    use super::super::sink::SinkDir as GenericSinkDir;
    use super::*;
    use crate::backend::fake::FakeBackend;

    type SinkDir = GenericSinkDir<FakeBackend>;

    #[test]
    fn xrun_report_preserves_duration_and_quality() {
        let config = PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48_000,
            channels: 4,
            positions: vec![],
            flags: 0,
            stride: 8,
        };
        assert_eq!(
            pending_xrun(
                123,
                backend::XrunDelta {
                    events: 1,
                    lost_frames: Some(480),
                    lost_bytes: None,
                    quality: Some(backend::ObservationQuality::Estimated),
                },
                Some(&config),
            ),
            PendingXrun {
                trigger_us: 123,
                delay_us: 10_000,
                quality: Some(backend::ObservationQuality::Estimated),
            }
        );
        assert_eq!(
            pending_xrun(
                456,
                backend::XrunDelta {
                    events: 1,
                    lost_frames: None,
                    lost_bytes: Some(3_840),
                    quality: Some(backend::ObservationQuality::Exact),
                },
                Some(&config),
            )
            .delay_us,
            10_000
        );
    }

    fn test_port(fd: c_int) -> Port<SinkDir> {
        Port {
            config: None,
            buffers: vec![],
            io: IoArea::null(),
            rate_match: IoArea::null(),
            dsp: backend::fake::FakeStream::test_on_fd(fd, 8),
            dll: Default::default(),
            setup_period: 0,
            bw_adapt: Default::default(),
            delivery_quantum_bytes: 0,
            rebuild_pending: false,
            generation: 0,
            stream_token: backend::StreamToken::for_port(0),
            was_matching: false,
            warn_limit: RateLimit::new(),
            pending_xrun: None,
            stream_wake: None,
            rebuild_required: false,
            xrun_tracker: backend::XrunTracker::default(),
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
        let (r, w) = backend::test_transport::pipe_pair(true, true);
        let mut port = test_port(w);
        let log = Log::test_null();
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
