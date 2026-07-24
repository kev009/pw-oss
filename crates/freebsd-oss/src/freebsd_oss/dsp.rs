use libc::ssize_t;
use nix::errno::Errno;
use std::cell::Cell;
use std::ffi::{CString, c_int};
use std::time::Duration;

use super::abi::*;
use super::buffer::{
    MAX_BUFFER_BYTES, MIN_BUFFER_BYTES, capture_applied_geometry, capture_buffer_plan,
    playback_applied_geometry, playback_buffer_plan, playback_retuned_geometry,
};
use super::devices::delivery_quantum;
use super::event::OssWakeDriver;
use super::identity::OssNodeProperties;
use super::sys::LibcFd;
use crate::backend::{
    BufferLayout, CaptureBufferGeometry, CaptureBufferRequest, CaptureRetune, DeliveryQuantum,
    IoStatus, PauseOutcome, PlaybackBufferGeometry, PlaybackBufferRequest, PlaybackRetune,
    ReadOutcome, SilencePattern, StreamError, StreamIdentity, WakeBufferState, WakeError,
    WriteOutcome, XrunObservation,
};
use crate::spa::Log;
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct AppliedNativeConfig {
    pub(super) format: u32,
    pub(super) channels: u32,
    pub(super) rate: u32,
}

#[derive(Default)]
struct CaptureBufferState {
    period_bytes: u32,
    quantum_bytes: u32,
    capacity_bytes: u32,
    mismatch_cycles: u32,
    pinned_cycles: u32,
}

#[derive(Default)]
struct PlaybackBufferState {
    period_bytes: u32,
    quantum_bytes: u32,
    capacity_bytes: u32,
    mismatch_cycles: u32,
    last_retune_log_ns: u64,
    suppressed_retune_logs: u32,
}

fn io_status(error: Errno) -> IoStatus {
    if error == Errno::EAGAIN || error == Errno::EWOULDBLOCK || error == Errno::EINTR {
        IoStatus::WouldBlock
    } else {
        match error {
            Errno::EBADF | Errno::ENODEV | Errno::ENXIO | Errno::EPIPE => IoStatus::Disconnected,
            // OSS commonly reports EIO for a dying channel whose descriptor
            // still exists. Treat every remaining non-retryable errno as
            // fatal so the shared shell replaces the stream instead of
            // repeatedly dropping graph buffers on the same descriptor.
            _ => IoStatus::Fatal(StreamError::from_native_code(error as i32)),
        }
    }
}

fn native_frame_stride(format: u32, channels: u32) -> u32 {
    afmt_frame_bytes(format)
        .max(1)
        .saturating_mul(channels.max(1))
}

fn wake_threshold_changed(current: u32, desired: u32, quantum: u32) -> bool {
    current == 0 || current.abs_diff(desired) >= quantum.max(1)
}

fn capture_wake_threshold(buffer: WakeBufferState) -> u32 {
    buffer.target_fill_bytes.max(buffer.period_bytes).max(1)
}

fn playback_wake_threshold(buffer: WakeBufferState) -> u32 {
    buffer
        .capacity_bytes
        .saturating_sub(buffer.target_fill_bytes)
        .max(1)
}

fn install_wake_threshold(
    fd: c_int,
    current: &Cell<u32>,
    desired: u32,
    quantum: u32,
) -> Result<(), WakeError> {
    if !wake_threshold_changed(current.get(), desired, quantum) {
        return Ok(());
    }
    if !set_low_water(fd, desired) {
        return Err(WakeError::threshold(desired, Errno::last()));
    }
    current.set(desired);
    Ok(())
}

pub(crate) struct Dsp {
    path: CString,
    delivery_quantum: DeliveryQuantum,
    fd: Option<LibcFd>,
    state: DspState,
    needs_trigger: bool, // trigger-suspended: NOTRIGGER must be cleared on restart
    hw_caps: u32,        // best-effort per-open SNDCTL_DSP_GETCAPS snapshot
    stride: u32,         // negotiated frame bytes; reads must consume whole frames
    skip: u32,           // tail bytes of a torn frame to discard before the next read
    wake_threshold: Cell<u32>,
    buffer: CaptureBufferState,
}

impl Dsp {
    pub(crate) fn new(path: &str) -> Self {
        Self {
            path: CString::new(path).unwrap(),
            delivery_quantum: delivery_quantum(path, false),
            fd: None,
            state: DspState::Closed,
            needs_trigger: false,
            hw_caps: 0,
            stride: 1,
            skip: 0,
            wake_threshold: Cell::new(0),
            buffer: CaptureBufferState::default(),
        }
    }

    pub(crate) fn is_closed(&self) -> bool {
        self.state == DspState::Closed
    }

    pub(crate) fn path(&self) -> &str {
        self.path.to_str().unwrap_or("") // constructed from &str; always valid
    }

    pub(crate) fn delivery_quantum(&self) -> DeliveryQuantum {
        self.delivery_quantum
    }

    // on direct opens the hardware blocksize is per-session state; call after
    // Configure first so the cadence snapshot reflects this session.
    pub(crate) fn refresh_delivery_quantum(&mut self) {
        if let Ok(path) = self.path.to_str() {
            self.delivery_quantum = delivery_quantum(path, false);
        }
    }

    pub(crate) fn is_running(&self) -> bool {
        self.state == DspState::Running
    }

    pub(crate) fn hw_caps(&self) -> u32 {
        self.hw_caps
    }

    pub(crate) fn is_virtual_channel(&self) -> bool {
        self.hw_caps & PCM_CAP_VIRTUAL as u32 != 0
    }

    fn descriptor(&self) -> &LibcFd {
        self.fd
            .as_ref()
            .expect("an active DSP state owns its descriptor")
    }

    fn raw_fd(&self) -> c_int {
        self.descriptor().raw()
    }

    pub(crate) fn register_wake(
        &self,
        driver: &mut OssWakeDriver,
        stream: StreamIdentity,
        buffer: WakeBufferState,
    ) -> Result<(), WakeError> {
        let fd = self
            .fd
            .as_ref()
            .ok_or_else(|| WakeError::new(Errno::ENODEV))?;
        let threshold = capture_wake_threshold(buffer);
        install_wake_threshold(
            fd.raw(),
            &self.wake_threshold,
            threshold,
            buffer.quantum_bytes,
        )?;
        driver.register_stream(fd.raw(), false, stream, buffer.frame_stride)
    }

    pub(crate) fn open(&mut self) -> Result<(), Errno> {
        assert_eq!(self.state, DspState::Closed);

        // O_RDONLY, not O_RDWR: on devices with asymmetric play/rec channel
        // counts (e.g. RODECaster) the kernel won't take per-direction counts on
        // one fd (shkhln/pw-oss#3)
        // This descriptor is consumed on the graph/data loop. GETISPACE
        // normally bounds each read to queued bytes, but a signal-shortened
        // frame leaves a realignment tail which is consumed before the next
        // bounded read. CHN_F_NBIO is the hard guarantee that neither phase
        // can sleep in chn_read.
        let fd =
            LibcFd::open(&self.path, libc::O_RDONLY | libc::O_NONBLOCK).ok_or_else(Errno::last)?;

        self.fd = Some(fd);
        self.state = DspState::Setup;

        Ok(())
    }

    pub(crate) fn close(&mut self) {
        assert_ne!(self.state, DspState::Closed);
        drop(
            self.fd
                .take()
                .expect("an active DSP state owns its descriptor"),
        );
        self.state = DspState::Closed;
        self.needs_trigger = false;
        self.hw_caps = 0;
        self.skip = 0;
        self.buffer = CaptureBufferState::default();
        self.wake_threshold.set(0);
    }

    #[cfg(test)]
    pub(crate) fn is_nonblocking(&self) -> bool {
        let flags = unsafe { libc::fcntl(self.raw_fd(), libc::F_GETFL) };
        flags >= 0 && flags & libc::O_NONBLOCK != 0
    }

    pub(crate) fn clear_overrun_observation(&mut self) {
        self.buffer.pinned_cycles = 0;
    }

    /// Classify and recover a FreeBSD capture overrun. chn_rdfeed disposes a
    /// hardware lump upstream when bufsoft is full, so a counter tick alone
    /// does not corrupt the queued data. Re-prime only when the soft ring stays
    /// pinned across consecutive cycles: that is evidence the catch-up read
    /// cannot drain it. This avoids turning a short kernel drop into a much
    /// longer backlog discard, silence period, and servo relock.
    ///
    /// `Some(reset_epoch)` means recovery was selected; the boolean reports
    /// whether trigger suspension established a fresh native event epoch.
    pub(crate) fn recover_overrun(
        &mut self,
        overrun_count: u32,
        pre_read_fill: Option<u32>,
        log: &Log,
    ) -> Option<bool> {
        const PINNED_CYCLE_LIMIT: u32 = 3;

        let pinned = match (pre_read_fill, self.buffer.capacity_bytes) {
            (Some(fill), capacity) if capacity > 0 => {
                fill > capacity.saturating_sub(self.buffer.quantum_bytes)
            }
            (Some(_), _) => true,
            (None, _) => false,
        };
        self.buffer.pinned_cycles = if pinned {
            self.buffer.pinned_cycles.saturating_add(1)
        } else {
            0
        };
        if self.buffer.pinned_cycles < PINNED_CYCLE_LIMIT {
            crate::debug!(
                log,
                "{} overrun counts ignored (kernel disposed upstream; fill {:?} of ring {})",
                overrun_count,
                pre_read_fill,
                self.buffer.capacity_bytes
            );
            return None;
        }

        self.buffer.pinned_cycles = 0;
        Some(self.suspend())
    }

    pub(crate) fn log_overrun_recovery(
        &self,
        overrun_count: u32,
        now: u64,
        suppressed: u32,
        log: &Log,
    ) {
        crate::warn!(
            log,
            "OSS reported {:3} overruns @ {} with the ring pinned; re-priming (+{} warnings suppressed)",
            overrun_count,
            now,
            suppressed
        );
    }

    pub(super) fn configure(
        &mut self,
        format: u32,
        channels: u32,
        rate: u32,
        channel_order: Option<u64>,
    ) -> Result<AppliedNativeConfig, Errno> {
        assert_eq!(self.state, DspState::Setup);
        let format = set_format(self.raw_fd(), format)?;
        let channels = set_channels(self.raw_fd(), channels)?;
        // Derive frame alignment from the successful native readback. The
        // current FreeBSD selectors reject a changed format/count, but keeping
        // the stream state tied to the grant makes that invariant explicit.
        self.stride = native_frame_stride(format, channels);
        if let Some(order) = channel_order {
            set_channel_order(self.raw_fd(), order)?;
        }
        let rate = set_rate(self.raw_fd(), rate)?;
        self.hw_caps = channel_caps(self.raw_fd());
        Ok(AppliedNativeConfig {
            format,
            channels,
            rate,
        })
    }

    // Size the capture ring into small fragments and make poll byte-accurate.
    // Small fragments set the DMA delivery granularity (the servo's measurement
    // quantization); the hw.snd.latency default can exceed a small graph
    // period. The low-water mark then decouples the poll trigger from the
    // GRANTED fragment size (chn_polltrigger fires at lw, which SETFRAGMENT
    // resets to blksz (channel.c:1980) - so the order here matters, and the
    // mark survives a trigger suspend since chn_resetbuf doesn't touch it).
    // `fragment` is the normalized oss.fragment override (0 = the 1 KiB
    // default); either way the ring keeps the MIN_BUFFER_BYTES budget.
    pub(crate) fn set_small_fragments(&self, fragment: u32, ring: u32) {
        if self.state != DspState::Setup {
            return; // triggered channels can't retune; the next re-prime will
        }
        self.wake_threshold.set(0);
        // max-then-min, not clamp: the kernel cap must win over the floor.
        let ring = ring.max(MIN_BUFFER_BYTES).min(MAX_BUFFER_BYTES as u32);
        if fragment == 0 {
            set_fragment(self.raw_fd(), (ring >> 10).min(u16::MAX as u32) as u16, 10);
        // 1 KiB fragments
        } else {
            // fragment is normalized by the backend to a power of two in
            // [64, 16384], so the selector stays inside the kernel's
            // RANGE(fragln, 4, 16) (dsp.c:1251) and the count never drops under
            // the kernel minimum of 2 (dsp.c:1256)
            let count = (ring >> fragment.trailing_zeros()).max(2u32);
            set_fragment(
                self.raw_fd(),
                count.min(u16::MAX as u32) as u16,
                fragment.trailing_zeros() as u16,
            );
        }
        // best-effort: without it, poll readiness is merely fragment-coarse
        let _ = set_low_water(self.raw_fd(), 1);
    }

    pub(crate) fn prime_buffer(
        &mut self,
        request: CaptureBufferRequest,
        properties: &OssNodeProperties,
        scratch: &mut [u8],
        log: &Log,
    ) -> CaptureBufferGeometry {
        if !self.is_running() {
            let (fragment, capacity) = capture_buffer_plan(request, properties.fragment_bytes());
            self.set_small_fragments(fragment, capacity);
        }

        let ready = self.ready_for_reading(0);
        let layout = self.buffer_layout();
        let mut device_lost = false;
        if ready {
            let mut backlog = layout.queued_bytes;
            while backlog > 0 {
                let chunk = (backlog.min(scratch.len() as u32) / request.stride.max(1))
                    * request.stride.max(1);
                if chunk == 0 {
                    break;
                }
                let outcome = self.read(&mut scratch[..chunk as usize]);
                device_lost |= outcome.status.requires_rebuild();
                if outcome.bytes == 0 {
                    break;
                }
                backlog = backlog.saturating_sub(outcome.bytes as u32);
            }
        }

        let mut geometry = capture_applied_geometry(
            request,
            layout.capacity_bytes,
            layout.quantum_bytes,
            self.delivery_quantum().duration_ns(),
        );
        geometry.device_lost = device_lost;
        self.buffer = CaptureBufferState {
            period_bytes: request.period_bytes,
            quantum_bytes: geometry.quantum_bytes,
            capacity_bytes: geometry.capacity_bytes,
            mismatch_cycles: 0,
            pinned_cycles: 0,
        };
        if geometry.capacity_bytes > 0 && geometry.capacity_bytes < geometry.required_capacity_bytes
        {
            crate::warn!(
                log,
                "granted OSS capture ring ({}) is smaller than the fill geometry needs ({}); \
                 audio will glitch. Lower the PipeWire quantum; we set the fragment size \
                 explicitly, so hw.snd.latency has no effect",
                geometry.capacity_bytes,
                geometry.required_capacity_bytes
            );
        }
        geometry
    }

    pub(crate) fn retune_buffer(
        &mut self,
        request: CaptureBufferRequest,
        primed: bool,
        log: &Log,
    ) -> CaptureRetune {
        if !primed
            || self.buffer.period_bytes == 0
            || request.period_bytes == 0
            || request.period_bytes == self.buffer.period_bytes
        {
            self.buffer.mismatch_cycles = 0;
            return CaptureRetune::Unchanged;
        }
        self.buffer.mismatch_cycles = self.buffer.mismatch_cycles.saturating_add(1);
        if self.buffer.mismatch_cycles < 2 {
            return CaptureRetune::Pending;
        }

        let geometry = capture_applied_geometry(
            request,
            self.buffer.capacity_bytes,
            self.buffer.quantum_bytes,
            self.delivery_quantum().duration_ns(),
        );
        if geometry.capacity_bytes >= geometry.required_capacity_bytes {
            self.buffer.period_bytes = request.period_bytes;
            self.buffer.mismatch_cycles = 0;
            CaptureRetune::Applied(geometry)
        } else if self.suspend() {
            crate::info!(
                log,
                "capture period {} -> {} bytes exceeds the ring ({}); re-priming",
                self.buffer.period_bytes,
                request.period_bytes,
                self.buffer.capacity_bytes
            );
            self.buffer.mismatch_cycles = 0;
            CaptureRetune::Reprime
        } else {
            CaptureRetune::Rebuild
        }
    }

    // Stop the channel but keep the fd: SETTRIGGER(0) aborts, resets the ring
    // and clears TRIGGERED, so the next prime retunes and poll() force-starts
    // the channel again (chn_poll ignores NOTRIGGER). false = driver refused;
    // the caller falls back to closing.
    pub(crate) fn suspend(&mut self) -> bool {
        if self.state != DspState::Running {
            return true; // nothing runs; already primable
        }
        if !set_trigger(self.raw_fd(), 0) {
            return false;
        }
        self.state = DspState::Setup;
        self.needs_trigger = true;
        self.skip = 0; // the ring reset discarded the torn frame with it
        true
    }

    /// Read up to `count` bytes, keeping every returned buffer frame-aligned:
    /// the stream's sample boundaries are fixed by total bytes consumed, so an
    /// unaligned read makes the NEXT buffer start mid-sample and turns it into
    /// static. Callers floor their requests to the stride; if the kernel still
    /// returns short mid-frame (signals), the torn frame's tail is discarded on
    /// the next call and its consumed head hidden from this one - one frame
    /// dropped, alignment kept. Returns a frame-aligned count.
    pub(crate) fn read(&mut self, buf: &mut [u8]) -> ReadOutcome {
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }
        assert_eq!(self.state, DspState::Running);
        while self.skip != 0 {
            let mut scratch = [0u8; 64];
            let len = (self.skip as usize).min(scratch.len());
            match self.descriptor().read(&mut scratch[..len]) {
                Ok(0) => {
                    return ReadOutcome {
                        bytes: 0,
                        status: IoStatus::Disconnected,
                    };
                }
                Ok(count) => self.skip -= count as u32,
                Err(error) => {
                    return ReadOutcome {
                        bytes: 0,
                        status: io_status(error),
                    };
                }
            }
        }
        let count = match self.descriptor().read(buf) {
            Ok(0) => {
                return ReadOutcome {
                    bytes: 0,
                    status: IoStatus::Disconnected,
                };
            }
            Ok(count) => count,
            Err(error) => {
                return ReadOutcome {
                    bytes: 0,
                    status: io_status(error),
                };
            }
        };
        let rem = count % self.stride.max(1) as usize;
        if rem != 0 {
            self.skip = self.stride - rem as u32;
            return ReadOutcome {
                bytes: count - rem,
                status: IoStatus::Progress,
            }; // hide the torn frame's head
        }
        ReadOutcome {
            bytes: count,
            status: IoStatus::Progress,
        }
    }

    pub(crate) fn ready_for_reading(&mut self, timeout_ms: usize) -> bool {
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }

        assert_eq!(self.state, DspState::Running);

        // Capture must be started before its first sound kevent can arrive.
        // The enriched event backend takes over after this prime-time poll;
        // older kernels continue on the timer/ioctl path.
        // poll(2), not select(2): FD_SET writes out of bounds past FD_SETSIZE
        // (1024) fds, which a busy daemon can reach; poll also triggers the
        // capture channel just like select/read do (dsp_poll -> chn_poll)
        let ready = self
            .descriptor()
            .poll(libc::POLLIN, timeout_ms.min(c_int::MAX as usize) as c_int)
            .is_ok_and(|events| events & libc::POLLIN != 0);
        // poll force-starts a trigger-suspended channel but leaves NOTRIGGER
        // set, which would keep the channel from ever auto-restarting; clear it
        if self.needs_trigger {
            self.needs_trigger = false;
            let _ = set_trigger(self.raw_fd(), PCM_ENABLE_INPUT);
        }
        ready
    }

    pub(crate) fn queued_bytes(&self) -> u32 {
        assert_eq!(self.state, DspState::Running);
        input_space(self.raw_fd()).map_or(0, |info| info.bytes.max(0) as u32)
    }

    // fill, granted fragment and total ring from ONE GETISPACE: the prime path
    // needs all three and they come from the same struct (fragsize/fragstotal
    // are layout constants after SETFRAGMENT; only `bytes` moves). (0, 0, 0) =
    // ioctl failed (e.g. device unplugged mid-stream).
    pub(crate) fn buffer_layout(&self) -> BufferLayout {
        assert_eq!(self.state, DspState::Running);
        let Some(info) = input_space(self.raw_fd()) else {
            return BufferLayout::default();
        };
        BufferLayout {
            queued_bytes: info.bytes.max(0) as u32,
            quantum_bytes: info.fragsize.max(0) as u32,
            capacity_bytes: (info.fragstotal.max(0) as u32)
                .saturating_mul(info.fragsize.max(0) as u32),
        }
    }

    pub(crate) fn overruns(&self) -> XrunObservation {
        assert_eq!(self.state, DspState::Running);
        XrunObservation::resetting_events(xrun_counter_bits(get_error(self.raw_fd()).rec_overruns))
    }
}

impl Drop for Dsp {
    fn drop(&mut self) {
        if !self.is_closed() {
            self.close();
        }
    }
}

// Capture half of the pipe-backed test-constructor contract documented with
// DspWriter::test_on_fd; the first read transitions it from setup to running.
#[cfg(test)]
impl Dsp {
    pub(crate) fn test_on_fd(fd: c_int, stride: u32) -> Self {
        Self {
            path: c"test-fd".to_owned(),
            delivery_quantum: DeliveryQuantum::unavailable(),
            // The test constructor takes ownership of this pipe endpoint.
            fd: Some(unsafe { LibcFd::from_raw(fd) }),
            state: DspState::Setup,
            needs_trigger: false,
            hw_caps: 0,
            stride,
            skip: 0,
            wake_threshold: Cell::new(0),
            buffer: CaptureBufferState::default(),
        }
    }
}

#[derive(Clone, Copy)]
struct NativeWrite {
    bytes: ssize_t,
    error: Option<Errno>,
}

pub(crate) struct DspWriter {
    path: String,
    delivery_quantum: DeliveryQuantum,
    fd: Option<LibcFd>,
    state: DspState,
    needs_trigger: bool,  // trigger-suspended: writes buffer until armed
    pause_shadowed: bool, // SILENCE saved bufsoft; Start must pair it with SKIP
    hw_caps: u32,         // best-effort per-open SNDCTL_DSP_GETCAPS snapshot
    stride: u32,          // negotiated frame bytes; the byte stream must stay frame-aligned
    silence_pattern: SilencePattern,
    playback_delay_eighths: u32,
    frame_off: u32, // bytes into a frame a short write left the stream at (0 = aligned)
    wake_threshold: Cell<u32>,
    buffer: PlaybackBufferState,
    #[cfg(debug_assertions)]
    prev_ns: u64,
}

// Keep the dominant signed/float and U8 fills at the maximum ring size: prime
// and recovery commonly need one write, and these live on the data-loop path.
static ZERO_SILENCE: [u8; MAX_BUFFER_BYTES] = [0; MAX_BUFFER_BYTES];
static U8_SILENCE: [u8; MAX_BUFFER_BYTES] = [0x80; MAX_BUFFER_BYTES];

// Companded silence is also a repeated byte, but its low-bandwidth streams do
// not justify another pair of maximum-ring statics. Reuse these small shared
// blocks without regenerating them or allocating on the data loop.
static ULAW_SILENCE: [u8; SILENCE_CHUNK_BYTES] = [0xff; SILENCE_CHUNK_BYTES];
static ALAW_SILENCE: [u8; SILENCE_CHUNK_BYTES] = [0x55; SILENCE_CHUNK_BYTES];

// Four KiB bounds stack use for the less common nonuniform encodings while
// one core-owned, allocation-free pattern covers every configured format.
const SILENCE_CHUNK_BYTES: usize = 4096;

impl DspWriter {
    // Debug-build diagnostics for the FreeBSD scheduling class/priority the
    // data loop actually received. RT setup problems show up here first.
    #[cfg(debug_assertions)]
    pub(crate) fn debug_log_priorities(log: &Log) {
        fn prio_type(type_: std::ffi::c_ushort) -> &'static str {
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

    /// Threshold for a recoverable FreeBSD playback underrun. A vchan mixer
    /// can count a momentarily short child and pad it with silence while the
    /// queue remains healthy. Gate recovery on a genuinely low fill, capped
    /// by the normal delivery sawtooth; otherwise a delivery quantum wider
    /// than the graph period would trigger recovery on every accounting tick.
    pub(crate) fn underrun_low(
        target_fill: u32,
        delivery_quantum: u32,
        period_bytes: u32,
        drained_bytes: u32,
    ) -> u32 {
        let low = period_bytes
            .min(target_fill.saturating_sub(delivery_quantum))
            .max(period_bytes / 4);
        let wander = (period_bytes / 4).max(delivery_quantum);
        low.min(
            target_fill
                .saturating_sub(drained_bytes)
                .saturating_sub(wander),
        )
        .max(period_bytes / 16)
    }

    pub(crate) fn log_underrun_recovery(&self, count: u32, now: u64, suppressed: u32, log: &Log) {
        crate::warn!(
            log,
            "{}: OSS reported {:3} underruns @ {} (+{} warnings suppressed)",
            self.path(),
            count,
            now,
            suppressed
        );
    }

    pub(crate) fn log_ignored_underruns(
        &self,
        count: u32,
        observed_fill: u32,
        recovery_threshold: u32,
        log: &Log,
    ) {
        crate::debug!(
            log,
            "{}: {} underrun counts ignored (fill {} >= {})",
            self.path(),
            count,
            observed_fill,
            recovery_threshold
        );
    }

    pub(crate) fn new(path: &str) -> Self {
        Self {
            path: path.to_string(),
            delivery_quantum: delivery_quantum(path, true), // main thread; nodes are built there
            fd: None,
            state: DspState::Closed,
            needs_trigger: false,
            pause_shadowed: false,
            hw_caps: 0,
            stride: 1,
            silence_pattern: SilencePattern::zero(1),
            playback_delay_eighths: 10,
            frame_off: 0,
            wake_threshold: Cell::new(0),
            buffer: PlaybackBufferState::default(),
            #[cfg(debug_assertions)]
            prev_ns: 0,
        }
    }

    pub(crate) fn is_closed(&self) -> bool {
        self.state == DspState::Closed
    }

    pub(crate) fn path(&self) -> &str {
        &self.path
    }

    pub(crate) fn delivery_quantum(&self) -> DeliveryQuantum {
        self.delivery_quantum
    }

    pub(crate) fn refresh_delivery_quantum(&mut self) {
        self.delivery_quantum = delivery_quantum(&self.path, true);
    }

    pub(crate) fn is_running(&self) -> bool {
        self.state == DspState::Running
    }

    pub(crate) fn hw_caps(&self) -> u32 {
        self.hw_caps
    }

    pub(crate) fn is_virtual_channel(&self) -> bool {
        self.hw_caps & PCM_CAP_VIRTUAL as u32 != 0
    }

    fn descriptor(&self) -> &LibcFd {
        self.fd
            .as_ref()
            .expect("an active DSP writer state owns its descriptor")
    }

    fn raw_fd(&self) -> c_int {
        self.descriptor().raw()
    }

    pub(crate) fn register_wake(
        &self,
        driver: &mut OssWakeDriver,
        stream: StreamIdentity,
        buffer: WakeBufferState,
    ) -> Result<(), WakeError> {
        let fd = self
            .fd
            .as_ref()
            .ok_or_else(|| WakeError::new(Errno::ENODEV))?;
        // EVFILT_WRITE reports free bytes. Wake when draining the live target
        // makes enough space for the next graph write.
        let threshold = playback_wake_threshold(buffer);
        install_wake_threshold(
            fd.raw(),
            &self.wake_threshold,
            threshold,
            buffer.quantum_bytes,
        )?;
        driver.register_stream(fd.raw(), true, stream, buffer.frame_stride)
    }

    pub(super) fn set_silence_pattern(&mut self, pattern: SilencePattern) {
        self.silence_pattern = pattern;
    }

    pub(crate) fn open(&mut self) -> Result<(), Errno> {
        assert_eq!(self.state, DspState::Closed);
        let path = CString::new(self.path.clone()).unwrap();
        let fd = LibcFd::open(&path, libc::O_WRONLY | libc::O_NONBLOCK).ok_or_else(Errno::last)?;
        self.fd = Some(fd);
        self.state = DspState::Setup;
        Ok(())
    }

    pub(crate) fn close(&mut self) {
        assert_ne!(self.state, DspState::Closed);
        // discard the queued buffer so close() doesn't block draining it
        halt(self.raw_fd());
        drop(
            self.fd
                .take()
                .expect("an active DSP writer state owns its descriptor"),
        );
        self.state = DspState::Closed;
        self.needs_trigger = false;
        self.pause_shadowed = false;
        self.hw_caps = 0;
        self.frame_off = 0;
        self.buffer = PlaybackBufferState::default();
        self.wake_threshold.set(0);
    }

    // Stop the channel but keep the fd: SETTRIGGER(0) aborts, resets the ring
    // (discarding queued audio, exactly like close's HALT) and sets NOTRIGGER
    // (writes only buffer until armed), and clears TRIGGERED so the next
    // prime's SETFRAGMENT is legal again; write() arms the channel once real
    // data is buffered (write_silence only buffers, it never arms). false =
    // driver refused; the caller falls back to rebuilding.
    pub(crate) fn suspend(&mut self) -> bool {
        if self.state != DspState::Running {
            return true; // nothing runs; already primable
        }
        if !set_trigger(self.raw_fd(), 0) {
            return false;
        }
        self.state = DspState::Setup;
        self.needs_trigger = true;
        self.pause_shadowed = false;
        self.frame_off = 0; // the ring reset discarded any split frame with it
        true
    }

    /// Pause without losing queued playback. FreeBSD moves the ready portion
    /// of bufsoft into its shadow buffer and substitutes silence, allowing the
    /// hardware side to keep running while graph processing is stopped. The
    /// ioctl may wait for an in-progress channel write, so this is called only
    /// from the serialized Pause handoff, never from process().
    fn queued_pause_outcome(&self) -> PauseOutcome {
        if self.silence_pattern.uniform_byte().is_some() {
            PauseOutcome::Preserved
        } else {
            PauseOutcome::Reprime
        }
    }

    pub(crate) fn pause(&mut self) -> Result<PauseOutcome, Errno> {
        self.pause_shadowed = false;
        if self.state != DspState::Running {
            return Ok(PauseOutcome::Preserved);
        }
        // FreeBSD refreshes the shadow length only when bufsoft has ready
        // bytes. Do not arm a later SKIP for an empty queue: there is no audio
        // to preserve, and the kernel shadow length may still describe an
        // older/reset buffer.
        if try_odelay(self.raw_fd())? <= 0 {
            return Ok(PauseOutcome::Preserved);
        }
        // FreeBSD implements SNDCTL_DSP_SILENCE with memset(zerodata). That is
        // format-correct only when a repeated byte represents silence; for an
        // unsigned multibyte midpoint it would play 0x8080... instead. Ask the
        // shared Pause path to reset and re-prime those streams safely.
        if self.queued_pause_outcome() == PauseOutcome::Reprime {
            return Ok(PauseOutcome::Reprime);
        }
        if !shadow_pause(self.raw_fd()) {
            return Err(Errno::last());
        }
        // SILENCE is a successful no-op if the soft ring drained between the
        // observation above and the ioctl. Only arm SKIP when the post-SILENCE
        // queue shows that FreeBSD actually installed the silence shadow.
        self.pause_shadowed = try_odelay(self.raw_fd())? > 0;
        Ok(PauseOutcome::Preserved)
    }

    /// Restore the queued samples saved by `pause`. This intentionally pairs
    /// FreeBSD's SILENCE/SKIP operations; issuing SKIP independently has
    /// different semantics in the generic OSS API. Like Pause, Start invokes
    /// this only from its serialized data-loop handoff.
    pub(crate) fn resume(&mut self) -> Result<(), Errno> {
        if !std::mem::take(&mut self.pause_shadowed) {
            return Ok(());
        }
        if !restore_shadow(self.raw_fd()) {
            return Err(Errno::last());
        }
        Ok(())
    }

    // start a trigger-suspended channel with whatever is buffered
    fn arm(&mut self) {
        if self.needs_trigger {
            self.needs_trigger = false;
            if !set_trigger(self.raw_fd(), PCM_ENABLE_OUTPUT) {
                eprintln!(
                    "{}: SETTRIGGER(OUTPUT) failed after a trigger suspend",
                    self.path
                );
            }
        }
    }

    pub(super) fn configure(
        &mut self,
        format: u32,
        channels: u32,
        rate: u32,
        channel_order: Option<u64>,
    ) -> Result<AppliedNativeConfig, Errno> {
        assert_eq!(self.state, DspState::Setup);
        let format = set_format(self.raw_fd(), format)?;
        let channels = set_channels(self.raw_fd(), channels)?;
        self.stride = native_frame_stride(format, channels);
        if let Some(order) = channel_order {
            set_channel_order(self.raw_fd(), order)?;
        }
        let rate = set_rate(self.raw_fd(), rate)?;
        self.hw_caps = channel_caps(self.raw_fd());
        Ok(AppliedNativeConfig {
            format,
            channels,
            rate,
        })
    }

    /// Request a `len`-byte output buffer and return the applied geometry.
    /// FreeBSD clamps the fragment count, so the grant can be much smaller than
    /// requested; size the target delay to the return value, not `len`.
    /// `fragment` is the normalized oss.fragment override (0 = 1 KiB default).
    /// The returned capacity and quantum are zero when their readback fails
    /// (for example, after detach). The caller caches the capacity across
    /// period changes, so a fictitious value would gate later changes onto an
    /// in-place retune path with a fill target the real ring cannot hold.
    pub(crate) fn set_buffer_size(&self, len: u32, fragment: u32) -> BufferLayout {
        assert_eq!(self.state, DspState::Setup);
        self.wake_threshold.set(0);
        if fragment == 0 {
            // the fragment count field is 16 bits; an extreme oss.delay x quantum
            // request must clamp, not truncate
            set_fragment(
                self.raw_fd(),
                len.div_ceil(1024).min(u16::MAX as u32) as u16,
                10,
            );
        } else {
            // fragment is normalized by the backend to a power of two in
            // [64, 16384], keeping the selector inside the kernel's
            // RANGE(fragln, 4, 16) (dsp.c:1251); the count clamp mirrors the
            // kernel's own bounds (min 2, total <= MAX_BUFFER_BYTES, dsp.c:1256-1259)
            let count = len
                .div_ceil(fragment)
                .clamp(2, MAX_BUFFER_BYTES as u32 / fragment);
            set_fragment(
                self.raw_fd(),
                count as u16,
                fragment.trailing_zeros() as u16,
            );
        }
        // nothing's written yet, so GETOSPACE reports the granted buffer size
        BufferLayout {
            queued_bytes: 0,
            quantum_bytes: blocksize(self.raw_fd()).max(0) as u32,
            capacity_bytes: ospace_in_bytes(self.raw_fd()).max(0) as u32,
        }
    }

    pub(crate) fn prime_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        properties: &OssNodeProperties,
        log: &Log,
    ) -> PlaybackBufferGeometry {
        self.playback_delay_eighths = properties.playback_delay_eighths();
        let fragment_bytes = properties.fragment_bytes();
        let (capacity_request, _) = playback_buffer_plan(
            request,
            self.delivery_quantum().duration_ns(),
            fragment_bytes,
            self.playback_delay_eighths,
        );
        let applied = self.set_buffer_size(capacity_request, fragment_bytes);
        let geometry = playback_applied_geometry(
            request,
            applied.capacity_bytes,
            applied.quantum_bytes,
            self.delivery_quantum().duration_ns(),
            self.playback_delay_eighths,
        );
        self.buffer = PlaybackBufferState {
            period_bytes: request.period_bytes,
            quantum_bytes: geometry.quantum_bytes,
            capacity_bytes: geometry.capacity_bytes,
            ..PlaybackBufferState::default()
        };

        crate::warn!(
            log,
            "{}: granted {}, blocksize {}, period {}, target delay {}",
            self.path(),
            geometry.capacity_bytes,
            geometry.quantum_bytes,
            request.period_bytes,
            geometry.target_fill_bytes
        );
        self.log_delay_capped(geometry, log);
        if geometry.capacity_bytes < request.period_bytes.saturating_mul(2) {
            crate::warn!(
                log,
                "{}: granted OSS buffer ({}) is smaller than two quanta ({}); \
                 audio will glitch. Lower the PipeWire quantum; we set the fragment size \
                 explicitly, so hw.snd.latency has no effect",
                self.path(),
                geometry.capacity_bytes,
                request.period_bytes.saturating_mul(2)
            );
        }
        self.write_silence(geometry.target_fill_bytes);
        geometry
    }

    pub(crate) fn retune_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        current_fill_bytes: u32,
        now_ns: u64,
        log: &Log,
    ) -> PlaybackRetune {
        if !self.is_running()
            || self.buffer.period_bytes == 0
            || request.period_bytes == 0
            || request.period_bytes == self.buffer.period_bytes
        {
            self.buffer.mismatch_cycles = 0;
            return PlaybackRetune::Unchanged;
        }
        self.buffer.mismatch_cycles = self.buffer.mismatch_cycles.saturating_add(1);
        if self.buffer.mismatch_cycles < 2 {
            return PlaybackRetune::Pending;
        }

        let geometry = playback_retuned_geometry(
            request,
            self.buffer.capacity_bytes,
            self.buffer.quantum_bytes,
            self.delivery_quantum().duration_ns(),
            current_fill_bytes,
            self.playback_delay_eighths,
        );
        if geometry.capacity_bytes >= geometry.required_capacity_bytes {
            crate::info!(
                log,
                "{}: period {} -> {} bytes; retuned in place (granted {}, target delay {} -> {})",
                self.path(),
                self.buffer.period_bytes,
                request.period_bytes,
                geometry.capacity_bytes,
                geometry.target_fill_bytes,
                geometry.target_goal_bytes
            );
            self.buffer.period_bytes = request.period_bytes;
            self.buffer.mismatch_cycles = 0;
            self.log_delay_capped(geometry, log);
            PlaybackRetune::Applied(geometry)
        } else if self.suspend() {
            crate::info!(
                log,
                "{}: period {} -> {} bytes exceeds the ring ({}); re-priming",
                self.path(),
                self.buffer.period_bytes,
                request.period_bytes,
                self.buffer.capacity_bytes
            );
            self.buffer.mismatch_cycles = 0;
            PlaybackRetune::Reprime
        } else {
            if now_ns.saturating_sub(self.buffer.last_retune_log_ns) >= 1_000_000_000 {
                crate::info!(
                    log,
                    "{}: period {} -> {} bytes; reconfiguring (+{} messages suppressed)",
                    self.path(),
                    self.buffer.period_bytes,
                    request.period_bytes,
                    self.buffer.suppressed_retune_logs
                );
                self.buffer.last_retune_log_ns = now_ns;
                self.buffer.suppressed_retune_logs = 0;
            } else {
                self.buffer.suppressed_retune_logs =
                    self.buffer.suppressed_retune_logs.saturating_add(1);
            }
            PlaybackRetune::Rebuild
        }
    }

    fn log_delay_capped(&self, geometry: PlaybackBufferGeometry, log: &Log) {
        if geometry.delay_capped {
            crate::info!(
                log,
                "{}: the {} target is capped by the granted buffer ({})",
                self.path(),
                super::identity::PLAYBACK_DELAY,
                geometry.capacity_bytes
            );
        }
    }

    /// Write `count` bytes, keeping the device byte stream frame-aligned. The
    /// fd is O_NONBLOCK and chn_write is byte-granular: a short return can
    /// split a frame, after which the kernel parses every later sample offset
    /// by the remainder - loud static with the audio faintly underneath. A
    /// split frame is completed from the next `buf` slice (the true
    /// continuation bytes) with a bounded retry; the ring drains continuously,
    /// so the few missing bytes fit within microseconds. The result counts only
    /// bytes accepted from this slice and preserves errno at the write(2)
    /// boundary, before debug diagnostics can issue more ioctls. Callers retain
    /// the unaccepted suffix and pass it back on the next call.
    pub(crate) fn write(&mut self, buf: &[u8]) -> WriteOutcome {
        let count = buf.len() as u32;
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }
        let mut done = 0u32;
        let mut error = None;
        let mut frame_complete = true;

        // A prior short write left the device in the middle of this PCM
        // frame. The retained input suffix starts with its missing bytes, so
        // finish it from real audio before presenting another bulk write.
        if self.frame_off != 0 && count != 0 {
            let need = (self.stride - self.frame_off).min(count);
            let tail = self.write_exact(&buf[..need as usize]);
            done = tail.bytes.max(0) as u32;
            if done < need {
                error = tail.error;
                frame_complete = false;
            }
        }

        if frame_complete && done < count {
            let first = self.write_buffered(&buf[done as usize..]);
            if first.bytes <= 0 {
                error = first.error;
            } else {
                done += first.bytes as u32;
                if self.frame_off != 0 && done < count {
                    // Complete only the split frame here. Any whole-frame
                    // suffix stays in the caller's retained buffer and can be
                    // retried after the next device drain.
                    let need = (self.stride - self.frame_off).min(count - done);
                    let tail = self.write_exact(&buf[done as usize..(done + need) as usize]);
                    done += tail.bytes.max(0) as u32;
                    if tail.bytes < need as ssize_t {
                        error = tail.error;
                    }
                }
            }
        }

        // a trigger-suspended channel starts once real data is buffered
        self.arm();
        WriteOutcome {
            bytes: done as usize,
            status: if done == 0 && error.is_none() {
                IoStatus::WouldBlock
            } else if done < count
                && let Some(error) = error
            {
                io_status(error)
            } else {
                IoStatus::Progress
            },
        }
    }

    // Synthetic fill has no real continuation to retain. Close an open frame
    // with format silence before writing more silence; this path is used only
    // after recovery deliberately chose synthetic audio over continuity.
    #[inline(never)]
    fn realign_with_silence(&mut self) -> Result<(), Option<Errno>> {
        if self.frame_off != 0 {
            let need = self.stride - self.frame_off;
            let mut silence = [0; SILENCE_CHUNK_BYTES];
            self.silence_pattern
                .fill_at(self.frame_off as usize, &mut silence[..need as usize]);
            let result = self.write_exact(&silence[..need as usize]);
            if self.frame_off != 0 {
                return Err(result.error);
            }
        }
        Ok(())
    }

    fn aligned_silence_chunk(&self, count: u32, buffer_len: usize) -> u32 {
        let mut chunk = count.min(buffer_len as u32);
        if chunk < count {
            chunk -= (self.frame_off + chunk) % self.stride.max(1);
        }
        chunk
    }

    fn write_silence_chunk(&mut self, silence: &[u8]) -> Option<u32> {
        let result = self.write_buffered(silence);
        if result.bytes < 0 {
            if let Some(errno) = result.error.filter(|errno| *errno != Errno::EAGAIN) {
                // EAGAIN is just a full buffer; surface anything else.
                eprintln!("{}: write_silence: {}", self.path, errno);
            }
            return None;
        }
        (result.bytes != 0).then_some(result.bytes as u32)
    }

    fn write_uniform_silence(&mut self, mut count: u32, silence: &'static [u8]) {
        while count > 0 {
            debug_assert_eq!((self.frame_off + count) % self.stride.max(1), 0);
            let chunk = self.aligned_silence_chunk(count, silence.len());
            if chunk == 0 {
                break;
            }
            let Some(written) = self.write_silence_chunk(&silence[..chunk as usize]) else {
                break;
            };
            count -= written;
        }
    }

    // Keep the 4 KiB scratch frame out of the common zero/U8 call frame.
    #[inline(never)]
    fn write_pattern_silence(&mut self, mut count: u32) {
        let mut generated = [0; SILENCE_CHUNK_BYTES];
        while count > 0 {
            debug_assert_eq!((self.frame_off + count) % self.stride.max(1), 0);
            let chunk = self.aligned_silence_chunk(count, generated.len());
            if chunk == 0 {
                break;
            }
            self.silence_pattern
                .fill_at(self.frame_off as usize, &mut generated[..chunk as usize]);
            let Some(written) = self.write_silence_chunk(&generated[..chunk as usize]) else {
                break;
            };
            count -= written;
        }
    }

    /// End a retained input sequence before its backing buffer disappears.
    /// Complete an open PCM frame with format silence so the next sequence
    /// starts on a frame boundary. If the channel cannot accept that small
    /// tail, reset its ring instead; `true` tells the caller that device event
    /// state was invalidated by the reset.
    pub(crate) fn end_buffer_sequence(&mut self) -> bool {
        if self.frame_off == 0 {
            return false;
        }
        if self.state != DspState::Running {
            // Setup and Closed own no accepted stream bytes. Their normal
            // transitions also clear frame_off; keep this boundary robust if
            // this branch is reached with stale bookkeeping.
            self.frame_off = 0;
            return false;
        }
        if !self.pause_shadowed && self.realign_with_silence().is_ok() {
            return false;
        }
        self.suspend()
    }

    // push exactly `count` bytes (a partial frame's tail), waiting out EAGAIN
    // briefly: at audio rates the ring frees a byte every few microseconds, so
    // the tail fits well inside the retry budget - unless the channel is
    // trigger-suspended and nothing drains, where waiting is pointless.
    fn write_exact(&mut self, buf: &[u8]) -> NativeWrite {
        let count = buf.len() as u32;
        let mut done = 0u32;
        let mut tries = 0;
        let mut error = None;
        while done < count {
            let attempt = self.write_buffered(&buf[done as usize..]);
            if attempt.bytes > 0 {
                done += attempt.bytes as u32;
                error = None;
                continue;
            }
            error = attempt.error;
            if (attempt.bytes < 0 && error != Some(Errno::EAGAIN)) || self.needs_trigger {
                break;
            }
            tries += 1;
            if tries > 100 {
                eprintln!(
                    "{}: could not complete a split frame ({} of {} bytes)",
                    self.path, done, count
                );
                break;
            }
            std::thread::sleep(Duration::from_nanos(2_000));
        }
        NativeWrite {
            bytes: done as ssize_t,
            error: if done < count { error } else { None },
        }
    }

    fn write_buffered(&mut self, buf: &[u8]) -> NativeWrite {
        let count = buf.len() as u32;
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }
        assert_eq!(self.state, DspState::Running);

        #[cfg(debug_assertions)]
        let space = ospace_in_bytes(self.raw_fd()) as usize;
        #[cfg(debug_assertions)]
        let delay = odelay(self.raw_fd());

        let (nbytes, error) = match self.descriptor().write(&buf[..count as usize]) {
            Ok(nbytes) => (nbytes as ssize_t, None),
            Err(error) => (-1, Some(error)),
        };
        if nbytes > 0 {
            // frame phase of the stream: every accepted byte counts, whoever wrote it
            self.frame_off = (self.frame_off + nbytes as u32) % self.stride.max(1);
        }

        #[cfg(debug_assertions)]
        {
            let now = super::sys::monotonic_time_ns();
            let space_after = ospace_in_bytes(self.raw_fd()) as usize;
            let delay_after = odelay(self.raw_fd());
            eprintln!(
                "{}: {:9} @ {}, count = {:5}, ospace = {:5} -> {:5}, odelay = {:5} -> {:5}",
                self.path,
                now - self.prev_ns,
                now,
                count,
                space,
                space_after,
                delay,
                delay_after
            );
            self.prev_ns = now;
        }

        NativeWrite {
            bytes: nbytes,
            error,
        }
    }

    pub(crate) fn write_silence(&mut self, mut count: u32) {
        // even a zero-length prime must leave the writer Running: callers assume
        // the space/underrun ioctls are usable after priming
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }
        if self.frame_off != 0 && self.realign_with_silence().is_err() {
            return;
        }
        // whole frames only: callers derive `count` from byte-granular ioctls
        // (odelay through a vchan can sit mid-frame), and a split frame turns
        // every later sample into static
        count -= count % self.stride.max(1);
        // The fd is O_NONBLOCK, so a
        // short write or EAGAIN is normal; prime best-effort rather than asserting and
        // panicking out of the `extern "C"` callback (which aborts the process).
        // An early break can leave a frame split; frame_off records it. A
        // later real write completes it from retained audio, while another
        // synthetic fill closes it with the format's silence value.
        match self.silence_pattern.uniform_byte() {
            Some(0) => self.write_uniform_silence(count, &ZERO_SILENCE),
            Some(0x80) => self.write_uniform_silence(count, &U8_SILENCE),
            Some(0xff) => self.write_uniform_silence(count, &ULAW_SILENCE),
            Some(0x55) => self.write_uniform_silence(count, &ALAW_SILENCE),
            _ => self.write_pattern_silence(count),
        }
    }

    pub(crate) fn queued_bytes(&self) -> u32 {
        assert_eq!(self.state, DspState::Running);
        odelay(self.raw_fd()).max(0) as u32
    }

    /// The fragment size the driver actually granted (may differ from what
    /// SETFRAGMENT asked for; some drivers force a fixed period).
    pub(crate) fn underruns(&self) -> XrunObservation {
        assert_eq!(self.state, DspState::Running);
        // Timer-driven and follower fallback. Enriched driver wakes consume
        // the xrun count from the same kevent snapshot as their queued fill.
        XrunObservation::resetting_events(xrun_counter_bits(
            get_error(self.raw_fd()).play_underruns,
        ))
    }
}

// Pipe-backed constructors for the alignment and recovery tests: a pipe
// write end is byte-granular under O_NONBLOCK exactly like chn_write, with
// byte-exact buffer accounting, so short writes can be forced
// deterministically. The device starts in setup, like a freshly configured
// channel; the first write/read transitions it to running.
#[cfg(test)]
impl DspWriter {
    pub(crate) fn test_on_fd(fd: c_int, stride: u32) -> Self {
        Self {
            path: "test-fd".to_string(),
            delivery_quantum: DeliveryQuantum::unavailable(),
            // The test constructor takes ownership of this pipe endpoint.
            fd: Some(unsafe { LibcFd::from_raw(fd) }),
            state: DspState::Setup,
            needs_trigger: false,
            pause_shadowed: false,
            hw_caps: 0,
            stride,
            silence_pattern: SilencePattern::zero(stride as usize),
            playback_delay_eighths: 10,
            frame_off: 0,
            wake_threshold: Cell::new(0),
            buffer: PlaybackBufferState::default(),
            #[cfg(debug_assertions)]
            prev_ns: 0,
        }
    }
}

#[cfg(test)]
mod wake_policy_tests {
    use crate::backend::WakeBufferState;

    #[test]
    fn native_thresholds_derive_from_applied_buffer_state() {
        let buffer = WakeBufferState {
            frame_stride: 8,
            period_bytes: 16_384,
            quantum_bytes: 2_048,
            capacity_bytes: 65_536,
            target_fill_bytes: 20_480,
        };
        assert_eq!(super::capture_wake_threshold(buffer), 20_480);
        assert_eq!(super::playback_wake_threshold(buffer), 45_056);

        assert_eq!(super::capture_wake_threshold(WakeBufferState::default()), 1);
        assert_eq!(
            super::playback_wake_threshold(WakeBufferState {
                capacity_bytes: 4_096,
                target_fill_bytes: 4_096,
                ..WakeBufferState::default()
            }),
            1
        );
    }

    #[test]
    fn threshold_updates_follow_native_quantum_granularity() {
        assert!(super::wake_threshold_changed(0, 16_384, 2_048));
        assert!(!super::wake_threshold_changed(16_384, 17_407, 2_048));
        assert!(super::wake_threshold_changed(16_384, 18_432, 2_048));
        assert!(super::wake_threshold_changed(18_432, 16_384, 2_048));
        assert!(super::wake_threshold_changed(8, 9, 0));
    }
}

#[cfg(test)]
mod playback_tests {
    use crate::backend::{
        StreamConfig,
        test_transport::{drain, fill_pipe, free_space, pattern, pipe_pair},
    };

    fn silence_pattern(format: u32, stride: u32) -> crate::backend::SilencePattern {
        StreamConfig {
            format: libspa::param::audio::AudioFormat(format),
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride,
        }
        .silence_pattern()
    }

    #[test]
    fn oss_underrun_threshold_tracks_delivery_and_lateness() {
        let underrun_low = super::DspWriter::underrun_low;
        assert_eq!(underrun_low(20_480, 2_048, 16_384, 0), 16_384);
        assert!(20_480 - 2_048 >= underrun_low(20_480, 2_048, 16_384, 0));
        assert_eq!(underrun_low(20_480, 18_432, 16_384, 0), 20_480 - 18_432);
        assert_eq!(
            underrun_low(20_480, 2_048, 16_384, 8_192),
            20_480 - 8_192 - 4_096
        );
        assert_eq!(underrun_low(20_480, 2_048, 16_384, 1 << 30), 16_384 / 16);
    }

    #[test]
    fn oss_playback_retune_requires_two_matching_cycles() {
        use crate::backend::{PlaybackBufferRequest, PlaybackRetune};

        let (read_fd, write_fd) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(write_fd, 8);
        dsp.write_silence(0);
        dsp.buffer = super::PlaybackBufferState {
            period_bytes: 2_048,
            quantum_bytes: 1_024,
            capacity_bytes: 65_536,
            ..Default::default()
        };
        let request = PlaybackBufferRequest {
            period_bytes: 4_096,
            graph_rate: 0,
            stride: 8,
            device_rate: 48_000,
            write_bytes: 4_096,
            maximum_write_bytes: 4_096,
        };
        let log = crate::spa::Log::test_null();

        assert_eq!(
            dsp.retune_buffer(request, 4_096, 0, &log),
            PlaybackRetune::Pending
        );
        assert_eq!(dsp.buffer.mismatch_cycles, 1);
        assert!(matches!(
            dsp.retune_buffer(request, 4_096, 0, &log),
            PlaybackRetune::Applied(_)
        ));
        assert_eq!(dsp.buffer.period_bytes, 4_096);
        assert_eq!(dsp.buffer.mismatch_cycles, 0);
        unsafe { libc::close(read_fd) };
    }

    #[test]
    fn native_errors_map_to_semantic_io_statuses() {
        use crate::backend::IoStatus;

        assert_eq!(
            super::io_status(nix::errno::Errno::EAGAIN),
            IoStatus::WouldBlock
        );
        assert_eq!(
            super::io_status(nix::errno::Errno::EINTR),
            IoStatus::WouldBlock
        );
        assert_eq!(
            super::io_status(nix::errno::Errno::ENODEV),
            IoStatus::Disconnected
        );
        assert_eq!(
            super::io_status(nix::errno::Errno::EIO),
            IoStatus::Fatal(crate::backend::StreamError::from_native_code(
                nix::errno::Errno::EIO as i32
            ))
        );
    }

    #[test]
    fn write_silence_floors_to_frames() {
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 8);
        dsp.write_silence(2047); // odelay through a vchan can produce counts like this
        let got = drain(r);
        assert_eq!(got.len(), 2040);
        assert!(got.iter().all(|&b| b == 0));
        unsafe { libc::close(r) };
    }

    #[test]
    fn u8_silence_uses_the_biased_midpoint() {
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 2);
        // Apply the same state derived from a successful native readback; a
        // pipe cannot accept the configuration ioctls themselves.
        dsp.stride = super::native_frame_stride(super::AFMT_U8, 2);
        dsp.set_silence_pattern(silence_pattern(
            libspa::sys::SPA_AUDIO_FORMAT_U8,
            dsp.stride,
        ));
        assert_eq!(dsp.stride, 2);
        dsp.write_silence(8);
        assert_eq!(drain(r), vec![0x80; 8]);

        // A synthetic fill after a short prior write repairs the open frame
        // with biased silence; 0x00 here would be a full-scale U8 sample.
        dsp.frame_off = 1;
        dsp.write_silence(0);
        assert_eq!(drain(r), vec![0x80]);
        assert_eq!(dsp.frame_off, 0);

        let data = [0x91, 0x92];
        assert_eq!(dsp.write(&data).bytes, data.len());
        assert_eq!(drain(r), data);
        unsafe { libc::close(r) };
    }

    #[test]
    fn companded_silence_uses_pipewire_compatible_zero_codes() {
        for (format, expected) in [(super::AFMT_MU_LAW, 0xff), (super::AFMT_A_LAW, 0x55)] {
            let (r, w) = pipe_pair(true, true);
            let mut dsp = super::DspWriter::test_on_fd(w, 2);
            dsp.stride = super::native_frame_stride(format, 2);
            let spa_format = if format == super::AFMT_MU_LAW {
                libspa::sys::SPA_AUDIO_FORMAT_ULAW
            } else {
                libspa::sys::SPA_AUDIO_FORMAT_ALAW
            };
            dsp.set_silence_pattern(silence_pattern(spa_format, dsp.stride));
            assert_eq!(dsp.stride, 2);
            let count = super::SILENCE_CHUNK_BYTES as u32 * 2;
            dsp.write_silence(count);
            assert_eq!(drain(r), vec![expected; count as usize]);
            unsafe { libc::close(r) };
        }
    }

    #[test]
    fn unsigned_multibyte_silence_and_partial_frame_repair_use_core_patterns() {
        const FORMATS: &[(u32, u32, u32, &[u8])] = &[
            (super::AFMT_S8, libspa::sys::SPA_AUDIO_FORMAT_S8, 2, &[0x00]),
            (
                super::AFMT_U16_LE,
                libspa::sys::SPA_AUDIO_FORMAT_U16_LE,
                4,
                &[0x00, 0x80],
            ),
            (
                super::AFMT_U16_BE,
                libspa::sys::SPA_AUDIO_FORMAT_U16_BE,
                4,
                &[0x80, 0x00],
            ),
            (
                super::AFMT_U24_LE,
                libspa::sys::SPA_AUDIO_FORMAT_U24_LE,
                6,
                &[0x00, 0x00, 0x80],
            ),
            (
                super::AFMT_U24_BE,
                libspa::sys::SPA_AUDIO_FORMAT_U24_BE,
                6,
                &[0x80, 0x00, 0x00],
            ),
            (
                super::AFMT_U32_LE,
                libspa::sys::SPA_AUDIO_FORMAT_U32_LE,
                8,
                &[0x00, 0x00, 0x00, 0x80],
            ),
            (
                super::AFMT_U32_BE,
                libspa::sys::SPA_AUDIO_FORMAT_U32_BE,
                8,
                &[0x80, 0x00, 0x00, 0x00],
            ),
        ];

        for &(native, spa, stride, expected) in FORMATS {
            let (r, w) = pipe_pair(true, true);
            let mut dsp = super::DspWriter::test_on_fd(w, stride);
            assert_eq!(super::native_frame_stride(native, 2), stride);
            dsp.set_silence_pattern(silence_pattern(spa, stride));
            dsp.write_silence(stride * 2);
            assert_eq!(
                drain(r),
                expected
                    .iter()
                    .copied()
                    .cycle()
                    .take((stride * 2) as usize)
                    .collect::<Vec<_>>()
            );
            unsafe { libc::close(r) };
        }

        // The reusable fill chunk is not divisible by a three-byte sample or
        // six-byte stereo frame. Crossing it must not restart the midpoint
        // pattern in the middle of a sample.
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 6);
        dsp.set_silence_pattern(silence_pattern(libspa::sys::SPA_AUDIO_FORMAT_U24_LE, 6));
        dsp.write_silence(8_196);
        let silence = drain(r);
        assert_eq!(silence.len(), 8_196);
        assert!(
            silence
                .iter()
                .copied()
                .eq([0x00, 0x00, 0x80].into_iter().cycle().take(8_196))
        );
        unsafe { libc::close(r) };

        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 4);
        dsp.set_silence_pattern(silence_pattern(libspa::sys::SPA_AUDIO_FORMAT_U16_LE, 4));
        assert_eq!(dsp.write(&[0x42]).bytes, 1);
        assert!(!dsp.end_buffer_sequence());
        assert_eq!(drain(r), [0x42, 0x80, 0x00, 0x80]);
        unsafe { libc::close(r) };
    }

    #[test]
    fn multibyte_unsigned_queue_reprimes_before_kernel_byte_fill() {
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 4);
        dsp.set_silence_pattern(silence_pattern(libspa::sys::SPA_AUDIO_FORMAT_U16_LE, 4));

        assert_eq!(
            dsp.queued_pause_outcome(),
            crate::backend::PauseOutcome::Reprime
        );
        assert!(!dsp.pause_shadowed);
        unsafe { libc::close(r) };
    }

    #[test]
    fn empty_or_failed_shadow_pause_does_not_leave_a_stale_resume() {
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 8);
        dsp.write_silence(0); // transition the test writer to Running

        // A pipe rejects GETODELAY. A failed Pause must not make a later Start
        // issue an unrelated SKIP against the descriptor.
        assert!(dsp.pause().is_err());
        assert!(!dsp.pause_shadowed);
        assert!(dsp.resume().is_ok());

        // Likewise, consume the pairing token before trying SKIP so a failure
        // cannot replay the command on every later Start.
        dsp.pause_shadowed = true;
        assert!(dsp.resume().is_err());
        assert!(!dsp.pause_shadowed);
        assert!(dsp.resume().is_ok());
        unsafe { libc::close(r) };
    }

    // A short write that splits a frame must not shift the device byte stream:
    // every later sample would otherwise be stitched from two neighbors
    // (white noise with the audio faintly underneath).
    #[test]
    fn short_write_keeps_stream_frame_aligned() {
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 8);

        // fill the pipe to capacity, then free a mid-frame hole: the next write
        // is forced short at an unaligned count, like a full OSS ring
        let total_fill = fill_pipe(w);
        free_space(r, 2046);

        // 2046 = 255 frames + 6 bytes: the kernel takes all of it, the 2-byte
        // frame tail can't fit, and the split is recorded rather than dropped
        let a = pattern(4096, 1);
        let ret = dsp.write(&a);
        assert_eq!(ret.bytes, 2046);
        assert_eq!(dsp.frame_off, 6);

        let queued = drain(r); // remaining filler, then the accepted head
        assert_eq!(queued.len(), total_fill); // the 2046-byte hole was exactly refilled
        assert_eq!(&queued[queued.len() - 2046..], &a[..2046]);

        // With space available again, retry the untouched suffix. Its first
        // two bytes complete the split frame, so no samples are dropped or
        // replaced while the stream returns to a frame boundary.
        let ret = dsp.write(&a[2046..]);
        assert_eq!(ret.bytes, 2050);
        assert_eq!(dsp.frame_off, 0);
        assert_eq!(drain(r), &a[2046..]);

        // The next graph buffer then starts on its natural frame boundary.
        let b = pattern(4096, 2);
        let ret = dsp.write(&b);
        assert_eq!(ret.bytes, 4096);
        assert_eq!(dsp.frame_off, 0);
        assert_eq!(drain(r), b);
        unsafe { libc::close(r) };
    }

    #[test]
    fn ending_a_buffer_sequence_does_not_stitch_it_to_the_next_one() {
        let (r, w) = pipe_pair(true, true);
        let mut dsp = super::DspWriter::test_on_fd(w, 8);

        let total_fill = fill_pipe(w);
        free_space(r, 2046);
        let old = pattern(4096, 1);
        assert_eq!(dsp.write(&old).bytes, 2046);
        assert_eq!(dsp.frame_off, 6);
        assert_eq!(drain(r).len(), total_fill);

        // A buffer-pool replacement abandons old[2046..]. Close that frame
        // with silence before bytes from the new pool reach the device.
        assert!(!dsp.end_buffer_sequence());
        assert_eq!(dsp.frame_off, 0);

        let new = pattern(4096, 2);
        assert_eq!(dsp.write(&new).bytes, new.len());
        let queued = drain(r);
        assert_eq!(&queued[..2], &[0, 0]);
        assert_eq!(&queued[2..], new);
        unsafe { libc::close(r) };
    }
}

impl Drop for DspWriter {
    fn drop(&mut self) {
        if !self.is_closed() {
            self.close();
        }
    }
}

#[cfg(test)]
mod capture_tests {
    use super::{Dsp, DspWriter, PCM_CAP_VIRTUAL};
    use crate::backend::test_transport::{pattern, pipe_pair};

    #[test]
    fn virtual_channel_detection_tests_the_capability_bit() {
        let mut capture = Dsp::new("/dev/dsp0");
        let mut playback = DspWriter::new("/dev/dsp0");
        assert!(!capture.is_virtual_channel());
        assert!(!playback.is_virtual_channel());

        capture.hw_caps = PCM_CAP_VIRTUAL as u32;
        playback.hw_caps = PCM_CAP_VIRTUAL as u32;
        assert!(capture.is_virtual_channel());
        assert!(playback.is_virtual_channel());
    }

    #[test]
    fn oss_overrun_recovery_requires_three_pinned_cycles() {
        let (read_fd, write_fd) = pipe_pair(false, false);
        let mut dsp = super::Dsp::test_on_fd(read_fd, 8);
        dsp.buffer = super::CaptureBufferState {
            period_bytes: 1_024,
            quantum_bytes: 1_024,
            capacity_bytes: 8_192,
            ..Default::default()
        };
        let log = crate::spa::Log::test_null();

        assert_eq!(dsp.recover_overrun(4, Some(8_000), &log), None);
        assert_eq!(dsp.recover_overrun(4, Some(8_000), &log), None);
        assert_eq!(dsp.buffer.pinned_cycles, 2);
        assert_eq!(dsp.recover_overrun(4, Some(100), &log), None);
        assert_eq!(dsp.buffer.pinned_cycles, 0);
        assert_eq!(dsp.recover_overrun(4, Some(8_000), &log), None);
        assert_eq!(dsp.recover_overrun(4, Some(8_000), &log), None);
        assert_eq!(dsp.recover_overrun(4, Some(8_000), &log), Some(true));
        assert_eq!(dsp.buffer.pinned_cycles, 0);
        unsafe { libc::close(write_fd) };
    }

    // capture mirror image: a read that lands mid-frame must hide the torn
    // frame's head and discard its tail, so every returned buffer starts on a
    // frame boundary
    #[test]
    fn read_hides_torn_frame_and_realigns() {
        let (r, w) = pipe_pair(true, false);
        let mut dsp = super::Dsp::test_on_fd(r, 8);
        let s = pattern(2056, 3);
        assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 2046) }, 2046);

        // 2046 available < 4096 requested: the pipe returns it all, mid-frame
        let mut buf = vec![0u8; 4096];
        let n = dsp.read(&mut buf[..4096]);
        assert_eq!(n.bytes, 2040);
        assert_eq!(&buf[..2040], &s[..2040]);
        assert_eq!(dsp.skip, 2);

        // the stream continues; the torn frame's tail is skipped and the next
        // buffer starts exactly on the following frame boundary
        assert_eq!(
            unsafe { libc::write(w, s.as_ptr().add(2046).cast(), 10) },
            10
        );
        let n = dsp.read(&mut buf[..8]);
        assert_eq!(n.bytes, 8);
        assert_eq!(&buf[..8], &s[2048..2056]);
        assert_eq!(dsp.skip, 0);
        unsafe { libc::close(w) };
    }
}
