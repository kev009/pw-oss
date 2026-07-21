use libc::{size_t, ssize_t};
use nix::errno::Errno;
use std::ffi::{CString, c_int};

use crate::freebsd::{LibcFd, ioctl_int, ioctl_read};

use super::abi::*;
use super::devices::{CHN_2NDBUFMAXSIZE, MIN_RING_BYTES, drain_quantum_ns};

pub(crate) struct Dsp {
    path: CString,
    pub hw_quantum_ns: u64, // the hardware drain quantum (sndstat); 0 = fragment-accurate
    fd: Option<LibcFd>,
    state: DspState,
    needs_trigger: bool, // trigger-suspended: NOTRIGGER must be cleared on restart
    stride: u32,         // negotiated frame bytes; reads must consume whole frames
    skip: u32,           // tail bytes of a torn frame to discard before the next read
}

impl Dsp {
    pub(crate) fn new(path: &str) -> Self {
        Self {
            path: CString::new(path).unwrap(),
            hw_quantum_ns: drain_quantum_ns(path, false),
            fd: None,
            state: DspState::Closed,
            needs_trigger: false,
            stride: 1,
            skip: 0,
        }
    }

    pub(crate) fn is_closed(&self) -> bool {
        self.state == DspState::Closed
    }

    pub(crate) fn path(&self) -> &str {
        self.path.to_str().unwrap_or("") // constructed from &str; always valid
    }

    // on direct opens the hardware blocksize is per-session state; call after
    // configure so the snapshot reflects THIS session (see drain_quantum_ns)
    pub(crate) fn refresh_hw_quantum(&mut self) {
        if let Ok(path) = self.path.to_str() {
            self.hw_quantum_ns = drain_quantum_ns(path, false);
        }
    }

    pub(crate) fn is_running(&self) -> bool {
        self.state == DspState::Running
    }

    fn raw_fd(&self) -> c_int {
        self.fd
            .as_ref()
            .expect("an active DSP state owns its descriptor")
            .raw()
    }

    pub(crate) fn fd(&self) -> Option<c_int> {
        self.fd.as_ref().map(LibcFd::raw)
    }

    pub(crate) fn set_low_water(&self, bytes: u32) -> bool {
        self.fd.as_ref().is_some_and(|fd| {
            ioctl_int(
                fd.raw(),
                SNDCTL_DSP_LOW_WATER,
                bytes.clamp(1, c_int::MAX as u32) as c_int,
            )
            .is_some()
        })
    }

    pub(crate) fn open(&mut self) -> Result<(), Errno> {
        assert_eq!(self.state, DspState::Closed);

        // O_RDONLY, not O_RDWR: on devices with asymmetric play/rec channel
        // counts (e.g. RODECaster) the kernel won't take per-direction counts on
        // one fd (shkhln/pw-oss#3)
        let fd = LibcFd::open(&self.path, libc::O_RDONLY).ok_or_else(Errno::last)?;

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
        self.skip = 0;
    }

    pub(crate) fn configure(
        &mut self,
        format: u32,
        channels: u32,
        rate: u32,
        channel_order: Option<u64>,
    ) -> Result<(), Errno> {
        assert_eq!(self.state, DspState::Setup);
        // plain AFMT selector (no channel field), so this yields the sample width
        self.stride = afmt_frame_bytes(format)
            .max(1)
            .saturating_mul(channels.max(1));
        set_value(self.raw_fd(), SNDCTL_DSP_SETFMT, format, 0)?;
        set_value(self.raw_fd(), SNDCTL_DSP_CHANNELS, channels, 0)?;
        if let Some(order) = channel_order {
            set_channel_order(self.raw_fd(), order)?;
        }
        set_value(self.raw_fd(), SNDCTL_DSP_SPEED, rate, feeder_rate_round())
    }

    // Size the capture ring into small fragments and make poll byte-accurate.
    // Small fragments set the DMA delivery granularity (the servo's measurement
    // quantization); the hw.snd.latency default can exceed a small graph
    // period. The low-water mark then decouples the poll trigger from the
    // GRANTED fragment size (chn_polltrigger fires at lw, which SETFRAGMENT
    // resets to blksz (channel.c:1980) - so the order here matters, and the
    // mark survives a trigger suspend since chn_resetbuf doesn't touch it).
    // `fragment` is the normalized oss.fragment override (0 = the 1 KiB
    // default); either way the ring keeps the MIN_RING_BYTES budget.
    pub(crate) fn set_small_fragments(&self, fragment: u32, ring: u32) {
        if self.state != DspState::Setup {
            return; // triggered channels can't retune; the next re-prime will
        }
        // max-then-min, not clamp: the kernel cap must win over the floor (and
        // clamp panics if a future rate-dependent cap undercuts MIN_RING_BYTES)
        let ring = ring.max(MIN_RING_BYTES).min(CHN_2NDBUFMAXSIZE as u32);
        if fragment == 0 {
            set_fragment(self.raw_fd(), (ring >> 10).min(u16::MAX as u32) as u16, 10);
        // 1 KiB fragments
        } else {
            // fragment is a power of two in [64, 16384] (node::normalize_fragment
            // normalize_fragment), so the selector stays inside the kernel's
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
        let _ = ioctl_int(self.raw_fd(), SNDCTL_DSP_LOW_WATER, 1);
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
    pub(crate) fn read(&mut self, buf: &mut [u8]) -> ssize_t {
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }
        assert_eq!(self.state, DspState::Running);
        while self.skip != 0 {
            let mut scratch = [0u8; 64];
            let n = unsafe {
                libc::read(
                    self.raw_fd(),
                    scratch.as_mut_ptr().cast(),
                    (self.skip as usize).min(scratch.len()),
                )
            };
            if n <= 0 {
                return n; // capture is running when reads happen; treat as the caller's error
            }
            self.skip -= n as u32;
        }
        let n = unsafe { libc::read(self.raw_fd(), buf.as_mut_ptr().cast(), buf.len()) };
        if n <= 0 {
            return n;
        }
        let rem = n as usize % self.stride.max(1) as usize;
        if rem != 0 {
            self.skip = self.stride - rem as u32;
            return (n as usize - rem) as ssize_t; // hide the torn frame's head
        }
        n
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
        let mut pfd = libc::pollfd {
            fd: self.raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let n = unsafe { libc::poll(&mut pfd, 1, timeout_ms as i32) };
        // poll force-starts a trigger-suspended channel but leaves NOTRIGGER
        // set, which would keep the channel from ever auto-restarting; clear it
        if self.needs_trigger {
            self.needs_trigger = false;
            let _ = set_trigger(self.raw_fd(), PCM_ENABLE_INPUT);
        }
        n > 0 && (pfd.revents & libc::POLLIN) != 0
    }

    pub(crate) fn ispace_in_bytes(&self) -> c_int {
        assert_eq!(self.state, DspState::Running);
        unsafe { ioctl_read::<audio_buf_info>(self.raw_fd(), SNDCTL_DSP_GETISPACE) }
            .map_or(0, |info| info.bytes)
    }

    // fill, granted fragment and total ring from ONE GETISPACE: the prime path
    // needs all three and they come from the same struct (fragsize/fragstotal
    // are layout constants after SETFRAGMENT; only `bytes` moves). (0, 0, 0) =
    // ioctl failed (e.g. device unplugged mid-stream).
    pub(crate) fn ispace_layout(&mut self) -> (u32, u32, u32) {
        assert_eq!(self.state, DspState::Running);
        let Some(info) =
            (unsafe { ioctl_read::<audio_buf_info>(self.raw_fd(), SNDCTL_DSP_GETISPACE) })
        else {
            return (0, 0, 0);
        };
        (
            info.bytes.max(0) as u32,
            info.fragsize.max(0) as u32,
            (info.fragstotal.max(0) as u32).saturating_mul(info.fragsize.max(0) as u32),
        )
    }

    pub(crate) fn overruns(&self) -> u32 {
        assert_eq!(self.state, DspState::Running);
        get_error(self.raw_fd()).rec_overruns.max(0) as u32
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
            hw_quantum_ns: 0,
            // The test constructor takes ownership of this pipe endpoint.
            fd: Some(unsafe { LibcFd::from_raw(fd) }),
            state: DspState::Setup,
            needs_trigger: false,
            stride,
            skip: 0,
        }
    }
}

#[cfg(debug_assertions)]
fn now_ns_libc() -> u64 {
    let mut now = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
    assert!(err != -1);
    (now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PlaybackWrite {
    pub bytes: ssize_t,
    pub error: Option<Errno>,
}

impl PlaybackWrite {
    pub(crate) fn consumed(bytes: ssize_t) -> Self {
        Self { bytes, error: None }
    }

    pub(crate) fn would_block(&self) -> bool {
        (self.bytes < 0 && self.error == Some(Errno::EAGAIN))
            || (self.bytes == 0 && (self.error.is_none() || self.error == Some(Errno::EAGAIN)))
    }
}

pub(crate) struct DspWriter {
    pub path: String,
    pub hw_quantum_ns: u64, // the hardware drain quantum (sndstat); 0 = fragment-accurate
    fd: Option<LibcFd>,
    state: DspState,
    needs_trigger: bool,  // trigger-suspended: writes buffer until armed
    pause_shadowed: bool, // SILENCE saved bufsoft; Start must pair it with SKIP
    stride: u32,          // negotiated frame bytes; the byte stream must stay frame-aligned
    silence_byte: u8,     // 0x80 for biased U8 PCM; zero for every other supported format
    frame_off: u32,       // bytes into a frame a short write left the stream at (0 = aligned)
    #[cfg(debug_assertions)]
    prev_ns: u64,
}

static ZERO_SILENCE: [u8; CHN_2NDBUFMAXSIZE] = [0; CHN_2NDBUFMAXSIZE];
static U8_SILENCE: [u8; CHN_2NDBUFMAXSIZE] = [0x80; CHN_2NDBUFMAXSIZE];

impl DspWriter {
    pub(crate) fn new(path: &str) -> Self {
        Self {
            path: path.to_string(),
            hw_quantum_ns: drain_quantum_ns(path, true), // main thread; nodes are built there
            fd: None,
            state: DspState::Closed,
            needs_trigger: false,
            pause_shadowed: false,
            stride: 1,
            silence_byte: 0,
            frame_off: 0,
            #[cfg(debug_assertions)]
            prev_ns: 0,
        }
    }

    pub(crate) fn is_closed(&self) -> bool {
        self.state == DspState::Closed
    }

    pub(crate) fn is_running(&self) -> bool {
        self.state == DspState::Running
    }

    fn raw_fd(&self) -> c_int {
        self.fd
            .as_ref()
            .expect("an active DSP writer state owns its descriptor")
            .raw()
    }

    pub(crate) fn fd(&self) -> Option<c_int> {
        self.fd.as_ref().map(LibcFd::raw)
    }

    pub(crate) fn set_low_water(&self, bytes: u32) -> bool {
        self.fd.as_ref().is_some_and(|fd| {
            ioctl_int(
                fd.raw(),
                SNDCTL_DSP_LOW_WATER,
                bytes.clamp(1, c_int::MAX as u32) as c_int,
            )
            .is_some()
        })
    }

    fn silence_buffer(&self) -> &'static [u8; CHN_2NDBUFMAXSIZE] {
        if self.silence_byte == 0x80 {
            &U8_SILENCE
        } else {
            &ZERO_SILENCE
        }
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
        unsafe {
            libc::ioctl(self.raw_fd(), SNDCTL_DSP_HALT);
        }
        drop(
            self.fd
                .take()
                .expect("an active DSP writer state owns its descriptor"),
        );
        self.state = DspState::Closed;
        self.needs_trigger = false;
        self.pause_shadowed = false;
        self.frame_off = 0;
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
    pub(crate) fn pause(&mut self) -> Result<(), Errno> {
        self.pause_shadowed = false;
        if self.state != DspState::Running {
            return Ok(());
        }
        // FreeBSD refreshes the shadow length only when bufsoft has ready
        // bytes. Do not arm a later SKIP for an empty queue: there is no audio
        // to preserve, and the kernel shadow length may still describe an
        // older/reset buffer.
        if try_odelay(self.raw_fd())? <= 0 {
            return Ok(());
        }
        if !shadow_pause(self.raw_fd()) {
            return Err(Errno::last());
        }
        // SILENCE is a successful no-op if the soft ring drained between the
        // observation above and the ioctl. Only arm SKIP when the post-SILENCE
        // queue shows that FreeBSD actually installed the silence shadow.
        self.pause_shadowed = try_odelay(self.raw_fd())? > 0;
        Ok(())
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

    pub(crate) fn configure(
        &mut self,
        format: u32,
        channels: u32,
        rate: u32,
        silence_byte: u8,
        channel_order: Option<u64>,
    ) -> Result<(), Errno> {
        assert_eq!(self.state, DspState::Setup);
        // plain AFMT selector (no channel field), so this yields the sample width
        self.stride = afmt_frame_bytes(format)
            .max(1)
            .saturating_mul(channels.max(1));
        self.silence_byte = silence_byte;
        set_value(self.raw_fd(), SNDCTL_DSP_SETFMT, format, 0)?;
        set_value(self.raw_fd(), SNDCTL_DSP_CHANNELS, channels, 0)?;
        if let Some(order) = channel_order {
            set_channel_order(self.raw_fd(), order)?;
        }
        set_value(self.raw_fd(), SNDCTL_DSP_SPEED, rate, feeder_rate_round())
    }

    /// Request a `len`-byte output buffer and return the size the device granted.
    /// FreeBSD clamps the fragment count, so the grant can be much smaller than
    /// requested; size the target delay to the return value, not `len`.
    /// `fragment` is the normalized oss.fragment override (0 = 1 KiB default).
    /// Returns 0 when the grant can't be read back (the device vanished): the
    /// caller caches this value across period changes, and a fictitious
    /// capacity would gate quantum changes onto the in-place retune path
    /// forever with a fill target the real ring can't hold.
    pub(crate) fn set_buffer_size(&self, len: u32, fragment: u32) -> u32 {
        assert_eq!(self.state, DspState::Setup);
        if fragment == 0 {
            // the fragment count field is 16 bits; an extreme oss.delay x quantum
            // request must clamp, not truncate
            set_fragment(
                self.raw_fd(),
                len.div_ceil(1024).min(u16::MAX as u32) as u16,
                10,
            );
        } else {
            // fragment is a power of two in [64, 16384] (node::normalize_fragment
            // normalize_fragment), keeping the selector inside the kernel's
            // RANGE(fragln, 4, 16) (dsp.c:1251); the count clamp mirrors the
            // kernel's own bounds (min 2, total <= CHN_2NDBUFMAXSIZE, dsp.c:1256-1259)
            let count = len
                .div_ceil(fragment)
                .clamp(2, CHN_2NDBUFMAXSIZE as u32 / fragment);
            set_fragment(
                self.raw_fd(),
                count as u16,
                fragment.trailing_zeros() as u16,
            );
        }
        // nothing's written yet, so GETOSPACE reports the granted buffer size
        ospace_in_bytes(self.raw_fd()).max(0) as u32
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
    pub(crate) fn write(&mut self, buf: &[u8]) -> PlaybackWrite {
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
        PlaybackWrite {
            bytes: done as ssize_t,
            error: if done < count { error } else { None },
        }
    }

    // Synthetic fill has no real continuation to retain. Close an open frame
    // with format silence before writing more silence; this path is used only
    // after recovery deliberately chose synthetic audio over continuity.
    fn realign_with_silence(&mut self) -> Result<(), Option<Errno>> {
        if self.frame_off != 0 {
            let need = self.stride - self.frame_off;
            let silence = self.silence_buffer();
            let result = self.write_exact(&silence[..need as usize]);
            if self.frame_off != 0 {
                return Err(result.error);
            }
        }
        Ok(())
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
            // a future path reaches it with stale bookkeeping.
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
    fn write_exact(&mut self, buf: &[u8]) -> PlaybackWrite {
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
            let ts = libc::timespec {
                tv_sec: 0,
                tv_nsec: 2_000,
            };
            unsafe { libc::nanosleep(&ts, std::ptr::null_mut()) };
        }
        PlaybackWrite {
            bytes: done as ssize_t,
            error: if done < count { error } else { None },
        }
    }

    fn write_buffered(&mut self, buf: &[u8]) -> PlaybackWrite {
        let count = buf.len() as u32;
        if self.state == DspState::Setup {
            self.state = DspState::Running;
        }
        assert_eq!(self.state, DspState::Running);

        #[cfg(debug_assertions)]
        let space = ospace_in_bytes(self.raw_fd()) as usize;
        #[cfg(debug_assertions)]
        let delay = odelay(self.raw_fd());

        let nbytes = unsafe { libc::write(self.raw_fd(), buf.as_ptr().cast(), count as size_t) };
        let error = (nbytes < 0).then(Errno::last);
        if nbytes > 0 {
            // frame phase of the stream: every accepted byte counts, whoever wrote it
            self.frame_off = (self.frame_off + nbytes as u32) % self.stride.max(1);
        }

        #[cfg(debug_assertions)]
        {
            let now = now_ns_libc();
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

        PlaybackWrite {
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
        if self.realign_with_silence().is_err() {
            return;
        }
        // whole frames only: callers derive `count` from byte-granular ioctls
        // (odelay through a vchan can sit mid-frame), and a split frame turns
        // every later sample into static
        count -= count % self.stride.max(1);
        let silence = self.silence_buffer();
        // Chunk from the static silence buffer (`count` can exceed its length).
        // The fd is O_NONBLOCK, so a
        // short write or EAGAIN is normal; prime best-effort rather than asserting and
        // panicking out of the `extern "C"` callback (which aborts the process).
        // An early break can leave a frame split; frame_off records it. A
        // later real write completes it from retained audio, while another
        // synthetic fill closes it with the format's silence value.
        while count > 0 {
            let chunk = count.min(silence.len() as u32);
            let result = self.write_buffered(&silence[..chunk as usize]);
            if result.bytes < 0 {
                if let Some(errno) = result.error.filter(|errno| *errno != Errno::EAGAIN) {
                    // EAGAIN is just a full buffer; surface anything else
                    eprintln!("{}: write_silence: {}", self.path, errno);
                }
                break;
            }
            if result.bytes == 0 {
                break;
            }
            count -= result.bytes as u32;
        }
    }

    pub(crate) fn odelay(&self) -> u32 {
        assert_eq!(self.state, DspState::Running);
        odelay(self.raw_fd()).max(0) as u32
    }

    /// The fragment size the driver actually granted (may differ from what
    /// SETFRAGMENT asked for; some drivers force a fixed period).
    pub(crate) fn blocksize(&self) -> u32 {
        blocksize(self.raw_fd()).max(0) as u32
    }

    pub(crate) fn underruns(&self) -> u32 {
        assert_eq!(self.state, DspState::Running);
        // Timer-driven and follower fallback. Enriched driver wakes consume
        // the xrun count from the same kevent snapshot as their queued fill.
        get_error(self.raw_fd()).play_underruns.max(0) as u32
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
            hw_quantum_ns: 0,
            // The test constructor takes ownership of this pipe endpoint.
            fd: Some(unsafe { LibcFd::from_raw(fd) }),
            state: DspState::Setup,
            needs_trigger: false,
            pause_shadowed: false,
            stride,
            silence_byte: 0,
            frame_off: 0,
            #[cfg(debug_assertions)]
            prev_ns: 0,
        }
    }
}

#[cfg(test)]
mod playback_tests {
    use crate::oss::test_util::{drain, pattern, pipe_pair};

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
        // A pipe rejects the OSS format ioctl, but configure stores the
        // negotiated frame geometry and silence byte before issuing it.
        assert!(
            dsp.configure(super::AFMT_U8, 2, 48_000, 0x80, None)
                .is_err()
        );
        assert_eq!(dsp.stride, 2);
        assert_eq!(dsp.silence_byte, 0x80);
        dsp.write_silence(8);
        assert_eq!(drain(r), vec![0x80; 8]);

        // A synthetic fill after a short prior write repairs the open frame
        // with biased silence; 0x00 here would be a full-scale U8 sample.
        dsp.frame_off = 1;
        dsp.write_silence(0);
        assert_eq!(drain(r), vec![0x80]);
        assert_eq!(dsp.frame_off, 0);

        let data = [0x91, 0x92];
        assert_eq!(dsp.write(&data).bytes, data.len() as isize);
        assert_eq!(drain(r), data);
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
        let total_fill = crate::oss::test_util::fill_pipe(w);
        crate::oss::test_util::free_space(r, 2046);

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

        let total_fill = crate::oss::test_util::fill_pipe(w);
        crate::oss::test_util::free_space(r, 2046);
        let old = pattern(4096, 1);
        assert_eq!(dsp.write(&old).bytes, 2046);
        assert_eq!(dsp.frame_off, 6);
        assert_eq!(drain(r).len(), total_fill);

        // A buffer-pool replacement abandons old[2046..]. Close that frame
        // with silence before bytes from the new pool reach the device.
        assert!(!dsp.end_buffer_sequence());
        assert_eq!(dsp.frame_off, 0);

        let new = pattern(4096, 2);
        assert_eq!(dsp.write(&new).bytes, new.len() as isize);
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
    use crate::oss::test_util::{pattern, pipe_pair};
    // capture mirror image: a read that lands mid-frame must hide the torn
    // frame's head and discard its tail, so every returned buffer starts on a
    // frame boundary
    #[test]
    fn read_hides_torn_frame_and_realigns() {
        let (r, w) = pipe_pair(false, false);
        let mut dsp = super::Dsp::test_on_fd(r, 8);
        let s = pattern(2056, 3);
        assert_eq!(unsafe { libc::write(w, s.as_ptr().cast(), 2046) }, 2046);

        // 2046 available < 4096 requested: the pipe returns it all, mid-frame
        let mut buf = vec![0u8; 4096];
        let n = dsp.read(&mut buf[..4096]);
        assert_eq!(n, 2040);
        assert_eq!(&buf[..2040], &s[..2040]);
        assert_eq!(dsp.skip, 2);

        // the stream continues; the torn frame's tail is skipped and the next
        // buffer starts exactly on the following frame boundary
        assert_eq!(
            unsafe { libc::write(w, s.as_ptr().add(2046).cast(), 10) },
            10
        );
        let n = dsp.read(&mut buf[..8]);
        assert_eq!(n, 8);
        assert_eq!(&buf[..8], &s[2048..2056]);
        assert_eq!(dsp.skip, 0);
        unsafe { libc::close(w) };
    }
}
