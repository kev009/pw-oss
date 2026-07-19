use libc::sysctlbyname;
use nix::errno::Errno;
use std::ffi::{CStr, CString};
use std::os::raw::{c_int, c_ulong, c_void};

// slice::from_raw_parts requires the byte size to fit in isize even when the
// host claims the backing allocation is valid.
pub(crate) fn raw_slice_len_ok<T>(len: usize) -> bool {
    let size = std::mem::size_of::<T>();
    size == 0 || len <= (isize::MAX as usize) / size
}

/// An owned libc descriptor closed with `libc::close`.
pub(crate) struct LibcFd(c_int);

impl LibcFd {
    pub(crate) fn open(path: &CStr, flags: c_int) -> Option<Self> {
        let fd = unsafe { libc::open(path.as_ptr(), flags | libc::O_CLOEXEC) };
        (fd != -1).then(|| Self(fd))
    }

    /// Take ownership of an existing descriptor.
    ///
    /// # Safety
    /// `fd` must be open and exclusively transferred to the returned owner.
    #[cfg(test)]
    pub(crate) unsafe fn from_raw(fd: c_int) -> Self {
        assert!(fd >= 0);
        Self(fd)
    }

    pub(crate) fn raw(&self) -> c_int {
        self.0
    }
}

impl Drop for LibcFd {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.0);
        }
    }
}

/// A C-compatible plain-data type that an ioctl may initialize byte-for-byte.
///
/// # Safety
/// Implementors must be `Copy`, contain no references or drop state, accept
/// the all-zero value and every bit pattern the kernel can return, and have
/// the exact C layout encoded by the corresponding ioctl request.
pub(crate) unsafe trait IoctlPod: Copy {}

unsafe impl IoctlPod for c_int {}

pub(crate) fn ioctl_zeroed<T: IoctlPod>() -> T {
    // IoctlPod requires the all-zero value to be valid.
    unsafe { std::mem::zeroed() }
}

pub(crate) fn ioctl_int(fd: c_int, req: c_ulong, value: c_int) -> Option<c_int> {
    unsafe { ioctl_value(fd, req, value) }
}

/// Pass an initialized POD value through an ioctl that may update it.
///
/// # Safety
/// `req` must address exactly `T` and may not retain the pointer.
pub(crate) unsafe fn ioctl_value<T: IoctlPod>(fd: c_int, req: c_ulong, mut value: T) -> Option<T> {
    (unsafe { libc::ioctl(fd, req, &mut value) } != -1).then_some(value)
}

/// Read a POD value fully initialized by an ioctl.
///
/// # Safety
/// `req` must address exactly `T`, fully initialize it on success, and not
/// retain the pointer.
pub(crate) unsafe fn ioctl_read<T: IoctlPod>(fd: c_int, req: c_ulong) -> Option<T> {
    let mut value = std::mem::MaybeUninit::<T>::uninit();
    if unsafe { libc::ioctl(fd, req, value.as_mut_ptr()) } == -1 {
        None
    } else {
        Some(unsafe { value.assume_init() })
    }
}

// the shared read-only sysctlbyname shape (no new value): `buf` may be null
// for a size probe, `len` is in/out. Callers pass a `buf` valid for `len`
// bytes (or null).
unsafe fn sysctl_read(name: &CStr, buf: *mut c_void, len: &mut usize) -> Result<(), Errno> {
    if unsafe { sysctlbyname(name.as_ptr(), buf, len, std::ptr::null(), 0) } == -1 {
        return Err(Errno::last());
    }
    Ok(())
}

// a NUL-terminated sysctl name
pub(crate) struct SysctlName(CString);

impl From<&str> for SysctlName {
    fn from(str: &str) -> Self {
        SysctlName(CString::new(str).unwrap())
    }
}

impl From<String> for SysctlName {
    fn from(str: String) -> Self {
        SysctlName(CString::new(str).unwrap())
    }
}

pub(crate) struct SysctlReader {
    scratch_buffer: Vec<u8>,
}

impl SysctlReader {
    pub(crate) fn new() -> Self {
        Self {
            scratch_buffer: Vec::with_capacity(32),
        }
    }

    pub(crate) fn read_string<T: Into<SysctlName>>(
        &mut self,
        name: T,
        max_len: usize,
    ) -> Result<String, Errno> {
        let SysctlName(name) = name.into();

        let mut len = 0;
        unsafe { sysctl_read(&name, std::ptr::null_mut(), &mut len) }?;

        if len > max_len {
            return Err(Errno::ENOMEM);
        }

        if len == 0 {
            return Ok("".to_string());
        }

        self.scratch_buffer.resize(len, 0);
        unsafe { sysctl_read(&name, self.scratch_buffer.as_mut_ptr().cast(), &mut len) }?;

        // classic string sysctls (e.g. kern.ostype) count the terminating NUL
        // in the returned length; device-tree ones don't - trim either way, or
        // the NUL poisons map keys and C-string conversions downstream
        let mut bytes = &self.scratch_buffer[0..len];
        while let [head @ .., 0] = bytes {
            bytes = head;
        }
        Ok(String::from_utf8_lossy(bytes).to_string())
    }

    pub(crate) fn read_u32<T: Into<SysctlName>>(&self, name: T) -> Result<u32, Errno> {
        let SysctlName(name) = name.into();
        let mut value: u32 = 0;
        let mut len = std::mem::size_of::<u32>();
        unsafe { sysctl_read(&name, (&mut value as *mut u32).cast(), &mut len) }?;
        Ok(value)
    }
}

use std::os::fd::AsRawFd;
use std::os::fd::RawFd;
use uds::UnixSeqpacketConn;

pub(crate) struct DevdSocket {
    socket: UnixSeqpacketConn,
    buffer: Vec<u8>,
}

impl DevdSocket {
    pub(crate) fn open() -> Result<Self, std::io::Error> {
        let socket = UnixSeqpacketConn::connect("/var/run/devd.seqpacket.pipe")?;
        let buffer = [0; 8192 /* DEVCTL_MAXBUF */].to_vec();
        Ok(Self { socket, buffer })
    }

    pub(crate) fn fd(&self) -> RawFd {
        self.socket.as_raw_fd()
    }

    // false when the connection is dead (EOF or error): the fd stays readable
    // forever then, and the caller must deregister it or the loop busy-spins
    pub(crate) fn read_event(&mut self, mut apply: impl FnMut(&str)) -> bool {
        match self.socket.recv(&mut self.buffer) {
            Ok(0) => false, // EOF: devd went away (e.g. service devd restart)
            Ok(len) => {
                assert!(len <= self.buffer.len());
                // devd events should be ASCII, but don't abort on a stray byte
                apply(&String::from_utf8_lossy(&self.buffer[..len]));
                true
            }
            Err(err) => matches!(
                err.kind(),
                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted
            ),
        }
    }
}

// sys/dev/sound/pcm/matrix.h interleave order; note 5.1/7.1 put FC/LF after
// the rears, unlike WAV/ALSA
// hand-formatted: one line per speaker pair keeps the interleave order legible
#[rustfmt::skip]
pub(crate) fn channel_positions(channels: u32) -> Option<&'static [u32]> {
    use libspa::sys::*;
    static C1: [u32; 1] = [SPA_AUDIO_CHANNEL_MONO];
    static C2: [u32; 2] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR];
    static C4: [u32; 4] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                           SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR];
    static C6: [u32; 6] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                           SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR,
                           SPA_AUDIO_CHANNEL_FC, SPA_AUDIO_CHANNEL_LFE];
    static C8: [u32; 8] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR,
                           SPA_AUDIO_CHANNEL_RL, SPA_AUDIO_CHANNEL_RR,
                           SPA_AUDIO_CHANNEL_FC, SPA_AUDIO_CHANNEL_LFE,
                           SPA_AUDIO_CHANNEL_SL, SPA_AUDIO_CHANNEL_SR];
    match channels {
        1 => Some(&C1),
        2 => Some(&C2),
        4 => Some(&C4),
        6 => Some(&C6),
        8 => Some(&C8),
        _ => None
    }
}

// (OSS AFMT, SPA audio format, bytes per sample) triples we can produce,
// best first; the single source of truth for the format surface - EnumFormat,
// the negotiation snap and the per-config stride all derive from it
// (hand-formatted: one triple per line keeps the mapping scannable)
#[rustfmt::skip]
pub(crate) const FORMAT_MAP: [(u32, u32, u32); 4] = [
    (crate::sound::AFMT_S32_LE, libspa::sys::SPA_AUDIO_FORMAT_S32_LE, 4),
    (crate::sound::AFMT_S32_BE, libspa::sys::SPA_AUDIO_FORMAT_S32_BE, 4),
    (crate::sound::AFMT_S16_LE, libspa::sys::SPA_AUDIO_FORMAT_S16_LE, 2),
    (crate::sound::AFMT_S16_BE, libspa::sys::SPA_AUDIO_FORMAT_S16_BE, 2)
];

// the (OSS AFMT, bytes per sample) behind a SPA audio format; None for
// anything outside the map (rejected at negotiation)
pub(crate) fn oss_format_info(spa_format: u32) -> Option<(u32, u32)> {
    FORMAT_MAP
        .iter()
        .find(|(_, f, _)| *f == spa_format)
        .map(|(m, _, b)| (*m, *b))
}

// the formats a device gets offered: native ones when any exist, all of ours
// otherwise (the kernel feeder converts), nothing on a convertless device
// without a native match (bitperfect has no feeder; a snap-and-mismatch
// would just fail negotiation)
fn offered_formats(caps: &crate::sound::DspCaps) -> Vec<u32> {
    let native = FORMAT_MAP
        .iter()
        .filter(|(m, _, _)| caps.formats & m != 0)
        .map(|(_, f, _)| *f)
        .collect::<Vec<_>>();
    if !native.is_empty() {
        native
    } else if caps.convertless {
        vec![]
    } else {
        FORMAT_MAP.iter().map(|(_, f, _)| *f).collect()
    }
}

// Snap a requested raw format onto the advertised caps for callers that pass
// SPA_NODE_PARAM_FLAG_NEAREST - audioadapter always negotiates the follower
// that way (audioadapter.c:758, :1059) - mirroring alsa's set_*_near handling
// (alsa-pcm.c:2364, :2388). Returns true when anything was adjusted; the
// caller then returns 1 (alsa-pcm.c:2548) so the adapter re-reads our Format
// param for the actual values (audioadapter.c:596).
pub(crate) fn snap_raw_to_caps(
    caps: &crate::sound::DspCaps,
    raw: &mut libspa::sys::spa_audio_info_raw,
) -> bool {
    let mut changed = false;

    let offered = offered_formats(caps);
    if !offered.contains(&raw.format) {
        if let Some(&best) = offered.first() {
            raw.format = best;
            changed = true;
        } // else: convertless with no native format; the exact path rejects it
    }

    // the position array is 64 wide; garbage caps must not push past it
    let channels = raw
        .channels
        .clamp(caps.min_channels, caps.max_channels)
        .min(libspa::sys::SPA_AUDIO_MAX_CHANNELS);
    if channels != raw.channels {
        raw.channels = channels;
        // the requested layout no longer applies; hand out the kernel interleave
        // order (or AUX slots), same as EnumFormat
        match channel_positions(channels) {
            Some(positions) => {
                for (slot, &p) in raw.position.iter_mut().zip(positions.iter()) {
                    *slot = p;
                }
            }
            None => {
                for (i, slot) in raw.position.iter_mut().take(channels as usize).enumerate() {
                    *slot = libspa::sys::SPA_AUDIO_CHANNEL_AUX0 + i as u32;
                }
            }
        }
        changed = true;
    }

    let rate = if !caps.rates.is_empty() {
        // discrete native rates (exclusive devices): nearest wins
        *caps
            .rates
            .iter()
            .min_by_key(|r| r.abs_diff(raw.rate))
            .unwrap()
    } else {
        raw.rate.clamp(caps.min_rate, caps.max_rate)
    };
    if rate != raw.rate {
        raw.rate = rate;
        changed = true;
    }

    changed
}

// The offered channel widths, in EnumFormat order: standard widths in range,
// then the native max if missing (AUX for non-std), with 2 pinned first (host
// default) and last (pulse-server falls back to the LAST EnumFormat map when
// Format is gone; HW routes always report 2ch volume, so a last width of
// 1/max would thrash cvolume.channels). That can mean two entries for stereo.
fn enum_format_widths(min_channels: u32, max_channels: u32) -> Vec<u32> {
    let mut counts = [2u32, 4, 6, 8, 1]
        .iter()
        .copied()
        .filter(|c| *c >= min_channels && *c <= max_channels)
        .collect::<Vec<_>>();
    if !counts.contains(&max_channels) {
        counts.push(max_channels);
    }
    // pin 2 first and last; no-op when already only [2]
    if min_channels <= 2 && max_channels >= 2 {
        counts.retain(|c| *c != 2);
        counts.insert(0, 2);
        if counts.last() != Some(&2) {
            counts.push(2);
        }
    }
    counts
}

// Serialize a Value tree into a standalone pod byte buffer. Infallible in
// practice: the output is in-memory and every Value we build is
// serializable, so an error here is a programming bug.
pub(crate) fn serialize_pod(value: &libspa::pod::Value) -> Vec<u8> {
    use libspa::pod::serialize::PodSerializer;
    PodSerializer::serialize(std::io::Cursor::new(Vec::new()), value)
        .expect("serializing a pod Value into a Vec cannot fail")
        .0
        .into_inner()
}

// a flag-less object property (the common case)
pub(crate) fn pod_prop(key: u32, value: libspa::pod::Value) -> libspa::pod::Property {
    libspa::pod::Property {
        key,
        flags: libspa::pod::PropertyFlags::empty(),
        value,
    }
}

// an Int range choice (default, min, max)
fn pod_int_range(default: i32, min: i32, max: i32) -> libspa::pod::Value {
    use libspa::pod::{ChoiceValue, Value};
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags};
    Value::Choice(ChoiceValue::Int(Choice(
        ChoiceFlags::empty(),
        ChoiceEnum::Range { default, min, max },
    )))
}

// One EnumFormat pod per offered channel width (enum_format_widths order),
// positions from the kernel interleave. None when `index` is past the last
// result.
pub(crate) fn build_enum_format_info(caps: &crate::sound::DspCaps, index: u32) -> Option<Vec<u8>> {
    use libspa::pod::{ChoiceValue, Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    // formats supported by both us and the device, best first
    let formats = offered_formats(caps);
    if formats.is_empty() {
        return None;
    }

    let counts = enum_format_widths(caps.min_channels, caps.max_channels);
    let &channels = counts.get(index as usize)?;

    let format = if let [format] = formats[..] {
        Value::Id(Id(format))
    } else {
        Value::Choice(ChoiceValue::Id(Choice(
            ChoiceFlags::empty(),
            ChoiceEnum::Enum {
                default: Id(formats[0]),
                alternatives: formats[1..].iter().map(|f| Id(*f)).collect(),
            },
        )))
    };

    let rate = if caps.rates.len() > 1 {
        // discrete native rates (exclusive devices); a range would admit
        // in-between rates the hardware can't run (see sound::native_rates)
        let target = caps.preferred_rate.unwrap_or(48000);
        let default = *caps
            .rates
            .iter()
            .min_by_key(|r| r.abs_diff(target))
            .unwrap();
        Value::Choice(ChoiceValue::Int(Choice(
            ChoiceFlags::empty(),
            ChoiceEnum::Enum {
                default: default as i32,
                alternatives: caps.rates.iter().map(|r| *r as i32).collect(),
            },
        )))
    } else if let [rate] = caps.rates[..] {
        Value::Int(rate as i32)
    } else if caps.min_rate == caps.max_rate {
        Value::Int(caps.min_rate as i32)
    } else {
        pod_int_range(
            caps.preferred_rate
                .unwrap_or(48000)
                .clamp(caps.min_rate, caps.max_rate) as i32,
            caps.min_rate as i32,
            caps.max_rate as i32,
        )
    };

    let positions: Vec<Id> = match channel_positions(channels) {
        Some(positions) => positions.iter().map(|&p| Id(p)).collect(),
        None => (0..channels)
            .map(|i| Id(SPA_AUDIO_CHANNEL_AUX0 + i))
            .collect(),
    };

    Some(serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_Format,
        id: SPA_PARAM_EnumFormat,
        properties: vec![
            pod_prop(SPA_FORMAT_mediaType, Value::Id(Id(SPA_MEDIA_TYPE_audio))),
            pod_prop(
                SPA_FORMAT_mediaSubtype,
                Value::Id(Id(SPA_MEDIA_SUBTYPE_raw)),
            ),
            pod_prop(SPA_FORMAT_AUDIO_format, format),
            pod_prop(SPA_FORMAT_AUDIO_rate, rate),
            pod_prop(SPA_FORMAT_AUDIO_channels, Value::Int(channels as i32)),
            pod_prop(
                SPA_FORMAT_AUDIO_position,
                Value::ValueArray(ValueArray::Id(positions)),
            ),
        ],
    })))
}

pub(crate) fn build_buffers_info(stride: u32) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    // The point here is dataType = MemPtr: process() maps the buffer memory
    // directly, so a MemFd/DmaBuf block would be unusable.
    //
    // Capacity floors at two graph periods (2048 frames at the 1024-frame
    // reference quantum). The capture catch-up read (source.rs) drains a device
    // ring excursion by handing the graph MORE than one period in a cycle; a
    // one-period buffer clamps that read back to a period, so the ring stays
    // pinned at its ceiling and the kernel overruns every late cycle. Two
    // periods of *capacity* cost no latency - we still deliver one period per
    // cycle - it only widens the container so the drain can happen. The adapter
    // sizes the buffer to the graph quantum and clamps up to this floor, so the
    // headroom is present at the common quanta (a quantum coarser than the floor
    // needs the ring-quantum cap in node.rs to stay glitch-free anyway).
    let floor = (2048 * stride) as i32;
    let max = (16384 * stride) as i32;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamBuffers,
        id: SPA_PARAM_Buffers,
        properties: vec![
            pod_prop(SPA_PARAM_BUFFERS_buffers, pod_int_range(2, 1, 32)),
            pod_prop(SPA_PARAM_BUFFERS_blocks, Value::Int(1)),
            pod_prop(SPA_PARAM_BUFFERS_size, pod_int_range(floor, floor, max)),
            pod_prop(SPA_PARAM_BUFFERS_stride, Value::Int(stride as i32)),
            pod_prop(SPA_PARAM_BUFFERS_align, Value::Int(16)),
            pod_prop(
                SPA_PARAM_BUFFERS_dataType,
                Value::Int(1i32 << SPA_DATA_MemPtr),
            ),
        ],
    }))
}

// Marks a captured value as allowed to cross onto the loop thread inside an
// invoke closure. For host pointers (io areas, the callback table, buffer
// arrays) whose validity the SPA contract ties to the node's lifetime rather
// than to a Rust Send impl; the loop invoke is the serialization point.
// The field is private on purpose: closures capture fields precisely, so a
// public .0 (or a destructuring pattern of it) would be captured
// field-by-field, skipping the wrapper's Send; into_inner takes self whole,
// forcing whole-value capture.
pub(crate) struct SendWrap<T>(T);
unsafe impl<T> Send for SendWrap<T> {}
impl<T> SendWrap<T> {
    /// Allow `v` to cross onto the loop thread.
    ///
    /// # Safety
    /// The caller asserts that this particular value stays valid and usable
    /// from the loop thread for as long as it is used there. For host
    /// pointers that is the SPA lifetime contract (the host keeps callback
    /// tables, io areas and buffer arrays valid while they are set); the
    /// blocking loop invoke is the serialization point.
    pub(crate) unsafe fn new(v: T) -> Self {
        SendWrap(v)
    }
    pub(crate) fn into_inner(self) -> T {
        self.0
    }
}

// Run `f` on the data loop and wait for it; serializes main-thread
// reconfiguration against process()/on_timeout() (runs inline when already on
// the loop thread). The closure and target cross a thread boundary; callers
// only capture raw pointers and plain data (F: Send; the blocking call keeps
// stack borrows sound, so no 'static is needed). Returns false when the
// invoke failed or the closure panicked - the closure then may not have run.
pub(crate) unsafe fn block_on_loop<T, F: FnOnce(&mut T) + Send>(
    loop_: &crate::spa::Loop,
    target: *mut T,
    f: F,
) -> bool {
    struct Ctx<T, F> {
        target: *mut T,
        f: Option<F>,
    }

    unsafe extern "C" fn trampoline<T, F: FnOnce(&mut T) + Send>(
        _loop: *mut libspa::sys::spa_loop,
        _async: bool,
        _seq: u32,
        _data: *const std::os::raw::c_void,
        _size: usize,
        user_data: *mut std::os::raw::c_void,
    ) -> std::os::raw::c_int {
        // user_data is the &mut Ctx the blocking invoke below keeps alive
        let ctx = unsafe { user_data.cast::<Ctx<T, F>>().as_mut() }
            .expect("user_data is not supposed to be null");
        let f = ctx.f.take().expect("the invoked function only runs once");
        let target = ctx.target;
        // a panic must not unwind into the C loop (that aborts the daemon)
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            // target validity is the caller's contract on block_on_loop
            f(unsafe { target.as_mut() }.expect("target is not supposed to be null"));
        }));
        if ok.is_err() { -libc::ECANCELED } else { 0 }
    }

    // blocking, so `ctx` outlives the call
    let mut ctx = Ctx { target, f: Some(f) };
    let err = unsafe {
        loop_.invoke(
            Some(trampoline::<T, F>),
            0,
            std::ptr::null(),
            0,
            true,
            &mut ctx as *mut _ as *mut std::os::raw::c_void,
        )
    };
    err >= 0
}

pub(crate) fn latency_info_default(
    direction: libspa::sys::spa_direction,
) -> libspa::sys::spa_latency_info {
    libspa::sys::spa_latency_info {
        direction,
        min_quantum: 0.0,
        max_quantum: 0.0,
        min_rate: 0,
        max_rate: 0,
        min_ns: 0,
        max_ns: 0,
    }
}

// spa_latency_parse is static inline C, so reimplemented here; takes the
// already-deserialized Value (the extern fns call deserialize_pod at the
// FFI boundary), so the parse itself is safe code
pub(crate) fn parse_latency_info(
    value: Option<&libspa::pod::Value>,
) -> Option<libspa::sys::spa_latency_info> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    match value {
        Some(Value::Object(Object {
            type_, properties, ..
        })) if *type_ == SPA_TYPE_OBJECT_ParamLatency => {
            let mut info = latency_info_default(SPA_DIRECTION_INPUT);
            for p in properties {
                #[allow(non_upper_case_globals)]
                match (p.key, &p.value) {
                    (SPA_PARAM_LATENCY_direction, Value::Id(v)) => info.direction = v.0 & 1,
                    (SPA_PARAM_LATENCY_minQuantum, Value::Float(v)) => info.min_quantum = *v,
                    (SPA_PARAM_LATENCY_maxQuantum, Value::Float(v)) => info.max_quantum = *v,
                    (SPA_PARAM_LATENCY_minRate, Value::Int(v)) => info.min_rate = *v,
                    (SPA_PARAM_LATENCY_maxRate, Value::Int(v)) => info.max_rate = *v,
                    (SPA_PARAM_LATENCY_minNs, Value::Long(v)) => info.min_ns = *v,
                    (SPA_PARAM_LATENCY_maxNs, Value::Long(v)) => info.max_ns = *v,
                    _ => (),
                }
            }
            Some(info)
        }
        _ => None,
    }
}

// spa_latency_build is static inline C, so reimplemented here
pub(crate) fn build_latency_info(info: &libspa::sys::spa_latency_info) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;
    use libspa::utils::Id;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamLatency,
        id: SPA_PARAM_Latency,
        properties: vec![
            pod_prop(SPA_PARAM_LATENCY_direction, Value::Id(Id(info.direction))),
            pod_prop(SPA_PARAM_LATENCY_minQuantum, Value::Float(info.min_quantum)),
            pod_prop(SPA_PARAM_LATENCY_maxQuantum, Value::Float(info.max_quantum)),
            pod_prop(SPA_PARAM_LATENCY_minRate, Value::Int(info.min_rate)),
            pod_prop(SPA_PARAM_LATENCY_maxRate, Value::Int(info.max_rate)),
            pod_prop(SPA_PARAM_LATENCY_minNs, Value::Long(info.min_ns)),
            pod_prop(SPA_PARAM_LATENCY_maxNs, Value::Long(info.max_ns)),
        ],
    }))
}

pub(crate) fn process_latency_default() -> libspa::sys::spa_process_latency_info {
    libspa::sys::spa_process_latency_info {
        quantum: 0.0,
        rate: 0,
        ns: 0,
    }
}

// spa_process_latency_parse is static inline C, so reimplemented here;
// takes the already-deserialized Value (see parse_latency_info)
pub(crate) fn parse_process_latency_info(
    value: Option<&libspa::pod::Value>,
) -> Option<libspa::sys::spa_process_latency_info> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    match value {
        Some(Value::Object(Object {
            type_, properties, ..
        })) if *type_ == SPA_TYPE_OBJECT_ParamProcessLatency => {
            let mut info = process_latency_default();
            for p in properties {
                #[allow(non_upper_case_globals)]
                match (p.key, &p.value) {
                    (SPA_PARAM_PROCESS_LATENCY_quantum, Value::Float(v)) => info.quantum = *v,
                    (SPA_PARAM_PROCESS_LATENCY_rate, Value::Int(v)) => info.rate = *v,
                    (SPA_PARAM_PROCESS_LATENCY_ns, Value::Long(v)) => info.ns = *v,
                    _ => (),
                }
            }
            Some(info)
        }
        _ => None,
    }
}

// spa_process_latency_build is static inline C, so reimplemented here
pub(crate) fn build_process_latency_info(info: &libspa::sys::spa_process_latency_info) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamProcessLatency,
        id: SPA_PARAM_ProcessLatency,
        properties: vec![
            pod_prop(
                SPA_PARAM_PROCESS_LATENCY_quantum,
                Value::Float(info.quantum),
            ),
            pod_prop(SPA_PARAM_PROCESS_LATENCY_rate, Value::Int(info.rate)),
            pod_prop(SPA_PARAM_PROCESS_LATENCY_ns, Value::Long(info.ns)),
        ],
    }))
}

// spa_process_latency_info_add is static inline C, so reimplemented here
pub(crate) fn process_latency_info_add(
    process: &libspa::sys::spa_process_latency_info,
    info: &mut libspa::sys::spa_latency_info,
) {
    info.min_quantum += process.quantum;
    info.max_quantum += process.quantum;
    info.min_rate += process.rate;
    info.max_rate += process.rate;
    info.min_ns += process.ns;
    info.max_ns += process.ns;
}

pub(crate) fn build_latency_offset_prop_info() -> Vec<u8> {
    use libspa::pod::{ChoiceValue, Object, Value};
    use libspa::sys::*;
    use libspa::utils::{Choice, ChoiceEnum, ChoiceFlags, Id};

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_PropInfo,
        id: SPA_PARAM_PropInfo,
        properties: vec![
            pod_prop(SPA_PROP_INFO_id, Value::Id(Id(SPA_PROP_latencyOffsetNsec))),
            pod_prop(
                SPA_PROP_INFO_description,
                Value::String("Latency offset (ns)".to_string()),
            ),
            pod_prop(
                SPA_PROP_INFO_type,
                Value::Choice(ChoiceValue::Long(Choice(
                    ChoiceFlags::empty(),
                    ChoiceEnum::Range {
                        default: 0,
                        min: 0,
                        max: 2 * SPA_NSEC_PER_SEC as i64,
                    },
                ))),
            ),
        ],
    }))
}

pub(crate) fn build_latency_offset_props(ns: i64, params: &[(&str, u32)]) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    let mut properties = vec![pod_prop(SPA_PROP_latencyOffsetNsec, Value::Long(ns))];

    // custom key/value props (oss.delay, oss.fragment) ride in the params struct
    if !params.is_empty() {
        let fields = params
            .iter()
            .flat_map(|(key, value)| [Value::String((*key).to_string()), Value::Int(*value as i32)])
            .collect();
        properties.push(pod_prop(SPA_PROP_params, Value::Struct(fields)));
    }

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_Props,
        id: SPA_PARAM_Props,
        properties,
    }))
}

// PropInfo for a custom u32 tunable carried in the Props params struct; the
// advertised default is the CURRENT (effective) value, like the ALSA plugin
pub(crate) fn build_params_prop_info(
    name: &str,
    description: &str,
    current: u32,
    max: u32,
) -> Vec<u8> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_PropInfo,
        id: SPA_PARAM_PropInfo,
        properties: vec![
            pod_prop(SPA_PROP_INFO_name, Value::String(name.to_string())),
            pod_prop(
                SPA_PROP_INFO_description,
                Value::String(description.to_string()),
            ),
            pod_prop(
                SPA_PROP_INFO_type,
                pod_int_range(current as i32, 0, max as i32),
            ),
            // settable through the Props params struct
            pod_prop(SPA_PROP_INFO_params, Value::Bool(true)),
        ],
    }))
}

// identify our device clock (spa_io_clock.name) so consumers can tell whether
// two nodes tick from the same hardware
pub(crate) fn set_clock_name(clock: &mut libspa::sys::spa_io_clock, name: &std::ffi::CStr) {
    // at most 63 bytes plus the forced terminator fit the 64-byte name field
    let bytes = name.to_bytes_with_nul();
    for (dst, &src) in clock.name.iter_mut().take(63).zip(bytes.iter()) {
        *dst = src as _;
    }
    clock.name[63] = 0;
}

// does the driver's clock in `position` carry our clock name? (then we tick
// from the same device and rate matching is pointless - ALSA does the same
// clock-name comparison)
pub(crate) fn same_clock(position: &libspa::sys::spa_io_position, name: &std::ffi::CStr) -> bool {
    let theirs = &position.clock.name;
    let ours = name.to_bytes();
    if ours.is_empty() || ours.len() >= theirs.len() || theirs[0] == 0 {
        return false;
    }
    for (i, &b) in ours.iter().enumerate() {
        if theirs[i] as u8 != b {
            return false;
        }
    }
    theirs[ours.len()] == 0
}

// one graph cycle expressed in device bytes; the device rate can differ from
// the graph rate (the adapter's resampler makes up the difference)
pub(crate) fn device_period_bytes(
    target_duration: u64,
    device_rate: u32,
    graph_rate: u32,
    stride: u32,
) -> u32 {
    if graph_rate == 0 {
        return 0;
    }
    // saturate: a corrupt duration must not wrap (or panic in debug builds)
    (target_duration.saturating_mul(device_rate as u64) / graph_rate as u64)
        .saturating_mul(stride as u64)
        .min(u32::MAX as u64) as u32
}

// a nanosecond interval (hardware drain quantum, elapsed time) expressed in
// device bytes; saturating and clamped - the inputs are device- or
// clock-provided and an overflow here would abort the data loop
pub(crate) fn ns_to_bytes(ns: u64, rate: u32, stride: u32) -> u32 {
    ((ns as u128)
        .saturating_mul(rate as u128)
        .saturating_mul(stride as u128)
        / 1_000_000_000)
        .min(u32::MAX as u128) as u32
}

// ns_to_bytes rounded up to a whole frame (the division floors: a 2048-byte
// hardware quantum reads as 2047); a saturated conversion stays saturated at
// the largest frame multiple instead of overflowing the round-up
pub(crate) fn ns_to_frame_bytes(ns: u64, rate: u32, stride: u32) -> u32 {
    let stride = stride.max(1);
    ns_to_bytes(ns, rate, stride)
        .checked_next_multiple_of(stride)
        .unwrap_or(u32::MAX - u32::MAX % stride)
}

// Deserialize a host-supplied pod without trusting it: libspa's
// deserializer divides by a pod-declared child size (Choice pods) and
// pre-allocates from declared lengths, so a hostile pod can panic it -
// which must not unwind across our extern "C" boundaries.
pub(crate) unsafe fn deserialize_pod(
    param: *const libspa::sys::spa_pod,
) -> Option<libspa::pod::Value> {
    use libspa::pod::deserialize::PodDeserializer;
    let bytes = unsafe { libspa::pod::Pod::from_raw(param).as_bytes() };
    std::panic::catch_unwind(|| PodDeserializer::deserialize_any_from(bytes).ok())
        .ok()
        .flatten()
        // the parse remainder rode a lifetime fabricated from the raw pod;
        // every caller wants only the owned Value
        .map(|(_, value)| value)
}

// Test-only: parse a serialized pod back through the same PodDeserializer
// consumers run, insisting the buffer holds exactly one complete pod.
#[cfg(test)]
pub(crate) fn parse_back(pod: &[u8]) -> libspa::pod::Value {
    let (rest, value) = libspa::pod::deserialize::PodDeserializer::deserialize_any_from(pod)
        .expect("an advertised pod must deserialize");
    assert!(rest.is_empty(), "trailing bytes after the pod");
    value
}

// Queue an owned closure on the target loop. spa_loop.invoke may execute it
// inline when called from that loop, so callers must release reentrant state
// borrows first and real-time callers must not enqueue blocking work. A false
// return means the closure and its payload were dropped on the calling thread.
//
// # Safety
// The loop must outlive the queued item's execution: host loops come from
// the spa_support array and live for the plugin host's lifetime.
pub(crate) unsafe fn queue_task<F: FnOnce() + Send + 'static>(
    loop_: &crate::spa::Loop,
    f: F,
) -> bool {
    unsafe extern "C" fn trampoline<F: FnOnce() + Send + 'static>(
        _loop: *mut libspa::sys::spa_loop,
        _async: bool,
        _seq: u32,
        _data: *const std::os::raw::c_void,
        _size: usize,
        user_data: *mut std::os::raw::c_void,
    ) -> std::os::raw::c_int {
        // user_data is the Box::into_raw'd closure below; the loop runs each
        // queued item exactly once, so this is the sole owner
        let f = unsafe { Box::from_raw(user_data.cast::<F>()) };
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
        if ok.is_err() {
            eprintln!("freebsd-oss: panic in a queued loop task (swallowed)");
        }
        // never negative: an inline flush returns this value to the caller,
        // and a negative would make it free the closure a second time
        0
    }

    let ctx = Box::into_raw(Box::new(f));
    let err = unsafe {
        loop_.invoke(
            Some(trampoline::<F>),
            0,
            std::ptr::null(),
            0,
            false,
            ctx as *mut std::os::raw::c_void,
        )
    };
    if err < 0 {
        // a negative here uniquely means the item was never queued (the
        // trampoline never ran, so this is still the sole owner)
        drop(unsafe { Box::from_raw(ctx) });
        return false;
    }
    true
}

// at most one message a second from a per-cycle warn site, with a count of
// what went unsaid (ALSA uses spa_ratelimit for the same purpose)
pub(crate) struct RateLimit {
    last: u64,
    suppressed: u32,
}

impl Default for RateLimit {
    fn default() -> Self {
        Self::new()
    }
}

impl RateLimit {
    pub(crate) const fn new() -> Self {
        Self {
            last: 0,
            suppressed: 0,
        }
    }

    // Some(previously suppressed count) when the caller may log now
    pub(crate) fn check(&mut self, now: u64) -> Option<u32> {
        if now.saturating_sub(self.last) >= 1_000_000_000 {
            self.last = now;
            Some(std::mem::take(&mut self.suppressed))
        } else {
            self.suppressed += 1;
            None
        }
    }
}

// CLOCK_MONOTONIC through the host system vtable; None when the read fails.
// Fallible on purpose: every caller runs on the data loop under extern "C",
// where an assert would abort the whole daemon - each caller has a soft
// path (park the timer, reuse the previous stamp, skip a cycle).
pub(crate) fn try_now_ns(system: &crate::spa::System) -> Option<u64> {
    let mut now = libspa::sys::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now);
    if err < 0 {
        return None;
    }
    Some((now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64)
}

#[cfg(debug_assertions)]
pub(crate) fn now_ns_libc() -> u64 {
    let mut now = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
    assert!(err != -1);
    (now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64
}

pub(crate) fn spa_command_to_str(body: &libspa::sys::spa_pod_object_body) -> &'static str {
    use libspa::sys::*;
    #[allow(non_upper_case_globals)]
    match (body.type_, body.id) {
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start) => "SPA_NODE_COMMAND_Start",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => "SPA_NODE_COMMAND_Suspend",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => "SPA_NODE_COMMAND_Pause",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin) => "SPA_NODE_COMMAND_ParamBegin",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamEnd) => "SPA_NODE_COMMAND_ParamEnd",
        (SPA_TYPE_COMMAND_Node, _) => "SPA_NODE_COMMAND_???",
        _ => "???",
    }
}

#[cfg(test)]
mod tests;
