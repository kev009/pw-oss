use libc::sysctlbyname;
use nix::errno::Errno;
use std::ffi::{CStr, CString};
use std::os::raw::c_void;

// the shared read-only sysctlbyname shape (no new value): `buf` may be null
// for a size probe, `len` is in/out. Callers pass a `buf` valid for `len`
// bytes (or null).
unsafe fn sysctl_read(name: &CStr, buf: *mut c_void, len: &mut usize) -> Result<(), Errno> {
    if sysctlbyname(name.as_ptr(), buf, len, std::ptr::null(), 0) == -1 {
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

    pub(crate) fn read_u32<T: Into<SysctlName>>(&mut self, name: T) -> Result<u32, Errno> {
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

// (OSS AFMT, SPA audio format) pairs we can produce, best first; keep in
// sync with the per-direction parse_config/oss_format matches
// (hand-formatted: one pair per line keeps the mapping scannable)
#[rustfmt::skip]
pub(crate) const FORMAT_MAP: [(u32, u32); 4] = [
    (crate::sound::AFMT_S32_LE, libspa::sys::SPA_AUDIO_FORMAT_S32_LE),
    (crate::sound::AFMT_S32_BE, libspa::sys::SPA_AUDIO_FORMAT_S32_BE),
    (crate::sound::AFMT_S16_LE, libspa::sys::SPA_AUDIO_FORMAT_S16_LE),
    (crate::sound::AFMT_S16_BE, libspa::sys::SPA_AUDIO_FORMAT_S16_BE)
];

// the formats a device gets offered: native ones when any exist, all of ours
// otherwise (the kernel feeder converts), nothing on a convertless device
// without a native match (bitperfect has no feeder; a snap-and-mismatch
// would just fail negotiation)
fn offered_formats(caps: &crate::sound::DspCaps) -> Vec<u32> {
    let native = FORMAT_MAP
        .iter()
        .filter(|(m, _)| caps.formats & m != 0)
        .map(|(_, f)| *f)
        .collect::<Vec<_>>();
    if !native.is_empty() {
        native
    } else if caps.convertless {
        vec![]
    } else {
        FORMAT_MAP.iter().map(|(_, f)| *f).collect()
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

// One EnumFormat pod per offered channel width (enum_format_widths order),
// positions from the kernel interleave. Returns false when `index` is past
// the last result.
pub(crate) fn build_enum_format_info(
    b: &mut libspa::pod::builder::Builder,
    caps: &crate::sound::DspCaps,
    index: u32,
) -> Result<bool, rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
        use libspa::sys::*;

        // formats supported by both us and the device, best first
        let formats = offered_formats(caps);
        if formats.is_empty() {
            return Ok(false);
        }

        let counts = enum_format_widths(caps.min_channels, caps.max_channels);
        let Some(&channels) = counts.get(index as usize) else {
            return Ok(false);
        };

        let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
        let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(&mut outer, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat)?;

        b.add_prop(SPA_FORMAT_mediaType, 0)?;
        b.add_id(libspa::utils::Id(SPA_MEDIA_TYPE_audio))?;

        b.add_prop(SPA_FORMAT_mediaSubtype, 0)?;
        b.add_id(libspa::utils::Id(SPA_MEDIA_SUBTYPE_raw))?;

        b.add_prop(SPA_FORMAT_AUDIO_format, 0)?;
        if formats.len() == 1 {
            b.add_id(libspa::utils::Id(formats[0]))?;
        } else {
            b.push_choice(&mut inner, SPA_CHOICE_Enum, 0)?;
            for fmt in &formats {
                b.add_id(libspa::utils::Id(*fmt))?;
            }
            b.pop(inner.assume_init_mut());
        }

        b.add_prop(SPA_FORMAT_AUDIO_rate, 0)?;
        if caps.rates.len() > 1 {
            // discrete native rates (exclusive devices); a range would admit
            // in-between rates the hardware can't run (see sound::native_rates)
            let target = caps.preferred_rate.unwrap_or(48000);
            let default = *caps
                .rates
                .iter()
                .min_by_key(|r| r.abs_diff(target))
                .unwrap();
            b.push_choice(&mut inner, SPA_CHOICE_Enum, 0)?;
            b.add_int(default as i32)?;
            for rate in &caps.rates {
                b.add_int(*rate as i32)?;
            }
            b.pop(inner.assume_init_mut());
        } else if let [rate] = caps.rates[..] {
            b.add_int(rate as i32)?;
        } else if caps.min_rate == caps.max_rate {
            b.add_int(caps.min_rate as i32)?;
        } else {
            b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
            b.add_int(
                caps.preferred_rate
                    .unwrap_or(48000)
                    .clamp(caps.min_rate, caps.max_rate) as i32,
            )?;
            b.add_int(caps.min_rate as i32)?;
            b.add_int(caps.max_rate as i32)?;
            b.pop(inner.assume_init_mut());
        }

        b.add_prop(SPA_FORMAT_AUDIO_channels, 0)?;
        b.add_int(channels as i32)?;

        let aux_positions;
        let positions: &[u32] = match channel_positions(channels) {
            Some(positions) => positions,
            None => {
                aux_positions = (0..channels)
                    .map(|i| SPA_AUDIO_CHANNEL_AUX0 + i)
                    .collect::<Vec<u32>>();
                &aux_positions
            }
        };

        b.add_prop(SPA_FORMAT_AUDIO_position, 0)?;
        b.add_array(
            std::mem::size_of::<u32>() as u32,
            SPA_TYPE_Id,
            positions.len() as u32,
            positions.as_ptr().cast(),
        )?;

        b.pop(outer.assume_init_mut());

        Ok(true)
    }
}

pub(crate) fn build_buffers_info(
    b: &mut libspa::pod::builder::Builder,
    stride: u32,
) -> Result<(), rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
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
        let floor = 2048 * stride;
        let max = 16384 * stride;

        let mut obj = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
        let mut choice = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(&mut obj, SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers)?;

        b.add_prop(SPA_PARAM_BUFFERS_buffers, 0)?;
        b.push_choice(&mut choice, SPA_CHOICE_Range, 0)?;
        b.add_int(2)?;
        b.add_int(1)?;
        b.add_int(32)?; // default, min, max
        b.pop(choice.assume_init_mut());

        b.add_prop(SPA_PARAM_BUFFERS_blocks, 0)?;
        b.add_int(1)?;

        b.add_prop(SPA_PARAM_BUFFERS_size, 0)?;
        b.push_choice(&mut choice, SPA_CHOICE_Range, 0)?;
        b.add_int(floor as i32)?;
        b.add_int(floor as i32)?;
        b.add_int(max as i32)?; // default, min, max
        b.pop(choice.assume_init_mut());

        b.add_prop(SPA_PARAM_BUFFERS_stride, 0)?;
        b.add_int(stride as i32)?;

        b.add_prop(SPA_PARAM_BUFFERS_align, 0)?;
        b.add_int(16)?;

        b.add_prop(SPA_PARAM_BUFFERS_dataType, 0)?;
        b.add_int(1i32 << SPA_DATA_MemPtr)?;

        b.pop(obj.assume_init_mut());

        Ok(())
    }
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
    pub(crate) fn new(v: T) -> Self {
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
        let ctx = user_data
            .cast::<Ctx<T, F>>()
            .as_mut()
            .expect("user_data is not supposed to be null");
        let f = ctx.f.take().expect("the invoked function only runs once");
        let target = ctx.target;
        // a panic must not unwind into the C loop (that aborts the daemon)
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            f(target.as_mut().expect("target is not supposed to be null"));
        }));
        if ok.is_err() {
            -libc::ECANCELED
        } else {
            0
        }
    }

    // blocking, so `ctx` outlives the call
    let mut ctx = Ctx { target, f: Some(f) };
    let err = loop_.invoke(
        Some(trampoline::<T, F>),
        0,
        std::ptr::null(),
        0,
        true,
        &mut ctx as *mut _ as *mut std::os::raw::c_void,
    );
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

// spa_latency_parse is static inline C, so reimplemented here
pub(crate) unsafe fn parse_latency_info(
    param: *const libspa::sys::spa_pod,
) -> Option<libspa::sys::spa_latency_info> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    match crate::utils::deserialize_pod(param) {
        Some(Value::Object(Object {
            type_, properties, ..
        })) if type_ == SPA_TYPE_OBJECT_ParamLatency => {
            let mut info = latency_info_default(SPA_DIRECTION_INPUT);
            for p in properties {
                #[allow(non_upper_case_globals)]
                match (p.key, p.value) {
                    (SPA_PARAM_LATENCY_direction, Value::Id(v)) => info.direction = v.0 & 1,
                    (SPA_PARAM_LATENCY_minQuantum, Value::Float(v)) => info.min_quantum = v,
                    (SPA_PARAM_LATENCY_maxQuantum, Value::Float(v)) => info.max_quantum = v,
                    (SPA_PARAM_LATENCY_minRate, Value::Int(v)) => info.min_rate = v,
                    (SPA_PARAM_LATENCY_maxRate, Value::Int(v)) => info.max_rate = v,
                    (SPA_PARAM_LATENCY_minNs, Value::Long(v)) => info.min_ns = v,
                    (SPA_PARAM_LATENCY_maxNs, Value::Long(v)) => info.max_ns = v,
                    _ => (),
                }
            }
            Some(info)
        }
        _ => None,
    }
}

// spa_latency_build is static inline C, so reimplemented here
pub(crate) fn build_latency_info(
    b: &mut libspa::pod::builder::Builder,
    info: &libspa::sys::spa_latency_info,
) -> Result<(), rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
        use libspa::sys::*;

        let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(&mut frame, SPA_TYPE_OBJECT_ParamLatency, SPA_PARAM_Latency)?;

        b.add_prop(SPA_PARAM_LATENCY_direction, 0)?;
        b.add_id(libspa::utils::Id(info.direction))?;

        b.add_prop(SPA_PARAM_LATENCY_minQuantum, 0)?;
        b.add_float(info.min_quantum)?;
        b.add_prop(SPA_PARAM_LATENCY_maxQuantum, 0)?;
        b.add_float(info.max_quantum)?;

        b.add_prop(SPA_PARAM_LATENCY_minRate, 0)?;
        b.add_int(info.min_rate)?;
        b.add_prop(SPA_PARAM_LATENCY_maxRate, 0)?;
        b.add_int(info.max_rate)?;

        b.add_prop(SPA_PARAM_LATENCY_minNs, 0)?;
        b.add_long(info.min_ns)?;
        b.add_prop(SPA_PARAM_LATENCY_maxNs, 0)?;
        b.add_long(info.max_ns)?;

        b.pop(frame.assume_init_mut());

        Ok(())
    }
}

pub(crate) fn process_latency_default() -> libspa::sys::spa_process_latency_info {
    libspa::sys::spa_process_latency_info {
        quantum: 0.0,
        rate: 0,
        ns: 0,
    }
}

// spa_process_latency_parse is static inline C, so reimplemented here
pub(crate) unsafe fn parse_process_latency_info(
    param: *const libspa::sys::spa_pod,
) -> Option<libspa::sys::spa_process_latency_info> {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;

    match crate::utils::deserialize_pod(param) {
        Some(Value::Object(Object {
            type_, properties, ..
        })) if type_ == SPA_TYPE_OBJECT_ParamProcessLatency => {
            let mut info = process_latency_default();
            for p in properties {
                #[allow(non_upper_case_globals)]
                match (p.key, p.value) {
                    (SPA_PARAM_PROCESS_LATENCY_quantum, Value::Float(v)) => info.quantum = v,
                    (SPA_PARAM_PROCESS_LATENCY_rate, Value::Int(v)) => info.rate = v,
                    (SPA_PARAM_PROCESS_LATENCY_ns, Value::Long(v)) => info.ns = v,
                    _ => (),
                }
            }
            Some(info)
        }
        _ => None,
    }
}

// spa_process_latency_build is static inline C, so reimplemented here
pub(crate) fn build_process_latency_info(
    b: &mut libspa::pod::builder::Builder,
    info: &libspa::sys::spa_process_latency_info,
) -> Result<(), rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
        use libspa::sys::*;

        let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(
            &mut frame,
            SPA_TYPE_OBJECT_ParamProcessLatency,
            SPA_PARAM_ProcessLatency,
        )?;

        b.add_prop(SPA_PARAM_PROCESS_LATENCY_quantum, 0)?;
        b.add_float(info.quantum)?;
        b.add_prop(SPA_PARAM_PROCESS_LATENCY_rate, 0)?;
        b.add_int(info.rate)?;
        b.add_prop(SPA_PARAM_PROCESS_LATENCY_ns, 0)?;
        b.add_long(info.ns)?;

        b.pop(frame.assume_init_mut());

        Ok(())
    }
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

pub(crate) fn build_latency_offset_prop_info(
    b: &mut libspa::pod::builder::Builder,
) -> Result<(), rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
        use libspa::sys::*;

        let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
        let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(&mut outer, SPA_TYPE_OBJECT_PropInfo, SPA_PARAM_PropInfo)?;

        b.add_prop(SPA_PROP_INFO_id, 0)?;
        b.add_id(libspa::utils::Id(SPA_PROP_latencyOffsetNsec))?;

        b.add_prop(SPA_PROP_INFO_description, 0)?;
        b.add_string("Latency offset (ns)")?;

        b.add_prop(SPA_PROP_INFO_type, 0)?;
        b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
        b.add_long(0)?;
        b.add_long(0)?;
        b.add_long(2 * SPA_NSEC_PER_SEC as i64)?;
        b.pop(inner.assume_init_mut());

        b.pop(outer.assume_init_mut());

        Ok(())
    }
}

pub(crate) fn build_latency_offset_props(
    b: &mut libspa::pod::builder::Builder,
    ns: i64,
    params: &[(&str, u32)],
) -> Result<(), rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
        use libspa::sys::*;

        let mut frame = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(&mut frame, SPA_TYPE_OBJECT_Props, SPA_PARAM_Props)?;

        b.add_prop(SPA_PROP_latencyOffsetNsec, 0)?;
        b.add_long(ns)?;

        // custom key/value props (oss.delay, oss.fragment) ride in the params struct
        if !params.is_empty() {
            let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
            b.add_prop(SPA_PROP_params, 0)?;
            b.push_struct(&mut inner)?;
            for (key, value) in params {
                b.add_string(key)?;
                b.add_int(*value as i32)?;
            }
            b.pop(inner.assume_init_mut());
        }

        b.pop(frame.assume_init_mut());

        Ok(())
    }
}

// PropInfo for a custom u32 tunable carried in the Props params struct; the
// advertised default is the CURRENT (effective) value, like the ALSA plugin
pub(crate) fn build_params_prop_info(
    b: &mut libspa::pod::builder::Builder,
    name: &str,
    description: &str,
    current: u32,
    max: u32,
) -> Result<(), rustix::io::Errno> {
    // SAFETY: the frame pushes/pops below act on locally-owned frames in
    // strict LIFO order; the pod bytes land in the caller-owned builder
    unsafe {
        use libspa::sys::*;

        let mut outer = std::mem::MaybeUninit::<spa_pod_frame>::uninit();
        let mut inner = std::mem::MaybeUninit::<spa_pod_frame>::uninit();

        b.push_object(&mut outer, SPA_TYPE_OBJECT_PropInfo, SPA_PARAM_PropInfo)?;

        b.add_prop(SPA_PROP_INFO_name, 0)?;
        b.add_string(name)?;

        b.add_prop(SPA_PROP_INFO_description, 0)?;
        b.add_string(description)?;

        b.add_prop(SPA_PROP_INFO_type, 0)?;
        b.push_choice(&mut inner, SPA_CHOICE_Range, 0)?;
        b.add_int(current as i32)?;
        b.add_int(0)?;
        b.add_int(max as i32)?;
        b.pop(inner.assume_init_mut());

        b.add_prop(SPA_PROP_INFO_params, 0)?;
        b.add_bool(true)?; // settable through the Props params struct

        b.pop(outer.assume_init_mut());

        Ok(())
    }
}

// identify our device clock (spa_io_clock.name) so consumers can tell whether
// two nodes tick from the same hardware
pub(crate) unsafe fn set_clock_name(clock: *mut libspa::sys::spa_io_clock, name: &std::ffi::CStr) {
    if clock.is_null() {
        return;
    }
    let bytes = name.to_bytes_with_nul();
    let n = bytes.len().min(63);
    std::ptr::copy_nonoverlapping(bytes.as_ptr().cast(), (*clock).name.as_mut_ptr(), n);
    (*clock).name[63] = 0;
}

// does the driver's clock in `position` carry our clock name? (then we tick
// from the same device and rate matching is pointless - ALSA does the same
// clock-name comparison)
pub(crate) unsafe fn same_clock(
    position: *const libspa::sys::spa_io_position,
    name: &std::ffi::CStr,
) -> bool {
    if position.is_null() {
        return false;
    }
    let theirs = &(*position).clock.name;
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
    let bytes = libspa::pod::Pod::from_raw(param).as_bytes();
    std::panic::catch_unwind(|| PodDeserializer::deserialize_any_from(bytes).ok())
        .ok()
        .flatten()
        // the parse remainder rode a lifetime fabricated from the raw pod;
        // every caller wants only the owned Value
        .map(|(_, value)| value)
}

// Fire-and-forget: queue `f` to run once on the given loop (from any thread;
// non-blocking, RT-safe on the caller side). The closure is boxed and freed
// after it runs. Returns false when it could not even be queued.
// F: Send + 'static - the boxed closure crosses to the loop thread and runs
// after this call returns, so captured stack borrows would dangle
pub(crate) unsafe fn invoke_on_loop<T, F: FnOnce(&mut T) + Send + 'static>(
    loop_: &crate::spa::Loop,
    target: *mut T,
    f: F,
) -> bool {
    struct Ctx<T, F> {
        target: *mut T,
        f: F,
    }

    unsafe extern "C" fn trampoline<T, F: FnOnce(&mut T) + Send + 'static>(
        _loop: *mut libspa::sys::spa_loop,
        _async: bool,
        _seq: u32,
        _data: *const std::os::raw::c_void,
        _size: usize,
        user_data: *mut std::os::raw::c_void,
    ) -> std::os::raw::c_int {
        let ctx = Box::from_raw(user_data.cast::<Ctx<T, F>>());
        let target = ctx.target;
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            (ctx.f)(target.as_mut().expect("target is not supposed to be null"));
        }));
        if ok.is_err() {
            eprintln!("freebsd-oss: panic in a queued main-loop task (swallowed)");
        }
        // never return negative: when the loop flushes the item INLINE (same
        // thread, or the loop not currently entered) the invoke returns this
        // value, and a negative would make the caller free the ctx a second time
        0
    }

    let ctx = Box::into_raw(Box::new(Ctx { target, f }));
    let err = loop_.invoke(
        Some(trampoline::<T, F>),
        0,
        std::ptr::null(),
        0,
        false,
        ctx as *mut std::os::raw::c_void,
    );
    if err < 0 {
        // a negative here uniquely means the item was never queued
        drop(Box::from_raw(ctx));
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

pub(crate) fn now_ns(system: &crate::spa::System) -> u64 {
    let mut now = libspa::sys::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let err = unsafe { system.clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
    assert!(err != -1);
    (now.tv_sec * libspa::sys::SPA_NSEC_PER_SEC as i64 + now.tv_nsec) as u64
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
mod tests {
    use super::{device_period_bytes, enum_format_widths, ns_to_bytes, ns_to_frame_bytes};

    #[test]
    fn frame_bytes_round_up_and_stay_saturated() {
        // the floored hardware quantum comes back up to a whole frame
        assert_eq!(ns_to_frame_bytes(5_333_333, 48000, 8), 2048);
        assert_eq!(ns_to_frame_bytes(0, 48000, 8), 0);
        // a saturated conversion must not overflow the round-up (debug builds
        // would abort the data loop; release would wrap the chunk to zero) -
        // it stays pinned at the largest frame multiple
        assert_eq!(
            ns_to_frame_bytes(u64::MAX, u32::MAX, 8),
            u32::MAX - u32::MAX % 8
        );
        assert_eq!(ns_to_frame_bytes(u64::MAX, u32::MAX, 1), u32::MAX);
    }

    // the 9875023 invariant: pulse falls back to the LAST EnumFormat map when
    // Format is gone, and HW route volume is always 2ch - so whenever the
    // device can do stereo, the list must open with it (host default) and end
    // with it (fallback), or cvolume.channels flips and pops the volume OSD
    #[test]
    fn stereo_pins_both_ends_of_enum_format() {
        for (min, max) in [(1u32, 2u32), (2, 2), (1, 8), (2, 8), (1, 10), (2, 32)] {
            let widths = enum_format_widths(min, max);
            assert_eq!(
                *widths.first().unwrap(),
                2,
                "min {min} max {max}: {widths:?}"
            );
            assert_eq!(
                *widths.last().unwrap(),
                2,
                "min {min} max {max}: {widths:?}"
            );
            assert!(
                widths.contains(&max),
                "native width lost: min {min} max {max}: {widths:?}"
            );
            assert!(
                widths.iter().all(|w| *w >= min && *w <= max),
                "width out of range: min {min} max {max}: {widths:?}"
            );
            assert_eq!(
                widths.iter().filter(|w| **w == 2).count().min(2),
                if widths.len() == 1 { 1 } else { 2 }
            );
        }
        // concrete shapes: this machine's HDMI 8ch and a 10ch USB mixer
        assert_eq!(enum_format_widths(2, 8), [2, 4, 6, 8, 2]);
        assert_eq!(enum_format_widths(1, 10), [2, 4, 6, 8, 1, 10, 2]);
        assert_eq!(enum_format_widths(2, 2), [2]);
        // no stereo support: no pinning, the native width still leads/closes
        assert_eq!(enum_format_widths(1, 1), [1]);
        assert_eq!(enum_format_widths(4, 8), [4, 6, 8]);
        assert_eq!(enum_format_widths(3, 3), [3]);
    }

    #[test]
    fn ns_to_bytes_floors() {
        // the production hw-quantum shape: 256 frames @ 48k S32 stereo is
        // 5333333 ns, which floors to 2047 - call sites round back up to the
        // stride and rely on this direction; a rounding change here silently
        // shifts every geometry derived from it
        assert_eq!(ns_to_bytes(5_333_333, 48000, 8), 2047);
        assert_eq!(ns_to_bytes(5_333_334, 48000, 8), 2048);
        assert_eq!(ns_to_bytes(0, 48000, 8), 0);
        assert_eq!(ns_to_bytes(1_000_000_000, 48000, 8), 384_000);
        // saturates instead of wrapping
        assert_eq!(ns_to_bytes(u64::MAX, u32::MAX, u32::MAX), u32::MAX);
    }

    #[test]
    fn device_period_scales_with_rate_ratio() {
        assert_eq!(device_period_bytes(2048, 48000, 48000, 8), 16384);
        assert_eq!(device_period_bytes(2048, 96000, 48000, 8), 32768);
        assert_eq!(device_period_bytes(2048, 44100, 48000, 8), 15048); // floors
        assert_eq!(device_period_bytes(2048, 48000, 0, 8), 0);
        assert_eq!(
            device_period_bytes(u64::MAX, u32::MAX, 1, u32::MAX),
            u32::MAX
        );
    }
}
