use nix::errno::Errno;
use std::ffi::{c_char, c_int, c_long, c_uint, c_ulong, c_void};

use super::sys::{IoctlPod, SysctlReader, ioctl_int, ioctl_read, ioctl_value, ioctl_zeroed};

pub(crate) const AFMT_MU_LAW: u32 = 0x00000001;
pub(crate) const AFMT_A_LAW: u32 = 0x00000002;
pub(crate) const AFMT_U8: u32 = 0x00000008;
pub(crate) const AFMT_S16_LE: u32 = 0x00000010;
pub(crate) const AFMT_S16_BE: u32 = 0x00000020;
pub(crate) const AFMT_S8: u32 = 0x00000040;
pub(crate) const AFMT_U16_LE: u32 = 0x00000080;
pub(crate) const AFMT_U16_BE: u32 = 0x00000100;
pub(crate) const AFMT_S32_LE: u32 = 0x00001000;
pub(crate) const AFMT_S32_BE: u32 = 0x00002000;
pub(crate) const AFMT_U32_LE: u32 = 0x00004000;
pub(crate) const AFMT_U32_BE: u32 = 0x00008000;
pub(crate) const AFMT_S24_LE: u32 = 0x00010000;
pub(crate) const AFMT_S24_BE: u32 = 0x00020000;
pub(crate) const AFMT_U24_LE: u32 = 0x00040000;
pub(crate) const AFMT_U24_BE: u32 = 0x00080000;
pub(crate) const AFMT_F32_LE: u32 = 0x10000000;
pub(crate) const AFMT_F32_BE: u32 = 0x20000000;

const SNDCTL_DSP_SPEED: c_ulong = nix::request_code_readwrite!(b'P', 2, size_of::<c_int>());
const SNDCTL_DSP_SETFMT: c_ulong = nix::request_code_readwrite!(b'P', 5, size_of::<c_int>());
const SNDCTL_DSP_CHANNELS: c_ulong = nix::request_code_readwrite!(b'P', 6, size_of::<c_int>());
const SNDCTL_DSP_SETFRAGMENT: c_ulong = nix::request_code_readwrite!(b'P', 10, size_of::<c_int>());
const SNDCTL_DSP_LOW_WATER: c_ulong = nix::request_code_write!(b'P', 34, size_of::<c_int>());
const SNDCTL_DSP_GETFMTS: c_ulong = nix::request_code_read!(b'P', 11, size_of::<c_int>());
const SNDCTL_DSP_GETOSPACE: c_ulong =
    nix::request_code_read!(b'P', 12, size_of::<audio_buf_info>());
const SNDCTL_DSP_GETISPACE: c_ulong =
    nix::request_code_read!(b'P', 13, size_of::<audio_buf_info>());
const SNDCTL_DSP_GETCAPS: c_ulong = nix::request_code_read!(b'P', 15, size_of::<c_int>());
const SNDCTL_DSP_SETTRIGGER: c_ulong = nix::request_code_write!(b'P', 16, size_of::<c_int>());
const SNDCTL_DSP_GETODELAY: c_ulong = nix::request_code_read!(b'P', 23, size_of::<c_int>());
const SNDCTL_DSP_GETERROR: c_ulong = nix::request_code_read!(b'P', 25, size_of::<audio_errinfo>());
const SNDCTL_DSP_GET_CHNORDER: c_ulong =
    nix::request_code_read!(b'P', 42, size_of::<OssChannelOrder>());
const SNDCTL_DSP_SET_CHNORDER: c_ulong =
    nix::request_code_readwrite!(b'P', 42, size_of::<OssChannelOrder>());
const SNDCTL_DSP_HALT: c_ulong = nix::request_code_none!(b'P', 0); // aka SNDCTL_DSP_RESET
const SNDCTL_DSP_SILENCE: c_ulong = nix::request_code_none!(b'P', 31);
const SNDCTL_DSP_SKIP: c_ulong = nix::request_code_none!(b'P', 32);
const SNDCTL_ENGINEINFO: c_ulong =
    nix::request_code_readwrite!(b'X', 12, size_of::<oss_audioinfo>());

#[repr(C)]
struct SndstiocNvArg {
    nbytes: usize,
    buf: *mut c_void,
}

const SNDSTIOC_REFRESH_DEVS: c_ulong = nix::request_code_none!(b'D', 100);
const SNDSTIOC_GET_DEVS: c_ulong =
    nix::request_code_readwrite!(b'D', 101, size_of::<SndstiocNvArg>());

// sys/soundcard.h; the ioctl encodes the size, so a layout mismatch fails
// cleanly instead of corrupting memory
#[repr(C)]
#[derive(Clone, Copy)]
pub(super) struct oss_audioinfo {
    pub(super) dev: c_int,
    pub(super) name: [c_char; 64],
    pub(super) busy: c_int,
    pub(super) pid: c_int,
    pub(super) caps: c_int,
    pub(super) iformats: c_int,
    pub(super) oformats: c_int,
    pub(super) magic: c_int,
    pub(super) cmd: [c_char; 64],
    pub(super) card_number: c_int,
    pub(super) port_number: c_int,
    pub(super) mixer_dev: c_int,
    pub(super) legacy_device: c_int,
    pub(super) enabled: c_int,
    pub(super) flags: c_int,
    pub(super) min_rate: c_int,
    pub(super) max_rate: c_int,
    pub(super) min_channels: c_int,
    pub(super) max_channels: c_int,
    pub(super) binding: c_int,
    pub(super) rate_source: c_int,
    pub(super) handle: [c_char; 32],
    pub(super) nrates: c_uint,
    pub(super) rates: [c_uint; 20],
    pub(super) song_name: [c_char; 64],
    pub(super) label: [c_char; 16],
    pub(super) latency: c_int,
    pub(super) devnode: [c_char; 32],
    pub(super) next_play_engine: c_int,
    pub(super) next_rec_engine: c_int,
    pub(super) filler: [c_int; 184],
}

unsafe impl IoctlPod for oss_audioinfo {}

// sys/dev/sound/pcm/matrix.h: SETCHANNELS requests are clamped to this
const SND_CHN_MAX: c_int = 8;

pub(super) const PCM_ENABLE_INPUT: c_int = 0x00000001;
pub(super) const PCM_ENABLE_OUTPUT: c_int = 0x00000002;

// sound(4) PCM_CAP_* (sys/soundcard.h) — ENGINEINFO / sndstat channel caps
pub(crate) const PCM_CAP_INPUT: c_int = 0x0001_0000;
pub(crate) const PCM_CAP_OUTPUT: c_int = 0x0002_0000;
pub(crate) const PCM_CAP_VIRTUAL: c_int = 0x0004_0000;
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(super) struct audio_buf_info {
    pub(super) fragments: c_int,
    pub(super) fragstotal: c_int,
    pub(super) fragsize: c_int,
    pub(super) bytes: c_int,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(super) struct audio_errinfo {
    pub(super) play_underruns: c_int,
    pub(super) rec_overruns: c_int,
    pub(super) play_ptradjust: c_uint,
    pub(super) rec_ptradjust: c_uint,
    pub(super) play_errorcount: c_int,
    pub(super) rec_errorcount: c_int,
    pub(super) play_lasterror: c_int,
    pub(super) rec_lasterror: c_int,
    pub(super) play_errorparm: c_long,
    pub(super) rec_errorparm: c_long,
    pub(super) filler: [c_int; 16],
}

// sys/soundcard.h uses unsigned long long for channel-order maps.
// Keep a distinct type so the ioctl size and the value being verified cannot
// accidentally drift apart.
#[repr(transparent)]
#[derive(Clone, Copy)]
struct OssChannelOrder(u64);

unsafe impl IoctlPod for OssChannelOrder {}

unsafe impl IoctlPod for audio_buf_info {}
unsafe impl IoctlPod for audio_errinfo {}

#[derive(Debug, PartialEq)]
pub(super) enum DspState {
    Closed,
    Setup,
    Running,
}
// hw.snd.feeder_rate_round: the kernel snaps a requested rate within this of
// the hardware clock to the exact hardware rate (channel.c chn_setparam);
// it's a runtime sysctl (0..500), so read it, falling back to the default
pub(super) const FEEDER_RATE_ROUND_DEFAULT: u32 = 25;

pub(crate) fn feeder_rate_round() -> u32 {
    SysctlReader::new()
        .read_u32("hw.snd.feeder_rate_round")
        .unwrap_or(FEEDER_RATE_ROUND_DEFAULT)
        .min(500)
}

// OSS grants the nearest supported value instead of failing, so a grant that
// differs from the request beyond `tolerance` is a rejection here
fn set_value(fd: c_int, req: c_ulong, value: u32, tolerance: u32) -> Result<u32, Errno> {
    let Some(v) = ioctl_int(fd, req, value as c_int) else {
        return Err(Errno::last());
    };
    let actual = u32::try_from(v).map_err(|_| Errno::EINVAL)?;
    if (actual as i64 - value as i64).unsigned_abs() > tolerance as u64 {
        return Err(Errno::EINVAL);
    }
    Ok(actual)
}

pub(super) fn set_format(fd: c_int, format: u32) -> Result<u32, Errno> {
    set_value(fd, SNDCTL_DSP_SETFMT, format, 0)
}

pub(super) fn set_channels(fd: c_int, channels: u32) -> Result<u32, Errno> {
    set_value(fd, SNDCTL_DSP_CHANNELS, channels, 0)
}

pub(super) fn set_rate(fd: c_int, rate: u32) -> Result<u32, Errno> {
    set_value(fd, SNDCTL_DSP_SPEED, rate, feeder_rate_round())
}

pub(super) fn set_low_water(fd: c_int, bytes: u32) -> bool {
    ioctl_int(
        fd,
        SNDCTL_DSP_LOW_WATER,
        bytes.clamp(1, c_int::MAX as u32) as c_int,
    )
    .is_some()
}

pub(super) fn input_space(fd: c_int) -> Option<audio_buf_info> {
    unsafe { ioctl_read(fd, SNDCTL_DSP_GETISPACE) }
}

pub(super) fn halt(fd: c_int) {
    // Best-effort: HALT is used immediately before closing the descriptor.
    let _ = unsafe { libc::ioctl(fd, SNDCTL_DSP_HALT) };
}

pub(super) fn supported_formats(fd: c_int) -> Option<c_int> {
    ioctl_int(fd, SNDCTL_DSP_GETFMTS, 0)
}

pub(super) fn channel_caps(fd: c_int) -> u32 {
    ioctl_int(fd, SNDCTL_DSP_GETCAPS, 0).map_or(0, |caps| caps as u32)
}

pub(super) fn engine_info(fd: c_int) -> Option<oss_audioinfo> {
    engine_info_at(fd, -1)
}

pub(super) fn engine_info_at(fd: c_int, device: c_int) -> Option<oss_audioinfo> {
    let mut info = ioctl_zeroed::<oss_audioinfo>();
    info.dev = device;
    unsafe { ioctl_value(fd, SNDCTL_ENGINEINFO, info) }
}

pub(super) fn probe_min_channels(fd: c_int) -> Option<c_int> {
    ioctl_int(fd, SNDCTL_DSP_CHANNELS, 1)
}

pub(super) fn probe_max_channels(fd: c_int) -> Option<c_int> {
    ioctl_int(fd, SNDCTL_DSP_CHANNELS, SND_CHN_MAX)
}

pub(super) fn probe_rate(fd: c_int, rate: c_int) -> Option<c_int> {
    ioctl_int(fd, SNDCTL_DSP_SPEED, rate)
}

pub(super) fn sndstat_snapshot_bytes(fd: c_int) -> Option<Vec<u8>> {
    // Best-effort; GET still returns the last snapshot if refresh fails.
    let _ = unsafe { libc::ioctl(fd, SNDSTIOC_REFRESH_DEVS) };

    // The snapshot is per-open cdevpriv, so it cannot change between the size
    // probe and fill calls. A too-small second buffer returns nbytes = 0.
    let mut buffer: Vec<u8> = Vec::new();
    for _ in 0..2 {
        let mut arg = SndstiocNvArg {
            nbytes: buffer.len(),
            buf: if buffer.is_empty() {
                std::ptr::null_mut()
            } else {
                buffer.as_mut_ptr().cast()
            },
        };
        if unsafe { libc::ioctl(fd, SNDSTIOC_GET_DEVS, &mut arg) } == -1 {
            return None;
        }
        if !buffer.is_empty() && arg.nbytes <= buffer.len() {
            buffer.truncate(arg.nbytes);
            break;
        }
        buffer = vec![0; arg.nbytes];
    }
    Some(buffer)
}

pub(super) fn ospace_in_bytes(fd: c_int) -> c_int {
    // e.g. the device was unplugged mid-stream
    unsafe { ioctl_read::<audio_buf_info>(fd, SNDCTL_DSP_GETOSPACE) }.map_or(0, |info| info.bytes)
}

pub(super) fn set_fragment(fd: c_int, n_frags: u16, frag_size_selector: u16) {
    let s = (((n_frags as u32) << 16) | frag_size_selector as u32) as c_int;
    // best-effort: the caller reads the real grant back via GETOSPACE
    let _ = ioctl_int(fd, SNDCTL_DSP_SETFRAGMENT, s);
    // FreeBSD can grant a smaller layout than requested. The caller reads the real
    // size from GETOSPACE, so don't assert the request was honored.
}

pub(super) fn set_trigger(fd: c_int, mask: c_int) -> bool {
    ioctl_int(fd, SNDCTL_DSP_SETTRIGGER, mask).is_some()
}

// Reorder the application-side PCM interleave, then verify what the channel
// exposes. FreeBSD implements this for convertible PCM formats; direct or
// alternate OSS implementations may reject it, which must fail negotiation
// rather than leave the SPA channel labels disagreeing with the byte stream.
pub(super) fn set_channel_order(fd: c_int, order: u64) -> Result<(), Errno> {
    let requested = OssChannelOrder(order);
    if unsafe { ioctl_value(fd, SNDCTL_DSP_SET_CHNORDER, requested) }.is_none() {
        return Err(Errno::last());
    }
    let Some(actual) = (unsafe { ioctl_read::<OssChannelOrder>(fd, SNDCTL_DSP_GET_CHNORDER) })
    else {
        return Err(Errno::last());
    };
    if actual.0 != order {
        return Err(Errno::EINVAL);
    }
    Ok(())
}

pub(super) fn odelay(fd: c_int) -> c_int {
    // e.g. the device was unplugged mid-stream
    try_odelay(fd).unwrap_or(0)
}

pub(super) fn try_odelay(fd: c_int) -> Result<c_int, Errno> {
    // FreeBSD reports bufsoft only. The hardware buffer is prefilled at
    // trigger and omitted, so callers may use this as a stable queue signal
    // but not as complete physical latency or proof that playback stopped.
    ioctl_int(fd, SNDCTL_DSP_GETODELAY, -1).ok_or_else(Errno::last)
}

// The fragment size the driver actually granted, which need not match the
// SETFRAGMENT request: some drivers (e.g. snd_hdspe) force a fixed period.
// GETBLKSIZE returns EINVAL here, so read GETOSPACE's fragsize field.
pub(super) fn blocksize(fd: c_int) -> c_int {
    unsafe { ioctl_read::<audio_buf_info>(fd, SNDCTL_DSP_GETOSPACE) }
        .map_or(0, |info| info.fragsize)
}

pub(super) fn get_error(fd: c_int) -> audio_errinfo {
    // GETERROR consumes pcm_channel::xruns, whereas enriched dsp_kqevent()
    // snapshots the same counter without clearing it. The shared shell keeps
    // an event-side baseline so a later timer/follower poll cannot report the
    // same errors twice. A failed ioctl (for example after unplug) reads as an
    // empty snapshot and the stream I/O path supplies the disconnect status.
    unsafe {
        ioctl_read::<audio_errinfo>(fd, SNDCTL_DSP_GETERROR).unwrap_or_else(|| std::mem::zeroed())
    }
}

// The kernel counter is unsigned, but the historical OSS ABI exposes it in
// an `int` field. Preserve the 32-bit representation instead of treating the
// sign bit as an error; GETERROR has already used ioctl success for validity.
pub(super) const fn xrun_counter_bits(value: c_int) -> u32 {
    value as u32
}

// FreeBSD's paired pause operations: SILENCE saves the ready part of bufsoft
// in its shadow buffer and substitutes format-correct silence; SKIP discards
// the remaining pause silence and restores those saved samples. Keep the raw
// no-argument ioctls here so callers cannot accidentally use generic OSS
// SKIP's otherwise surprising "discard queued output" meaning on its own.
pub(super) fn shadow_pause(fd: c_int) -> bool {
    unsafe { libc::ioctl(fd, SNDCTL_DSP_SILENCE) != -1 }
}

pub(super) fn restore_shadow(fd: c_int) -> bool {
    unsafe { libc::ioctl(fd, SNDCTL_DSP_SKIP) != -1 }
}

// Exact frame storage for a sound4 AFMT value: bytes per encoded sample times
// the channel count carried in AFMT_CHANNEL (sound.h:344). This supplies both
// negotiated stream stride and delivery-quantum geometry.
pub(super) fn afmt_frame_bytes(format: u32) -> u32 {
    let width: u32 = if format
        & (AFMT_S32_LE | AFMT_S32_BE | AFMT_U32_LE | AFMT_U32_BE | AFMT_F32_LE | AFMT_F32_BE)
        != 0
    {
        4 // S32/U32/F32
    } else if format & (AFMT_S24_LE | AFMT_S24_BE | AFMT_U24_LE | AFMT_U24_BE) != 0 {
        // 3-byte S24/U24
        3
    } else if format & (AFMT_S16_LE | AFMT_S16_BE | AFMT_U16_LE | AFMT_U16_BE) != 0 {
        2
    } else {
        1
    };
    let channels = ((format & 0x07f00000) >> 20).max(1); // AFMT_CHANNEL (sound.h:344)
    width * channels
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::{CStr, c_char};
    use std::fmt::Write as _;

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct CAbiEntry {
        name: *const c_char,
        value: u64,
    }

    unsafe extern "C" {
        fn pw_oss_native_abi_report(count: *mut usize) -> *const CAbiEntry;
    }

    fn native_abi_report() -> Vec<(String, u64)> {
        let mut count = 0;
        let entries = unsafe {
            let entries = pw_oss_native_abi_report(&raw mut count);
            assert!(!entries.is_null());
            std::slice::from_raw_parts(entries, count)
        };

        entries
            .iter()
            .map(|entry| {
                assert!(!entry.name.is_null());
                let name = unsafe { CStr::from_ptr(entry.name) }
                    .to_str()
                    .expect("C ABI report names are ASCII")
                    .to_owned();
                (name, entry.value)
            })
            .collect()
    }

    fn rust_abi_report() -> Vec<(&'static str, u64)> {
        macro_rules! abi_size {
            ($name:literal, $type:ty) => {
                (concat!("size.", $name), size_of::<$type>() as u64)
            };
        }
        macro_rules! abi_align {
            ($name:literal, $type:ty) => {
                (concat!("align.", $name), align_of::<$type>() as u64)
            };
        }
        macro_rules! abi_offset {
            ($name:literal, $type:ty, $field:ident) => {
                (
                    concat!("offset.", $name, ".", stringify!($field)),
                    std::mem::offset_of!($type, $field) as u64,
                )
            };
        }
        macro_rules! abi_const {
            ($name:ident) => {
                (concat!("const.", stringify!($name)), $name as u64)
            };
        }

        vec![
            abi_size!("SndstiocNvArg", SndstiocNvArg),
            abi_align!("SndstiocNvArg", SndstiocNvArg),
            abi_offset!("SndstiocNvArg", SndstiocNvArg, nbytes),
            abi_offset!("SndstiocNvArg", SndstiocNvArg, buf),
            abi_size!("audio_buf_info", audio_buf_info),
            abi_align!("audio_buf_info", audio_buf_info),
            abi_offset!("audio_buf_info", audio_buf_info, fragments),
            abi_offset!("audio_buf_info", audio_buf_info, fragstotal),
            abi_offset!("audio_buf_info", audio_buf_info, fragsize),
            abi_offset!("audio_buf_info", audio_buf_info, bytes),
            abi_size!("audio_errinfo", audio_errinfo),
            abi_align!("audio_errinfo", audio_errinfo),
            abi_offset!("audio_errinfo", audio_errinfo, play_underruns),
            abi_offset!("audio_errinfo", audio_errinfo, rec_overruns),
            abi_offset!("audio_errinfo", audio_errinfo, play_ptradjust),
            abi_offset!("audio_errinfo", audio_errinfo, rec_ptradjust),
            abi_offset!("audio_errinfo", audio_errinfo, play_errorcount),
            abi_offset!("audio_errinfo", audio_errinfo, rec_errorcount),
            abi_offset!("audio_errinfo", audio_errinfo, play_lasterror),
            abi_offset!("audio_errinfo", audio_errinfo, rec_lasterror),
            abi_offset!("audio_errinfo", audio_errinfo, play_errorparm),
            abi_offset!("audio_errinfo", audio_errinfo, rec_errorparm),
            abi_offset!("audio_errinfo", audio_errinfo, filler),
            abi_size!("oss_audioinfo", oss_audioinfo),
            abi_align!("oss_audioinfo", oss_audioinfo),
            abi_offset!("oss_audioinfo", oss_audioinfo, dev),
            abi_offset!("oss_audioinfo", oss_audioinfo, name),
            abi_offset!("oss_audioinfo", oss_audioinfo, busy),
            abi_offset!("oss_audioinfo", oss_audioinfo, pid),
            abi_offset!("oss_audioinfo", oss_audioinfo, caps),
            abi_offset!("oss_audioinfo", oss_audioinfo, iformats),
            abi_offset!("oss_audioinfo", oss_audioinfo, oformats),
            abi_offset!("oss_audioinfo", oss_audioinfo, magic),
            abi_offset!("oss_audioinfo", oss_audioinfo, cmd),
            abi_offset!("oss_audioinfo", oss_audioinfo, card_number),
            abi_offset!("oss_audioinfo", oss_audioinfo, port_number),
            abi_offset!("oss_audioinfo", oss_audioinfo, mixer_dev),
            abi_offset!("oss_audioinfo", oss_audioinfo, legacy_device),
            abi_offset!("oss_audioinfo", oss_audioinfo, enabled),
            abi_offset!("oss_audioinfo", oss_audioinfo, flags),
            abi_offset!("oss_audioinfo", oss_audioinfo, min_rate),
            abi_offset!("oss_audioinfo", oss_audioinfo, max_rate),
            abi_offset!("oss_audioinfo", oss_audioinfo, min_channels),
            abi_offset!("oss_audioinfo", oss_audioinfo, max_channels),
            abi_offset!("oss_audioinfo", oss_audioinfo, binding),
            abi_offset!("oss_audioinfo", oss_audioinfo, rate_source),
            abi_offset!("oss_audioinfo", oss_audioinfo, handle),
            abi_offset!("oss_audioinfo", oss_audioinfo, nrates),
            abi_offset!("oss_audioinfo", oss_audioinfo, rates),
            abi_offset!("oss_audioinfo", oss_audioinfo, song_name),
            abi_offset!("oss_audioinfo", oss_audioinfo, label),
            abi_offset!("oss_audioinfo", oss_audioinfo, latency),
            abi_offset!("oss_audioinfo", oss_audioinfo, devnode),
            abi_offset!("oss_audioinfo", oss_audioinfo, next_play_engine),
            abi_offset!("oss_audioinfo", oss_audioinfo, next_rec_engine),
            abi_offset!("oss_audioinfo", oss_audioinfo, filler),
            abi_size!("OssChannelOrder", OssChannelOrder),
            abi_align!("OssChannelOrder", OssChannelOrder),
            abi_const!(AFMT_MU_LAW),
            abi_const!(AFMT_A_LAW),
            abi_const!(AFMT_U8),
            abi_const!(AFMT_S16_LE),
            abi_const!(AFMT_S16_BE),
            abi_const!(AFMT_S8),
            abi_const!(AFMT_U16_LE),
            abi_const!(AFMT_U16_BE),
            abi_const!(AFMT_S32_LE),
            abi_const!(AFMT_S32_BE),
            abi_const!(AFMT_U32_LE),
            abi_const!(AFMT_U32_BE),
            abi_const!(AFMT_S24_LE),
            abi_const!(AFMT_S24_BE),
            abi_const!(AFMT_U24_LE),
            abi_const!(AFMT_U24_BE),
            abi_const!(AFMT_F32_LE),
            abi_const!(AFMT_F32_BE),
            abi_const!(PCM_ENABLE_INPUT),
            abi_const!(PCM_ENABLE_OUTPUT),
            abi_const!(PCM_CAP_INPUT),
            abi_const!(PCM_CAP_OUTPUT),
            abi_const!(PCM_CAP_VIRTUAL),
            abi_const!(SNDCTL_DSP_SPEED),
            abi_const!(SNDCTL_DSP_SETFMT),
            abi_const!(SNDCTL_DSP_CHANNELS),
            abi_const!(SNDCTL_DSP_SETFRAGMENT),
            abi_const!(SNDCTL_DSP_LOW_WATER),
            abi_const!(SNDCTL_DSP_GETFMTS),
            abi_const!(SNDCTL_DSP_GETOSPACE),
            abi_const!(SNDCTL_DSP_GETISPACE),
            abi_const!(SNDCTL_DSP_GETCAPS),
            abi_const!(SNDCTL_DSP_SETTRIGGER),
            abi_const!(SNDCTL_DSP_GETODELAY),
            abi_const!(SNDCTL_DSP_GETERROR),
            abi_const!(SNDCTL_DSP_GET_CHNORDER),
            abi_const!(SNDCTL_DSP_SET_CHNORDER),
            abi_const!(SNDCTL_DSP_HALT),
            abi_const!(SNDCTL_DSP_SILENCE),
            abi_const!(SNDCTL_DSP_SKIP),
            abi_const!(SNDCTL_ENGINEINFO),
            abi_const!(SNDSTIOC_REFRESH_DEVS),
            abi_const!(SNDSTIOC_GET_DEVS),
        ]
    }

    #[test]
    fn c_and_rust_abi_match() {
        let native = native_abi_report();
        let rust = rust_abi_report();
        let entry_count = native.len();
        let mut report = format!("FreeBSD OSS C/Rust ABI comparison ({entry_count} entries)\n");

        for ((native_name, native_value), (rust_name, rust_value)) in native.iter().zip(&rust) {
            let status = if native_name == rust_name && native_value == rust_value {
                "ok"
            } else {
                "MISMATCH"
            };
            writeln!(
                report,
                "{native_name:<48} C={native_value:#018x} Rust={rust_value:#018x} {status}"
            )
            .expect("writing to a String cannot fail");
        }
        print!("{report}");

        assert_eq!(
            native.len(),
            rust.len(),
            "C and Rust ABI reports cover different numbers of values"
        );
        for ((native_name, native_value), (rust_name, rust_value)) in native.iter().zip(&rust) {
            assert_eq!(native_name, rust_name, "ABI report ordering differs");
            assert_eq!(native_value, rust_value, "ABI mismatch for {native_name}");
        }
    }

    #[test]
    fn signed_oss_xrun_field_preserves_unsigned_counter_bits() {
        assert_eq!(super::xrun_counter_bits(0), 0);
        assert_eq!(super::xrun_counter_bits(-1), u32::MAX);
        assert_eq!(super::xrun_counter_bits(i32::MIN), 1 << 31);
    }
}
