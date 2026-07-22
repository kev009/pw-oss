use nix::errno::Errno;
use std::ffi::{c_char, c_int, c_long, c_uint, c_ulong};

use crate::freebsd::{IoctlPod, SysctlReader, ioctl_int, ioctl_read, ioctl_value};

pub(crate) const AFMT_U8: u32 = 0x00000008;
pub(crate) const AFMT_S16_LE: u32 = 0x00000010;
pub(crate) const AFMT_S16_BE: u32 = 0x00000020;
pub(crate) const AFMT_S32_LE: u32 = 0x00001000;
pub(crate) const AFMT_S32_BE: u32 = 0x00002000;
pub(crate) const AFMT_S24_LE: u32 = 0x00010000;
pub(crate) const AFMT_S24_BE: u32 = 0x00020000;
pub(crate) const AFMT_F32_LE: u32 = 0x10000000;
pub(crate) const AFMT_F32_BE: u32 = 0x20000000;

pub(super) const SNDCTL_DSP_SPEED: c_ulong =
    nix::request_code_readwrite!(b'P', 2, size_of::<c_int>());
pub(super) const SNDCTL_DSP_SETFMT: c_ulong =
    nix::request_code_readwrite!(b'P', 5, size_of::<c_int>());
pub(super) const SNDCTL_DSP_CHANNELS: c_ulong =
    nix::request_code_readwrite!(b'P', 6, size_of::<c_int>());
pub(super) const SNDCTL_DSP_SETFRAGMENT: c_ulong =
    nix::request_code_readwrite!(b'P', 10, size_of::<c_int>());
pub(super) const SNDCTL_DSP_LOW_WATER: c_ulong =
    nix::request_code_write!(b'P', 34, size_of::<c_int>());
pub(super) const SNDCTL_DSP_GETFMTS: c_ulong =
    nix::request_code_read!(b'P', 11, size_of::<c_int>());
pub(super) const SNDCTL_DSP_GETOSPACE: c_ulong =
    nix::request_code_read!(b'P', 12, size_of::<audio_buf_info>());
pub(super) const SNDCTL_DSP_GETISPACE: c_ulong =
    nix::request_code_read!(b'P', 13, size_of::<audio_buf_info>());
pub(super) const SNDCTL_DSP_SETTRIGGER: c_ulong =
    nix::request_code_write!(b'P', 16, size_of::<c_int>());
pub(super) const SNDCTL_DSP_GETODELAY: c_ulong =
    nix::request_code_read!(b'P', 23, size_of::<c_int>());
pub(super) const SNDCTL_DSP_GETERROR: c_ulong =
    nix::request_code_read!(b'P', 25, size_of::<audio_errinfo>());
const SNDCTL_DSP_GET_CHNORDER: c_ulong =
    nix::request_code_read!(b'P', 42, size_of::<OssChannelOrder>());
const SNDCTL_DSP_SET_CHNORDER: c_ulong =
    nix::request_code_readwrite!(b'P', 42, size_of::<OssChannelOrder>());
pub(super) const SNDCTL_DSP_HALT: c_ulong = nix::request_code_none!(b'P', 0); // aka SNDCTL_DSP_RESET
pub(super) const SNDCTL_DSP_SILENCE: c_ulong = nix::request_code_none!(b'P', 31);
pub(super) const SNDCTL_DSP_SKIP: c_ulong = nix::request_code_none!(b'P', 32);
pub(super) const SNDCTL_ENGINEINFO: c_ulong =
    nix::request_code_readwrite!(b'X', 12, size_of::<oss_audioinfo>());

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
pub(super) const SND_CHN_MAX: c_int = 8;

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
pub(super) fn set_value(fd: c_int, req: c_ulong, value: u32, tolerance: u32) -> Result<u32, Errno> {
    let Some(v) = ioctl_int(fd, req, value as c_int) else {
        return Err(Errno::last());
    };
    let actual = u32::try_from(v).map_err(|_| Errno::EINVAL)?;
    if (actual as i64 - value as i64).unsigned_abs() > tolerance as u64 {
        return Err(Errno::EINVAL);
    }
    Ok(actual)
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
    // e.g. the device was unplugged mid-stream
    unsafe {
        ioctl_read::<audio_errinfo>(fd, SNDCTL_DSP_GETERROR).unwrap_or_else(|| std::mem::zeroed())
    }
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

// frame bytes for a sound4 AFMT value (width by encoding bit, channels from
// the AFMT_CHANNEL field, sound.h:344); approximate widths are fine - the
// quantum this feeds is a floor, and overstating errs toward more margin
pub(super) fn afmt_frame_bytes(format: u32) -> u32 {
    const AFMT_U16_MASK: u32 = 0x00000180;
    const AFMT_U24_MASK: u32 = 0x000c0000;
    const AFMT_U32_MASK: u32 = 0x0000c000;

    let width: u32 =
        if format & (AFMT_S32_LE | AFMT_S32_BE | AFMT_F32_LE | AFMT_F32_BE | AFMT_U32_MASK) != 0 {
            4 // S32/U32/F32
        } else if format & (AFMT_S24_LE | AFMT_S24_BE | AFMT_U24_MASK) != 0 {
            // 3-byte S24/U24
            3
        } else if format & (AFMT_S16_LE | AFMT_S16_BE | AFMT_U16_MASK) != 0 {
            2
        } else {
            1
        };
    let channels = ((format & 0x07f00000) >> 20).max(1); // AFMT_CHANNEL (sound.h:344)
    width * channels
}
