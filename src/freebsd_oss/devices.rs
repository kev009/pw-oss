use nix::errno::Errno;
use std::collections::BTreeMap;
use std::ffi::{CString, c_int};

use super::abi::*;
use super::sys::{LibcFd, NvList, NvRef, SysctlReader};
use crate::backend::{ConversionKind, StreamCaps, StreamConfiguration};

// Ask the device what it actually supports. Two sources, merged:
// - empirical SETCHANNELS/SPEED probes at the extremes (OSS grants the nearest
//   supported value) - but the kernel clamps channel requests to SND_CHN_MAX
//   and bitperfect devices reject unsupported values instead of snapping;
// - SNDCTL_AUDIOINFO, which reports the real hardware limits (dsp.c
//   aggregates chn_getcaps over the device), covering both gaps above.
// Uses a transient open; the caller falls back if the device is busy.

// native per-direction device info from the sndstat(4) nvlist interface -
// no dsp open, so an exclusive device's only channel stays unclaimed
pub(crate) struct SndstatDspInfo {
    pub formats: u32,
    pub min_rate: u32,
    pub max_rate: u32,
    pub min_chn: u32,
    pub max_chn: u32,
    pub exclusive: Option<bool>, // vchans off for this direction; None = can't tell
    pub vchan_rate: u32,         // the parent's mix rate while vchans are on
    pub bitperfect: bool,
}

// one packed snapshot of every sound device from sndstat(4)
fn sndstat_snapshot() -> Option<NvList> {
    let fd = LibcFd::open(c"/dev/sndstat", libc::O_RDONLY)?;
    NvList::unpack(&sndstat_snapshot_bytes(fd.raw())?)
}

// pcm unit numbers plus per-direction channel presence; user-registered
// devices (virtual_oss) are excluded explicitly instead of by name prefix
fn sndstat_pcm_devices() -> Option<Vec<(u32, bool, bool)>> {
    let nvl = sndstat_snapshot()?;
    let mut out = vec![];
    for dev in nvl.root().nvlist_array(c"dsps") {
        if dev.boolean(c"from_user").unwrap_or(false) {
            continue;
        }
        let Some(unit) = dev
            .string(c"nameunit")
            .and_then(|nu| nu.strip_prefix("pcm"))
            .and_then(|u| u.parse::<u32>().ok())
        else {
            continue;
        };
        out.push((
            unit,
            dev.number(c"pchan").unwrap_or(0) > 0,
            dev.number(c"rchan").unwrap_or(0) > 0,
        ));
    }
    Some(out)
}

pub(crate) fn sndstat_dsp_info(devnode: &str, play: bool) -> Option<SndstatDspInfo> {
    let nvl = sndstat_snapshot()?;
    let root = nvl.root();
    for dev in root.nvlist_array(c"dsps") {
        // a user-registered device (virtual_oss) may carry any devnode string;
        // don't let it shadow a kernel one
        if dev.boolean(c"from_user").unwrap_or(false) || dev.string(c"devnode") != Some(devnode) {
            continue;
        }
        // absent for a direction with no channels
        let info = dev.nvlist(if play { c"info_play" } else { c"info_rec" })?;
        let num = |r: NvRef, k: &std::ffi::CStr| r.number(k).unwrap_or(0) as u32;

        let (mut exclusive, mut vchan_rate, mut bitperfect) = (None, 0, false);
        if let Some(p) = dev.nvlist(c"provider_info") {
            // pvchan/rvchan is a NUMBER of LIVE vchans on 14.x (an idle device
            // reads 0 with vchans enabled!) and a BOOL enabled flag on 15.0+
            // (sndstat.c 0c0bb4c1401c). Only the bool and a positive count are
            // unambiguous; a zero count means "can't tell, probe".
            let key = if play { c"pvchan" } else { c"rvchan" };
            exclusive = match (p.boolean(key), p.number(key)) {
                (Some(enabled), _) => Some(!enabled),
                (None, Some(n)) if n > 0 => Some(false),
                _ => None,
            };
            vchan_rate = num(p, if play { c"pvchanrate" } else { c"rvchanrate" });
            bitperfect = p.boolean(c"bitperfect").unwrap_or(false);
        }

        return Some(SndstatDspInfo {
            formats: num(info, c"formats"),
            min_rate: num(info, c"min_rate"),
            max_rate: num(info, c"max_rate"),
            min_chn: num(info, c"min_chn"),
            max_chn: num(info, c"max_chn"),
            exclusive,
            vchan_rate,
            bitperfect,
        });
    }
    None
}

fn caps_from_sndstat(nv: &SndstatDspInfo, rates: Vec<u32>) -> StreamCaps {
    StreamCaps {
        configurations: vec![StreamConfiguration {
            formats: super::backend::formats_from_native_mask(nv.formats, nv.bitperfect),
            min_channels: nv.min_chn.max(1),
            max_channels: nv.max_chn.max(nv.min_chn).max(1),
            min_rate: nv.min_rate.max(1),
            max_rate: nv.max_rate.max(nv.min_rate).max(1),
            preferred_rate: None, // the native values are the preference
            rates,
            rate_tolerance: feeder_rate_round(),
        }],
        preferred: 0,
        conversion: if nv.bitperfect {
            ConversionKind::None
        } else {
            ConversionKind::Backend
        },
    }
}

// The native rate SET of an exclusive device, from a brief ENGINEINFO-only
// open. Bitperfect rates aren't a dense range: the kernel snaps the DMA to
// the nearest native rate but SNDCTL_DSP_SPEED echoes the REQUEST back for
// playback (feeder_chain keeps c->speed = target), so an in-range non-native
// rate would negotiate fine and play pitch-shifted with no diagnostics.
fn native_rates(path: &str, play: bool) -> Vec<u32> {
    let Ok(cpath) = CString::new(path) else {
        return vec![];
    };
    let mode = if play { libc::O_WRONLY } else { libc::O_RDONLY };
    let Some(fd) = LibcFd::open(&cpath, mode | libc::O_NONBLOCK) else {
        return vec![]; // busy: the caller keeps the min..max range
    };
    let mut rates = vec![];
    if let Some(ai) = engine_info(fd.raw()) {
        for i in 0..ai.nrates.min(20) as usize {
            rates.push(ai.rates[i]);
        }
    }
    rates.retain(|r| *r > 0);
    rates.sort_unstable();
    rates.dedup();
    rates
}

pub(crate) fn probe_caps(path: &str, play: bool) -> Option<StreamCaps> {
    let native = sndstat_dsp_info(path.trim_start_matches("/dev/"), play);

    // An exclusive channel (bitperfect or vchans off) negotiates the native
    // values verbatim, and a probe open would briefly claim the only channel;
    // build the caps from sndstat without opening at all.
    if let Some(nv) = &native
        && (nv.bitperfect || nv.exclusive == Some(true))
    {
        let mut rates = native_rates(path, play);
        // native_rates opens the device briefly; if that failed (busy) on a
        // bitperfect device, fall back to the native EXTREMES - they are
        // themselves native, while a dense min..max range would admit
        // pitch-shifting non-native rates (playback echoes the request back)
        if nv.bitperfect && rates.is_empty() && nv.min_rate != nv.max_rate {
            rates = vec![nv.min_rate.max(1), nv.max_rate.max(nv.min_rate).max(1)];
        }
        return Some(caps_from_sndstat(nv, rates));
    }

    let cpath = CString::new(path).ok()?;
    let mode = if play { libc::O_WRONLY } else { libc::O_RDONLY };
    let Some(fd) = LibcFd::open(&cpath, mode | libc::O_NONBLOCK) else {
        // busy or transiently gone: the native info still beats the caller's
        // conservative stereo fallback
        return native.as_ref().map(|nv| caps_from_sndstat(nv, vec![]));
    };
    let raw_fd = fd.raw();

    let formats = supported_formats(raw_fd);

    // ENGINEINFO with dev == -1 resolves the channel bound to THIS fd, so the
    // limits are per-direction (AUDIOINFO blends play and rec across the
    // device). Note: kernels before the 15.x sound rewrite report a vchan's
    // fixed rate here instead of the feeder range; harmless, since these
    // values are only consulted when the empirical probe fails.
    let (ai_min_ch, ai_max_ch, ai_min_rate, ai_max_rate, ai_caps) =
        engine_info(raw_fd).map_or((0, 0, 0, 0, 0), |ai| {
            (
                ai.min_channels,
                ai.max_channels,
                ai.min_rate,
                ai.max_rate,
                ai.caps,
            )
        });
    // a failed or degenerate probe defers to the audioinfo limits
    let pick = |probed: Option<c_int>, ai_val: c_int| probed.filter(|&v| v >= 1).unwrap_or(ai_val);

    // On a vchan the feeder converts and SETCHANNELS clamps at SND_CHN_MAX, so
    // advertising the engine's wider native count would only fail at configure
    // time. On a DIRECT channel (bitperfect / vchans off) the grant snaps to a
    // native format and wider counts are genuinely negotiable, so the engine
    // width extends the probe there (e.g. 10-channel USB mixers).
    let direct = ai_caps != 0 && ai_caps & PCM_CAP_VIRTUAL == 0;
    let min_channels = pick(probe_min_channels(raw_fd), ai_min_ch);
    let max_channels = {
        let probed = pick(probe_max_channels(raw_fd), ai_max_ch);
        if direct {
            probed.max(ai_max_ch)
        } else {
            probed
        }
    };
    let min_rate = pick(probe_rate(raw_fd, 8000), ai_min_rate);
    let max_rate = pick(probe_rate(raw_fd, 192000), ai_max_rate);

    let formats = formats?;
    if min_channels < 1 || max_channels < min_channels || min_rate < 1 || max_rate < min_rate {
        return None;
    }

    // On a vchan the parent hardware mixes at its vchanrate (from the sndstat
    // nvlist); preferring it avoids a second in-kernel resample on non-48k
    // parents. Zero/absent (direct channel) just means no preference.
    let preferred_rate = native
        .as_ref()
        .map(|nv| nv.vchan_rate)
        .filter(|r| *r != 0 && (min_rate as u32..=max_rate as u32).contains(r));

    Some(StreamCaps {
        configurations: vec![StreamConfiguration {
            formats: super::backend::formats_from_native_mask(formats as u32, false),
            min_channels: min_channels as u32,
            max_channels: max_channels as u32,
            min_rate: min_rate as u32,
            max_rate: max_rate as u32,
            preferred_rate,
            rates: vec![], // the feeder converts; the range really is dense
            rate_tolerance: feeder_rate_round(),
        }],
        preferred: 0,
        conversion: ConversionKind::Backend,
    })
}
// The device's real drain quantum, as TIME: the hardware buffer blocksize of
// the primary (non-virtual) channel for the direction, from the sndstat
// channel info. The soft fragsize GETOSPACE reports can understate it badly
// on drivers that ignore SETFRAGMENT and pull fixed transfers (uaudio:
// buffer_ms of audio per completion) - and for vchan children the parent's
// hardware cadence governs the mix pull the same way. Time-domain so a later
// rate renegotiation converts cleanly; drivers with rate-proportional blocks
// (fixed frame counts) read slightly large or small across rates, which only
// shifts a floor. 0 = unknown, use the soft fragsize alone.
pub(crate) fn drain_quantum_ns(devnode: &str, play: bool) -> u64 {
    let devnode = devnode.trim_start_matches("/dev/"); // sndstat devnodes are bare
    let want_dir = if play {
        PCM_CAP_OUTPUT as u64
    } else {
        PCM_CAP_INPUT as u64
    };
    let mut quantum: u64 = 0;
    let Some(nvl) = sndstat_snapshot() else {
        return 0;
    };
    for dev in nvl.root().nvlist_array(c"dsps") {
        if dev.boolean(c"from_user").unwrap_or(false) || dev.string(c"devnode") != Some(devnode) {
            continue;
        }
        let Some(p) = dev.nvlist(c"provider_info") else {
            return 0;
        };
        for chan in p.nvlist_array(c"channel_info") {
            let caps = chan.number(c"caps").unwrap_or(0);
            if caps & PCM_CAP_VIRTUAL as u64 != 0 || caps & want_dir == 0 {
                continue;
            }
            let blksz = chan.number(c"hwbuf_blksz").unwrap_or(0);
            let rate = chan.number(c"hwbuf_rate").unwrap_or(0);
            let stride = afmt_frame_bytes(chan.number(c"hwbuf_format").unwrap_or(0) as u32) as u64;
            if blksz > 0 && rate > 0 {
                quantum = quantum.max(blksz.saturating_mul(1_000_000_000) / (rate * stride));
            }
        }
        break;
    }
    quantum
}
fn read_sndstat() -> Result<Vec<u32>, Errno> {
    // sndstat's nvlist interface; the plugin assumes FreeBSD 14.4+
    sndstat_pcm_devices()
        .map(|devs| devs.into_iter().map(|(unit, _, _)| unit).collect())
        .ok_or(Errno::ENXIO)
}

#[derive(Debug)]
pub(crate) struct PcmDevice {
    pub index: u32,
    pub desc: String,
    pub location: String,
    pub play: bool,
    pub rec: bool,
}

pub(crate) fn read_pcm_device_description(sysctl: &mut SysctlReader, index: u32) -> Option<String> {
    let parent = sysctl
        .read_string(format!("dev.pcm.{index}.%parent"), 1024)
        .ok()?; // the device can detach mid-enumeration
    if let Some(unit_str) = parent.strip_prefix("uaudio")
        && let Ok(unit) = unit_str.parse::<u32>()
        && let Ok(desc) = sysctl.read_string(format!("dev.uaudio.{unit}.%desc"), 1024)
    {
        // let's get rid of ", class %d/%d, rev %x.%02x/%x.%02x, addr %d" suffix
        static RE: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
            regex::Regex::new(r"^(.*?), class \d+/\d+, rev [^\s]+, addr \d$").unwrap()
        });
        if let Some(groups) = RE.captures(&desc)
            && let Some(prefix) = groups.get(1)
        {
            return Some(prefix.as_str().to_string());
        }
        return Some(desc);
    }

    sysctl
        .read_string(format!("dev.pcm.{index}.%desc"), 1024)
        .ok()
}

fn group_pcm_devices_by_parent(indexes: &[u32]) -> BTreeMap<String, Vec<u32>> {
    let mut sysctl = SysctlReader::new();
    let mut indexes_by_parent: BTreeMap<String, Vec<u32>> = BTreeMap::new();
    for index in indexes {
        if let Ok(parent) = sysctl.read_string(format!("dev.pcm.{index}.%parent"), 1024) {
            let values = indexes_by_parent.entry(parent).or_default();
            values.push(*index);
        }
    }
    indexes_by_parent
}

pub(crate) fn read_device_groups() -> Result<BTreeMap<String, Vec<u32>>, Errno> {
    read_sndstat().map(|indexes| group_pcm_devices_by_parent(&indexes))
}

pub(crate) fn list_audio_devices(indexes: &[u32]) -> Vec<PcmDevice> {
    let mut result = Vec::with_capacity(indexes.len());
    let mut sysctl = SysctlReader::new();
    // Direction support from the nvlist channel counts (vchans on or off);
    // dev.pcm.N.mode (1 = mixer, 2 = play, 4 = rec) only covers a transient
    // nvlist failure.
    let chans = sndstat_pcm_devices();

    for index in indexes {
        if let Some(desc) = read_pcm_device_description(&mut sysctl, *index)
            && let Ok(location) = sysctl.read_string(format!("dev.pcm.{index}.%location"), 1024)
        {
            let from_nv = chans.as_ref().and_then(|c| {
                c.iter()
                    .find(|(unit, _, _)| unit == index)
                    .map(|&(_, play, rec)| (play, rec))
            });
            let (play, rec) = match from_nv {
                Some(dirs) => dirs,
                None => match sysctl.read_u32(format!("dev.pcm.{index}.mode")) {
                    Ok(mode) => (mode & 2 != 0, mode & 4 != 0),
                    Err(_) => (false, false),
                },
            };
            result.push(PcmDevice {
                index: *index,
                desc,
                location,
                play,
                rec,
            });
        }
    }

    result
}

#[cfg(test)]
mod tests {
    #[test]
    fn pcm_format_widths_cover_u8_float_and_three_byte_24() {
        const STEREO: u32 = 2 << 20;

        assert_eq!(super::afmt_frame_bytes(super::AFMT_U8), 1);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U8 | STEREO), 2);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_S24_LE), 3);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_S24_BE | STEREO), 6);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_F32_LE), 4);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_F32_BE | STEREO), 8);
    }

    #[test]
    fn drain_quantum_probe() {
        for unit in [0u32, 1, 6] {
            let node = format!("/dev/dsp{unit}"); // the production string shape
            println!(
                "{}: play {} ns, rec {} ns",
                node,
                super::drain_quantum_ns(&node, true),
                super::drain_quantum_ns(&node, false)
            );
        }
    }
}
