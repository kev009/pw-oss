use nix::errno::Errno;
use std::collections::BTreeMap;
use std::ffi::{CString, c_char, c_int};

use super::abi::*;
use super::sys::{LibcFd, NvList, NvRef, SysctlReader};
use crate::backend::{
    CatalogChange, CatalogError, CatalogGroupSnapshot, CatalogRescan, ConfigurationFlags,
    ConversionPath, DeliveryQuantum, DeviceKey, DeviceSnapshot, EndpointKey, EndpointSnapshot,
    QuantumQuality, RateSet, StreamCaps, StreamConfiguration, StreamDirection, StreamLocator,
};

// Capability discovery merges sndstat's per-direction limits with OSS engine
// details. Exclusive endpoints stay unclaimed: indexed ENGINEINFO requests go
// through their mixer control descriptor. Shareable endpoints additionally
// use a transient DSP open for empirical SETCHANNELS/SPEED probes; OSS grants
// the nearest supported value, and the caller falls back if that open is busy.

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

#[cfg(test)]
pub(super) fn snd_dummy_unit() -> Option<u32> {
    let mut sysctl = SysctlReader::new();
    sndstat_pcm_devices()?
        .into_iter()
        .map(|(unit, _, _)| unit)
        .find(|unit| {
            sysctl
                .read_string(format!("dev.pcm.{unit}.%desc"), 1024)
                .is_ok_and(|description| description == "Dummy Audio Device")
        })
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
            channels: super::backend::channel_layouts(
                nv.min_chn.max(1),
                nv.max_chn.max(nv.min_chn).max(1),
            ),
            rates: if rates.is_empty() {
                RateSet::Range {
                    min: nv.min_rate.max(1),
                    max: nv.max_rate.max(nv.min_rate).max(1),
                }
            } else {
                RateSet::Discrete(rates)
            },
            preferred_rate: None, // the native values are the preference
            rate_tolerance: feeder_rate_round(),
            conversion: if nv.bitperfect {
                ConversionPath::None
            } else {
                ConversionPath::Kernel
            },
            flags: if nv.bitperfect {
                ConfigurationFlags::with_opaque_layout()
            } else {
                ConfigurationFlags::with_layout_reorder_and_opaque()
            },
        }],
        preferred: 0,
    }
}

fn mixer_control_path(path: &str) -> Option<(CString, u32)> {
    let unit = path
        .trim_start_matches("/dev/")
        .strip_prefix("dsp")?
        .parse::<u32>()
        .ok()?;
    Some((CString::new(format!("/dev/mixer{unit}")).ok()?, unit))
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct BusyOwner {
    pid: Option<c_int>,
    command: Option<String>,
}

fn c_char_text(value: &[c_char]) -> Option<String> {
    let end = value
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(value.len());
    let bytes = value[..end]
        .iter()
        .map(|byte| *byte as u8)
        .collect::<Vec<_>>();
    let text = String::from_utf8_lossy(&bytes).trim().to_string();
    (!text.is_empty()).then_some(text)
}

fn select_busy_owner(
    unit: u32,
    play: bool,
    infos: impl IntoIterator<Item = oss_audioinfo>,
) -> Option<BusyOwner> {
    let direction_cap = if play { PCM_CAP_OUTPUT } else { PCM_CAP_INPUT };
    let open_mode = if play {
        PCM_ENABLE_OUTPUT
    } else {
        PCM_ENABLE_INPUT
    };
    // A busy primary channel hosting vchans may precede the process-owned
    // virtual channel and still carry the kernel's "<UNUSED>" identity.
    // Prefer an attributed engine, while retaining an unattributed match so
    // the diagnostic can still distinguish a busy channel from no match.
    // ENGINEINFO cannot prove which allocation blocked the failed open, so
    // even the preferred identity remains diagnostic context only.
    let mut best: Option<BusyOwner> = None;
    for info in infos.into_iter().filter(|info| {
        info.card_number == unit as c_int
            && info.caps & direction_cap != 0
            && info.busy & open_mode != 0
    }) {
        let owner = BusyOwner {
            pid: (info.pid > 0).then_some(info.pid),
            command: c_char_text(&info.cmd).filter(|command| command != "<UNUSED>"),
        };
        if owner.pid.is_some() {
            return Some(owner);
        }
        if best
            .as_ref()
            .is_none_or(|current| current.command.is_none() && owner.command.is_some())
        {
            best = Some(owner);
        }
    }
    best
}

fn busy_owner(path: &str, play: bool) -> Option<BusyOwner> {
    let (control_path, unit) = mixer_control_path(path)?;
    let fd = LibcFd::open(&control_path, libc::O_RDONLY | libc::O_NONBLOCK)?;
    let infos = (0..4096).map_while(|device| engine_info_at(fd.raw(), device));
    select_busy_owner(unit, play, infos)
}

pub(super) fn open_failure_diagnostic(path: &str, play: bool, error: Errno) -> Option<String> {
    let direction = if play { "playback" } else { "capture" };
    match error {
        Errno::EACCES | Errno::EPERM => Some(format!(
            "{path}: {direction} access denied; check devfs permissions and ACLs for the PipeWire service user"
        )),
        Errno::EBUSY => {
            let detail = match busy_owner(path, play) {
                Some(BusyOwner {
                    pid: Some(pid),
                    command: Some(command),
                }) => format!("held by {command}, pid {pid}"),
                Some(BusyOwner {
                    pid: Some(pid),
                    command: None,
                }) => format!("held by pid {pid}"),
                Some(BusyOwner {
                    pid: None,
                    command: Some(command),
                }) => format!("held by {command}"),
                Some(BusyOwner {
                    pid: None,
                    command: None,
                })
                | None => "owner unavailable".to_string(),
            };
            Some(format!("{path}: {direction} channel is busy ({detail})"))
        }
        _ => None,
    }
}

fn collect_native_rates(
    unit: u32,
    play: bool,
    infos: impl IntoIterator<Item = oss_audioinfo>,
) -> Vec<u32> {
    let direction = if play { PCM_CAP_OUTPUT } else { PCM_CAP_INPUT };
    let mut rates = vec![];
    for ai in infos {
        if ai.card_number != unit as c_int
            || ai.caps & direction == 0
            || ai.caps & PCM_CAP_VIRTUAL != 0
        {
            continue;
        }
        for i in 0..ai.nrates.min(20) as usize {
            rates.push(ai.rates[i]);
        }
    }
    rates.retain(|r| *r > 0);
    rates.sort_unstable();
    rates.dedup();
    rates
}

// The native rate set of an exclusive device. Query indexed ENGINEINFO
// records through the mixer control descriptor: unlike opening /dev/dspN,
// this never claims the device's only PCM channel. Bitperfect rates are not a
// dense range; admitting an arbitrary in-range playback rate can pitch-shift
// while SNDCTL_DSP_SPEED still echoes the requested value.
fn native_rates(path: &str, play: bool) -> Vec<u32> {
    let Some((control_path, unit)) = mixer_control_path(path) else {
        return vec![];
    };
    let Some(fd) = LibcFd::open(&control_path, libc::O_RDONLY | libc::O_NONBLOCK) else {
        return vec![];
    };
    let infos = (0..4096).map_while(|device| engine_info_at(fd.raw(), device));
    collect_native_rates(unit, play, infos)
}

fn exclusive_rate_fallback(nv: &SndstatDspInfo, mut rates: Vec<u32>) -> Vec<u32> {
    if rates.is_empty() {
        let min = nv.min_rate.max(1);
        let max = nv.max_rate.max(nv.min_rate).max(1);
        rates.push(min);
        if max != min {
            rates.push(max);
        }
    }
    rates
}

pub(crate) fn probe_caps(path: &str, play: bool) -> Option<StreamCaps> {
    let native = sndstat_dsp_info(path.trim_start_matches("/dev/"), play);

    // An exclusive channel (bitperfect or vchans off) negotiates the native
    // values verbatim. Build its limits from sndstat and obtain any discrete
    // rate list through a non-PCM control descriptor.
    if let Some(nv) = &native
        && (nv.bitperfect || nv.exclusive == Some(true))
    {
        // If indexed ENGINEINFO is unavailable, use the native extrema for
        // every exclusive endpoint. They are incomplete but safe, whereas a
        // dense range would admit pitch-shifting non-native rates.
        let rates = exclusive_rate_fallback(nv, native_rates(path, play));
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
            channels: super::backend::channel_layouts(min_channels as u32, max_channels as u32),
            rates: RateSet::Range {
                min: min_rate as u32,
                max: max_rate as u32,
            },
            preferred_rate,
            rate_tolerance: feeder_rate_round(),
            conversion: ConversionPath::Kernel,
            flags: ConfigurationFlags::with_layout_reorder_and_opaque(),
        }],
        preferred: 0,
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
// shifts a floor. Unavailable means to use the soft fragment size alone.
pub(crate) fn delivery_quantum(devnode: &str, play: bool) -> DeliveryQuantum {
    let devnode = devnode.trim_start_matches("/dev/"); // sndstat devnodes are bare
    let want_dir = if play {
        PCM_CAP_OUTPUT as u64
    } else {
        PCM_CAP_INPUT as u64
    };
    let mut quantum = DeliveryQuantum::unavailable();
    let Some(nvl) = sndstat_snapshot() else {
        return quantum;
    };
    for dev in nvl.root().nvlist_array(c"dsps") {
        if dev.boolean(c"from_user").unwrap_or(false) || dev.string(c"devnode") != Some(devnode) {
            continue;
        }
        let Some(p) = dev.nvlist(c"provider_info") else {
            return quantum;
        };
        for chan in p.nvlist_array(c"channel_info") {
            let caps = chan.number(c"caps").unwrap_or(0);
            if caps & PCM_CAP_VIRTUAL as u64 != 0 || caps & want_dir == 0 {
                continue;
            }
            let blksz = chan.number(c"hwbuf_blksz").unwrap_or(0);
            let rate = chan.number(c"hwbuf_rate").unwrap_or(0);
            let stride = afmt_frame_bytes(chan.number(c"hwbuf_format").unwrap_or(0) as u32) as u64;
            if blksz > 0 && rate > 0 && stride > 0 {
                let candidate = DeliveryQuantum {
                    frames: (blksz / stride).min(u64::from(u32::MAX)) as u32,
                    rate: rate.min(u64::from(u32::MAX)) as u32,
                    quality: QuantumQuality::Estimated,
                };
                if candidate.duration_ns() > quantum.duration_ns() {
                    quantum = candidate;
                }
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
struct PcmDevice {
    index: u32,
    desc: String,
    location: String,
    play: bool,
    rec: bool,
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

fn read_device_groups() -> Result<BTreeMap<String, Vec<u32>>, Errno> {
    read_sndstat().map(|indexes| group_pcm_devices_by_parent(&indexes))
}

fn list_audio_devices(indexes: &[u32]) -> Vec<PcmDevice> {
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

fn common_description(devices: &[PcmDevice]) -> String {
    // A sndstat group represents several PCM endpoints as one SPA device, so
    // name it with their longest common description prefix. Drivers commonly
    // append endpoint details in parentheses; trimming a dangling " (" keeps
    // the shared group label readable when that suffix is where they diverge.
    let mut description = devices[0].desc.clone();
    for device in &devices[1..] {
        let count = description
            .chars()
            .zip(device.desc.chars())
            .take_while(|(a, b)| a == b)
            .map(|(c, _)| c.len_utf8())
            .sum();
        description.truncate(count);
    }
    while description.ends_with(' ') || description.ends_with('(') {
        description.truncate(description.len() - 1);
    }
    description
}

pub(crate) fn device_snapshot(indexes: &[u32]) -> Option<DeviceSnapshot> {
    let devices = list_audio_devices(indexes);
    if devices.is_empty() {
        return None;
    }
    let description = common_description(&devices);
    let mut endpoints = Vec::new();
    for device in devices {
        let endpoint_description = if device.desc == description && !device.location.is_empty() {
            format!("{} @ {}", device.desc, device.location)
        } else {
            device.desc
        };
        for (direction, enabled, suffix) in [
            (StreamDirection::Playback, device.play, "play"),
            (StreamDirection::Capture, device.rec, "rec"),
        ] {
            if enabled {
                let name = format!("pcm{}.{}", device.index, suffix);
                endpoints.push(EndpointSnapshot {
                    key: EndpointKey::qualified(super::identity::DEVICE_API, &name),
                    object_id: device.index * 2 + u32::from(direction == StreamDirection::Capture),
                    direction,
                    name,
                    description: endpoint_description.clone(),
                    locator: StreamLocator::new(
                        super::identity::DEVICE_API,
                        super::identity::stream_path(device.index),
                    ),
                });
            }
        }
    }
    Some(DeviceSnapshot {
        description,
        endpoints,
    })
}

fn group_snapshot(key: String, indexes: Vec<u32>) -> Option<CatalogGroupSnapshot> {
    let object_id = *indexes.first()?;
    let indexes_value = indexes
        .iter()
        .map(u32::to_string)
        .collect::<Vec<_>>()
        .join(",");
    Some(CatalogGroupSnapshot {
        properties: vec![
            (super::identity::PARENT_DEVICE.to_string(), key.clone()),
            (super::identity::DEVICE_INDEXES.to_string(), indexes_value),
        ],
        key: DeviceKey::qualified(super::identity::DEVICE_API, &key),
        object_id,
    })
}

pub(crate) struct DeviceCatalog {
    groups: BTreeMap<String, Vec<u32>>,
}

impl DeviceCatalog {
    pub(crate) const fn open_error_context() -> &'static str {
        "Can't open /dev/sndstat"
    }

    pub(crate) const fn refresh_error_context() -> &'static str {
        "can't re-read sndstat"
    }

    pub(crate) fn scan() -> Result<Self, CatalogError> {
        read_device_groups()
            .map(|groups| Self { groups })
            .map_err(|error| CatalogError::new(error as i32, error.desc()))
    }

    pub(crate) fn snapshots(&self) -> Vec<CatalogGroupSnapshot> {
        self.groups
            .iter()
            .filter_map(|(key, indexes)| group_snapshot(key.clone(), indexes.clone()))
            .collect()
    }

    pub(super) fn rescan(&mut self, detached: &[String]) -> CatalogRescan {
        self.rescan_with(detached, read_device_groups)
    }

    fn rescan_with(
        &mut self,
        detached: &[String],
        read: impl FnOnce() -> Result<BTreeMap<String, Vec<u32>>, nix::errno::Errno>,
    ) -> CatalogRescan {
        let mut changes = Vec::new();
        // Force-retract a named group before reading sndstat. A fast replug can
        // deliver the '-' event after a replacement has attached with the same
        // nameunit and index set; a plain old/new map diff would then look
        // unchanged and leave nodes bound to the retired hardware instance.
        for subject in detached {
            let key = if let Some(unit) = subject
                .strip_prefix("pcm")
                .and_then(|unit| unit.parse::<u32>().ok())
            {
                self.groups
                    .iter()
                    .find(|(_, indexes)| indexes.contains(&unit))
                    .map(|(key, _)| key.clone())
            } else if self.groups.contains_key(subject) {
                Some(subject.clone())
            } else {
                None
            };
            if let Some(key) = key
                && let Some(indexes) = self.groups.remove(&key)
                && let Some(object_id) = indexes.first().copied()
            {
                changes.push(CatalogChange::Removed {
                    object_id,
                    diagnostic: format!("{key} ({indexes:?}) on detach"),
                });
            }
        }

        let new_groups = match read() {
            Ok(groups) => groups,
            Err(error) => {
                return CatalogRescan {
                    changes,
                    error: Some(CatalogError::new(error as i32, error.desc())),
                };
            }
        };
        let old_groups = std::mem::replace(&mut self.groups, new_groups);
        for (key, indexes) in &old_groups {
            if self.groups.get(key) != Some(indexes)
                && let Some(object_id) = indexes.first().copied()
            {
                changes.push(CatalogChange::Removed {
                    object_id,
                    diagnostic: format!("{key} ({indexes:?})"),
                });
            }
        }
        for (key, indexes) in &self.groups {
            if old_groups.get(key) != Some(indexes)
                && let Some(snapshot) = group_snapshot(key.clone(), indexes.clone())
            {
                changes.push(CatalogChange::Added {
                    snapshot,
                    diagnostic: format!("{key} ({indexes:?})"),
                });
            }
        }
        CatalogRescan {
            changes,
            error: None,
        }
    }
}

impl crate::backend::DeviceCatalog for DeviceCatalog {
    fn open_error_context() -> &'static str {
        Self::open_error_context()
    }

    fn refresh_error_context() -> &'static str {
        Self::refresh_error_context()
    }

    fn scan() -> Result<Self, CatalogError> {
        Self::scan()
    }

    fn snapshots(&self) -> Vec<CatalogGroupSnapshot> {
        self.snapshots()
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::ffi::c_char;

    use nix::errno::Errno;

    use super::{
        BusyOwner, DeviceCatalog, PcmDevice, SndstatDspInfo, collect_native_rates,
        common_description, exclusive_rate_fallback, mixer_control_path, open_failure_diagnostic,
        select_busy_owner,
    };
    use crate::freebsd_oss::abi::{
        PCM_CAP_INPUT, PCM_CAP_OUTPUT, PCM_CAP_VIRTUAL, PCM_ENABLE_INPUT, PCM_ENABLE_OUTPUT,
        oss_audioinfo,
    };

    fn pcm(desc: &str) -> PcmDevice {
        PcmDevice {
            index: 0,
            desc: desc.to_string(),
            location: String::new(),
            play: true,
            rec: false,
        }
    }

    fn engine(unit: i32, caps: i32, rates: &[u32]) -> oss_audioinfo {
        let mut info = unsafe { std::mem::zeroed::<oss_audioinfo>() };
        info.card_number = unit;
        info.caps = caps;
        info.nrates = rates.len() as u32;
        info.rates[..rates.len()].copy_from_slice(rates);
        info
    }

    fn busy_engine(unit: i32, caps: i32, busy: i32, pid: i32, command: &[u8]) -> oss_audioinfo {
        let mut info = engine(unit, caps, &[]);
        info.busy = busy;
        info.pid = pid;
        for (slot, byte) in info.cmd.iter_mut().zip(command.iter().copied()) {
            *slot = byte as c_char;
        }
        info
    }

    #[test]
    fn busy_owner_matches_pcm_unit_direction_and_open_state() {
        let infos = [
            busy_engine(3, PCM_CAP_OUTPUT, PCM_ENABLE_OUTPUT, 10, b"wrong-unit"),
            busy_engine(2, PCM_CAP_INPUT, PCM_ENABLE_INPUT, 11, b"capture"),
            busy_engine(2, PCM_CAP_OUTPUT, 0, 12, b"idle"),
            busy_engine(2, PCM_CAP_OUTPUT, PCM_ENABLE_OUTPUT, 13, b"music\0ignored"),
        ];
        assert_eq!(
            select_busy_owner(2, true, infos),
            Some(BusyOwner {
                pid: Some(13),
                command: Some("music".into()),
            })
        );
        assert_eq!(
            select_busy_owner(2, false, infos),
            Some(BusyOwner {
                pid: Some(11),
                command: Some("capture".into()),
            })
        );
    }

    #[test]
    fn busy_owner_prefers_attributed_vchan_over_unattributed_primary() {
        let infos = [
            busy_engine(2, PCM_CAP_OUTPUT, PCM_ENABLE_OUTPUT, -1, b"<UNUSED>"),
            busy_engine(
                2,
                PCM_CAP_OUTPUT | PCM_CAP_VIRTUAL,
                PCM_ENABLE_OUTPUT,
                42,
                b"pipewire",
            ),
        ];
        assert_eq!(
            select_busy_owner(2, true, infos),
            Some(BusyOwner {
                pid: Some(42),
                command: Some("pipewire".into()),
            })
        );
    }

    #[test]
    fn busy_owner_retains_an_unattributed_busy_engine() {
        assert_eq!(
            select_busy_owner(
                2,
                true,
                [busy_engine(
                    2,
                    PCM_CAP_OUTPUT,
                    PCM_ENABLE_OUTPUT,
                    -1,
                    b"<UNUSED>",
                )],
            ),
            Some(BusyOwner {
                pid: None,
                command: None,
            })
        );
    }

    #[test]
    fn open_failure_diagnostic_is_limited_to_access_and_busy_errors() {
        assert_eq!(
            open_failure_diagnostic("/dev/dsp2", true, Errno::EACCES),
            Some(
                "/dev/dsp2: playback access denied; check devfs permissions and ACLs for the PipeWire service user"
                    .into()
            )
        );
        assert!(
            open_failure_diagnostic("/dev/dsp.invalid", false, Errno::EBUSY)
                .is_some_and(|message| message.ends_with("(owner unavailable)"))
        );
        assert_eq!(
            open_failure_diagnostic("/dev/dsp2", true, Errno::ENODEV),
            None
        );
    }

    #[test]
    fn unterminated_engine_command_is_bounded_by_its_abi_field() {
        let info = busy_engine(2, PCM_CAP_OUTPUT, PCM_ENABLE_OUTPUT, 13, &[b'x'; 64]);
        assert_eq!(
            select_busy_owner(2, true, [info])
                .and_then(|owner| owner.command)
                .as_deref(),
            Some("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        );
    }

    #[test]
    fn exclusive_rates_use_matching_direct_control_engines() {
        let infos = [
            engine(3, PCM_CAP_OUTPUT, &[96_000, 48_000]),
            engine(3, PCM_CAP_OUTPUT | PCM_CAP_VIRTUAL, &[44_100]),
            engine(3, PCM_CAP_INPUT, &[32_000]),
            engine(4, PCM_CAP_OUTPUT, &[192_000]),
            engine(3, PCM_CAP_OUTPUT, &[48_000, 192_000]),
        ];
        assert_eq!(
            collect_native_rates(3, true, infos),
            [48_000, 96_000, 192_000]
        );
    }

    #[test]
    fn exclusive_rate_probe_uses_the_pcm_units_mixer() {
        let (path, unit) = mixer_control_path("/dev/dsp12").unwrap();
        assert_eq!(path.to_str().unwrap(), "/dev/mixer12");
        assert_eq!(unit, 12);
        assert!(mixer_control_path("/dev/dsp").is_none());
        assert!(mixer_control_path("/dev/audio12").is_none());
    }

    #[test]
    fn every_exclusive_endpoint_falls_back_to_discrete_rate_extrema() {
        let info = SndstatDspInfo {
            formats: 0,
            min_rate: 44_100,
            max_rate: 192_000,
            min_chn: 1,
            max_chn: 2,
            exclusive: Some(true),
            vchan_rate: 0,
            bitperfect: false,
        };
        assert_eq!(exclusive_rate_fallback(&info, vec![]), [44_100, 192_000]);
        assert_eq!(exclusive_rate_fallback(&info, vec![96_000]), [96_000]);
    }

    #[test]
    fn common_description_stops_at_a_utf8_character_boundary() {
        assert_eq!(
            common_description(&[pcm("Beyoncé DAC"), pcm("Beyoncê ADC")]),
            "Beyonc"
        );
    }

    #[test]
    fn pcm_format_widths_cover_every_supported_storage_width() {
        const STEREO: u32 = 2 << 20;

        assert_eq!(super::afmt_frame_bytes(super::AFMT_MU_LAW), 1);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_A_LAW | STEREO), 2);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U8), 1);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U8 | STEREO), 2);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_S8 | STEREO), 2);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U16_LE), 2);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U16_BE | STEREO), 4);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_S24_LE), 3);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_S24_BE | STEREO), 6);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U24_LE), 3);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U24_BE | STEREO), 6);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U32_LE), 4);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_U32_BE | STEREO), 8);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_F32_LE), 4);
        assert_eq!(super::afmt_frame_bytes(super::AFMT_F32_BE | STEREO), 8);
    }

    #[test]
    fn detach_survives_a_failed_catalog_read() {
        let mut catalog = DeviceCatalog {
            groups: BTreeMap::from([("usb-dac".to_string(), vec![3, 4])]),
        };

        let result = catalog.rescan_with(&["pcm3".to_string()], || Err(nix::errno::Errno::EIO));

        assert_eq!(
            result.changes,
            vec![crate::backend::CatalogChange::Removed {
                object_id: 3,
                diagnostic: "usb-dac ([3, 4]) on detach".into(),
            }]
        );
        assert!(result.error.is_some());
        assert!(catalog.groups.is_empty());
    }

    #[test]
    fn detach_retracts_and_readds_an_identical_replacement() {
        let mut catalog = DeviceCatalog {
            groups: BTreeMap::from([("usb-dac".to_string(), vec![3, 4])]),
        };

        let result = catalog.rescan_with(&["pcm3".to_string()], || {
            Ok(BTreeMap::from([("usb-dac".to_string(), vec![3, 4])]))
        });

        assert!(result.error.is_none());
        assert_eq!(result.changes.len(), 2);
        assert!(matches!(
            &result.changes[0],
            crate::backend::CatalogChange::Removed {
                object_id: 3,
                diagnostic
            } if diagnostic == "usb-dac ([3, 4]) on detach"
        ));
        assert!(matches!(
            &result.changes[1],
            crate::backend::CatalogChange::Added { snapshot, diagnostic }
                if snapshot.key.as_str() == "freebsd-oss:usb-dac"
                    && snapshot.object_id == 3
                    && diagnostic == "usb-dac ([3, 4])"
        ));
    }

    #[test]
    #[ignore = "manual FreeBSD hardware probe"]
    fn delivery_quantum_probe() {
        for unit in [0u32, 1, 6] {
            let node = format!("/dev/dsp{unit}"); // the production string shape
            println!(
                "{}: play {:?}, rec {:?}",
                node,
                super::delivery_quantum(&node, true),
                super::delivery_quantum(&node, false)
            );
        }
    }
}
