// OSS mixer access (/dev/mixerN) for hardware volume; main-thread only.
//
// The mixer unit number is the pcm unit number (mixer.c:649 creates
// "mixer%d" with the pcm unit), so the device is opened directly by unit
// instead of probing. SNDCTL_MIXERINFO{dev=N} is never used: resolving a
// foreign unit through it can NULL-deref in the kernel (mixer.c:1232-1245).
// Volume ioctls never go to a dsp fd either - those address the per-channel
// VPC volume, which every open() resets to 0 dB (channel.c:1216-1219).
//
// Every ioctl here degrades to None/false on failure: a mixer can go away
// with its (hotpluggable) device at any time.

use std::collections::BTreeMap;
use std::ffi::{CString, c_char, c_int, c_uint, c_ulong};

use libspa::sys::{SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR};

use super::event::HotplugMonitor;
use super::sys::{IoctlPod, LibcFd, ioctl_int, ioctl_read};
use crate::backend::{
    DeviceSnapshot, RouteAvailability, RouteChange, RouteDiagnostic, RouteKey, RouteMute,
    RouteSelectionOutcome, RouteSnapshot, RouteUpdate, RouteValueUpdate, RouteVolume,
    RouteWatchPolicy, StreamDirection,
};

pub(crate) const SOUND_MIXER_VOLUME: c_uint = 0;
pub(crate) const SOUND_MIXER_PCM: c_uint = 4;
pub(crate) const SOUND_MIXER_LINE: c_uint = 6;
pub(crate) const SOUND_MIXER_MIC: c_uint = 7;
pub(crate) const SOUND_MIXER_RECLEV: c_uint = 11;
pub(crate) const SOUND_MIXER_IGAIN: c_uint = 12;

pub(crate) const SOUND_MIXER_NRDEVICES: c_uint = 25;

const MIXER_SOURCE_MIC: c_uint = SOUND_MIXER_MIC;
const MIXER_SOURCE_LINE: c_uint = SOUND_MIXER_LINE;
pub(crate) const MIXER_SOURCE_COUNT: c_uint = SOUND_MIXER_NRDEVICES;

const SOUND_MIXER_MUTE: c_uint = 28;
const SOUND_MIXER_RECSRC: c_uint = 0xff;
const SOUND_MIXER_DEVMASK: c_uint = 0xfe;
const SOUND_MIXER_RECMASK: c_uint = 0xfd;

// sys/soundcard.h SOUND_DEVICE_NAMES, indexed by mixer device number: the
// stable identifiers mixer(8) exposes; capture route names derive from them
pub(crate) const SOUND_DEVICE_NAMES: [&str; SOUND_MIXER_NRDEVICES as usize] = [
    "vol", "bass", "treble", "synth", "pcm", "speaker", "line", "mic", "cd", "mix", "pcm2", "rec",
    "igain", "ogain", "line1", "line2", "line3", "dig1", "dig2", "dig3", "phin", "phout", "video",
    "radio", "monitor",
];
pub(crate) const MIXER_SOURCE_NAMES: [&str; MIXER_SOURCE_COUNT as usize] = SOUND_DEVICE_NAMES;

pub(crate) fn mixer_source_priority(source: c_uint) -> c_int {
    match source {
        MIXER_SOURCE_MIC => 100,
        MIXER_SOURCE_LINE => 90,
        _ => 80 - source as c_int,
    }
}

// sys/soundcard.h mixer_info; the ioctl encodes the size
#[repr(C)]
#[derive(Clone, Copy)]
struct MixerInfo {
    id: [c_char; 16],
    name: [c_char; 32],
    modify_counter: c_int,
    fillers: [c_int; 10],
}

unsafe impl IoctlPod for MixerInfo {}

fn read_req(dev: c_uint) -> c_ulong {
    nix::request_code_read!(b'M', dev, size_of::<c_int>())
}

// MIXER_WRITE is _IOWR but FreeBSD does NOT echo the stored value back
// (only the read branch writes *arg_i, mixer.c); never read the buffer after
fn write_req(dev: c_uint) -> c_ulong {
    nix::request_code_readwrite!(b'M', dev, size_of::<c_int>())
}

const SOUND_MIXER_INFO: c_ulong = nix::request_code_read!(b'M', 101, size_of::<MixerInfo>());

pub(crate) struct Mixer {
    fd: LibcFd,
    devmask: u32,
    recmask: u32,
}

impl Mixer {
    // None when the pcm device has no mixer (ENOENT) or it can't be queried
    pub(crate) fn open(unit: u32) -> Option<Self> {
        let path = CString::new(format!("/dev/mixer{unit}")).ok()?;
        let fd = LibcFd::open(&path, libc::O_RDWR)?;
        let mut mixer = Self {
            fd,
            devmask: 0,
            recmask: 0,
        };
        let devmask = mixer.read_int(SOUND_MIXER_DEVMASK)?; // Drop closes fd on the error path
        mixer.devmask = devmask as u32;
        mixer.recmask = mixer.read_int(SOUND_MIXER_RECMASK).unwrap_or(0) as u32
            & ((1 << SOUND_MIXER_NRDEVICES) - 1);
        Some(mixer)
    }

    pub(crate) fn recmask(&self) -> u32 {
        self.recmask
    }

    fn read_int(&self, dev: c_uint) -> Option<c_int> {
        ioctl_int(self.fd.raw(), read_req(dev), 0)
    }

    fn write_int(&self, dev: c_uint, value: c_int) -> bool {
        ioctl_int(self.fd.raw(), write_req(dev), value).is_some()
    }

    fn has(&self, dev: c_uint) -> bool {
        self.devmask & (1 << dev) != 0
    }

    // Playback: vol, else pcm, else nothing. vol is nearly always present -
    // either a real codec amp or a synthetic parent rescaling pcm
    // (mixer.c:258-276); userland can't tell them apart, and vol matches what
    // mixer(8) users expect. pcm is deliberately not normalized: real pcm amps
    // can carry positive gain at 100.
    pub(crate) fn output_control(&self) -> Option<c_uint> {
        [SOUND_MIXER_VOLUME, SOUND_MIXER_PCM]
            .into_iter()
            .find(|d| self.has(*d))
    }

    // Capture: RECLEV (ADC-side on hdaa, survives RECSRC changes), else the
    // level control of the current RECSRC source, else IGAIN, else nothing.
    // the bool marks a recsrc-derived choice, which must be re-resolved when
    // the recording source changes (RECSRC writes don't tick modify_counter)
    pub(crate) fn input_control(&self) -> Option<(c_uint, bool)> {
        if self.has(SOUND_MIXER_RECLEV) {
            return Some((SOUND_MIXER_RECLEV, false));
        }
        if let Some(recsrc) = self.read_int(SOUND_MIXER_RECSRC) {
            let named = recsrc as u32 & self.recmask & self.devmask;
            for dev in 0..SOUND_MIXER_NRDEVICES {
                if named & (1 << dev) != 0 {
                    return Some((dev, true));
                }
            }
        }
        if self.has(SOUND_MIXER_IGAIN) {
            return Some((SOUND_MIXER_IGAIN, false));
        }
        None
    }

    // volume control for one selectable recording source: the source's own
    // level control when it has one, else the shared ADC-side RECLEV
    pub(crate) fn source_volume_control(&self, dev: c_uint) -> Option<c_uint> {
        [dev, SOUND_MIXER_RECLEV].into_iter().find(|d| self.has(*d))
    }

    // current recording source mask; the kernel keeps it inside recdevs and
    // non-empty whenever recdevs is (mixer_setrecsrc, mixer.c:347-357)
    pub(crate) fn recsrc(&self) -> Option<u32> {
        self.read_int(SOUND_MIXER_RECSRC).map(|v| v as u32)
    }

    // Mask write: the kernel strips non-recdevs bits, falls back MIC ->
    // MONITOR -> LINE -> lowest recdevs bit when the result is empty, and
    // stores whatever the driver actually applied (mixer.c:334-361), so a
    // caller must read back to learn the outcome. mixer_setrecsrc never ticks
    // modify_counter - RECSRC changes have to be polled by value.
    pub(crate) fn set_recsrc(&self, mask: u32) -> bool {
        self.write_int(SOUND_MIXER_RECSRC, mask as c_int)
    }

    // (left, right), each 0-100; the kernel reports the logical level even
    // while muted (mixer.c mixer_get returns level_muted)
    pub(crate) fn level(&self, dev: c_uint) -> Option<(u32, u32)> {
        let v = self.read_int(dev)?;
        if v < 0 {
            return None;
        }
        Some((v as u32 & 0x7f, (v as u32 >> 8) & 0x7f))
    }

    pub(crate) fn set_level(&self, dev: c_uint, left: u32, right: u32) -> bool {
        let v = left.min(100) | (right.min(100) << 8);
        self.write_int(dev, v as c_int)
    }

    pub(crate) fn muted(&self, dev: c_uint) -> Option<bool> {
        let mask = self.read_int(SOUND_MIXER_MUTE)?;
        Some(mask as u32 & (1 << dev) != 0)
    }

    // SOUND_MIXER_MUTE writes REPLACE the whole mutedevs mask
    // (mix_setmutedevs, mixer.c:312), so read-modify-write it; a blind write
    // would clobber mutes set elsewhere (e.g. uaudio HID mute buttons).
    pub(crate) fn set_muted(&self, dev: c_uint, mute: bool) -> bool {
        let Some(mask) = self.read_int(SOUND_MIXER_MUTE) else {
            return false;
        };
        let bit = 1u32 << dev;
        let mask = if mute {
            mask as u32 | bit
        } else {
            mask as u32 & !bit
        };
        self.write_int(SOUND_MIXER_MUTE, mask as c_int)
    }

    // Bumped by mixer_set (mixer.c:293) for any control, so it covers
    // mixer(8)/other-process changes. It does NOT tick for RECSRC changes or
    // for level writes while muted (mixer_set stores level_muted and returns
    // early), so callers must value-diff and treat the counter as a hint.
    pub(crate) fn modify_counter(&self) -> Option<c_int> {
        unsafe { ioctl_read::<MixerInfo>(self.fd.raw(), SOUND_MIXER_INFO) }
            .map(|info| info.modify_counter)
    }
}

struct RouteBinding {
    mixer: usize,
    volume_control: Option<c_uint>,
    mute_control: Option<c_uint>,
    follows_recsrc: bool,
    source: Option<c_uint>,
}

struct MixerHandle {
    mixer: Mixer,
    counter: c_int,
}

pub(crate) struct RouteController {
    bindings: Vec<RouteBinding>,
    mixers: Vec<MixerHandle>,
}

fn capitalize(value: &str) -> String {
    let mut chars = value.chars();
    match chars.next() {
        Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
        None => String::new(),
    }
}

fn level_to_volume(level: u32) -> f32 {
    let value = level.min(100) as f32 / 100.0;
    value * value * value
}

fn volume_to_level(volume: f32) -> u32 {
    if volume.is_nan() || volume <= 0.0 {
        0
    } else {
        (volume.min(1.0).cbrt() * 100.0).round() as u32
    }
}

fn route_volume(levels: (u32, u32), hardware: bool) -> RouteVolume {
    RouteVolume {
        values: vec![level_to_volume(levels.0), level_to_volume(levels.1)],
        channels: vec![SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR],
        base: 1.0,
        // Preserve the public step historically advertised for the native
        // integer 0--100 controls.
        step: 1.0 / 101.0,
        hardware,
    }
}

fn native_levels(volume: &RouteVolume) -> (u32, u32) {
    let left = volume.values.first().copied().unwrap_or(1.0);
    let right = volume.values.get(1).copied().unwrap_or(left);
    (volume_to_level(left), volume_to_level(right))
}

fn set_native_levels(volume: &mut RouteVolume, levels: (u32, u32)) {
    volume.values = vec![level_to_volume(levels.0), level_to_volume(levels.1)];
}

fn selected_recording_sources(recmask: u32, recsrc: u32) -> u32 {
    let selected = recsrc & recmask;
    if selected != 0 || recmask == 0 {
        selected
    } else {
        // Match the kernel's non-empty fallback for a non-empty recmask.
        1 << recmask.trailing_zeros()
    }
}

fn sync_source_routes(
    routes: &mut [RouteSnapshot],
    bindings: &[RouteBinding],
    mixer_index: usize,
    selected: u32,
) -> Vec<usize> {
    let mut changed = Vec::new();
    for (pos, (route, binding)) in routes.iter_mut().zip(bindings).enumerate() {
        let Some(source) = binding.source else {
            continue;
        };
        if binding.mixer != mixer_index {
            continue;
        }
        let active = selected & (1 << source) != 0;
        if route.active != active {
            route.active = active;
            changed.push(pos);
        }
    }
    changed
}

fn external_selection_change(route: &RouteSnapshot) -> RouteChange {
    RouteChange {
        key: Some(route.key),
        selection: RouteSelectionOutcome::Applied,
        // A control-less source route is soft volume. Re-announcing hardware
        // ObjectConfig for it would overwrite the session manager's softvol.
        // A route which just became inactive needs only the Route serial.
        volume: route.active && route.volume.hardware,
        mute: route.active && route.mute.hardware,
        refresh: true,
        diagnostic: Some(RouteDiagnostic::info(format!(
            "recording source changed externally: route {} {}",
            route.name,
            if route.active { "enabled" } else { "disabled" }
        ))),
    }
}

fn selection_notification_flags(
    routes: &[RouteSnapshot],
    requested_pos: usize,
    selected: Option<usize>,
    mut volume: bool,
    mut mute: bool,
) -> (usize, bool, bool) {
    let changed_pos = selected.unwrap_or(requested_pos);
    if selected == Some(requested_pos) {
        // Preserve properties supplied in the same Route request. Native
        // controls also need a readback announcement after becoming active.
        volume |= routes[changed_pos].volume.hardware;
        mute |= routes[changed_pos].mute.hardware;
    } else if selected.is_some() {
        // The backend selected a different route. Reannounce only that
        // route's native state; requested soft values belong to the inactive
        // route and must not be sent to the selected node.
        volume = routes[changed_pos].volume.hardware;
        mute = routes[changed_pos].mute.hardware;
    } else if !routes[requested_pos].active {
        volume = false;
        mute = false;
    }
    (changed_pos, volume, mute)
}

impl RouteController {
    pub(crate) fn probe(snapshot: &DeviceSnapshot) -> (Self, Vec<RouteSnapshot>) {
        let mut devices = BTreeMap::<u32, (bool, bool)>::new();
        for endpoint in &snapshot.endpoints {
            let directions = devices.entry(endpoint.object_id / 2).or_default();
            match endpoint.direction {
                StreamDirection::Playback => directions.0 = true,
                StreamDirection::Capture => directions.1 = true,
            }
        }

        let device_count = devices.len();
        let mut routes = Vec::new();
        let mut bindings = Vec::new();
        let mut mixers = Vec::new();
        let mut n_out = 0;
        let mut n_in = 0;
        for (unit, (play, capture)) in devices {
            let Some(mixer) = Mixer::open(unit) else {
                continue;
            };
            let probe_recsrc = mixer.recsrc().unwrap_or(0);
            let mixer_index = mixers.len();
            let mut used = false;

            for (direction, enabled) in [
                (StreamDirection::Playback, play),
                (StreamDirection::Capture, capture),
            ] {
                if !enabled {
                    continue;
                }
                if direction == StreamDirection::Capture && mixer.recmask().count_ones() >= 2 {
                    let recmask = mixer.recmask();
                    let selected = selected_recording_sources(recmask, probe_recsrc);
                    for source in 0..MIXER_SOURCE_COUNT {
                        if recmask & (1 << source) == 0 {
                            continue;
                        }
                        let candidate = mixer.source_volume_control(source);
                        let levels = candidate.and_then(|value| mixer.level(value));
                        let volume_control = candidate.filter(|_| levels.is_some());
                        let mute_readback = candidate.and_then(|value| mixer.muted(value));
                        let mute_control = candidate.filter(|_| mute_readback.is_some());
                        let source_name = MIXER_SOURCE_NAMES[source as usize];
                        let (name, description) = if device_count == 1 {
                            (format!("oss-input-{source_name}"), capitalize(source_name))
                        } else {
                            (
                                format!("oss-input-pcm{unit}-{source_name}"),
                                format!("{} (pcm{unit})", capitalize(source_name)),
                            )
                        };
                        let levels = levels.unwrap_or((100, 100));
                        routes.push(RouteSnapshot {
                            key: RouteKey(routes.len() as u64),
                            node_id: unit * 2 + 1,
                            direction,
                            name,
                            description,
                            priority: mixer_source_priority(source),
                            // FreeBSD exposes no complete per-jack state to
                            // userland. SND CONN names a preferred device, not
                            // a jack, so retain the established compatibility
                            // value instead of fabricating availability.
                            availability: RouteAvailability::Yes,
                            active: selected & (1 << source) != 0,
                            volume: route_volume(levels, volume_control.is_some()),
                            mute: RouteMute {
                                value: mute_readback.unwrap_or(false),
                                hardware: mute_control.is_some(),
                            },
                            save: false,
                        });
                        bindings.push(RouteBinding {
                            mixer: mixer_index,
                            volume_control,
                            mute_control,
                            follows_recsrc: false,
                            source: Some(source),
                        });
                        used = true;
                    }
                    continue;
                }

                let picked = if direction == StreamDirection::Capture {
                    mixer.input_control()
                } else {
                    mixer.output_control().map(|control| (control, false))
                };
                let Some((control, follows_recsrc)) = picked else {
                    continue;
                };
                let Some(levels) = mixer.level(control) else {
                    continue;
                };
                let mute_readback = mixer.muted(control);
                // Route names are WirePlumber persistence keys. Derive every
                // non-singleton name from the stable PCM unit, never the
                // aggregate's attach-order ordinal.
                let (name, description) = if direction == StreamDirection::Capture {
                    n_in += 1;
                    if n_in == 1 && device_count == 1 {
                        ("oss-input".to_string(), "Input".to_string())
                    } else {
                        (format!("oss-input-pcm{unit}"), format!("Input (pcm{unit})"))
                    }
                } else {
                    n_out += 1;
                    if n_out == 1 && device_count == 1 {
                        ("oss-output".to_string(), "Output".to_string())
                    } else {
                        (
                            format!("oss-output-pcm{unit}"),
                            format!("Output (pcm{unit})"),
                        )
                    }
                };
                routes.push(RouteSnapshot {
                    key: RouteKey(routes.len() as u64),
                    node_id: unit * 2 + u32::from(direction == StreamDirection::Capture),
                    direction,
                    name,
                    description,
                    priority: 100,
                    availability: RouteAvailability::Yes,
                    active: true,
                    volume: route_volume(levels, true),
                    mute: RouteMute {
                        value: mute_readback.unwrap_or(false),
                        hardware: mute_readback.is_some(),
                    },
                    save: false,
                });
                bindings.push(RouteBinding {
                    mixer: mixer_index,
                    volume_control: Some(control),
                    mute_control: mute_readback.map(|_| control),
                    follows_recsrc,
                    source: None,
                });
                used = true;
            }
            if used {
                mixers.push(MixerHandle {
                    counter: mixer.modify_counter().unwrap_or(0),
                    mixer,
                });
            }
        }
        (Self { bindings, mixers }, routes)
    }

    fn position(&self, routes: &[RouteSnapshot], key: RouteKey) -> Option<usize> {
        routes.iter().position(|route| route.key == key)
    }

    fn resolve_controls(&mut self, pos: usize) {
        let binding = &mut self.bindings[pos];
        if binding.follows_recsrc
            && let Some((control, true)) = self.mixers[binding.mixer].mixer.input_control()
        {
            let mixer = &self.mixers[binding.mixer].mixer;
            binding.volume_control = mixer.level(control).map(|_| control);
            binding.mute_control = mixer.muted(control).map(|_| control);
        }
    }

    fn refresh(&mut self, routes: &mut [RouteSnapshot], pos: usize) {
        self.resolve_controls(pos);
        let binding = &self.bindings[pos];
        routes[pos].volume.hardware = binding.volume_control.is_some();
        routes[pos].mute.hardware = binding.mute_control.is_some();
        if let Some((left, right)) = binding
            .volume_control
            .and_then(|control| self.mixers[binding.mixer].mixer.level(control))
        {
            set_native_levels(&mut routes[pos].volume, (left, right));
        }
        if let Some(mute) = binding
            .mute_control
            .and_then(|control| self.mixers[binding.mixer].mixer.muted(control))
        {
            routes[pos].mute.value = mute;
        }
    }

    fn sync_source(
        &mut self,
        routes: &mut [RouteSnapshot],
        mixer_index: usize,
    ) -> Option<Vec<usize>> {
        if !self
            .bindings
            .iter()
            .any(|binding| binding.mixer == mixer_index && binding.source.is_some())
        {
            return None;
        }
        let recsrc = self.mixers[mixer_index].mixer.recsrc()?;
        let selected = selected_recording_sources(self.mixers[mixer_index].mixer.recmask(), recsrc);
        let changed = sync_source_routes(routes, &self.bindings, mixer_index, selected);
        for &pos in &changed {
            if routes[pos].active {
                self.refresh(routes, pos);
            }
        }
        Some(changed)
    }

    pub(crate) fn refresh_all(&mut self, routes: &mut [RouteSnapshot]) {
        for mixer in 0..self.mixers.len() {
            let _ = self.sync_source(routes, mixer);
        }
        for pos in 0..routes.len() {
            self.refresh(routes, pos);
        }
    }

    pub(crate) fn poll(&mut self, routes: &mut [RouteSnapshot]) -> Vec<RouteChange> {
        let mut changes = Vec::new();
        for mixer_index in 0..self.mixers.len() {
            let Some(counter) = self.mixers[mixer_index].mixer.modify_counter() else {
                continue;
            };
            // modify_counter is only a poll hint: RECSRC does not increment it,
            // writes to a muted control can bypass it, and an external write can
            // land inside our write/readback window. Always diff the values;
            // that is also what suppresses spurious notifications.
            self.mixers[mixer_index].counter = counter;
            if let Some(changed) = self.sync_source(routes, mixer_index) {
                changes.extend(
                    changed
                        .into_iter()
                        .map(|pos| external_selection_change(&routes[pos])),
                );
            }
            for pos in 0..routes.len() {
                if self.bindings[pos].mixer != mixer_index {
                    continue;
                }
                let before = (routes[pos].volume.clone(), routes[pos].mute);
                self.refresh(routes, pos);
                let volume = before.0 != routes[pos].volume;
                let mute = before.1 != routes[pos].mute;
                if routes[pos].active && (volume || mute) {
                    changes.push(RouteChange {
                        key: Some(routes[pos].key),
                        volume,
                        mute,
                        selection: RouteSelectionOutcome::Unchanged,
                        refresh: true,
                        diagnostic: Some(RouteDiagnostic::info(format!(
                            "route {} changed externally: levels {:?}, mute {}",
                            routes[pos].name,
                            native_levels(&routes[pos].volume),
                            routes[pos].mute.value
                        ))),
                    });
                }
            }
        }
        changes
    }

    pub(crate) fn apply(
        &mut self,
        routes: &mut [RouteSnapshot],
        key: RouteKey,
        update: RouteUpdate,
    ) -> RouteChange {
        let Some(pos) = self.position(routes, key) else {
            return RouteChange::default();
        };
        let mut selected = None;
        let mut selection = RouteSelectionOutcome::Unchanged;
        if update.activate && !routes[pos].active {
            let binding = &self.bindings[pos];
            if let Some(source) = binding.source {
                if self.mixers[binding.mixer].mixer.set_recsrc(1 << source) {
                    let mixer_index = binding.mixer;
                    let _ = self.sync_source(routes, mixer_index);
                    selected = if routes[pos].active {
                        Some(pos)
                    } else {
                        self.bindings
                            .iter()
                            .enumerate()
                            .find_map(|(candidate, binding)| {
                                (binding.mixer == mixer_index
                                    && binding.source.is_some()
                                    && routes[candidate].active)
                                    .then_some(candidate)
                            })
                    };
                    // The session manager applies a requested route
                    // optimistically. A kernel-deflected RECSRC must therefore
                    // be re-announced even when no level or mute changed.
                    selection = if selected == Some(pos) {
                        RouteSelectionOutcome::Applied
                    } else {
                        RouteSelectionOutcome::Deflected
                    };
                } else {
                    selection = RouteSelectionOutcome::Failed;
                }
            }
        }

        self.resolve_controls(pos);
        let binding = &self.bindings[pos];
        let volume_control = binding.volume_control;
        let mute_control = binding.mute_control;
        let mut volume = false;
        let mut mute = false;
        let mut volume_write_failed = false;
        let mut mute_write_failed = false;
        for value in update.values {
            match value {
                RouteValueUpdate::Mute(requested) if requested != routes[pos].mute.value => {
                    match mute_control {
                        None => {
                            routes[pos].mute.value = requested;
                            mute = true;
                        }
                        Some(control)
                            if self.mixers[binding.mixer]
                                .mixer
                                .set_muted(control, requested) =>
                        {
                            routes[pos].mute.value = requested;
                            mute = true;
                        }
                        Some(_) => mute_write_failed = true,
                    }
                }
                RouteValueUpdate::Volume(values) if !values.is_empty() => {
                    let left = values[0];
                    let right = values.get(1).copied().unwrap_or(left);
                    let levels = (volume_to_level(left), volume_to_level(right));
                    let current = native_levels(&routes[pos].volume);
                    if levels != current {
                        match volume_control {
                            None => {
                                set_native_levels(&mut routes[pos].volume, levels);
                                volume = true;
                            }
                            Some(control)
                                if self.mixers[binding.mixer]
                                    .mixer
                                    .set_level(control, levels.0, levels.1) =>
                            {
                                set_native_levels(&mut routes[pos].volume, levels);
                                volume = true;
                            }
                            Some(_) => volume_write_failed = true,
                        }
                    }
                }
                _ => {}
            }
        }
        if let Some(counter) = self.mixers[binding.mixer].mixer.modify_counter() {
            self.mixers[binding.mixer].counter = counter;
        }
        let (changed_pos, volume, mute) =
            selection_notification_flags(routes, pos, selected, volume, mute);
        let mut diagnostics = Vec::new();
        let mut warning = false;
        match selection {
            RouteSelectionOutcome::Failed => {
                warning = true;
                diagnostics.push(format!(
                    "can't select the recording source for route {}",
                    routes[pos].name
                ));
            }
            RouteSelectionOutcome::Deflected => diagnostics.push(format!(
                "kernel did not move the recording source to route {}",
                routes[pos].name
            )),
            RouteSelectionOutcome::Unchanged | RouteSelectionOutcome::Applied => {}
        }
        if volume_write_failed {
            warning = true;
            diagnostics.push(format!(
                "can't set hardware volume for route {}",
                routes[pos].name
            ));
        }
        if mute_write_failed {
            warning = true;
            diagnostics.push(format!(
                "can't set hardware mute for route {}",
                routes[pos].name
            ));
        }
        RouteChange {
            key: Some(routes[changed_pos].key),
            volume,
            mute,
            selection,
            refresh: matches!(
                selection,
                RouteSelectionOutcome::Applied | RouteSelectionOutcome::Deflected
            ) || volume
                || mute,
            diagnostic: (!diagnostics.is_empty()).then(|| {
                let message = diagnostics.join("; ");
                if warning {
                    RouteDiagnostic::warning(message)
                } else {
                    RouteDiagnostic::info(message)
                }
            }),
        }
    }

    pub(crate) const fn watch_policy(&self) -> RouteWatchPolicy {
        RouteWatchPolicy {
            // FreeBSD's mixer modify counter is only a hint and RECSRC does
            // not increment it, so native events cannot replace value polling.
            poll_interval_ns: Some(1_000_000_000),
            event_driven: true,
        }
    }

    fn handles_native_unit(&self, routes: &[RouteSnapshot], unit: u32) -> bool {
        routes.iter().any(|route| route.node_id / 2 == unit)
    }

    pub(crate) fn read_hotplug(
        &self,
        routes: &[RouteSnapshot],
        monitor: &mut HotplugMonitor,
    ) -> (bool, bool) {
        let (alive, unit, reconnected) = monitor.read_mixer_event();
        (
            alive,
            reconnected || unit.is_some_and(|unit| self.handles_native_unit(routes, unit)),
        )
    }
}

impl crate::backend::RouteController<HotplugMonitor> for RouteController {
    fn probe(snapshot: &DeviceSnapshot) -> (Self, Vec<RouteSnapshot>) {
        Self::probe(snapshot)
    }

    fn refresh_all(&mut self, routes: &mut [RouteSnapshot]) {
        self.refresh_all(routes);
    }

    fn poll(&mut self, routes: &mut [RouteSnapshot]) -> Vec<RouteChange> {
        self.poll(routes)
    }

    fn apply(
        &mut self,
        routes: &mut [RouteSnapshot],
        key: RouteKey,
        update: RouteUpdate,
    ) -> RouteChange {
        self.apply(routes, key, update)
    }

    fn watch_policy(&self) -> RouteWatchPolicy {
        self.watch_policy()
    }

    fn read_hotplug(&self, routes: &[RouteSnapshot], monitor: &mut HotplugMonitor) -> (bool, bool) {
        self.read_hotplug(routes, monitor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::StreamDirection;

    fn route(active: bool) -> RouteSnapshot {
        RouteSnapshot {
            key: RouteKey(1),
            node_id: 1,
            direction: StreamDirection::Capture,
            name: "mic".into(),
            description: "Microphone".into(),
            priority: 100,
            availability: RouteAvailability::Yes,
            active,
            volume: RouteVolume {
                values: vec![1.0, 1.0],
                channels: vec![],
                base: 1.0,
                step: 1.0 / 101.0,
                hardware: false,
            },
            mute: RouteMute {
                value: false,
                hardware: false,
            },
            save: false,
        }
    }

    #[test]
    fn failed_recording_source_write_does_not_report_a_refresh() {
        let (read_fd, write_fd) = crate::backend::test_transport::pipe_pair(false, false);
        let mixer = Mixer {
            // SAFETY: pipe_pair transfers the open descriptor to this owner.
            fd: unsafe { LibcFd::from_raw(write_fd) },
            devmask: 1,
            recmask: 1,
        };
        let mut controller = RouteController {
            bindings: vec![RouteBinding {
                mixer: 0,
                volume_control: None,
                mute_control: None,
                follows_recsrc: false,
                source: Some(0),
            }],
            mixers: vec![MixerHandle { mixer, counter: 0 }],
        };
        let mut routes = [route(false)];

        let change = controller.apply(
            &mut routes,
            RouteKey(1),
            RouteUpdate {
                activate: true,
                values: vec![],
            },
        );

        assert_eq!(change.selection, RouteSelectionOutcome::Failed);
        assert_eq!(
            change.diagnostic,
            Some(RouteDiagnostic::warning(
                "can't select the recording source for route mic"
            ))
        );
        assert!(!change.refresh);
        assert!(!change.volume);
        assert!(!change.mute);
        assert!(!routes[0].active);
        // SAFETY: ownership of the read end stayed with this test.
        unsafe { libc::close(read_fd) };
    }

    #[test]
    fn external_control_less_source_switch_does_not_emit_hardware_values() {
        let soft = external_selection_change(&route(true));
        assert!(soft.refresh);
        assert_eq!(soft.selection, RouteSelectionOutcome::Applied);
        assert!(!soft.volume);
        assert!(!soft.mute);

        let mut hardware = route(true);
        hardware.volume.hardware = true;
        hardware.mute.hardware = true;
        let hardware = external_selection_change(&hardware);
        assert!(hardware.volume);
        assert!(hardware.mute);

        let mut volume_only = route(true);
        volume_only.volume.hardware = true;
        let volume_only = external_selection_change(&volume_only);
        assert!(volume_only.volume);
        assert!(!volume_only.mute);

        let mut mute_only = route(true);
        mute_only.mute.hardware = true;
        let mute_only = external_selection_change(&mute_only);
        assert!(!mute_only.volume);
        assert!(mute_only.mute);

        let mut inactive = route(false);
        inactive.volume.hardware = true;
        inactive.mute.hardware = true;
        let inactive = external_selection_change(&inactive);
        assert!(inactive.refresh);
        assert!(!inactive.volume);
        assert!(!inactive.mute);
    }

    #[test]
    fn recording_source_masks_preserve_every_active_bit() {
        assert_eq!(selected_recording_sources(0b1110, 0b1010), 0b1010);
        assert_eq!(
            selected_recording_sources(0b1110, 0),
            0b0010,
            "a missing readback retains the kernel-compatible lowest-bit fallback"
        );

        let mut first = route(true);
        first.key = RouteKey(1);
        let mut second = route(false);
        second.key = RouteKey(2);
        let mut third = route(true);
        third.key = RouteKey(3);
        let mut routes = [first, second, third];
        let bindings = [
            RouteBinding {
                mixer: 0,
                volume_control: None,
                mute_control: None,
                follows_recsrc: false,
                source: Some(1),
            },
            RouteBinding {
                mixer: 0,
                volume_control: None,
                mute_control: None,
                follows_recsrc: false,
                source: Some(2),
            },
            RouteBinding {
                mixer: 0,
                volume_control: None,
                mute_control: None,
                follows_recsrc: false,
                source: Some(3),
            },
        ];

        let changed = sync_source_routes(&mut routes, &bindings, 0, 0b0110);
        assert_eq!(changed, [1, 2]);
        assert_eq!(
            routes.map(|route| route.active),
            [true, true, false],
            "the mask must not collapse to its lowest set bit"
        );
    }

    #[test]
    fn failed_hardware_volume_and_mute_writes_are_logged_without_fake_state() {
        let (read_fd, write_fd) = crate::backend::test_transport::pipe_pair(false, false);
        let mixer = Mixer {
            // SAFETY: pipe_pair transfers the open descriptor to this owner.
            fd: unsafe { LibcFd::from_raw(write_fd) },
            devmask: 1,
            recmask: 0,
        };
        let mut controller = RouteController {
            bindings: vec![RouteBinding {
                mixer: 0,
                volume_control: Some(0),
                mute_control: Some(0),
                follows_recsrc: false,
                source: None,
            }],
            mixers: vec![MixerHandle { mixer, counter: 0 }],
        };
        let mut routes = [route(true)];
        routes[0].volume.hardware = true;
        routes[0].mute.hardware = true;
        let before = (routes[0].volume.clone(), routes[0].mute);

        let change = controller.apply(
            &mut routes,
            RouteKey(1),
            RouteUpdate {
                activate: false,
                values: vec![
                    RouteValueUpdate::Volume(vec![0.125, 0.125]),
                    RouteValueUpdate::Mute(true),
                ],
            },
        );

        assert_eq!(routes[0].volume, before.0);
        assert_eq!(routes[0].mute, before.1);
        assert!(!change.volume);
        assert!(!change.mute);
        assert!(!change.refresh);
        assert_eq!(
            change.diagnostic,
            Some(RouteDiagnostic::warning(
                "can't set hardware volume for route mic; can't set hardware mute for route mic"
            ))
        );
        // SAFETY: ownership of the read end stayed with this test.
        unsafe { libc::close(read_fd) };
    }

    #[test]
    fn soft_mute_is_independent_of_a_hardware_volume_control() {
        let (read_fd, write_fd) = crate::backend::test_transport::pipe_pair(false, false);
        let mixer = Mixer {
            // SAFETY: pipe_pair transfers the open descriptor to this owner.
            fd: unsafe { LibcFd::from_raw(write_fd) },
            devmask: 1,
            recmask: 0,
        };
        let mut controller = RouteController {
            bindings: vec![RouteBinding {
                mixer: 0,
                volume_control: Some(0),
                mute_control: None,
                follows_recsrc: false,
                source: None,
            }],
            mixers: vec![MixerHandle { mixer, counter: 0 }],
        };
        let mut routes = [route(true)];
        routes[0].volume.hardware = true;

        let change = controller.apply(
            &mut routes,
            RouteKey(1),
            RouteUpdate {
                activate: false,
                values: vec![RouteValueUpdate::Mute(true)],
            },
        );

        assert!(routes[0].mute.value);
        assert!(!routes[0].mute.hardware);
        assert!(change.mute);
        assert!(!change.volume);
        // SAFETY: ownership of the read end stayed with this test.
        unsafe { libc::close(read_fd) };
    }

    #[test]
    fn successful_soft_route_switch_preserves_same_request_properties() {
        let routes = [route(true)];
        assert_eq!(
            selection_notification_flags(&routes, 0, Some(0), true, true),
            (0, true, true)
        );

        let mut volume_hardware = route(true);
        volume_hardware.volume.hardware = true;
        let routes = [volume_hardware];
        // An independently soft mute is still carried alongside the native
        // volume readback when both were requested in the activation pod.
        assert_eq!(
            selection_notification_flags(&routes, 0, Some(0), false, true),
            (0, true, true)
        );
    }

    #[test]
    fn deflected_route_does_not_apply_requested_soft_properties_to_actual_route() {
        let requested = route(false);
        let mut actual = route(true);
        actual.key = RouteKey(2);
        actual.volume.hardware = true;
        let routes = [requested, actual];
        assert_eq!(
            selection_notification_flags(&routes, 0, Some(1), true, true),
            (1, true, false)
        );
    }

    #[test]
    fn native_levels_round_trip_for_compatibility_diagnostics() {
        for level in 0..=100 {
            assert_eq!(volume_to_level(level_to_volume(level)), level);
        }
    }

    #[test]
    fn route_watch_policy_retains_polling_and_native_nudges() {
        let controller = RouteController {
            bindings: Vec::new(),
            mixers: Vec::new(),
        };
        assert_eq!(
            controller.watch_policy(),
            RouteWatchPolicy {
                poll_interval_ns: Some(1_000_000_000),
                event_driven: true,
            }
        );
    }
}
