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

use std::ffi::CString;
use std::os::raw::{c_char, c_int, c_uint, c_ulong};

use crate::utils::{LibcFd, ioctl_int, ioctl_read};

pub(crate) const SOUND_MIXER_VOLUME: c_uint = 0;
pub(crate) const SOUND_MIXER_PCM: c_uint = 4;
pub(crate) const SOUND_MIXER_LINE: c_uint = 6;
pub(crate) const SOUND_MIXER_MIC: c_uint = 7;
pub(crate) const SOUND_MIXER_RECLEV: c_uint = 11;
pub(crate) const SOUND_MIXER_IGAIN: c_uint = 12;

pub(crate) const SOUND_MIXER_NRDEVICES: c_uint = 25;

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

// sys/soundcard.h mixer_info; the ioctl encodes the size
#[repr(C)]
#[derive(Clone, Copy)]
struct MixerInfo {
    id: [c_char; 16],
    name: [c_char; 32],
    modify_counter: c_int,
    fillers: [c_int; 10],
}

unsafe impl crate::utils::IoctlPod for MixerInfo {}

fn read_req(dev: c_uint) -> c_ulong {
    nix::request_code_read!(b'M', dev, std::mem::size_of::<c_int>())
}

// MIXER_WRITE is _IOWR but FreeBSD does NOT echo the stored value back
// (only the read branch writes *arg_i, mixer.c); never read the buffer after
fn write_req(dev: c_uint) -> c_ulong {
    nix::request_code_readwrite!(b'M', dev, std::mem::size_of::<c_int>())
}

const SOUND_MIXER_INFO: c_ulong =
    nix::request_code_read!(b'M', 101, std::mem::size_of::<MixerInfo>());

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
