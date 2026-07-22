use std::collections::BTreeMap;
use std::ffi::{CStr, CString, c_int, c_uint};
use std::os::fd::RawFd;

use crate::{freebsd, oss};

pub(crate) use oss::{Mixer, PcmDevice as AudioDevice};

pub(crate) const DEVICE_API: &str = "freebsd-oss";
pub(crate) const DIAGNOSTIC_TAG: &str = DEVICE_API;

pub(crate) const MONITOR_FACTORY_NAME: &CStr = c"freebsd-oss.monitor";
pub(crate) const DEVICE_FACTORY_NAME: &CStr = c"freebsd-oss.device";
pub(crate) const SINK_FACTORY_NAME: &CStr = c"freebsd-oss.sink";
pub(crate) const SOURCE_FACTORY_NAME: &CStr = c"freebsd-oss.source";

pub(crate) const DEVICE_LOG_TOPIC: &CStr = c"spa.oss.device";
pub(crate) const MONITOR_LOG_TOPIC: &CStr = c"spa.oss.monitor";
pub(crate) const SINK_LOG_TOPIC: &CStr = c"spa.oss.sink";
pub(crate) const SOURCE_LOG_TOPIC: &CStr = c"spa.oss.source";

/// Name of the physical sound-card driver that owns an aggregate.
pub(crate) const PARENT_DEVICE: &str = "api.freebsd-oss.pcm-parent";
/// Comma-separated PCM unit numbers belonging to an aggregate.
pub(crate) const DEVICE_INDEXES: &str = "api.freebsd-oss.pcm-devices";
/// Stream device path passed from the device object to a source or sink.
pub(crate) const STREAM_PATH: &str = "api.freebsd-oss.dsp-path";
/// Creation-time switch forcing the portable timer wake path.
pub(crate) const FORCE_TIMER: &str = "api.freebsd-oss.force-timer";
/// Playback buffer target in eighths of a period.
pub(crate) const PLAYBACK_DELAY: &str = "oss.delay";
/// Backend fragment override in bytes; zero selects automatic sizing.
pub(crate) const FRAGMENT: &str = "oss.fragment";

pub(crate) fn stream_path(index: u32) -> String {
    format!("/dev/dsp{index}")
}

pub(crate) fn clock_name(stream_path: &str) -> CString {
    CString::new(format!(
        "{DEVICE_API}.{}",
        stream_path.trim_start_matches("/dev/")
    ))
    .unwrap_or_default()
}

pub(crate) fn read_device_groups() -> Result<BTreeMap<String, Vec<u32>>, nix::errno::Errno> {
    oss::read_sndstat().map(|indexes| oss::group_pcm_devices_by_parent(&indexes))
}

pub(crate) fn list_audio_devices(indexes: &[u32]) -> Vec<AudioDevice> {
    oss::list_pcm_devices(indexes)
}

pub(crate) const MIXER_SOURCE_MIC: c_uint = oss::SOUND_MIXER_MIC;
pub(crate) const MIXER_SOURCE_LINE: c_uint = oss::SOUND_MIXER_LINE;
pub(crate) const MIXER_SOURCE_COUNT: c_uint = oss::SOUND_MIXER_NRDEVICES;
pub(crate) const MIXER_SOURCE_NAMES: [&str; MIXER_SOURCE_COUNT as usize] = oss::SOUND_DEVICE_NAMES;

#[derive(Debug, Eq, PartialEq)]
pub(crate) enum MonitorHotplugEvent {
    Attached,
    Detached(String),
}

fn decode_monitor_event(line: &str) -> Option<MonitorHotplugEvent> {
    static RE: std::sync::LazyLock<regex::Regex> =
        std::sync::LazyLock::new(|| regex::Regex::new(r"^([\+-])((?:pcm|uaudio)\d+)").unwrap());

    let groups = RE.captures(line)?;
    if groups.get(1)?.as_str() == "-" {
        Some(MonitorHotplugEvent::Detached(
            groups.get(2)?.as_str().to_string(),
        ))
    } else {
        Some(MonitorHotplugEvent::Attached)
    }
}

fn decode_mixer_event(line: &str) -> Option<u32> {
    static RE: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
        regex::Regex::new(r"^!system=SND subsystem=CONN type=(?:IN|OUT) cdev=dsp([0-9]+)").unwrap()
    });

    RE.captures(line)
        .and_then(|groups| groups[1].parse::<u32>().ok())
}

/// devd connection that exposes decoded sound-device events.
pub(crate) struct HotplugMonitor(freebsd::DevdSocket);

impl HotplugMonitor {
    pub(crate) fn open() -> Result<Self, std::io::Error> {
        freebsd::DevdSocket::open().map(Self)
    }

    pub(crate) fn fd(&self) -> RawFd {
        self.0.fd()
    }

    /// Returns connection liveness plus one relevant attach/detach event.
    pub(crate) fn read_monitor_event(&mut self) -> (bool, Option<MonitorHotplugEvent>) {
        let mut event = None;
        let alive = self.0.read_event(|line| {
            event = decode_monitor_event(line);
        });
        (alive, event)
    }

    /// Returns connection liveness plus the PCM unit named by a sound event.
    pub(crate) fn read_mixer_event(&mut self) -> (bool, Option<u32>) {
        let mut unit = None;
        let alive = self.0.read_event(|line| {
            unit = decode_mixer_event(line);
        });
        (alive, unit)
    }
}

pub(crate) fn mixer_source_priority(source: c_uint) -> c_int {
    match source {
        MIXER_SOURCE_MIC => 100,
        MIXER_SOURCE_LINE => 90,
        _ => 80 - source as c_int,
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn compatibility_names_remain_stable() {
        assert_eq!(super::DEVICE_API, "freebsd-oss");
        assert_eq!(super::DIAGNOSTIC_TAG, "freebsd-oss");
        assert_eq!(
            super::clock_name("/dev/dsp3").to_bytes(),
            b"freebsd-oss.dsp3"
        );
        assert_eq!(super::STREAM_PATH, "api.freebsd-oss.dsp-path");
        assert_eq!(super::PLAYBACK_DELAY, "oss.delay");
        assert_eq!(super::FRAGMENT, "oss.fragment");
    }

    #[test]
    fn hotplug_payloads_are_decoded() {
        assert_eq!(
            super::decode_monitor_event("-uaudio3 at uhub2"),
            Some(super::MonitorHotplugEvent::Detached("uaudio3".into()))
        );
        assert_eq!(
            super::decode_monitor_event("+pcm7 at uaudio3"),
            Some(super::MonitorHotplugEvent::Attached)
        );
        assert_eq!(super::decode_monitor_event("!system=USB"), None);

        assert_eq!(
            super::decode_mixer_event("!system=SND subsystem=CONN type=OUT cdev=dsp12"),
            Some(12)
        );
        assert_eq!(
            super::decode_mixer_event("!system=SND subsystem=CONN type=NODEV"),
            None
        );
    }
}
