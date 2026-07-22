use std::ffi::{CStr, CString};

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
        assert_eq!(
            super::MONITOR_FACTORY_NAME.to_bytes(),
            b"freebsd-oss.monitor"
        );
        assert_eq!(super::DEVICE_FACTORY_NAME.to_bytes(), b"freebsd-oss.device");
        assert_eq!(super::SINK_FACTORY_NAME.to_bytes(), b"freebsd-oss.sink");
        assert_eq!(super::SOURCE_FACTORY_NAME.to_bytes(), b"freebsd-oss.source");
        assert_eq!(super::DEVICE_LOG_TOPIC.to_bytes(), b"spa.oss.device");
        assert_eq!(super::MONITOR_LOG_TOPIC.to_bytes(), b"spa.oss.monitor");
        assert_eq!(super::SINK_LOG_TOPIC.to_bytes(), b"spa.oss.sink");
        assert_eq!(super::SOURCE_LOG_TOPIC.to_bytes(), b"spa.oss.source");
        assert_eq!(super::PARENT_DEVICE, "api.freebsd-oss.pcm-parent");
        assert_eq!(super::DEVICE_INDEXES, "api.freebsd-oss.pcm-devices");
        assert_eq!(super::STREAM_PATH, "api.freebsd-oss.dsp-path");
        assert_eq!(super::FORCE_TIMER, "api.freebsd-oss.force-timer");
        assert_eq!(super::PLAYBACK_DELAY, "oss.delay");
        assert_eq!(super::FRAGMENT, "oss.fragment");
    }
}
