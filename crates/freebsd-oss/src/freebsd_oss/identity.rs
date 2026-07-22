use std::ffi::{CStr, CString};

use crate::backend::{
    BackendProperties, BackendPropertyDescriptor, DeviceInit, HotplugDiagnostic, NodeInit,
    NodeInitValues, WakeDiagnostic,
};

pub(crate) use crate::backend::InitItemResult;

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
pub(crate) const REBUILD_THREAD_PREFIX: &str = "pw-oss";
pub(crate) const SOURCE_COMMAND_PREFIX: &str = "oss-source: ";

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

pub(crate) const fn hotplug_diagnostic(kind: HotplugDiagnostic) -> &'static str {
    match kind {
        HotplugDiagnostic::MonitorDetachAbort => "can't detach the monitor devd source; aborting",
        HotplugDiagnostic::MonitorLost => "devd connection lost; hotplug disabled",
        HotplugDiagnostic::MonitorOpen => "can't connect to devd, hotplug disabled",
        HotplugDiagnostic::MonitorWatch => "can't watch devd",
        HotplugDiagnostic::RouteDetachAbort => "can't detach the devd source; aborting",
        HotplugDiagnostic::RouteLost => {
            "devd connection lost; falling back to the mixer poll alone"
        }
        HotplugDiagnostic::RouteNudge => "SND CONN event; re-polling the mixers",
        HotplugDiagnostic::RouteOpen => "no devd connection",
        HotplugDiagnostic::RouteOpenFallback => "jack events will wait for the mixer poll",
        HotplugDiagnostic::RouteTimerDetachAbort => "can't detach the mixer timer source; aborting",
        HotplugDiagnostic::RouteTimerArm => "can't arm the mixer poll timer",
        HotplugDiagnostic::RouteTimerCreate => {
            "can't create the mixer poll timer; external volume changes won't be noticed"
        }
        HotplugDiagnostic::RouteTimerWatch => {
            "can't watch the mixer; external volume changes won't be noticed"
        }
        HotplugDiagnostic::RouteWatch => {
            "can't watch devd; jack events will wait for the mixer poll"
        }
    }
}

pub(crate) const fn wake_diagnostic(kind: WakeDiagnostic) -> &'static str {
    match kind {
        WakeDiagnostic::Selected => "using enriched OSS kqueue device wakeups",
        WakeDiagnostic::Create => "can't create the OSS kqueue wake source",
        WakeDiagnostic::Read => "reading the OSS kqueue event",
        WakeDiagnostic::Remove => "removing the OSS kqueue device",
        WakeDiagnostic::Register => "can't register OSS kqueue events",
        WakeDiagnostic::Threshold => "can't set the OSS kqueue wake threshold",
        WakeDiagnostic::Arm => "arming the OSS kqueue timer",
    }
}

#[derive(Default)]
pub(crate) struct OssDeviceInit {
    parent_name: Option<String>,
    units: Vec<u32>,
}

impl OssDeviceInit {
    pub(crate) const fn missing_units_diagnostic() -> &'static str {
        "api.freebsd-oss.pcm-devices should contain pcm device indexes"
    }

    pub(crate) const fn snapshot_diagnostic() -> &'static str {
        "can't retrieve pcm device information"
    }

    pub(crate) fn parse(&mut self, key: &str, value: &str) {
        match key {
            PARENT_DEVICE => self.parent_name = Some(value.to_string()),
            DEVICE_INDEXES => self
                .units
                .extend(value.split(',').filter_map(|part| part.parse::<u32>().ok())),
            _ => {}
        }
    }

    pub(crate) fn parent_name(&self) -> Option<&str> {
        self.parent_name.as_deref()
    }

    pub(crate) fn units(&self) -> &[u32] {
        &self.units
    }
}

const FRAGMENT_DESCRIPTOR: BackendPropertyDescriptor = BackendPropertyDescriptor {
    key: FRAGMENT,
    description: "OSS fragment size (bytes, power of two, 0 = automatic)",
    maximum: 16_384,
};
const PLAYBACK_DELAY_DESCRIPTOR: BackendPropertyDescriptor = BackendPropertyDescriptor {
    key: PLAYBACK_DELAY,
    description: "Playback buffer fill target (1/8ths of a period)",
    maximum: 1_024,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum OssPropertyUpdate {
    Fragment(u32),
    PlaybackDelay(u32),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct OssNodeProperties {
    playback: bool,
    fragment_bytes: u32,
    fragment_default: u32,
    playback_delay_eighths: u32,
    playback_delay_default: u32,
}

impl OssNodeProperties {
    pub(crate) const fn new(playback: bool) -> Self {
        Self {
            playback,
            fragment_bytes: 0,
            fragment_default: 0,
            playback_delay_eighths: 10,
            playback_delay_default: 10,
        }
    }

    fn set_init(&mut self, update: OssPropertyUpdate) {
        self.apply(update);
        self.fragment_default = self.fragment_bytes;
        self.playback_delay_default = self.playback_delay_eighths;
    }

    pub(crate) fn descriptors(&self) -> &'static [BackendPropertyDescriptor] {
        if self.playback {
            &[PLAYBACK_DELAY_DESCRIPTOR, FRAGMENT_DESCRIPTOR]
        } else {
            &[FRAGMENT_DESCRIPTOR]
        }
    }

    pub(crate) fn values(&self) -> Vec<(&'static str, u32)> {
        if self.playback {
            vec![
                (PLAYBACK_DELAY, self.playback_delay_eighths),
                (FRAGMENT, self.fragment_bytes),
            ]
        } else {
            vec![(FRAGMENT, self.fragment_bytes)]
        }
    }

    pub(crate) fn decode_params(value: &libspa::pod::Value) -> Vec<OssPropertyUpdate> {
        use libspa::pod::Value;
        let Value::Struct(values) = value else {
            return Vec::new();
        };
        if values.len() % 2 != 0 {
            return Vec::new();
        }
        let mut playback_delay = None;
        let mut fragment = None;
        for pair in values.chunks(2) {
            match (&pair[0], &pair[1]) {
                (Value::String(key), Value::Int(value)) if key == FRAGMENT && *value >= 0 => {
                    fragment = Some(OssPropertyUpdate::Fragment(*value as u32));
                }
                (Value::String(key), Value::Int(value)) if key == PLAYBACK_DELAY && *value >= 0 => {
                    playback_delay = Some(OssPropertyUpdate::PlaybackDelay(*value as u32));
                }
                _ => {}
            }
        }
        playback_delay.into_iter().chain(fragment).collect()
    }

    pub(crate) fn apply(&mut self, update: OssPropertyUpdate) -> bool {
        match update {
            OssPropertyUpdate::Fragment(value) => {
                let value = super::buffer::normalize_fragment(value);
                let changed = value != self.fragment_bytes;
                self.fragment_bytes = value;
                changed
            }
            OssPropertyUpdate::PlaybackDelay(value) if self.playback => {
                let value = value.min(1_024);
                let changed = value != self.playback_delay_eighths;
                self.playback_delay_eighths = value;
                changed
            }
            OssPropertyUpdate::PlaybackDelay(_) => false,
        }
    }

    pub(crate) fn reset(&mut self) -> bool {
        let reset = Self {
            playback: self.playback,
            fragment_bytes: self.fragment_default,
            fragment_default: self.fragment_default,
            playback_delay_eighths: self.playback_delay_default,
            playback_delay_default: self.playback_delay_default,
        };
        let changed = *self != reset;
        *self = reset;
        changed
    }

    pub(crate) const fn fragment_bytes(&self) -> u32 {
        self.fragment_bytes
    }

    pub(crate) const fn playback_delay_eighths(&self) -> u32 {
        self.playback_delay_eighths
    }
}

pub(crate) struct OssNodeInit {
    pub(crate) stream_path: Option<String>,
    pub(crate) force_timer: bool,
    pub(crate) properties: OssNodeProperties,
}

impl BackendProperties for OssNodeProperties {
    type Update = OssPropertyUpdate;

    fn new(playback: bool) -> Self {
        Self::new(playback)
    }

    fn descriptors(&self) -> &'static [BackendPropertyDescriptor] {
        self.descriptors()
    }

    fn values(&self) -> Vec<(&'static str, u32)> {
        self.values()
    }

    fn decode_params(value: &libspa::pod::Value) -> Vec<Self::Update> {
        Self::decode_params(value)
    }

    fn apply(&mut self, update: Self::Update) -> bool {
        self.apply(update)
    }

    fn reset(&mut self) -> bool {
        self.reset()
    }
}

impl DeviceInit for OssDeviceInit {
    fn parse(&mut self, key: &str, value: &str) {
        self.parse(key, value);
    }

    fn parent_name(&self) -> Option<&str> {
        self.parent_name()
    }

    fn is_complete(&self) -> bool {
        !self.units().is_empty()
    }

    fn snapshot(&self) -> Option<crate::backend::DeviceSnapshot> {
        super::devices::device_snapshot(self.units())
    }

    fn missing_selector_diagnostic() -> &'static str {
        Self::missing_units_diagnostic()
    }

    fn snapshot_diagnostic() -> &'static str {
        Self::snapshot_diagnostic()
    }
}

impl NodeInit for OssNodeInit {
    type Properties = OssNodeProperties;

    fn new(playback: bool) -> Self {
        Self::new(playback)
    }

    fn force_timer_key() -> Option<&'static str> {
        Some(FORCE_TIMER)
    }

    fn parse(&mut self, key: &str, value: &str) -> InitItemResult {
        self.parse(key, value)
    }

    fn into_values(self) -> NodeInitValues<Self::Properties> {
        NodeInitValues {
            stream_path: self.stream_path,
            force_timer: self.force_timer,
            properties: self.properties,
        }
    }
}

impl OssNodeInit {
    pub(crate) const fn new(playback: bool) -> Self {
        Self {
            stream_path: None,
            force_timer: false,
            properties: OssNodeProperties::new(playback),
        }
    }

    pub(crate) fn parse(&mut self, key: &str, value: &str) -> InitItemResult {
        if key == STREAM_PATH {
            self.stream_path = Some(value.to_string());
            InitItemResult::Handled
        } else if key == FORCE_TIMER {
            match parse_bool(value) {
                Some(force_timer) => {
                    self.force_timer = force_timer;
                    InitItemResult::Handled
                }
                None => InitItemResult::InvalidBoolean,
            }
        } else if key == FRAGMENT {
            if let Ok(value) = value.parse::<u32>() {
                self.properties.set_init(OssPropertyUpdate::Fragment(value));
            }
            InitItemResult::Handled
        } else if key == PLAYBACK_DELAY {
            if let Ok(value) = value.parse::<u32>() {
                self.properties
                    .set_init(OssPropertyUpdate::PlaybackDelay(value));
            }
            InitItemResult::Handled
        } else {
            InitItemResult::Unknown
        }
    }
}

fn parse_bool(value: &str) -> Option<bool> {
    if matches!(value, "1")
        || value.eq_ignore_ascii_case("true")
        || value.eq_ignore_ascii_case("yes")
        || value.eq_ignore_ascii_case("on")
    {
        Some(true)
    } else if matches!(value, "0")
        || value.eq_ignore_ascii_case("false")
        || value.eq_ignore_ascii_case("no")
        || value.eq_ignore_ascii_case("off")
    {
        Some(false)
    } else {
        None
    }
}

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
    use libspa::pod::Value;

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
        assert_eq!(
            super::OssDeviceInit::missing_units_diagnostic(),
            "api.freebsd-oss.pcm-devices should contain pcm device indexes"
        );
        assert_eq!(
            super::OssDeviceInit::snapshot_diagnostic(),
            "can't retrieve pcm device information"
        );
    }

    #[test]
    fn force_timer_boolean_values_remain_compatible() {
        for value in ["1", "true", "TRUE", "yes", "on"] {
            assert_eq!(super::parse_bool(value), Some(true), "{value:?}");
        }
        for value in ["0", "false", "FALSE", "no", "off"] {
            assert_eq!(super::parse_bool(value), Some(false), "{value:?}");
        }
        for value in ["", "2", "maybe", "true "] {
            assert_eq!(super::parse_bool(value), None, "{value:?}");
        }
    }

    #[test]
    fn compatibility_diagnostics_remain_stable() {
        use crate::backend::{HotplugDiagnostic, WakeDiagnostic};

        let hotplug = [
            (
                HotplugDiagnostic::MonitorDetachAbort,
                "can't detach the monitor devd source; aborting",
            ),
            (
                HotplugDiagnostic::MonitorLost,
                "devd connection lost; hotplug disabled",
            ),
            (
                HotplugDiagnostic::MonitorOpen,
                "can't connect to devd, hotplug disabled",
            ),
            (HotplugDiagnostic::MonitorWatch, "can't watch devd"),
            (
                HotplugDiagnostic::RouteDetachAbort,
                "can't detach the devd source; aborting",
            ),
            (
                HotplugDiagnostic::RouteLost,
                "devd connection lost; falling back to the mixer poll alone",
            ),
            (
                HotplugDiagnostic::RouteNudge,
                "SND CONN event; re-polling the mixers",
            ),
            (HotplugDiagnostic::RouteOpen, "no devd connection"),
            (
                HotplugDiagnostic::RouteOpenFallback,
                "jack events will wait for the mixer poll",
            ),
            (
                HotplugDiagnostic::RouteTimerDetachAbort,
                "can't detach the mixer timer source; aborting",
            ),
            (
                HotplugDiagnostic::RouteTimerArm,
                "can't arm the mixer poll timer",
            ),
            (
                HotplugDiagnostic::RouteTimerCreate,
                "can't create the mixer poll timer; external volume changes won't be noticed",
            ),
            (
                HotplugDiagnostic::RouteTimerWatch,
                "can't watch the mixer; external volume changes won't be noticed",
            ),
            (
                HotplugDiagnostic::RouteWatch,
                "can't watch devd; jack events will wait for the mixer poll",
            ),
        ];
        for (kind, expected) in hotplug {
            assert_eq!(super::hotplug_diagnostic(kind), expected);
        }

        let wake = [
            (
                WakeDiagnostic::Selected,
                "using enriched OSS kqueue device wakeups",
            ),
            (
                WakeDiagnostic::Create,
                "can't create the OSS kqueue wake source",
            ),
            (WakeDiagnostic::Read, "reading the OSS kqueue event"),
            (WakeDiagnostic::Remove, "removing the OSS kqueue device"),
            (WakeDiagnostic::Register, "can't register OSS kqueue events"),
            (
                WakeDiagnostic::Threshold,
                "can't set the OSS kqueue wake threshold",
            ),
            (WakeDiagnostic::Arm, "arming the OSS kqueue timer"),
        ];
        for (kind, expected) in wake {
            assert_eq!(super::wake_diagnostic(kind), expected);
        }
    }

    #[test]
    fn property_pod_uses_last_value_and_canonical_application_order() {
        let value = Value::Struct(vec![
            Value::String(super::FRAGMENT.to_string()),
            Value::Int(1024),
            Value::String(super::PLAYBACK_DELAY.to_string()),
            Value::Int(8),
            Value::String(super::FRAGMENT.to_string()),
            Value::Int(4096),
        ]);

        assert_eq!(
            super::OssNodeProperties::decode_params(&value),
            vec![
                super::OssPropertyUpdate::PlaybackDelay(8),
                super::OssPropertyUpdate::Fragment(4096),
            ]
        );
    }
}
