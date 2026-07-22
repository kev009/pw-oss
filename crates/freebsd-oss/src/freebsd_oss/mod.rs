mod abi;
mod backend;
mod buffer;
mod devices;
mod dsp;
mod event;
mod identity;
mod mixer;
#[cfg(test)]
mod native_tests;
mod sys;

use abi::{
    AFMT_F32_BE, AFMT_F32_LE, AFMT_S16_BE, AFMT_S16_LE, AFMT_S24_BE, AFMT_S24_LE, AFMT_S32_BE,
    AFMT_S32_LE, AFMT_U8, feeder_rate_round,
};
pub(crate) use backend::{bytes_per_sample, fallback_caps, validate_config};
use buffer::MIN_BUFFER_BYTES;
pub(crate) use devices::{DeviceCatalog, probe_caps};
pub(crate) use dsp::{Dsp, DspWriter};
pub(crate) use event::{HotplugMonitor, OssWakeDriver, enriched_sound_kqueue_available};
pub(crate) use identity::{
    DEVICE_API, DEVICE_FACTORY_NAME, DEVICE_LOG_TOPIC, DIAGNOSTIC_TAG, MONITOR_FACTORY_NAME,
    MONITOR_LOG_TOPIC, OssDeviceInit, OssNodeInit, OssNodeProperties, REBUILD_THREAD_PREFIX,
    SINK_FACTORY_NAME, SINK_LOG_TOPIC, SOURCE_COMMAND_PREFIX, SOURCE_FACTORY_NAME,
    SOURCE_LOG_TOPIC, STREAM_PATH, clock_name, hotplug_diagnostic,
};
pub(crate) use mixer::RouteController;

pub(crate) enum FreeBsdOss {}

impl crate::backend::Backend for FreeBsdOss {
    type Capture = Dsp;
    type Playback = DspWriter;
    type Properties = OssNodeProperties;
    type DeviceInit = OssDeviceInit;
    type NodeInit = OssNodeInit;
    type Catalog = DeviceCatalog;
    type Hotplug = HotplugMonitor;
    type Routes = RouteController;

    const DEVICE_API: &'static str = DEVICE_API;
    const DIAGNOSTIC_TAG: &'static str = DIAGNOSTIC_TAG;
    const REBUILD_THREAD_PREFIX: &'static str = REBUILD_THREAD_PREFIX;
    const SOURCE_COMMAND_PREFIX: &'static str = SOURCE_COMMAND_PREFIX;
    const STREAM_PATH: &'static str = STREAM_PATH;
    const DEVICE_FACTORY_NAME: &'static std::ffi::CStr = DEVICE_FACTORY_NAME;
    const SINK_FACTORY_NAME: &'static std::ffi::CStr = SINK_FACTORY_NAME;
    const SOURCE_FACTORY_NAME: &'static std::ffi::CStr = SOURCE_FACTORY_NAME;

    fn bytes_per_sample(format: u32) -> Option<u32> {
        bytes_per_sample(format)
    }

    fn clock_name(stream_path: &str) -> std::ffi::CString {
        clock_name(stream_path)
    }

    fn fallback_caps() -> crate::backend::StreamCaps {
        fallback_caps()
    }

    fn hotplug_diagnostic(kind: crate::backend::HotplugDiagnostic) -> &'static str {
        hotplug_diagnostic(kind)
    }

    fn probe_caps(path: &str, playback: bool) -> Option<crate::backend::StreamCaps> {
        probe_caps(path, playback)
    }

    fn validate_config(
        caps: &crate::backend::StreamCaps,
        config: &crate::backend::StreamConfig,
    ) -> Result<(), crate::backend::ChannelMapError> {
        validate_config(caps, config)
    }

    fn device_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut crate::DEVICE_TOPIC)
            .expect("the device topic has a static address")
    }

    fn monitor_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut crate::MONITOR_TOPIC)
            .expect("the monitor topic has a static address")
    }

    fn sink_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut crate::SINK_TOPIC)
            .expect("the sink topic has a static address")
    }

    fn source_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut crate::SOURCE_TOPIC)
            .expect("the source topic has a static address")
    }
}
