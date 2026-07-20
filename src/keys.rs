/// Name of the actual sound card driver
pub(crate) const PCM_PARENT_DEVICE: &str = "api.freebsd-oss.pcm-parent";

/// Comma-separated list of pcm device numbers (there is typically more than one per sound card)
pub(crate) const PCM_DEVICE_INDEXES: &str = "api.freebsd-oss.pcm-devices";

/// Path to the dsp device file a source/sink node is supposed to open
pub(crate) const OSS_DSP_PATH: &str = "api.freebsd-oss.dsp-path";

/// Creation-time switch that keeps audio wakeups on the portable SPA timer
/// even when the kernel provides enriched OSS kqueue events.
pub(crate) const OSS_FORCE_TIMER: &str = "api.freebsd-oss.force-timer";

/// Sink buffer fill target in 1/8ths of a period; settable per device through
/// wireplumber node rules, or at runtime through the Props params struct
pub(crate) const OSS_DELAY: &str = "oss.delay";

/// Device fragment size in bytes (power of two, clamped to 64..16384;
/// 0 = automatic), shared by both directions; settable like oss.delay
pub(crate) const OSS_FRAGMENT: &str = "oss.fragment";
