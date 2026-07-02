/// Name of the actual sound card driver
pub const PCM_PARENT_DEVICE:  &str = "api.freebsd-oss.pcm-parent";

/// Comma-separated list of pcm device numbers (there is typically more than one per sound card)
pub const PCM_DEVICE_INDEXES: &str = "api.freebsd-oss.pcm-devices";

/// Path to the dsp device file a source/sink node is supposed to open
pub const OSS_DSP_PATH:       &str = "api.freebsd-oss.dsp-path";

/// Sink buffer fill target in 1/8ths of a period; settable per device through
/// wireplumber node rules, or at runtime through the Props params struct
pub const OSS_DELAY:          &str = "oss.delay";
