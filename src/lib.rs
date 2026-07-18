use libspa::sys::spa_handle_factory;
use std::os::raw::c_int;

#[allow(clippy::absurd_extreme_comparisons)]
mod device;
#[allow(clippy::absurd_extreme_comparisons)]
mod monitor;
#[allow(clippy::absurd_extreme_comparisons)]
mod node;
mod nv;
mod sink;
mod source;
#[allow(clippy::absurd_extreme_comparisons)]
mod spa;

mod dll;

mod keys;
mod mixer;
mod sound;
mod utils;

use device::OSS_DEVICE_FACTORY;
use monitor::OSS_MONITOR_FACTORY;
use sink::OSS_SINK_FACTORY;
use source::OSS_SOURCE_FACTORY;

#[allow(clippy::missing_safety_doc)]
#[no_mangle]
pub unsafe extern "C" fn spa_handle_factory_enum(
    factory: *mut *const spa_handle_factory,
    index: *mut u32,
) -> c_int {
    assert!(!factory.is_null());
    assert!(!index.is_null());
    match *index {
        0 => {
            *factory = &OSS_MONITOR_FACTORY;
            *index += 1;
            1
        }
        1 => {
            *factory = &OSS_DEVICE_FACTORY;
            *index += 1;
            1
        }
        2 => {
            *factory = &OSS_SINK_FACTORY;
            *index += 1;
            1
        }
        3 => {
            *factory = &OSS_SOURCE_FACTORY;
            *index += 1;
            1
        }
        _ => 0,
    }
}

// The static log-topic registration the host's plugin loader enumerates:
// one pointer per topic in a dedicated "spa_log_topic" ELF section plus a
// `spa_log_topic_enum` symbol spanning it (SPA_LOG_TOPIC_REGISTER /
// SPA_LOG_TOPIC_ENUM_DEFINE_REGISTERED in spa/support/log.h).
//
// One topic per module, spa.oss.{device,sink,source,monitor}, mirroring the
// spa.alsa layout. No parent spa.oss topic: the host matches PIPEWIRE_DEBUG
// patterns against each topic name exactly (globs aside), so a registered
// parent would never cascade to the per-module topics and setting it would
// silently do nothing - use PIPEWIRE_DEBUG="spa.oss.*:LEVEL" to cover all
// four.

use libspa::sys::{SPA_VERSION_LOG_TOPIC_ENUM, spa_log_topic};

// repr(transparent): the host walks __start..__stop as a plain C array of
// `struct spa_log_topic *`, so each entry must have exactly the size,
// alignment and layout of that pointer - which repr(transparent) pins by
// construction, on 32- and 64-bit targets alike (log.h's
// aligned(__alignof__(struct spa_log_topic *)) is the pointer's natural
// alignment).
#[repr(transparent)]
struct TopicPointer(*mut spa_log_topic);

// exactly one pointer per entry: no padding for the host's array walk
const _: () = assert!(
    std::mem::size_of::<TopicPointer>() == std::mem::size_of::<*mut spa_log_topic>()
        && std::mem::align_of::<TopicPointer>() == std::mem::align_of::<*mut spa_log_topic>()
);

// The entries are private (the host finds them through the section bounds,
// not by name), so no no_mangle; #[used] plus the section placement is what
// keeps them in the emitted spa_log_topic section.
#[unsafe(link_section = "spa_log_topic")]
#[used]
#[allow(non_upper_case_globals)]
static mut spa_log_topic_export_oss_device: TopicPointer =
    TopicPointer(&raw mut device::OSS_DEVICE_TOPIC);

#[unsafe(link_section = "spa_log_topic")]
#[used]
#[allow(non_upper_case_globals)]
static mut spa_log_topic_export_oss_sink: TopicPointer =
    TopicPointer(&raw mut sink::OSS_SINK_TOPIC);

#[unsafe(link_section = "spa_log_topic")]
#[used]
#[allow(non_upper_case_globals)]
static mut spa_log_topic_export_oss_source: TopicPointer =
    TopicPointer(&raw mut source::OSS_SOURCE_TOPIC);

#[unsafe(link_section = "spa_log_topic")]
#[used]
#[allow(non_upper_case_globals)]
static mut spa_log_topic_export_oss_monitor: TopicPointer =
    TopicPointer(&raw mut monitor::OSS_MONITOR_TOPIC);

// the linker generates these for the section
unsafe extern "C" {
    static __start_spa_log_topic: *mut spa_log_topic;
    static __stop_spa_log_topic: *mut spa_log_topic;
}

#[unsafe(no_mangle)]
#[used]
#[allow(non_upper_case_globals)]
static mut spa_log_topic_enum: libspa::sys::spa_log_topic_enum = libspa::sys::spa_log_topic_enum {
    version: SPA_VERSION_LOG_TOPIC_ENUM,
    topics: &raw const __start_spa_log_topic,
    topics_end: &raw const __stop_spa_log_topic,
};
