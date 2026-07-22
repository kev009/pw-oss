// the cdylib exports two symbols - spa_handle_factory_enum and the
// spa_log_topic_enum registration below; everything else is crate-internal,
// and pub items would otherwise be exempt from dead_code analysis
#![warn(unreachable_pub)]
// unsafe_op_in_unsafe_fn (warn-by-default on edition 2024) is honored: unsafe
// fns wrap only their actual unsafe operations in narrow blocks, except for
// short FFI trampolines and vtable-call wrappers whose whole body is the
// unsafe surface.
// mechanical-style clippy gates on top of the default set
// (not unreadable_literal: the hex masks mirror sys/soundcard.h and grep
// better without separators)
#![warn(clippy::uninlined_format_args)]
#![warn(clippy::semicolon_if_nothing_returned)]
#![warn(clippy::match_same_arms)]
#![warn(clippy::needless_pass_by_ref_mut)]
#![warn(clippy::used_underscore_binding)]

use libspa::sys::spa_handle_factory;
use std::ffi::c_int;

mod freebsd_oss;
use spa_kitchen_sink_core::{backend, device, monitor, node, spa};
pub(crate) use spa_kitchen_sink_core::{debug, info, warn};

type SelectedBackend = freebsd_oss::FreeBsdOss;

const MONITOR_FACTORY: spa_handle_factory =
    monitor::factory::<SelectedBackend>(freebsd_oss::MONITOR_FACTORY_NAME.as_ptr());
const DEVICE_FACTORY: spa_handle_factory =
    device::factory::<SelectedBackend>(freebsd_oss::DEVICE_FACTORY_NAME.as_ptr());
const SINK_FACTORY: spa_handle_factory =
    node::sink_factory::<SelectedBackend>(freebsd_oss::SINK_FACTORY_NAME.as_ptr());
const SOURCE_FACTORY: spa_handle_factory =
    node::source_factory::<SelectedBackend>(freebsd_oss::SOURCE_FACTORY_NAME.as_ptr());

/// The SPA plugin entry point, called by the PipeWire host loader.
///
/// # Safety
/// `factory` and `index` must be valid, writable pointers; `index` selects
/// the factory to return and is advanced by one on success (the host calls
/// this in a loop until it returns 0).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn spa_handle_factory_enum(
    factory: *mut *const spa_handle_factory,
    index: *mut u32,
) -> c_int {
    assert!(!factory.is_null());
    assert!(!index.is_null());
    // non-null asserted above; the caller contract makes both valid and writable
    unsafe {
        match *index {
            0 => {
                *factory = &MONITOR_FACTORY;
                *index += 1;
                1
            }
            1 => {
                *factory = &DEVICE_FACTORY;
                *index += 1;
                1
            }
            2 => {
                *factory = &SINK_FACTORY;
                *index += 1;
                1
            }
            3 => {
                *factory = &SOURCE_FACTORY;
                *index += 1;
                1
            }
            _ => 0,
        }
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

use libspa::sys::{
    SPA_LOG_LEVEL_NONE, SPA_VERSION_LOG_TOPIC, SPA_VERSION_LOG_TOPIC_ENUM, spa_log_topic,
};

// The plugin crate owns its mutable registered topics. The shared shells only
// receive their addresses through the compile-time Backend binding.
static mut DEVICE_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: freebsd_oss::DEVICE_LOG_TOPIC.as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
static mut SINK_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: freebsd_oss::SINK_LOG_TOPIC.as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
static mut SOURCE_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: freebsd_oss::SOURCE_LOG_TOPIC.as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
static mut MONITOR_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: freebsd_oss::MONITOR_LOG_TOPIC.as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};

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
    size_of::<TopicPointer>() == size_of::<*mut spa_log_topic>()
        && align_of::<TopicPointer>() == align_of::<*mut spa_log_topic>()
);

// The entries are private (the host finds them through the section bounds,
// not by name), so no no_mangle; #[used] plus the section placement is what
// keeps them in the emitted spa_log_topic section.
#[unsafe(link_section = "spa_log_topic")]
#[used]
#[expect(non_upper_case_globals)]
static mut spa_log_topic_export_oss_device: TopicPointer = TopicPointer(&raw mut DEVICE_TOPIC);

#[unsafe(link_section = "spa_log_topic")]
#[used]
#[expect(non_upper_case_globals)]
static mut spa_log_topic_export_oss_sink: TopicPointer = TopicPointer(&raw mut SINK_TOPIC);

#[unsafe(link_section = "spa_log_topic")]
#[used]
#[expect(non_upper_case_globals)]
static mut spa_log_topic_export_oss_source: TopicPointer = TopicPointer(&raw mut SOURCE_TOPIC);

#[unsafe(link_section = "spa_log_topic")]
#[used]
#[expect(non_upper_case_globals)]
static mut spa_log_topic_export_oss_monitor: TopicPointer = TopicPointer(&raw mut MONITOR_TOPIC);

// the linker generates these for the section
unsafe extern "C" {
    static __start_spa_log_topic: *mut spa_log_topic;
    static __stop_spa_log_topic: *mut spa_log_topic;
}

#[unsafe(no_mangle)]
#[used]
static mut spa_log_topic_enum: libspa::sys::spa_log_topic_enum = libspa::sys::spa_log_topic_enum {
    version: SPA_VERSION_LOG_TOPIC_ENUM,
    topics: &raw const __start_spa_log_topic,
    topics_end: &raw const __stop_spa_log_topic,
};
