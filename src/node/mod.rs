// The shared node core. The sink and source are the same SPA node modulo
// direction: everything direction-agnostic lives here once, generic over
// `Direction`, and the genuinely direction-specific logic (the process() data
// path, the servo error sign, priming/recovery semantics, the oss.delay prop)
// is supplied through the `Direction` hooks each module implements. The
// extern "C" vtable entries are generic and monomorphized per direction.

use std::ffi::{c_char, c_int, c_void};

use libspa::sys::*;

mod commands;
mod direction;
mod dll;
mod events;
mod factory;
mod format;
mod params;
mod ports;
mod process;
mod rebuild;
mod sink;
mod source;
mod state;
mod timing;

use dll::{BwAdapt, SpaDLL};
pub(crate) use events::handle_process_latency;
use events::{FormatPublication, MainEventTarget, NodeEvents};
use format::{build_buffers_info, build_enum_format_info, oss_format_info, snap_raw_to_caps};
use rebuild::{
    MainEvent, NodeShared, RebuildWork, RebuildWorkSlot, RebuildWorker, install_device,
    queue_main_event, release_rebuild_takeover,
};
pub(crate) use sink::{OSS_SINK_FACTORY, OSS_SINK_TOPIC};
pub(crate) use source::{OSS_SOURCE_FACTORY, OSS_SOURCE_TOPIC};
use state::*;
use timing::{
    RateLimit, device_period_bytes, ns_to_bytes, ns_to_frame_bytes, same_clock, set_clock_name,
    try_now_ns,
};
use timing::{on_wake, update_driver_wake};

use crate::oss::normalize_fragment;
use factory::{enum_interface_info, get_size, init};
use rebuild::{apply_props_param, poll_rebuild, queue_rebuild, store_and_rebuild};
use state::{DataControl, DataState, MainState, Port, valid_data_block};

use direction::MutexExt;
pub(crate) use direction::{DeviceOps, Direction, MAX_PORTS, ParamBuild, PortConfig};
