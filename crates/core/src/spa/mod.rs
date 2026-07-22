use libspa::sys::*;
use std::ffi::{CStr, CString, c_char, c_int, c_void};

mod hooks;
mod info;
mod log;
mod loop_api;
mod params;

pub(crate) use hooks::{
    ListenerList, LocalDispatchGuard, LocalListenerTarget, LocalNotificationQueue, dev_emit_result,
    node_emit_result,
};
#[cfg(debug_assertions)]
pub(crate) use info::dump_spa_dict;
pub(crate) use info::{DeviceInfo, Dictionary, NodeInfo, PortInfo, for_each_dict_item, key};
pub use log::Log;
pub(crate) use loop_api::{Loop, LoopSource, SendWrap, System, TimerFd, block_on_loop, queue_task};
#[cfg(test)]
pub(crate) use params::parse_back;
pub(crate) use params::{
    ParamStep, build_latency_info, build_latency_offset_prop_info, build_latency_offset_props,
    build_params_prop_info, build_process_latency_info, deserialize_pod, enum_params_loop,
    latency_info_default, parse_latency_info, parse_process_latency_info, pod_int_range, pod_prop,
    process_latency_default, process_latency_info_add, raw_slice_len_ok, serialize_pod,
};

pub(crate) const SPA_DEVICE_CHANGE_MASK_ALL: u32 =
    SPA_DEVICE_CHANGE_MASK_FLAGS | SPA_DEVICE_CHANGE_MASK_PARAMS | SPA_DEVICE_CHANGE_MASK_PROPS;

pub(crate) const SPA_DEVICE_OBJECT_CHANGE_MASK_ALL: u32 =
    SPA_DEVICE_OBJECT_CHANGE_MASK_FLAGS | SPA_DEVICE_OBJECT_CHANGE_MASK_PROPS;

pub(crate) const SPA_NODE_CHANGE_MASK_ALL: u32 =
    SPA_NODE_CHANGE_MASK_FLAGS | SPA_NODE_CHANGE_MASK_PARAMS | SPA_NODE_CHANGE_MASK_PROPS;

pub(crate) const SPA_PORT_CHANGE_MASK_ALL: u32 = SPA_PORT_CHANGE_MASK_FLAGS
    | SPA_PORT_CHANGE_MASK_PARAMS
    | SPA_PORT_CHANGE_MASK_PROPS
    | SPA_PORT_CHANGE_MASK_RATE;

// spa/node/node.h:241; the libspa-sys bindings don't carry the set_param flags
pub(crate) const SPA_NODE_PARAM_FLAG_NEAREST: u32 = 1 << 2;

// The listener-vtable version gate. The SPA_VERSION_* minimum constants are
// currently 0, so a literal `version >= MIN` comparison trips clippy's
// absurd_extreme_comparisons; routing MIN through a runtime parameter keeps
// the check future-proof without module-wide allows.
pub(crate) fn version_ok(version: u32, min: u32) -> bool {
    version >= min
}

// A host-shared io area (spa_io_clock/position/buffers/rate_match): a typed
// wrapper over the raw pointer the host hands to set_io/port_set_io. Plain
// data (one pointer), so it marshals through the SendWrap/block_on_loop
// paths unchanged. The single unsafe point is set(); read()/with() lean on
// its contract.
pub(crate) struct IoArea<T> {
    ptr: *mut T,
}

impl<T> IoArea<T> {
    pub(crate) const fn null() -> Self {
        Self {
            ptr: std::ptr::null_mut(),
        }
    }

    /// Point the area at host memory, or clear it with NULL.
    ///
    /// # Safety
    /// The caller has validated `data` against the area's size and alignment
    /// and the host keeps it valid while set (the set_io /
    /// port_set_io contract). The memory is host-shared by design; the
    /// data-loop invoke is what serializes our accesses against the swap.
    pub(crate) unsafe fn set(&mut self, data: *mut c_void) {
        self.ptr = data.cast();
    }

    pub(crate) fn is_null(&self) -> bool {
        self.ptr.is_null()
    }

    // Run `f` on the live area; None while cleared. &mut self so two live
    // &mut T over one area cannot coexist through safe calls (with &self a
    // nested with() would alias); no call site nests today, this keeps it
    // that way by construction.
    pub(crate) fn with<R>(&mut self, f: impl FnOnce(&mut T) -> R) -> Option<R> {
        // sound per set()'s contract (validity and serialization)
        unsafe { self.ptr.as_mut() }.map(f)
    }

    // read-only view of the live area; None while cleared
    pub(crate) fn with_ref<R>(&self, f: impl FnOnce(&T) -> R) -> Option<R> {
        // sound per set()'s contract (validity and serialization)
        unsafe { self.ptr.as_ref() }.map(f)
    }
}
