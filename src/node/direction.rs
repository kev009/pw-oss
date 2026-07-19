// Direction-agnostic node contracts. Sink and source monomorphize these hooks;
// shared state and FFI trampolines live in sibling modules.

use std::os::raw::c_int;

use libspa::sys::*;

use super::commands::{send_command, set_io};
use super::events::{add_listener, enum_params, set_callbacks, sync};
use super::params::set_param;
use super::ports::{add_port, port_enum_params, port_set_param, remove_port};
use super::process::{port_reuse_buffer, port_set_io, port_use_buffers, process};
use super::state::{DataControl, DataState, MainState, Port};

pub(crate) const MAX_PORTS: usize = 1;

pub(super) trait MutexExt<T> {
    fn lock_unpoisoned(&self) -> std::sync::MutexGuard<'_, T>;
}

impl<T> MutexExt<T> for std::sync::Mutex<T> {
    fn lock_unpoisoned(&self) -> std::sync::MutexGuard<'_, T> {
        self.lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

// the shared surface of oss::Dsp/oss::DspWriter used by the generic core;
// the direction-specific ops (write/odelay vs read/ispace) stay on the
// concrete types and are used from the Direction hooks
pub(crate) trait DeviceOps {
    fn new(path: &str) -> Self;
    fn path(&self) -> &str;
    fn is_closed(&self) -> bool;
    fn is_running(&self) -> bool;
    fn close(&mut self);
    fn suspend(&mut self) -> bool;
}

impl DeviceOps for crate::oss::Dsp {
    fn new(path: &str) -> Self {
        crate::oss::Dsp::new(path)
    }
    fn path(&self) -> &str {
        crate::oss::Dsp::path(self)
    }
    fn is_closed(&self) -> bool {
        crate::oss::Dsp::is_closed(self)
    }
    fn is_running(&self) -> bool {
        crate::oss::Dsp::is_running(self)
    }
    fn close(&mut self) {
        crate::oss::Dsp::close(self);
    }
    fn suspend(&mut self) -> bool {
        crate::oss::Dsp::suspend(self)
    }
}

impl DeviceOps for crate::oss::DspWriter {
    fn new(path: &str) -> Self {
        crate::oss::DspWriter::new(path)
    }
    fn path(&self) -> &str {
        &self.path
    }
    fn is_closed(&self) -> bool {
        crate::oss::DspWriter::is_closed(self)
    }
    fn is_running(&self) -> bool {
        crate::oss::DspWriter::is_running(self)
    }
    fn close(&mut self) {
        crate::oss::DspWriter::close(self);
    }
    fn suspend(&mut self) -> bool {
        crate::oss::DspWriter::suspend(self)
    }
}

// the negotiated format, shared by both directions (the stride is derived
// from the format map at parse time and stored)
#[derive(Debug, Clone)]
pub(crate) struct PortConfig {
    pub format: libspa::param::audio::AudioFormat,
    pub rate: u32,
    pub channels: u32,
    pub positions: Vec<u32>, // the negotiated channel positions, replayed in the Format readback
    pub flags: u32,
    pub stride: u32, // bytes per interleaved frame
}

impl PortConfig {
    pub(crate) fn oss_format(&self) -> u32 {
        // parse_config admits only formats from the map, so the lookup can't
        // miss; 0 (matching no AFMT) beats a panic across extern "C"
        super::format::oss_format_info(self.format.0)
            .map(|(m, _)| m)
            .unwrap_or(0)
    }
}

// outcome of a per-(id, index) node param build (the enum_params hook)
pub(crate) enum ParamBuild {
    Built(Vec<u8>), // the serialized pod for this (id, index)
    Exhausted,      // no more values for this param id
    Unknown,        // unknown param id
}

pub(crate) trait Direction: Sized + 'static {
    /// the port direction from the graph's perspective
    const DIRECTION: spa_direction;
    /// probe_caps()/install direction flag
    const PLAYBACK: bool;
    const MEDIA_CLASS: &'static str;
    /// status a driving node passes to ready(): a playback driver signals
    /// NEED_DATA; a capture driver signals HAVE_DATA (alsa-pcm.c capture_ready)
    const READY_STATUS: i32;
    /// Direction-specific prefix for unknown-command warnings.
    const CMD_WARN_PREFIX: &'static str;

    // Send: crosses onto the data loop through install_device's swap
    type Device: DeviceOps + Send;
    type MainExt: Default; // direction-specific main-loop/readback fields
    type DataExt: Default; // direction-specific data-loop fields
    type PortExt: Default; // direction-specific Port fields

    // Registered module log topic (see the lib.rs section entries). The host
    // mutates the pointee, so keep it as a raw pointer.
    fn log_topic() -> std::ptr::NonNull<spa_log_topic>;

    // Parse direction-specific node properties such as the sink's oss.delay.
    fn info_item(ext: &mut Self::MainExt, key: &str, value: &str);
    // Finalize direction-specific state after parsing the info dictionary.
    fn ext_ready(ext: &mut Self::MainExt);
    // Seed data-loop fields from the parsed control model.
    fn data_ext(ext: &Self::MainExt) -> Self::DataExt;

    // Serialize one node parameter pod for (id, index).
    fn build_node_param(state: &mut MainState<Self>, id: u32, index: u32) -> ParamBuild;
    // Reset Props to their defaults.
    fn reset_props(state: &mut MainState<Self>, data: &DataControl<Self>) -> c_int;
    // Apply oss.delay. The sink caps, stores, and rebuilds; the source ignores it.
    fn apply_oss_delay(state: &mut MainState<Self>, data: &DataControl<Self>, delay: u32) -> c_int;

    // used from the main thread only; returns 0 or -errno with the device
    // closed. `fragment` is the normalized oss.fragment (0 = automatic); the
    // source applies it at open time, the sink at prime time (the period is
    // only known then)
    fn try_open_configure(
        dsp: &mut Self::Device,
        config: &PortConfig,
        fragment: u32,
        log: &crate::spa::Log,
    ) -> c_int;
    // Reset direction-specific state during a device swap.
    fn on_device_swapped(state: &mut DataState<Self>, port_idx: usize);
    // port_use_buffers: direction-specific resets inside the loop-side swap
    fn on_buffers_swapped(state: &mut DataState<Self>, port_idx: usize);

    // send_command(Start): direction-specific resets, on the data loop
    fn on_start_loop(state: &mut DataState<Self>);
    // send_command(Suspend): direction-specific resets, on the data loop
    fn on_suspend_loop(state: &mut DataState<Self>);
    // set_io: the driver/follower role flipped on a live node
    fn on_role_flip(state: &mut DataState<Self>);

    // on_timeout: debug-build cycle tracing (the sink prints one line)
    fn debug_cycle(state: &DataState<Self>, now: u64, nsec: u64);
    // on_timeout servo hooks (see node::timeout_servo): the extra readiness
    // gate (the source's primed flag), the fill measurement, the recovery
    // hold (the sink's xrun window) and the signed servo error for a fill
    fn servo_ready(port: &Port<Self>) -> bool;
    fn servo_fill(port: &mut Port<Self>) -> u32;
    fn servo_hold(port: &Port<Self>) -> bool;
    fn servo_err(port: &Port<Self>, fill: u32) -> f64;

    // process(): the direction-specific data path over the ports
    fn process_ports(state: &mut DataState<Self>) -> c_int;

    const NODE_METHODS: spa_node_methods = spa_node_methods {
        version: SPA_VERSION_NODE_METHODS,
        add_listener: Some(add_listener::<Self>),
        set_callbacks: Some(set_callbacks::<Self>),
        sync: Some(sync::<Self>),
        enum_params: Some(enum_params::<Self>),
        set_param: Some(set_param::<Self>),
        set_io: Some(set_io::<Self>),
        send_command: Some(send_command::<Self>),
        add_port: Some(add_port),
        remove_port: Some(remove_port),
        port_enum_params: Some(port_enum_params::<Self>),
        port_set_param: Some(port_set_param::<Self>),
        port_use_buffers: Some(port_use_buffers::<Self>),
        port_set_io: Some(port_set_io::<Self>),
        port_reuse_buffer: Some(port_reuse_buffer),
        process: Some(process::<Self>),
    };
}
