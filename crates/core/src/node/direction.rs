// Direction-agnostic node contracts. Sink and source monomorphize these hooks;
// shared state and FFI trampolines live in sibling modules.

use std::ffi::c_int;

use libspa::sys::*;

use crate::backend;
use crate::spa::Log;

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

pub(crate) use backend::StreamConfig as PortConfig;
pub(crate) type BackendOf<D> = <D as Direction>::Backend;
pub(crate) type BackendPropertiesOf<D> = <BackendOf<D> as backend::Backend>::Properties;
pub(crate) type BackendPropertyUpdateOf<D> =
    <BackendPropertiesOf<D> as backend::BackendProperties>::Update;
pub(crate) type WakeDriverOf<D> =
    <<D as Direction>::Device as backend::StreamLifecycle>::WakeDriver;

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
    /// Status a driving node passes to ready(): playback requests input with
    /// NEED_DATA; capture publishes output with HAVE_DATA.
    const READY_STATUS: i32;
    /// Direction-specific prefix for unknown-command warnings.
    const CMD_WARN_PREFIX: &'static str;

    // Send: crosses onto the data loop through install_device's swap
    type Backend: backend::Backend;
    type Device: backend::StreamLifecycle + Send;
    type DataExt: Default; // direction-specific data-loop fields
    type PortExt: Default; // direction-specific Port fields

    // Registered module log topic (see the lib.rs section entries). The host
    // mutates the pointee, so keep it as a raw pointer.
    fn log_topic() -> std::ptr::NonNull<spa_log_topic>;

    // Seed data-loop fields from the parsed control model.
    fn data_ext(properties: &BackendPropertiesOf<Self>) -> Self::DataExt;
    fn sync_backend_properties(ext: &mut Self::DataExt, properties: &BackendPropertiesOf<Self>);

    // Serialize one node parameter pod for (id, index).
    fn build_node_param(state: &mut MainState<Self>, id: u32, index: u32) -> ParamBuild;
    // Reset Props to their defaults.
    fn reset_props(state: &mut MainState<Self>, data: &DataControl<Self>) -> c_int;
    // Used from the main thread only; returns the applied configuration or a
    // negative errno with the device closed. Backend properties are applied
    // at the lifecycle point where their required stream geometry is known.
    fn try_open_configure(
        stream: &mut Self::Device,
        config: &PortConfig,
        properties: &BackendPropertiesOf<Self>,
        log: &Log,
    ) -> Result<backend::ConfigureOutcome, c_int>;
    // Reset direction-specific state during a device swap.
    fn on_device_swapped(state: &mut DataState<Self>, port_idx: usize);
    // port_use_buffers: direction-specific resets inside the loop-side swap
    fn on_buffers_swapped(state: &mut DataState<Self>, port_idx: usize);

    // send_command(Start): direction-specific resets, on the data loop
    fn on_start_loop(state: &mut DataState<Self>);
    // send_command(Pause): snapshot direction-specific live state before the
    // device continues independently of graph processing.
    fn on_pause_loop(_state: &mut DataState<Self>) {}
    // send_command(Suspend): direction-specific resets, on the data loop
    fn on_suspend_loop(state: &mut DataState<Self>);
    // set_io: the driver/follower role flipped on a live node
    fn on_role_flip(state: &mut DataState<Self>);

    // driver wake: debug-build cycle tracing (the sink prints one line)
    fn debug_cycle(state: &DataState<Self>, now: u64, nsec: u64);
    // driver-servo hooks (see node::driver_servo): the extra readiness
    // gate (the source's primed flag), the fill measurement, the recovery
    // hold (the sink's xrun window) and the signed servo error for a fill
    fn servo_ready(port: &Port<Self>) -> bool;
    fn servo_fill(port: &mut Port<Self>) -> u32;
    fn servo_hold(port: &Port<Self>) -> bool;
    fn servo_err(port: &Port<Self>, fill: u32) -> f64;
    // Applied buffer state used by the backend to select its native wake
    // threshold. The shared node does not interpret native readiness units.
    fn wake_buffer_state(port: &Port<Self>) -> backend::WakeBufferState;

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
