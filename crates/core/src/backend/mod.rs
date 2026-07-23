//! Audio stream capabilities, events, configuration, and I/O outcomes.

use crate::spa::Log;
use libspa::sys::*;
use std::ffi::c_int;
use std::os::fd::RawFd;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StreamDirection {
    Playback,
    Capture,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct DeviceKey(String);

impl DeviceKey {
    pub fn qualified(backend: &str, value: &str) -> Self {
        Self(format!("{backend}:{value}"))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn into_string(self) -> String {
        self.0
    }
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct EndpointKey(String);

impl EndpointKey {
    pub fn qualified(backend: &str, value: &str) -> Self {
        Self(format!("{backend}:{value}"))
    }

    #[cfg(test)]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StreamLocator {
    pub backend: &'static str,
    pub value: String,
}

impl StreamLocator {
    pub fn new(backend: &'static str, value: impl Into<String>) -> Self {
        Self {
            backend,
            value: value.into(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EndpointSnapshot {
    pub key: EndpointKey,
    pub object_id: u32,
    pub direction: StreamDirection,
    pub name: String,
    pub description: String,
    pub locator: StreamLocator,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeviceSnapshot {
    pub description: String,
    pub endpoints: Vec<EndpointSnapshot>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CatalogGroupSnapshot {
    pub key: DeviceKey,
    pub object_id: u32,
    pub properties: Vec<(String, String)>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CatalogChange {
    Added {
        snapshot: CatalogGroupSnapshot,
        diagnostic: String,
    },
    Removed {
        object_id: u32,
        diagnostic: String,
    },
}

#[derive(Debug)]
pub struct CatalogRescan {
    pub changes: Vec<CatalogChange>,
    pub error: Option<CatalogError>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct RouteKey(pub u64);

#[derive(Clone, Debug, PartialEq)]
pub struct RouteSnapshot {
    pub key: RouteKey,
    pub node_id: u32,
    pub direction: StreamDirection,
    pub name: String,
    pub description: String,
    pub priority: i32,
    pub availability: RouteAvailability,
    pub active: bool,
    pub volume: RouteVolume,
    pub mute: RouteMute,
    pub save: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RouteVolume {
    /// Normalized SPA channel volumes. Native units and ranges remain in the
    /// backend which produced the snapshot.
    pub values: Vec<f32>,
    pub channels: Vec<u32>,
    pub base: f32,
    pub step: f32,
    pub hardware: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RouteMute {
    pub value: bool,
    /// Whether the route has a native mute control. This is independent of
    /// native volume support on APIs which expose only one of the two.
    pub hardware: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Neutral route snapshots support every availability state.
pub enum RouteAvailability {
    Yes,
    No,
    Unknown,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RouteChange {
    pub key: Option<RouteKey>,
    pub volume: bool,
    pub mute: bool,
    pub selection: RouteSelectionOutcome,
    pub refresh: bool,
    pub diagnostic: Option<RouteDiagnostic>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RouteDiagnostic {
    pub level: RouteDiagnosticLevel,
    pub message: String,
}

impl RouteDiagnostic {
    pub fn info(message: impl Into<String>) -> Self {
        Self {
            level: RouteDiagnosticLevel::Info,
            message: message.into(),
        }
    }

    pub fn warning(message: impl Into<String>) -> Self {
        Self {
            level: RouteDiagnosticLevel::Warning,
            message: message.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteDiagnosticLevel {
    Info,
    Warning,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RouteWatchPolicy {
    /// Periodic refresh cadence. `None` means the backend needs no timer.
    pub poll_interval_ns: Option<u64>,
    /// Whether a native event source can nudge the periodic refresh.
    pub event_driven: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum RouteSelectionOutcome {
    #[default]
    Unchanged,
    Applied,
    Deflected,
    Failed,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RouteUpdate {
    pub activate: bool,
    pub values: Vec<RouteValueUpdate>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum RouteValueUpdate {
    Volume(Vec<f32>),
    Mute(bool),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BackendPropertyDescriptor {
    pub key: &'static str,
    pub description: &'static str,
    pub maximum: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HotplugDiagnostic {
    MonitorDetachAbort,
    MonitorLost,
    MonitorOpen,
    MonitorWatch,
    RouteDetachAbort,
    RouteLost,
    RouteNudge,
    RouteOpen,
    RouteOpenFallback,
    RouteTimerDetachAbort,
    RouteTimerArm,
    RouteTimerCreate,
    RouteTimerWatch,
    RouteWatch,
}

#[derive(Debug)]
pub struct CatalogError {
    code: i32,
    message: String,
}

impl CatalogError {
    pub fn new(code: i32, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }

    pub fn code(&self) -> i32 {
        self.code
    }
}

impl std::fmt::Display for CatalogError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for CatalogError {}

/// Compile-time binding between the shared SPA shells and one plugin backend.
///
/// This is intentionally a type family and assembly contract. Native behavior
/// remains on the narrower stream, catalog, route, and property traits below;
/// the shared real-time path is monomorphized and never uses trait objects.
pub trait Backend: Sized + 'static {
    type Capture: CaptureOperations<Properties = Self::Properties> + Send;
    type Playback: PlaybackOperations<Properties = Self::Properties> + Send;
    type Properties: BackendProperties;
    type DeviceInit: DeviceInit;
    type NodeInit: NodeInit<Properties = Self::Properties>;
    type Catalog: DeviceCatalog;
    type Hotplug: HotplugMonitor<Self::Catalog>;
    type Routes: RouteController<Self::Hotplug>;

    const DEVICE_API: &'static str;
    const DIAGNOSTIC_TAG: &'static str;
    const REBUILD_THREAD_PREFIX: &'static str;
    const SOURCE_COMMAND_PREFIX: &'static str;
    const STREAM_PATH: &'static str;
    const DEVICE_FACTORY_NAME: &'static std::ffi::CStr;
    const SINK_FACTORY_NAME: &'static std::ffi::CStr;
    const SOURCE_FACTORY_NAME: &'static std::ffi::CStr;

    fn clock_name(stream_path: &str) -> std::ffi::CString;
    fn fallback_caps() -> StreamCaps;
    fn hotplug_diagnostic(kind: HotplugDiagnostic) -> &'static str;
    fn probe_caps(path: &str, playback: bool) -> Option<StreamCaps>;
    fn validate_config(caps: &StreamCaps, config: &StreamConfig) -> Result<(), ChannelMapError>;

    /// The plugin crate owns the mutable topic objects registered with SPA.
    fn device_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic>;
    fn monitor_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic>;
    fn sink_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic>;
    fn source_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic>;
}

pub trait BackendProperties: Copy + Send + 'static {
    type Update: Copy + std::fmt::Debug + PartialEq + Send + 'static;

    fn new(playback: bool) -> Self;
    fn descriptors(&self) -> &'static [BackendPropertyDescriptor];
    fn values(&self) -> Vec<(&'static str, u32)>;
    fn decode_params(value: &libspa::pod::Value) -> Vec<Self::Update>;
    fn apply(&mut self, update: Self::Update) -> bool;
    fn reset(&mut self) -> bool;
}

pub trait DeviceInit: Default {
    fn parse(&mut self, key: &str, value: &str);
    fn parent_name(&self) -> Option<&str>;
    fn is_complete(&self) -> bool;
    fn snapshot(&self) -> Option<DeviceSnapshot>;
    fn missing_selector_diagnostic() -> &'static str;
    fn snapshot_diagnostic() -> &'static str;
}

#[derive(Debug)]
pub struct NodeInitValues<P> {
    pub stream_path: Option<String>,
    pub force_timer: bool,
    pub properties: P,
}

pub trait NodeInit: Sized {
    type Properties: BackendProperties;

    fn new(playback: bool) -> Self;
    fn force_timer_key() -> Option<&'static str>;
    fn parse(&mut self, key: &str, value: &str) -> InitItemResult;
    fn into_values(self) -> NodeInitValues<Self::Properties>;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitItemResult {
    Handled,
    InvalidBoolean,
    Unknown,
}

pub trait DeviceCatalog: Sized + Send + 'static {
    fn open_error_context() -> &'static str;
    fn refresh_error_context() -> &'static str;
    fn scan() -> Result<Self, CatalogError>;
    fn snapshots(&self) -> Vec<CatalogGroupSnapshot>;
}

pub trait HotplugMonitor<C>: Sized + Send + 'static
where
    C: DeviceCatalog,
{
    fn open() -> Result<Self, std::io::Error>;
    fn fd(&self) -> std::os::fd::RawFd;
    fn read_catalog_rescan(&mut self, catalog: &mut C) -> (bool, Option<CatalogRescan>);
}

pub trait RouteController<H>: Sized + Send + 'static {
    fn probe(snapshot: &DeviceSnapshot) -> (Self, Vec<RouteSnapshot>);
    fn refresh_all(&mut self, routes: &mut [RouteSnapshot]);
    fn poll(&mut self, routes: &mut [RouteSnapshot]) -> Vec<RouteChange>;
    fn apply(
        &mut self,
        routes: &mut [RouteSnapshot],
        key: RouteKey,
        update: RouteUpdate,
    ) -> RouteChange;
    fn watch_policy(&self) -> RouteWatchPolicy;
    fn read_hotplug(&self, routes: &[RouteSnapshot], monitor: &mut H) -> (bool, bool);
}

#[cfg(any(test, feature = "test-support"))]
pub mod fake;
#[cfg(any(test, feature = "test-support"))]
pub mod test_transport;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StreamToken(u64);

impl StreamToken {
    pub const fn for_port(index: usize) -> Self {
        Self(index as u64 + 1)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StreamIdentity {
    pub token: StreamToken,
    pub generation: u64,
}

impl StreamIdentity {
    pub const fn new(token: StreamToken, generation: u64) -> Self {
        Self { token, generation }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StreamWake {
    pub stream: StreamIdentity,
    /// Declares whether this wake carries timing meaning beyond readiness.
    pub timing: WakeTiming,
    /// Readable capture bytes or writable playback bytes at the wake edge.
    /// Boolean-only readiness APIs leave this unavailable.
    pub ready_bytes: Option<u32>,
    /// Logical stream-queue fill, distinct from unobservable hardware or
    /// conversion queues.
    pub queue: Option<QueueObservation>,
    /// A stream/device position and timestamp when the backend can relate the
    /// two without fabricating precision.
    pub clock: Option<ClockObservation>,
    /// Backend-described counter update. Counter reset/wrap semantics and the
    /// unit are explicit; the shared shell never assumes an event count.
    pub xruns: Option<XrunObservation>,
    pub state: StreamWakeState,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Backends expose only the measurement qualities they can support.
pub enum ObservationQuality {
    Exact,
    Estimated,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QueueObservation {
    pub fill_bytes: u64,
    pub quality: ObservationQuality,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Each selected backend exposes only the timing modes it supports.
pub enum WakeTiming {
    /// Readiness only. Publish the graph deadline and steer from queue fill.
    Readiness,
    /// Native notification arrival is a backend-supported clock observation.
    NotificationTime,
    /// Use the independently supplied monotonic timestamp when present.
    ObservedTime,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Position scope is independently selected by each backend API.
pub enum ClockScope {
    Device,
    Stream,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClockObservation {
    pub position: Option<PositionObservation>,
    pub timestamp: Option<TimestampObservation>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PositionObservation {
    pub frames: u64,
    pub scope: ClockScope,
    pub quality: ObservationQuality,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimestampObservation {
    /// Timestamp in the host monotonic clock domain. A backend which cannot
    /// establish that mapping leaves it unavailable.
    pub monotonic_ns: u64,
    pub accuracy_ns: Option<u64>,
    pub quality: ObservationQuality,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Native xrun APIs report events, frames, or bytes.
pub enum XrunUnit {
    Events,
    Frames,
    Bytes,
}

impl XrunUnit {
    const fn index(self) -> usize {
        match self {
            Self::Events => 0,
            Self::Frames => 1,
            Self::Bytes => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Identity of the active cumulative counter for one unit. A backend must
/// aggregate simultaneous native counters of the same unit or report them as
/// deltas; switching this identity begins a new epoch, and a retired identity
/// must not later resume its old cumulative series.
pub struct XrunCounter(pub u64);

impl XrunCounter {
    pub const PRIMARY: Self = Self(0);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Native counters differ in reset and wrap behavior.
pub enum CounterUpdate {
    /// A count accumulated since the preceding observation.
    Delta,
    /// A cumulative counter which may reset to zero but does not wrap within
    /// its advertised representation.
    Snapshot,
    /// A wrapping cumulative counter. `modulus` is one past its maximum.
    WrappingSnapshot { modulus: u64 },
    /// A total returned by an operation which also resets the native counter.
    ResettingTotal,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct XrunObservation {
    pub counter: XrunCounter,
    pub value: u64,
    pub unit: XrunUnit,
    pub update: CounterUpdate,
    pub quality: ObservationQuality,
}

impl XrunObservation {
    #[cfg(test)]
    pub const fn cumulative_events(value: u32) -> Self {
        Self {
            counter: XrunCounter::PRIMARY,
            value: value as u64,
            unit: XrunUnit::Events,
            update: CounterUpdate::Snapshot,
            quality: ObservationQuality::Exact,
        }
    }

    pub const fn resetting_events(value: u32) -> Self {
        Self {
            counter: XrunCounter::PRIMARY,
            value: value as u64,
            unit: XrunUnit::Events,
            update: CounterUpdate::ResettingTotal,
            quality: ObservationQuality::Exact,
        }
    }

    pub const fn wrapping_events_u32(value: u32) -> Self {
        Self {
            counter: XrunCounter::PRIMARY,
            value: value as u64,
            unit: XrunUnit::Events,
            update: CounterUpdate::WrappingSnapshot {
                modulus: 1u64 << u32::BITS,
            },
            quality: ObservationQuality::Exact,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // A selected backend need not signal every state from its wake path.
pub enum StreamWakeState {
    Active,
    Reconfigure,
    Disconnected,
}

impl StreamWakeState {
    pub const fn requires_rebuild(self) -> bool {
        matches!(self, Self::Reconfigure | Self::Disconnected)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct XrunDelta {
    pub events: u32,
    pub lost_frames: Option<u64>,
    pub lost_bytes: Option<u64>,
    pub quality: Option<ObservationQuality>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct XrunTracker {
    // One allocation-free baseline per semantic unit. Counter identity marks
    // an epoch/source replacement within that unit and prevents cross-source
    // subtraction.
    previous: [Option<XrunBaseline>; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct XrunBaseline {
    counter: XrunCounter,
    value: u64,
    wrapping_modulus: Option<u64>,
}

fn cumulative_delta(current: u64, previous: Option<XrunBaseline>) -> u64 {
    let Some(previous) = previous else {
        return current;
    };
    match previous.wrapping_modulus {
        Some(modulus) if modulus > 0 => {
            let current = current % modulus;
            let previous = previous.value % modulus;
            if current >= previous {
                current - previous
            } else {
                modulus - previous + current
            }
        }
        _ if current >= previous.value => current - previous.value,
        // A non-wrapping counter reset began a new measurement epoch.
        _ => current,
    }
}

impl XrunTracker {
    pub fn observe(&mut self, observation: XrunObservation) -> XrunDelta {
        let previous = &mut self.previous[observation.unit.index()];
        let matching = (*previous).filter(|baseline| baseline.counter == observation.counter);
        let delta = match observation.update {
            CounterUpdate::Delta => observation.value,
            CounterUpdate::ResettingTotal => {
                let delta = cumulative_delta(observation.value, matching);
                *previous = None;
                delta
            }
            CounterUpdate::Snapshot => {
                let delta = cumulative_delta(observation.value, matching);
                *previous = Some(XrunBaseline {
                    counter: observation.counter,
                    value: observation.value,
                    wrapping_modulus: None,
                });
                delta
            }
            CounterUpdate::WrappingSnapshot { modulus } => {
                let prior = matching.map(|mut baseline| {
                    baseline.wrapping_modulus = Some(modulus);
                    baseline
                });
                let delta = cumulative_delta(observation.value, prior);
                *previous = Some(XrunBaseline {
                    counter: observation.counter,
                    value: observation.value,
                    wrapping_modulus: Some(modulus),
                });
                delta
            }
        };

        match observation.unit {
            XrunUnit::Events => XrunDelta {
                events: delta.min(u64::from(u32::MAX)) as u32,
                quality: Some(observation.quality),
                ..XrunDelta::default()
            },
            XrunUnit::Frames => XrunDelta {
                events: u32::from(delta != 0),
                lost_frames: Some(delta),
                lost_bytes: None,
                quality: Some(observation.quality),
            },
            XrunUnit::Bytes => XrunDelta {
                events: u32::from(delta != 0),
                lost_frames: None,
                lost_bytes: Some(delta),
                quality: Some(observation.quality),
            },
        }
    }

    pub fn reset(&mut self) {
        self.previous = [None; 3];
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WakeBufferState {
    pub frame_stride: u32,
    pub period_bytes: u32,
    pub quantum_bytes: u32,
    pub capacity_bytes: u32,
    pub target_fill_bytes: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WakeEvent {
    Timer,
    Stream(StreamWake),
}

#[derive(Debug)]
pub struct WakeError {
    message: String,
    threshold: Option<u32>,
}

impl WakeError {
    pub fn new(error: impl std::fmt::Display) -> Self {
        Self {
            message: error.to_string(),
            threshold: None,
        }
    }

    pub fn threshold(value: u32, error: impl std::fmt::Display) -> Self {
        Self {
            message: error.to_string(),
            threshold: Some(value),
        }
    }

    pub const fn threshold_value(&self) -> Option<u32> {
        self.threshold
    }
}

impl std::fmt::Display for WakeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for WakeError {}

pub trait WakeDriver {
    fn notification_fd(&self) -> RawFd;
    fn unregister_stream(&mut self) -> Result<(), WakeError>;
    fn arm_timer(&self, delay_ns: u64) -> Result<(), WakeError>;
    fn next_event(&self) -> Result<Option<WakeEvent>, WakeError>;
}

/// Lifecycle and wake operations shared by capture and playback streams.
/// Direction-specific I/O and buffer policy remain methods of the concrete
/// stream types selected by the compile-time backend binding.
pub trait StreamLifecycle {
    type WakeDriver: WakeDriver;

    fn new(path: &str) -> Self;
    fn path(&self) -> &str;
    fn is_closed(&self) -> bool;
    fn is_running(&self) -> bool;
    fn register_wake(
        &self,
        driver: &mut Self::WakeDriver,
        stream: StreamIdentity,
        buffer: WakeBufferState,
    ) -> Result<(), WakeError>;
    fn wake_available() -> bool;
    fn new_wake_driver() -> Result<Self::WakeDriver, WakeError>;
    fn wake_diagnostic(kind: WakeDiagnostic) -> &'static str;
    fn close(&mut self);
    fn suspend(&mut self) -> bool;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WakeDiagnostic {
    Selected,
    Create,
    Read,
    Remove,
    Register,
    Threshold,
    Arm,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WriteOutcome {
    pub bytes: usize,
    pub status: IoStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReadOutcome {
    pub bytes: usize,
    pub status: IoStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StreamError {
    pub native_code: i32,
}

impl StreamError {
    pub const fn from_native_code(native_code: i32) -> Self {
        Self { native_code }
    }
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::io::Error::from_raw_os_error(self.native_code).fmt(f)
    }
}

impl std::error::Error for StreamError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // Selected backends need not emit every semantic outcome.
pub enum IoStatus {
    Progress,
    WouldBlock,
    RecoveredXrun,
    Reconfigure,
    Disconnected,
    Fatal(StreamError),
}

impl IoStatus {
    pub fn requires_rebuild(self) -> bool {
        matches!(
            self,
            Self::Reconfigure | Self::Disconnected | Self::Fatal(_)
        )
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BufferLayout {
    pub queued_bytes: u32,
    pub quantum_bytes: u32,
    pub capacity_bytes: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CaptureBufferRequest {
    pub period_bytes: u32,
    /// Graph rate used only when planning initial capacity. Zero during a
    /// live retune means the backend must reuse the already-granted ring.
    pub graph_rate: u32,
    pub stride: u32,
    pub device_rate: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CaptureBufferGeometry {
    pub capacity_bytes: u32,
    pub quantum_bytes: u32,
    pub target_fill_bytes: u32,
    pub peak_fill_bytes: u32,
    pub required_capacity_bytes: u32,
    pub device_lost: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CaptureRetune {
    Unchanged,
    Pending,
    Applied(CaptureBufferGeometry),
    Reprime,
    Rebuild,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PlaybackBufferRequest {
    pub period_bytes: u32,
    /// Graph rate used only when planning initial capacity. Zero during a
    /// live retune means the backend must reuse the already-granted ring.
    pub graph_rate: u32,
    pub stride: u32,
    pub device_rate: u32,
    /// Bytes presented by this graph cycle.
    pub write_bytes: u32,
    /// Largest write the negotiated rate matcher can present.
    pub maximum_write_bytes: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BufferConstraints {
    pub capacity_limit_bytes: Option<u32>,
    pub quantum_cap_frames: Option<u32>,
    /// Backend-owned reason used by compatibility diagnostics. The shared
    /// shell treats it as an opaque description of the published cap.
    pub quantum_cap_basis: Option<&'static str>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PlaybackBufferGeometry {
    pub capacity_bytes: u32,
    pub quantum_bytes: u32,
    pub target_fill_bytes: u32,
    pub target_goal_bytes: u32,
    pub minimum_fill_bytes: u32,
    pub required_capacity_bytes: u32,
    pub delay_capped: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlaybackRetune {
    Unchanged,
    Pending,
    Applied(PlaybackBufferGeometry),
    Reprime,
    Rebuild,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PauseOutcome {
    /// The device queue was empty or was preserved for a later resume.
    Preserved,
    /// Queue preservation is unavailable; reset and prime again on Start.
    Reprime,
}

pub trait CaptureOperations: StreamLifecycle {
    type Properties: BackendProperties;

    fn configure(
        &mut self,
        config: &StreamConfig,
        properties: &Self::Properties,
        log: &Log,
    ) -> Result<ConfigureOutcome, c_int>;
    fn prime_buffer(
        &mut self,
        request: CaptureBufferRequest,
        properties: &Self::Properties,
        scratch: &mut [u8],
        log: &Log,
    ) -> CaptureBufferGeometry;
    fn retune_buffer(
        &mut self,
        request: CaptureBufferRequest,
        primed: bool,
        log: &Log,
    ) -> CaptureRetune;
    /// Read interleaved data. A successful outcome must report a whole number
    /// of negotiated frames. If native I/O consumes the head of a torn frame,
    /// the backend must hide it and discard the tail before returning later
    /// data so the graph never observes a shifted sample boundary.
    fn read(&mut self, data: &mut [u8]) -> ReadOutcome;
    /// Observe the live capture queue. The stream must be running.
    fn queued_bytes(&self) -> u32;
    /// Observe live capture overruns. The stream must be running.
    fn overruns(&self) -> XrunObservation;
    fn recover_overrun(
        &mut self,
        overrun_count: u32,
        pre_read_fill: Option<u32>,
        log: &Log,
    ) -> Option<bool>;
    fn log_overrun_recovery(&self, count: u32, now_ns: u64, suppressed: u32, log: &Log);
    fn clear_overrun_observation(&mut self);
}

pub trait PlaybackOperations: StreamLifecycle {
    type Properties: BackendProperties;

    fn configure(
        &mut self,
        config: &StreamConfig,
        properties: &Self::Properties,
        log: &Log,
    ) -> Result<ConfigureOutcome, c_int>;
    fn prime_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        properties: &Self::Properties,
        log: &Log,
    ) -> PlaybackBufferGeometry;
    fn retune_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        current_fill_bytes: u32,
        now_ns: u64,
        log: &Log,
    ) -> PlaybackRetune;
    fn write(&mut self, data: &[u8]) -> WriteOutcome;
    fn write_silence(&mut self, bytes: u32);
    fn end_buffer_sequence(&mut self) -> bool;
    /// Observe the live playback queue. The stream must be running.
    fn queued_bytes(&self) -> u32;
    /// Observe live playback underruns. The stream must be running.
    fn underruns(&self) -> XrunObservation;
    fn log_underrun_recovery(&self, count: u32, now_ns: u64, suppressed: u32, log: &Log);
    fn log_ignored_underruns(
        &self,
        count: u32,
        fill_bytes: u32,
        recovery_threshold_bytes: u32,
        log: &Log,
    );
    fn pause(&mut self) -> Result<PauseOutcome, StreamError>;
    fn resume(&mut self) -> Result<(), StreamError>;
    fn underrun_low(
        target_fill: u32,
        delivery_quantum: u32,
        period_bytes: u32,
        drained_bytes: u32,
    ) -> u32;
    fn debug_log_priorities(log: &Log);
}

impl WriteOutcome {
    pub fn consumed(bytes: usize) -> Self {
        Self {
            bytes,
            status: IoStatus::Progress,
        }
    }

    pub fn would_block(&self) -> bool {
        self.bytes == 0 && self.status == IoStatus::WouldBlock
    }

    pub fn retryable_partial(&self) -> bool {
        matches!(
            self.status,
            IoStatus::Progress | IoStatus::WouldBlock | IoStatus::RecoveredXrun
        )
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct StreamConfig {
    pub format: libspa::param::audio::AudioFormat,
    pub rate: u32,
    pub channels: u32,
    pub positions: Vec<u32>,
    pub flags: u32,
    pub stride: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConfigureOutcome {
    pub actual_config: AppliedStreamConfig,
    pub applied_buffer: AppliedBufferGeometry,
    pub buffer_constraints: BufferConstraints,
    pub adjusted: AdjustmentFlags,
}

pub type AppliedStreamConfig = StreamConfig;

/// Storage width of one sample for the interleaved raw formats understood by
/// the shared graph path. Backend capability sets still decide which of these
/// formats a concrete stream may negotiate.
pub const fn bytes_per_sample(format: u32) -> Option<u32> {
    match format {
        SPA_AUDIO_FORMAT_S8
        | SPA_AUDIO_FORMAT_U8
        | SPA_AUDIO_FORMAT_ULAW
        | SPA_AUDIO_FORMAT_ALAW => Some(1),
        SPA_AUDIO_FORMAT_S16_LE
        | SPA_AUDIO_FORMAT_S16_BE
        | SPA_AUDIO_FORMAT_U16_LE
        | SPA_AUDIO_FORMAT_U16_BE => Some(2),
        SPA_AUDIO_FORMAT_S24_LE
        | SPA_AUDIO_FORMAT_S24_BE
        | SPA_AUDIO_FORMAT_U24_LE
        | SPA_AUDIO_FORMAT_U24_BE
        | SPA_AUDIO_FORMAT_S20_LE
        | SPA_AUDIO_FORMAT_S20_BE
        | SPA_AUDIO_FORMAT_U20_LE
        | SPA_AUDIO_FORMAT_U20_BE
        | SPA_AUDIO_FORMAT_S18_LE
        | SPA_AUDIO_FORMAT_S18_BE
        | SPA_AUDIO_FORMAT_U18_LE
        | SPA_AUDIO_FORMAT_U18_BE => Some(3),
        SPA_AUDIO_FORMAT_S24_32_LE
        | SPA_AUDIO_FORMAT_S24_32_BE
        | SPA_AUDIO_FORMAT_U24_32_LE
        | SPA_AUDIO_FORMAT_U24_32_BE
        | SPA_AUDIO_FORMAT_S32_LE
        | SPA_AUDIO_FORMAT_S32_BE
        | SPA_AUDIO_FORMAT_U32_LE
        | SPA_AUDIO_FORMAT_U32_BE
        | SPA_AUDIO_FORMAT_F32_LE
        | SPA_AUDIO_FORMAT_F32_BE => Some(4),
        SPA_AUDIO_FORMAT_F64_LE | SPA_AUDIO_FORMAT_F64_BE => Some(8),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AppliedBufferGeometry {
    /// Ring geometry is negotiated later when the graph period is known.
    pub capacity_bytes: Option<u32>,
    pub quantum_bytes: Option<u32>,
    pub delivery: DeliveryQuantum,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[allow(dead_code)] // The neutral contract represents every delivery-quality state.
pub enum QuantumQuality {
    Exact,
    Estimated,
    Variable,
    #[default]
    Unavailable,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DeliveryQuantum {
    pub frames: u32,
    pub rate: u32,
    pub quality: QuantumQuality,
}

impl DeliveryQuantum {
    pub const fn unavailable() -> Self {
        Self {
            frames: 0,
            rate: 0,
            quality: QuantumQuality::Unavailable,
        }
    }

    pub fn duration_ns(self) -> u64 {
        if self.frames == 0 || self.rate == 0 {
            0
        } else {
            (u64::from(self.frames) * 1_000_000_000) / u64::from(self.rate)
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AdjustmentFlags(u32);

impl AdjustmentFlags {
    const FORMAT: u32 = 1 << 0;
    const RATE: u32 = 1 << 1;
    const CHANNELS: u32 = 1 << 2;
    const LAYOUT: u32 = 1 << 3;

    pub fn between(requested: &StreamConfig, applied: &StreamConfig) -> Self {
        let mut flags = 0;
        if requested.format != applied.format {
            flags |= Self::FORMAT;
        }
        if requested.rate != applied.rate {
            flags |= Self::RATE;
        }
        if requested.channels != applied.channels || requested.stride != applied.stride {
            flags |= Self::CHANNELS;
        }
        if requested.positions != applied.positions || requested.flags != applied.flags {
            flags |= Self::LAYOUT;
        }
        Self(flags)
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SilencePattern {
    bytes: [u8; 8],
    len: u8,
    pub frame_multiple: usize,
}

impl SilencePattern {
    pub const fn zero(frame_multiple: usize) -> Self {
        Self {
            bytes: [0; 8],
            len: 1,
            frame_multiple,
        }
    }

    pub fn for_config(config: &StreamConfig) -> Self {
        let (bytes, len) = match config.format.0 {
            SPA_AUDIO_FORMAT_U8 | SPA_AUDIO_FORMAT_U8P => ([0x80, 0, 0, 0, 0, 0, 0, 0], 1),
            SPA_AUDIO_FORMAT_U16_LE => ([0x00, 0x80, 0, 0, 0, 0, 0, 0], 2),
            SPA_AUDIO_FORMAT_U16_BE => ([0x80, 0x00, 0, 0, 0, 0, 0, 0], 2),
            SPA_AUDIO_FORMAT_U24_LE => ([0x00, 0x00, 0x80, 0, 0, 0, 0, 0], 3),
            SPA_AUDIO_FORMAT_U24_BE => ([0x80, 0x00, 0x00, 0, 0, 0, 0, 0], 3),
            SPA_AUDIO_FORMAT_U20_LE => ([0x00, 0x00, 0x08, 0, 0, 0, 0, 0], 3),
            SPA_AUDIO_FORMAT_U20_BE => ([0x08, 0x00, 0x00, 0, 0, 0, 0, 0], 3),
            SPA_AUDIO_FORMAT_U18_LE => ([0x00, 0x00, 0x02, 0, 0, 0, 0, 0], 3),
            SPA_AUDIO_FORMAT_U18_BE => ([0x02, 0x00, 0x00, 0, 0, 0, 0, 0], 3),
            SPA_AUDIO_FORMAT_U24_32_LE => ([0x00, 0x00, 0x80, 0x00, 0, 0, 0, 0], 4),
            SPA_AUDIO_FORMAT_U24_32_BE => ([0x00, 0x80, 0x00, 0x00, 0, 0, 0, 0], 4),
            SPA_AUDIO_FORMAT_U32_LE => ([0x00, 0x00, 0x00, 0x80, 0, 0, 0, 0], 4),
            SPA_AUDIO_FORMAT_U32_BE => ([0x80, 0x00, 0x00, 0x00, 0, 0, 0, 0], 4),
            SPA_AUDIO_FORMAT_ULAW => ([0xff, 0, 0, 0, 0, 0, 0, 0], 1),
            SPA_AUDIO_FORMAT_ALAW => ([0x55, 0, 0, 0, 0, 0, 0, 0], 1),
            _ => ([0; 8], 1),
        };
        Self {
            bytes,
            len,
            frame_multiple: config.stride.max(1) as usize,
        }
    }

    pub fn fill(self, output: &mut [u8]) {
        self.fill_at(0, output);
    }

    /// Fill silence beginning at `frame_offset` bytes into an interleaved
    /// frame. This is used when a short device write leaves a partial frame
    /// whose remaining bytes must be completed without shifting a multibyte
    /// unsigned sample's midpoint encoding.
    pub fn fill_at(self, frame_offset: usize, output: &mut [u8]) {
        let pattern = &self.bytes[..self.len as usize];
        let frame_multiple = self.frame_multiple.max(1);
        for (index, byte) in output.iter_mut().enumerate() {
            let offset = (frame_offset + index) % frame_multiple;
            *byte = pattern[offset % pattern.len()];
        }
    }

    /// Return the repeated byte when this encoding's silence can be produced
    /// by a byte fill. Concrete backends use this to detect native APIs whose
    /// silence operation cannot express a multibyte midpoint pattern.
    pub fn uniform_byte(self) -> Option<u8> {
        let pattern = &self.bytes[..self.len as usize];
        pattern
            .iter()
            .all(|byte| *byte == pattern[0])
            .then_some(pattern[0])
    }
}

impl StreamConfig {
    pub fn silence_pattern(&self) -> SilencePattern {
        SilencePattern::for_config(self)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[allow(dead_code)] // The neutral contract represents every conversion owner.
pub enum ConversionPath {
    None,
    Kernel,
    Server,
    Library,
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct ChannelLayout {
    pub channels: u32,
    /// `None` means the backend reports a channel count but no positions.
    pub positions: Option<Vec<u32>>,
}

#[derive(Debug, Clone, Eq, PartialEq)]
#[allow(dead_code)] // Range represents unpositioned count-only APIs.
pub enum ChannelSet {
    Discrete(Vec<ChannelLayout>),
    Range { min: u32, max: u32 },
}

impl ChannelSet {
    pub fn layouts(&self) -> Vec<ChannelLayout> {
        match self {
            Self::Discrete(layouts) => layouts.clone(),
            Self::Range { min, max } => (*min..=*max)
                .map(|channels| ChannelLayout {
                    channels,
                    positions: None,
                })
                .collect(),
        }
    }

    fn admits(&self, channels: u32, positions: Option<&[u32]>, flags: ConfigurationFlags) -> bool {
        match self {
            Self::Range { min, max } => positions.is_none() && (*min..=*max).contains(&channels),
            Self::Discrete(layouts) => layouts.iter().any(|layout| {
                if layout.channels != channels {
                    return false;
                }
                match (positions, layout.positions.as_deref()) {
                    (None, _) => true,
                    (Some(requested), None) => {
                        requested.len() == channels as usize
                            && requested.iter().all(|position| {
                                *position == SPA_AUDIO_CHANNEL_UNKNOWN
                                    || *position == SPA_AUDIO_CHANNEL_NA
                                    || *position >= SPA_AUDIO_CHANNEL_AUX0
                            })
                    }
                    (Some(requested), Some(advertised)) => {
                        (flags.allows_opaque_layout()
                            && requested.len() == channels as usize
                            && requested.iter().all(|position| {
                                *position == SPA_AUDIO_CHANNEL_UNKNOWN
                                    || *position == SPA_AUDIO_CHANNEL_NA
                                    || *position >= SPA_AUDIO_CHANNEL_AUX0
                            }))
                            || requested == advertised
                            || (flags.allows_layout_reorder()
                                && requested.len() == advertised.len()
                                && requested.iter().enumerate().all(|(index, position)| {
                                    !requested[..index].contains(position)
                                        && advertised.contains(position)
                                }))
                    }
                }
            }),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ConfigurationFlags(u32);

impl ConfigurationFlags {
    const LAYOUT_REORDER: u32 = 1 << 0;
    const OPAQUE_LAYOUT: u32 = 1 << 1;

    #[cfg(test)]
    pub const fn with_layout_reorder() -> Self {
        Self(Self::LAYOUT_REORDER)
    }

    pub const fn with_opaque_layout() -> Self {
        Self(Self::OPAQUE_LAYOUT)
    }

    pub const fn with_layout_reorder_and_opaque() -> Self {
        Self(Self::LAYOUT_REORDER | Self::OPAQUE_LAYOUT)
    }

    const fn allows_layout_reorder(self) -> bool {
        self.0 & Self::LAYOUT_REORDER != 0
    }

    const fn allows_opaque_layout(self) -> bool {
        self.0 & Self::OPAQUE_LAYOUT != 0
    }
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub enum RateSet {
    Discrete(Vec<u32>),
    Range { min: u32, max: u32 },
}

impl RateSet {
    pub fn admits(&self, rate: u32, tolerance: u32) -> bool {
        match self {
            Self::Discrete(rates) => rates.contains(&rate),
            Self::Range { min, max } => {
                rate.saturating_add(tolerance) >= *min && rate <= max.saturating_add(tolerance)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct StreamConfiguration {
    /// SPA formats offered to the graph, in preference order.
    pub formats: Vec<u32>,
    pub channels: ChannelSet,
    pub rates: RateSet,
    pub preferred_rate: Option<u32>,
    /// Tolerance used when a dense backend rate range snaps values.
    pub rate_tolerance: u32,
    pub conversion: ConversionPath,
    pub flags: ConfigurationFlags,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StreamCaps {
    pub configurations: Vec<StreamConfiguration>,
    pub preferred: usize,
}

impl StreamCaps {
    pub fn admits(
        &self,
        spa_format: u32,
        channels: u32,
        positions: Option<&[u32]>,
        rate: u32,
    ) -> bool {
        self.configurations.iter().any(|configuration| {
            configuration.formats.contains(&spa_format)
                && configuration
                    .channels
                    .admits(channels, positions, configuration.flags)
                && configuration
                    .rates
                    .admits(rate, configuration.rate_tolerance)
        })
    }

    pub fn conversion_for(
        &self,
        spa_format: u32,
        channels: u32,
        rate: u32,
    ) -> Option<ConversionPath> {
        self.configurations
            .iter()
            .find(|configuration| {
                configuration.formats.contains(&spa_format)
                    && configuration
                        .channels
                        .admits(channels, None, configuration.flags)
                    && configuration
                        .rates
                        .admits(rate, configuration.rate_tolerance)
            })
            .map(|configuration| configuration.conversion)
    }

    pub fn preferred_configuration(&self) -> Option<&StreamConfiguration> {
        let preferred = if self.preferred < self.configurations.len() {
            self.preferred
        } else {
            0
        };
        self.configurations.get(preferred)
    }

    pub fn configurations_in_preference_order(&self) -> impl Iterator<Item = &StreamConfiguration> {
        let preferred = if self.preferred < self.configurations.len() {
            self.preferred
        } else {
            0
        };
        self.preferred_configuration().into_iter().chain(
            self.configurations
                .iter()
                .enumerate()
                .filter(move |(index, _)| *index != preferred)
                .map(|(_, configuration)| configuration),
        )
    }
}

pub fn offered_formats(configuration: &StreamConfiguration) -> &[u32] {
    &configuration.formats
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChannelMapError {
    Unsupported,
    ConvertlessReorder,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn configuration(format: u32, channels: u32, rate: u32) -> StreamConfiguration {
        StreamConfiguration {
            formats: vec![format],
            channels: ChannelSet::Discrete(vec![ChannelLayout {
                channels,
                positions: None,
            }]),
            rates: RateSet::Discrete(vec![rate]),
            preferred_rate: None,
            rate_tolerance: 0,
            conversion: ConversionPath::Kernel,
            flags: ConfigurationFlags::default(),
        }
    }

    #[test]
    fn persistent_identity_is_backend_qualified() {
        assert_eq!(
            DeviceKey::qualified("test-audio", "card0").as_str(),
            "test-audio:card0"
        );
        assert_eq!(
            EndpointKey::qualified("test-audio", "card0.play").as_str(),
            "test-audio:card0.play"
        );
        assert_eq!(
            StreamLocator::new("test-audio", "test://card0/play"),
            StreamLocator {
                backend: "test-audio",
                value: "test://card0/play".into(),
            }
        );
    }

    #[test]
    fn backend_conversion_does_not_cross_configuration_constraints() {
        let caps = StreamCaps {
            configurations: vec![configuration(10, 2, 48_000), configuration(20, 6, 96_000)],
            preferred: 0,
        };

        assert!(caps.admits(10, 2, None, 48_000));
        assert!(caps.admits(20, 6, None, 96_000));
        assert!(!caps.admits(20, 2, None, 48_000));
        assert!(!caps.admits(10, 6, None, 96_000));
        assert!(!caps.admits(30, 2, None, 48_000));
    }

    #[test]
    fn positioned_layouts_require_an_advertised_or_explicitly_reorderable_map() {
        let mut strict = configuration(10, 2, 48_000);
        strict.channels = ChannelSet::Discrete(vec![ChannelLayout {
            channels: 2,
            positions: Some(vec![1, 2]),
        }]);
        let requested = [2, 1];
        let caps = StreamCaps {
            configurations: vec![strict.clone()],
            preferred: 0,
        };
        assert!(!caps.admits(10, 2, Some(&requested), 48_000));

        let mut opaque = strict.clone();
        opaque.flags = ConfigurationFlags::with_opaque_layout();
        let caps = StreamCaps {
            configurations: vec![opaque],
            preferred: 0,
        };
        assert!(caps.admits(
            10,
            2,
            Some(&[SPA_AUDIO_CHANNEL_AUX0, SPA_AUDIO_CHANNEL_AUX1]),
            48_000
        ));
        assert!(caps.admits(
            10,
            2,
            Some(&[SPA_AUDIO_CHANNEL_UNKNOWN, SPA_AUDIO_CHANNEL_NA]),
            48_000
        ));

        strict.flags = ConfigurationFlags::with_layout_reorder();
        let caps = StreamCaps {
            configurations: vec![strict],
            preferred: 0,
        };
        assert!(caps.admits(10, 2, Some(&requested), 48_000));
        assert!(!caps.admits(10, 2, Some(&[1, 1]), 48_000));

        let mut unpositioned = configuration(10, 2, 48_000);
        unpositioned.channels = ChannelSet::Discrete(vec![ChannelLayout {
            channels: 2,
            positions: None,
        }]);
        let caps = StreamCaps {
            configurations: vec![unpositioned],
            preferred: 0,
        };
        assert!(caps.admits(
            10,
            2,
            Some(&[SPA_AUDIO_CHANNEL_AUX0, SPA_AUDIO_CHANNEL_AUX1]),
            48_000
        ));
        assert!(!caps.admits(
            10,
            2,
            Some(&[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR]),
            48_000
        ));
    }

    #[test]
    fn semantic_io_outcomes_distinguish_progress_and_backpressure() {
        let consumed = WriteOutcome::consumed(128);
        assert_eq!(consumed.status, IoStatus::Progress);
        assert!(consumed.retryable_partial());
        assert!(!consumed.would_block());

        let blocked = WriteOutcome {
            bytes: 0,
            status: IoStatus::WouldBlock,
        };
        assert!(blocked.retryable_partial());
        assert!(blocked.would_block());

        let failed = WriteOutcome {
            bytes: 64,
            status: IoStatus::Fatal(StreamError::from_native_code(libc::EIO)),
        };
        assert!(!failed.retryable_partial());
        assert!(!failed.would_block());
        assert!(failed.status.requires_rebuild());
        assert!(
            WriteOutcome {
                bytes: 64,
                status: IoStatus::RecoveredXrun,
            }
            .retryable_partial()
        );
        assert!(!IoStatus::WouldBlock.requires_rebuild());
    }

    #[test]
    fn xrun_tracker_preserves_counter_units_and_epoch_semantics() {
        let mut tracker = XrunTracker::default();
        assert_eq!(
            tracker.observe(XrunObservation::cumulative_events(3)),
            XrunDelta {
                events: 3,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );
        assert_eq!(
            tracker.observe(XrunObservation::cumulative_events(3)),
            XrunDelta {
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );

        // The reset operation covers the same native counter as the wake
        // snapshot. Report only the portion not already published.
        assert_eq!(
            tracker.observe(XrunObservation::resetting_events(5)),
            XrunDelta {
                events: 2,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );
        assert_eq!(
            tracker.observe(XrunObservation::cumulative_events(1)),
            XrunDelta {
                events: 1,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );

        tracker.reset();
        assert_eq!(
            tracker.observe(XrunObservation {
                counter: XrunCounter::PRIMARY,
                value: 250,
                unit: XrunUnit::Bytes,
                update: CounterUpdate::WrappingSnapshot { modulus: 256 },
                quality: ObservationQuality::Exact,
            }),
            XrunDelta {
                events: 1,
                lost_frames: None,
                lost_bytes: Some(250),
                quality: Some(ObservationQuality::Exact),
            }
        );

        // Independently reported units retain independent cumulative epochs.
        assert_eq!(
            tracker.observe(XrunObservation::cumulative_events(2)),
            XrunDelta {
                events: 2,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );
        assert_eq!(
            tracker.observe(XrunObservation {
                counter: XrunCounter::PRIMARY,
                value: 5,
                unit: XrunUnit::Bytes,
                update: CounterUpdate::WrappingSnapshot { modulus: 256 },
                quality: ObservationQuality::Exact,
            }),
            XrunDelta {
                events: 1,
                lost_frames: None,
                lost_bytes: Some(11),
                quality: Some(ObservationQuality::Exact),
            }
        );
        assert_eq!(
            tracker.observe(XrunObservation {
                counter: XrunCounter::PRIMARY,
                value: 64,
                unit: XrunUnit::Frames,
                update: CounterUpdate::Delta,
                quality: ObservationQuality::Estimated,
            }),
            XrunDelta {
                events: 1,
                lost_frames: Some(64),
                lost_bytes: None,
                quality: Some(ObservationQuality::Estimated),
            }
        );

        tracker.reset();
        assert_eq!(
            tracker.observe(XrunObservation::wrapping_events_u32(u32::MAX - 1)),
            XrunDelta {
                events: u32::MAX - 1,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );
        assert_eq!(
            tracker.observe(XrunObservation::wrapping_events_u32(2)),
            XrunDelta {
                events: 4,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );

        tracker.reset();
        let _ = tracker.observe(XrunObservation::wrapping_events_u32(u32::MAX - 1));
        assert_eq!(
            tracker.observe(XrunObservation::resetting_events(2)),
            XrunDelta {
                events: 4,
                quality: Some(ObservationQuality::Exact),
                ..XrunDelta::default()
            }
        );
    }

    #[test]
    fn interleaved_sample_widths_cover_raw_audio_storage() {
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_S8), Some(1));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_ULAW), Some(1));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_ALAW), Some(1));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_S16_LE), Some(2));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U16_LE), Some(2));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U16_BE), Some(2));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_S24_BE), Some(3));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U24_LE), Some(3));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U24_BE), Some(3));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_F32_LE), Some(4));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U32_LE), Some(4));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_U32_BE), Some(4));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_F64_BE), Some(8));
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_S16P), None);
        assert_eq!(bytes_per_sample(SPA_AUDIO_FORMAT_ENCODED), None);
    }

    #[test]
    fn silence_pattern_is_format_correct_and_frame_tagged() {
        let mut config = StreamConfig {
            format: libspa::param::audio::AudioFormat::U8,
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 2,
        };
        let mut bytes = [0; 5];
        let pattern = config.silence_pattern();
        pattern.fill(&mut bytes);
        assert_eq!(bytes, [0x80; 5]);
        assert_eq!(pattern.frame_multiple, 2);

        config.format = libspa::param::audio::AudioFormat::S16LE;
        config.silence_pattern().fill(&mut bytes);
        assert_eq!(bytes, [0; 5]);

        config.format = libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_U16_LE);
        config.stride = 4;
        let mut multibyte = [0xff; 8];
        config.silence_pattern().fill(&mut multibyte);
        assert_eq!(multibyte, [0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80]);
        let mut tail = [0xff; 7];
        config.silence_pattern().fill_at(1, &mut tail);
        assert_eq!(tail, [0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80]);

        config.format = libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_ULAW);
        config.stride = 1;
        assert_eq!(config.silence_pattern().uniform_byte(), Some(0xff));
        config.format = libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_ALAW);
        assert_eq!(config.silence_pattern().uniform_byte(), Some(0x55));
    }
}
