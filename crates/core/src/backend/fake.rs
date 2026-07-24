//! Deterministic in-memory backend used by shared lifecycle tests.

use std::collections::VecDeque;

use super::*;
use crate::spa::Log;

pub enum FakeBackend {}

const FAKE_PROPERTY: BackendPropertyDescriptor = BackendPropertyDescriptor {
    key: "fake.quantum",
    description: "Fake delivery quantum",
    maximum: 65_536,
};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FakeProperties {
    playback: bool,
    quantum: u32,
    default_quantum: u32,
}

impl FakeProperties {
    pub fn new(playback: bool) -> Self {
        <Self as BackendProperties>::new(playback)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FakePropertyUpdate {
    Quantum(u32),
}

impl BackendProperties for FakeProperties {
    type Update = FakePropertyUpdate;

    fn new(playback: bool) -> Self {
        Self {
            playback,
            quantum: 0,
            default_quantum: 0,
        }
    }

    fn descriptors(&self) -> &'static [BackendPropertyDescriptor] {
        let _ = self.playback;
        &[FAKE_PROPERTY]
    }

    fn values(&self) -> Vec<(&'static str, u32)> {
        vec![(FAKE_PROPERTY.key, self.quantum)]
    }

    fn decode_params(value: &libspa::pod::Value) -> Vec<Self::Update> {
        use libspa::pod::Value;
        let Value::Struct(values) = value else {
            return Vec::new();
        };
        values
            .chunks_exact(2)
            .filter_map(|pair| match (&pair[0], &pair[1]) {
                (Value::String(key), Value::Int(value))
                    if key == FAKE_PROPERTY.key && *value >= 0 =>
                {
                    Some(FakePropertyUpdate::Quantum(*value as u32))
                }
                _ => None,
            })
            .collect()
    }

    fn apply(&mut self, update: Self::Update) -> bool {
        match update {
            FakePropertyUpdate::Quantum(value) => {
                let value = value.min(FAKE_PROPERTY.maximum);
                let changed = self.quantum != value;
                self.quantum = value;
                changed
            }
        }
    }

    fn reset(&mut self) -> bool {
        let changed = self.quantum != self.default_quantum;
        self.quantum = self.default_quantum;
        changed
    }
}

#[derive(Default)]
pub struct FakeDeviceInit {
    parent: Option<String>,
    units: Vec<u32>,
}

impl DeviceInit for FakeDeviceInit {
    fn parse(&mut self, key: &str, value: &str) {
        match key {
            "fake.parent" => self.parent = Some(value.to_string()),
            "fake.units" => {
                self.units = value
                    .split(',')
                    .filter_map(|part| part.parse().ok())
                    .collect();
            }
            _ => {}
        }
    }

    fn parent_name(&self) -> Option<&str> {
        self.parent.as_deref()
    }

    fn is_complete(&self) -> bool {
        !self.units.is_empty()
    }

    fn snapshot(&self) -> Option<DeviceSnapshot> {
        Some(DeviceSnapshot {
            description: "Fake audio device".into(),
            endpoints: self
                .units
                .iter()
                .map(|unit| EndpointSnapshot {
                    key: EndpointKey::qualified("fake", &unit.to_string()),
                    object_id: *unit,
                    direction: StreamDirection::Playback,
                    name: format!("fake-{unit}"),
                    description: format!("Fake endpoint {unit}"),
                    locator: StreamLocator::new("fake", format!("fake://{unit}")),
                })
                .collect(),
        })
    }

    fn missing_selector_diagnostic() -> &'static str {
        "fake.units should contain device indexes"
    }

    fn snapshot_diagnostic() -> &'static str {
        "can't retrieve fake device information"
    }
}

pub struct FakeNodeInit {
    path: Option<String>,
    force_timer: bool,
    properties: FakeProperties,
}

impl NodeInit for FakeNodeInit {
    type Properties = FakeProperties;

    fn new(playback: bool) -> Self {
        Self {
            path: None,
            force_timer: false,
            properties: FakeProperties::new(playback),
        }
    }

    fn force_timer_key() -> Option<&'static str> {
        Some("fake.force-timer")
    }

    fn parse(&mut self, key: &str, value: &str) -> InitItemResult {
        match key {
            "fake.stream.path" => {
                self.path = Some(value.to_string());
                InitItemResult::Handled
            }
            "fake.force-timer"
                if ["1", "true", "yes", "on"]
                    .iter()
                    .any(|accepted| value.eq_ignore_ascii_case(accepted)) =>
            {
                self.force_timer = true;
                InitItemResult::Handled
            }
            "fake.force-timer"
                if ["0", "false", "no", "off"]
                    .iter()
                    .any(|accepted| value.eq_ignore_ascii_case(accepted)) =>
            {
                self.force_timer = false;
                InitItemResult::Handled
            }
            "fake.force-timer" => InitItemResult::InvalidBoolean,
            "fake.quantum" => {
                if let Ok(value) = value.parse() {
                    self.properties.apply(FakePropertyUpdate::Quantum(value));
                    self.properties.default_quantum = self.properties.quantum;
                }
                InitItemResult::Handled
            }
            _ => InitItemResult::Unknown,
        }
    }

    fn into_values(self) -> NodeInitValues<Self::Properties> {
        NodeInitValues {
            stream_path: self.path,
            force_timer: self.force_timer,
            properties: self.properties,
        }
    }
}

pub struct FakeStream {
    path: String,
    config: Option<StreamConfig>,
    running: bool,
    detached: bool,
    capacity: usize,
    maximum_io: usize,
    playback: VecDeque<u8>,
    capture: VecDeque<u8>,
    xruns: u32,
    fd: Option<std::ffi::c_int>,
    frame_stride: u32,
    test_period: u32,
    test_quantum: u32,
    retune_pending_responses: u32,
    overrun_recovery_after: u32,
    overrun_observations: u32,
    pause_outcome: PauseOutcome,
    frame_off: u32,
    read_skip: u32,
}

impl FakeStream {
    pub fn new(path: &str) -> Self {
        Self {
            path: path.to_string(),
            config: None,
            running: false,
            detached: false,
            capacity: 65_536,
            maximum_io: usize::MAX,
            playback: VecDeque::new(),
            capture: VecDeque::new(),
            xruns: 0,
            fd: None,
            frame_stride: 1,
            test_period: 0,
            test_quantum: 0,
            retune_pending_responses: 0,
            overrun_recovery_after: 0,
            overrun_observations: 0,
            pause_outcome: PauseOutcome::Preserved,
            frame_off: 0,
            read_skip: 0,
        }
    }

    #[cfg(test)]
    pub fn test_on_fd(fd: std::ffi::c_int, frame_stride: u32) -> Self {
        let mut stream = Self::new("fake://test-fd");
        stream.fd = Some(fd);
        stream.frame_stride = frame_stride.max(1);
        stream
    }

    #[cfg(test)]
    pub fn test_buffered(frame_stride: u32) -> Self {
        let mut stream = Self::new("fake://test-buffered");
        stream.frame_stride = frame_stride.max(1);
        stream
    }

    #[cfg(test)]
    pub fn test_take_playback(&mut self) -> Vec<u8> {
        self.playback.drain(..).collect()
    }

    #[cfg(test)]
    pub fn test_set_buffer_geometry(
        &mut self,
        period_bytes: u32,
        quantum_bytes: u32,
        capacity_bytes: u32,
    ) {
        self.test_period = period_bytes;
        self.test_quantum = quantum_bytes;
        self.capacity = capacity_bytes as usize;
    }

    /// Script how many `Pending` responses precede the next retune verdict.
    #[cfg(test)]
    pub fn test_hold_retunes(&mut self, responses: u32) {
        self.retune_pending_responses = responses;
    }

    /// Select recovery after this many consecutive fake observations. Zero
    /// leaves recovery disabled until a test explicitly selects a verdict.
    #[cfg(test)]
    pub fn test_recover_overrun_after(&mut self, observations: u32) {
        self.overrun_recovery_after = observations;
        self.overrun_observations = 0;
    }

    #[cfg(test)]
    pub fn test_overrun_observations(&self) -> u32 {
        self.overrun_observations
    }

    #[cfg(test)]
    pub fn test_set_pause_outcome(&mut self, outcome: PauseOutcome) {
        self.pause_outcome = outcome;
    }

    pub fn path(&self) -> &str {
        &self.path
    }

    pub fn configure(&mut self, config: &StreamConfig) -> ConfigureOutcome {
        self.frame_stride = config.stride.max(1);
        self.config = Some(config.clone());
        ConfigureOutcome {
            actual_config: config.clone(),
            applied_buffer: AppliedBufferGeometry {
                capacity_bytes: Some(self.capacity.min(u32::MAX as usize) as u32),
                quantum_bytes: None,
                delivery: DeliveryQuantum::unavailable(),
            },
            buffer_constraints: BufferConstraints::default(),
            adjusted: AdjustmentFlags::default(),
        }
    }

    pub fn set_maximum_io(&mut self, bytes: usize) {
        self.maximum_io = bytes;
    }

    pub fn set_capacity(&mut self, bytes: usize) {
        self.capacity = bytes;
    }

    pub fn push_capture(&mut self, bytes: &[u8]) {
        self.capture.extend(bytes.iter().copied());
    }

    fn silence_bytes_at(&self, frame_offset: usize, bytes: usize) -> Vec<u8> {
        let mut silence = vec![0; bytes];
        if let Some(config) = &self.config {
            config.silence_pattern().fill_at(frame_offset, &mut silence);
        }
        silence
    }

    fn silence_bytes(&self, bytes: usize) -> Vec<u8> {
        self.silence_bytes_at(0, bytes)
    }

    pub fn write(&mut self, bytes: &[u8]) -> WriteOutcome {
        if self.detached {
            return WriteOutcome {
                bytes: 0,
                status: IoStatus::Disconnected,
            };
        }
        self.running = true;
        if let Some(fd) = self.fd {
            let count = unsafe { libc::write(fd, bytes.as_ptr().cast(), bytes.len()) };
            return if count < 0 {
                WriteOutcome {
                    bytes: 0,
                    status: fake_io_status(),
                }
            } else {
                self.record_written(count as usize);
                WriteOutcome {
                    bytes: count as usize,
                    status: IoStatus::Progress,
                }
            };
        }
        let count = bytes
            .len()
            .min(self.maximum_io)
            .min(self.capacity.saturating_sub(self.playback.len()));
        self.playback.extend(bytes[..count].iter().copied());
        self.record_written(count);
        WriteOutcome {
            bytes: count,
            status: if count == 0 {
                IoStatus::WouldBlock
            } else {
                IoStatus::Progress
            },
        }
    }

    fn record_written(&mut self, bytes: usize) {
        let stride = u64::from(self.frame_stride);
        let frame_bytes = bytes as u64 % stride;
        self.frame_off = ((u64::from(self.frame_off) + frame_bytes) % stride) as u32;
    }

    pub fn read(&mut self, output: &mut [u8]) -> ReadOutcome {
        if self.detached {
            return ReadOutcome {
                bytes: 0,
                status: IoStatus::Disconnected,
            };
        }
        self.running = true;
        if let Some(fd) = self.fd {
            while self.read_skip != 0 {
                let mut scratch = [0u8; 64];
                let len = (self.read_skip as usize).min(scratch.len());
                let count = unsafe { libc::read(fd, scratch.as_mut_ptr().cast(), len) };
                if count < 0 {
                    return ReadOutcome {
                        bytes: 0,
                        status: fake_io_status(),
                    };
                }
                if count == 0 {
                    return ReadOutcome {
                        bytes: 0,
                        status: IoStatus::Disconnected,
                    };
                }
                self.read_skip -= count as u32;
            }
            let count = unsafe { libc::read(fd, output.as_mut_ptr().cast(), output.len()) };
            return if count < 0 {
                ReadOutcome {
                    bytes: 0,
                    status: fake_io_status(),
                }
            } else if count == 0 {
                ReadOutcome {
                    bytes: 0,
                    status: IoStatus::Disconnected,
                }
            } else {
                let remainder = count as u32 % self.frame_stride;
                if remainder != 0 {
                    self.read_skip = self.frame_stride - remainder;
                }
                ReadOutcome {
                    bytes: count as usize - remainder as usize,
                    status: IoStatus::Progress,
                }
            };
        }
        while self.read_skip != 0 {
            let count = (self.read_skip as usize).min(self.capture.len());
            drop(self.capture.drain(..count));
            self.read_skip -= count as u32;
            if self.read_skip != 0 {
                return ReadOutcome {
                    bytes: 0,
                    status: IoStatus::WouldBlock,
                };
            }
        }
        let count = output.len().min(self.maximum_io).min(self.capture.len());
        for byte in &mut output[..count] {
            *byte = self
                .capture
                .pop_front()
                .expect("count is bounded by the queue");
        }
        let remainder = count as u32 % self.frame_stride;
        if remainder != 0 {
            self.read_skip = self.frame_stride - remainder;
        }
        ReadOutcome {
            bytes: count - remainder as usize,
            status: if count == 0 {
                IoStatus::WouldBlock
            } else {
                IoStatus::Progress
            },
        }
    }

    pub fn inject_xruns(&mut self, count: u32) {
        self.xruns = self.xruns.saturating_add(count);
    }

    pub fn take_xruns(&mut self) -> u32 {
        std::mem::take(&mut self.xruns)
    }

    pub fn queued_playback_bytes(&self) -> u32 {
        self.playback.len().min(u32::MAX as usize) as u32
    }

    pub fn detach(&mut self) {
        self.detached = true;
        self.running = false;
    }

    pub fn is_closed(&self) -> bool {
        self.detached
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    pub fn suspend(&mut self) -> bool {
        if self.running && self.fd.is_some() {
            return false;
        }
        self.running = false;
        self.frame_off = 0;
        self.read_skip = 0;
        self.playback.clear();
        self.capture.clear();
        true
    }

    pub fn close(&mut self) {
        self.detach();
        self.frame_off = 0;
        self.read_skip = 0;
        if let Some(fd) = self.fd.take() {
            unsafe { libc::close(fd) };
        }
    }

    #[cfg(test)]
    pub fn write_silence(&mut self, bytes: u32) {
        <Self as PlaybackOperations>::write_silence(self, bytes);
    }
}

fn fake_io_status() -> IoStatus {
    let native_code = std::io::Error::last_os_error()
        .raw_os_error()
        .unwrap_or(libc::EIO);
    match native_code {
        libc::EAGAIN | libc::EINTR => IoStatus::WouldBlock,
        libc::EPIPE | libc::ENODEV | libc::ENXIO => IoStatus::Disconnected,
        _ => IoStatus::Fatal(StreamError::from_native_code(native_code)),
    }
}

fn fake_capture_geometry(
    request: CaptureBufferRequest,
    capacity_bytes: u32,
    quantum_bytes: u32,
) -> CaptureBufferGeometry {
    let target_fill_bytes = request.period_bytes.saturating_add(quantum_bytes / 2);
    let ring_peak = capacity_bytes.saturating_sub(quantum_bytes);
    let minimum_peak = target_fill_bytes
        .saturating_add(quantum_bytes)
        .min(ring_peak);
    let peak_fill_bytes = target_fill_bytes
        .saturating_add(quantum_bytes / 2)
        .saturating_add(request.period_bytes / 2)
        .min(ring_peak)
        .max(minimum_peak);
    let required_capacity_bytes = target_fill_bytes
        .saturating_add(quantum_bytes.saturating_mul(2))
        .max(request.period_bytes.saturating_mul(2));
    CaptureBufferGeometry {
        capacity_bytes,
        quantum_bytes,
        target_fill_bytes,
        peak_fill_bytes,
        required_capacity_bytes,
        device_lost: false,
    }
}

fn fake_playback_geometry(
    request: PlaybackBufferRequest,
    capacity_bytes: u32,
    quantum_bytes: u32,
    current_fill_bytes: u32,
) -> PlaybackBufferGeometry {
    let desired = (request.period_bytes / 8).saturating_mul(10);
    let write_max = request.period_bytes.max(request.maximum_write_bytes);
    let minimum_fill_bytes = request
        .period_bytes
        .saturating_add((request.period_bytes / 4).max(quantum_bytes));
    let required_capacity_bytes = request
        .period_bytes
        .saturating_mul(2)
        .saturating_add(desired)
        .max(
            minimum_fill_bytes
                .saturating_add(write_max)
                .saturating_add(quantum_bytes),
        );
    let ceiling = capacity_bytes
        .saturating_sub(write_max.saturating_add(quantum_bytes))
        .max(request.period_bytes);
    let target_goal_bytes = desired
        .max(minimum_fill_bytes)
        .min(ceiling)
        .max(request.period_bytes);
    let predicted = current_fill_bytes
        .saturating_add(request.write_bytes)
        .saturating_sub(request.period_bytes);
    PlaybackBufferGeometry {
        capacity_bytes,
        quantum_bytes,
        target_fill_bytes: target_goal_bytes.min(predicted),
        target_goal_bytes,
        minimum_fill_bytes,
        required_capacity_bytes,
        delay_capped: target_goal_bytes < desired.max(minimum_fill_bytes),
    }
}

impl Drop for FakeStream {
    fn drop(&mut self) {
        if let Some(fd) = self.fd.take() {
            unsafe { libc::close(fd) };
        }
    }
}

#[derive(Default)]
pub struct FakeWakeDriver {
    events: std::cell::RefCell<VecDeque<WakeEvent>>,
    registered: std::cell::RefCell<Vec<StreamIdentity>>,
}

impl FakeWakeDriver {
    pub fn push(&self, event: WakeEvent) {
        self.events.borrow_mut().push_back(event);
    }

    pub fn register_stream(&self, stream: StreamIdentity) {
        let mut registered = self.registered.borrow_mut();
        registered.retain(|entry| entry.token != stream.token);
        registered.push(stream);
    }
}

impl WakeDriver for FakeWakeDriver {
    fn notification_fd(&self) -> std::os::fd::RawFd {
        -1
    }

    fn unregister_stream(&mut self) -> Result<(), WakeError> {
        self.registered.get_mut().clear();
        Ok(())
    }

    fn arm_timer(&self, _delay_ns: u64) -> Result<(), WakeError> {
        Ok(())
    }

    fn next_event(&self) -> Result<Option<WakeEvent>, WakeError> {
        loop {
            let Some(event) = self.events.borrow_mut().pop_front() else {
                return Ok(None);
            };
            if !matches!(event, WakeEvent::Stream(wake) if !self.registered.borrow().contains(&wake.stream))
            {
                return Ok(Some(event));
            }
        }
    }
}

pub struct FakeCatalog {
    groups: Vec<CatalogGroupSnapshot>,
}

impl FakeCatalog {
    pub fn new(groups: Vec<CatalogGroupSnapshot>) -> Self {
        Self { groups }
    }

    pub fn detach(&mut self, key: &str) -> Option<CatalogChange> {
        let position = self
            .groups
            .iter()
            .position(|group| group.key.as_str() == key)?;
        let group = self.groups.remove(position);
        Some(CatalogChange::Removed {
            object_id: group.object_id,
            diagnostic: group.key.into_string(),
        })
    }
}

impl DeviceCatalog for FakeCatalog {
    fn open_error_context() -> &'static str {
        "can't open fake catalog"
    }

    fn refresh_error_context() -> &'static str {
        "can't refresh fake catalog"
    }

    fn scan() -> Result<Self, CatalogError> {
        Ok(Self { groups: Vec::new() })
    }

    fn snapshots(&self) -> Vec<CatalogGroupSnapshot> {
        self.groups.clone()
    }
}

#[derive(Default)]
pub struct FakeHotplug;

impl HotplugMonitor<FakeCatalog> for FakeHotplug {
    fn open() -> Result<Self, std::io::Error> {
        Ok(Self)
    }

    fn fd(&self) -> std::os::fd::RawFd {
        -1
    }

    fn read_catalog_rescan(&mut self, _catalog: &mut FakeCatalog) -> (bool, Option<CatalogRescan>) {
        (true, None)
    }
}

#[derive(Default)]
pub struct FakeRoutes;

impl RouteController<FakeHotplug> for FakeRoutes {
    fn probe(_snapshot: &DeviceSnapshot) -> (Self, Vec<RouteSnapshot>) {
        (Self, Vec::new())
    }

    fn refresh_all(&mut self, _routes: &mut [RouteSnapshot]) {}

    fn poll(&mut self, _routes: &mut [RouteSnapshot]) -> Vec<RouteChange> {
        Vec::new()
    }

    fn apply(
        &mut self,
        _routes: &mut [RouteSnapshot],
        _key: RouteKey,
        _update: RouteUpdate,
    ) -> RouteChange {
        RouteChange::default()
    }

    fn watch_policy(&self) -> RouteWatchPolicy {
        RouteWatchPolicy {
            poll_interval_ns: None,
            event_driven: false,
        }
    }

    fn read_hotplug(&self, _routes: &[RouteSnapshot], _monitor: &mut FakeHotplug) -> (bool, bool) {
        (true, false)
    }
}

impl StreamLifecycle for FakeStream {
    type WakeDriver = FakeWakeDriver;

    fn new(path: &str) -> Self {
        Self::new(path)
    }

    fn path(&self) -> &str {
        self.path()
    }

    fn is_closed(&self) -> bool {
        self.is_closed()
    }

    fn is_running(&self) -> bool {
        self.is_running()
    }

    fn register_wake(
        &self,
        driver: &mut Self::WakeDriver,
        stream: StreamIdentity,
        _buffer: WakeBufferState,
    ) -> Result<(), WakeError> {
        driver.register_stream(stream);
        Ok(())
    }

    fn wake_available() -> bool {
        true
    }

    fn new_wake_driver() -> Result<Self::WakeDriver, WakeError> {
        Ok(FakeWakeDriver::default())
    }

    fn wake_diagnostic(kind: WakeDiagnostic) -> &'static str {
        match kind {
            WakeDiagnostic::Selected => "using fake wake events",
            WakeDiagnostic::Create => "can't create the fake wake source",
            WakeDiagnostic::Read => "reading the fake wake event",
            WakeDiagnostic::Remove => "removing the fake wake stream",
            WakeDiagnostic::Register => "can't register fake wake events",
            WakeDiagnostic::Threshold => "can't set the fake wake threshold",
            WakeDiagnostic::Arm => "arming the fake wake timer",
        }
    }

    fn close(&mut self) {
        self.close();
    }

    fn suspend(&mut self) -> bool {
        self.suspend()
    }
}

impl CaptureOperations for FakeStream {
    type Properties = FakeProperties;

    fn configure(
        &mut self,
        config: &StreamConfig,
        _properties: &Self::Properties,
        _log: &Log,
    ) -> Result<ConfigureOutcome, std::ffi::c_int> {
        if self.path == "/nonexistent/dsp" {
            return Err(-libc::ENOENT);
        }
        Ok(self.configure(config))
    }

    fn prime_buffer(
        &mut self,
        request: CaptureBufferRequest,
        _properties: &Self::Properties,
        _scratch: &mut [u8],
        _log: &Log,
    ) -> CaptureBufferGeometry {
        self.running = true;
        CaptureBufferGeometry {
            capacity_bytes: self.capacity.min(u32::MAX as usize) as u32,
            quantum_bytes: request.period_bytes,
            target_fill_bytes: request.period_bytes,
            peak_fill_bytes: request.period_bytes.saturating_mul(2),
            required_capacity_bytes: request.period_bytes.saturating_mul(4),
            device_lost: self.detached,
        }
    }

    fn retune_buffer(
        &mut self,
        request: CaptureBufferRequest,
        primed: bool,
        _log: &Log,
    ) -> CaptureRetune {
        if !primed
            || self.test_period == 0
            || request.period_bytes == 0
            || request.period_bytes == self.test_period
        {
            return CaptureRetune::Unchanged;
        }
        if self.retune_pending_responses != 0 {
            self.retune_pending_responses -= 1;
            return CaptureRetune::Pending;
        }

        let geometry = fake_capture_geometry(
            request,
            self.capacity.min(u32::MAX as usize) as u32,
            self.test_quantum,
        );
        if geometry.capacity_bytes >= geometry.required_capacity_bytes {
            self.test_period = request.period_bytes;
            CaptureRetune::Applied(geometry)
        } else if self.suspend() {
            CaptureRetune::Reprime
        } else {
            CaptureRetune::Rebuild
        }
    }

    fn read(&mut self, data: &mut [u8]) -> ReadOutcome {
        self.read(data)
    }

    fn queued_bytes(&self) -> u32 {
        assert!(self.running, "capture queue sampled before prime");
        self.capture.len().min(u32::MAX as usize) as u32
    }

    fn overruns(&self) -> XrunObservation {
        assert!(self.running, "capture overruns sampled before prime");
        XrunObservation {
            counter: XrunCounter::PRIMARY,
            value: u64::from(self.xruns),
            unit: XrunUnit::Events,
            update: CounterUpdate::Snapshot,
            quality: ObservationQuality::Exact,
        }
    }

    fn recover_overrun(
        &mut self,
        _overrun_count: u32,
        _pre_read_fill: Option<u32>,
        _log: &Log,
    ) -> Option<bool> {
        if self.overrun_recovery_after == 0 {
            self.overrun_observations = 0;
            return None;
        }
        self.overrun_observations = self.overrun_observations.saturating_add(1);
        if self.overrun_observations < self.overrun_recovery_after {
            return None;
        }
        self.overrun_observations = 0;
        Some(self.suspend())
    }

    fn log_overrun_recovery(&self, _count: u32, _now_ns: u64, _suppressed: u32, _log: &Log) {}

    fn clear_overrun_observation(&mut self) {
        self.overrun_observations = 0;
    }
}

impl PlaybackOperations for FakeStream {
    type Properties = FakeProperties;

    fn configure(
        &mut self,
        config: &StreamConfig,
        _properties: &Self::Properties,
        _log: &Log,
    ) -> Result<ConfigureOutcome, std::ffi::c_int> {
        if self.path == "/nonexistent/dsp" {
            return Err(-libc::ENOENT);
        }
        Ok(self.configure(config))
    }

    fn prime_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        _properties: &Self::Properties,
        _log: &Log,
    ) -> PlaybackBufferGeometry {
        self.running = true;
        let capacity = self.capacity.min(u32::MAX as usize) as u32;
        PlaybackBufferGeometry {
            capacity_bytes: capacity,
            quantum_bytes: request.period_bytes,
            target_fill_bytes: request.period_bytes,
            target_goal_bytes: request.period_bytes,
            minimum_fill_bytes: request.period_bytes / 2,
            required_capacity_bytes: request.period_bytes.saturating_mul(4),
            delay_capped: false,
        }
    }

    fn retune_buffer(
        &mut self,
        request: PlaybackBufferRequest,
        current_fill_bytes: u32,
        _now_ns: u64,
        _log: &Log,
    ) -> PlaybackRetune {
        if !self.running
            || self.test_period == 0
            || request.period_bytes == 0
            || request.period_bytes == self.test_period
        {
            return PlaybackRetune::Unchanged;
        }
        if self.retune_pending_responses != 0 {
            self.retune_pending_responses -= 1;
            return PlaybackRetune::Pending;
        }

        let geometry = fake_playback_geometry(
            request,
            self.capacity.min(u32::MAX as usize) as u32,
            self.test_quantum,
            current_fill_bytes,
        );
        if geometry.capacity_bytes >= geometry.required_capacity_bytes {
            self.test_period = request.period_bytes;
            PlaybackRetune::Applied(geometry)
        } else if self.suspend() {
            PlaybackRetune::Reprime
        } else {
            PlaybackRetune::Rebuild
        }
    }

    fn write(&mut self, data: &[u8]) -> WriteOutcome {
        self.write(data)
    }

    fn write_silence(&mut self, bytes: u32) {
        if self.frame_off != 0 {
            let remaining = self.frame_stride - self.frame_off;
            let silence = self.silence_bytes_at(self.frame_off as usize, remaining as usize);
            let _ = self.write(&silence);
            if self.frame_off != 0 {
                return;
            }
        }
        let bytes = bytes / self.frame_stride * self.frame_stride;
        let silence = self.silence_bytes(bytes as usize);
        let _ = self.write(&silence);
    }

    fn end_buffer_sequence(&mut self) -> bool {
        if self.frame_off == 0 {
            return false;
        }
        let remaining = self.frame_stride - self.frame_off;
        let silence = self.silence_bytes_at(self.frame_off as usize, remaining as usize);
        let outcome = self.write(&silence);
        if outcome.bytes == silence.len() {
            false
        } else {
            self.suspend()
        }
    }

    fn queued_bytes(&self) -> u32 {
        assert!(self.running, "playback queue sampled before prime");
        self.queued_playback_bytes()
    }

    fn underruns(&self) -> XrunObservation {
        assert!(self.running, "playback underruns sampled before prime");
        XrunObservation {
            counter: XrunCounter::PRIMARY,
            value: u64::from(self.xruns),
            unit: XrunUnit::Events,
            update: CounterUpdate::Snapshot,
            quality: ObservationQuality::Exact,
        }
    }

    fn log_underrun_recovery(&self, _count: u32, _now_ns: u64, _suppressed: u32, _log: &Log) {}

    fn log_ignored_underruns(
        &self,
        _count: u32,
        _fill_bytes: u32,
        _recovery_threshold_bytes: u32,
        _log: &Log,
    ) {
    }

    fn pause(&mut self) -> Result<PauseOutcome, StreamError> {
        Ok(self.pause_outcome)
    }

    fn resume(&mut self) -> Result<(), StreamError> {
        self.running = true;
        Ok(())
    }

    fn underrun_low(
        _target_fill: u32,
        _delivery_quantum: u32,
        period_bytes: u32,
        _drained_bytes: u32,
    ) -> u32 {
        // Deliberately simple: shared tests exercise the node's reaction to a
        // selected threshold, not any concrete backend's threshold formula.
        period_bytes.max(1)
    }

    fn debug_log_priorities(_log: &Log) {}
}

fn fake_caps() -> StreamCaps {
    StreamCaps {
        configurations: vec![StreamConfiguration {
            formats: vec![
                libspa::sys::SPA_AUDIO_FORMAT_S16_LE,
                libspa::sys::SPA_AUDIO_FORMAT_ULAW,
                libspa::sys::SPA_AUDIO_FORMAT_ALAW,
                libspa::sys::SPA_AUDIO_FORMAT_S8,
                libspa::sys::SPA_AUDIO_FORMAT_U16_LE,
                libspa::sys::SPA_AUDIO_FORMAT_U16_BE,
                libspa::sys::SPA_AUDIO_FORMAT_U24_LE,
                libspa::sys::SPA_AUDIO_FORMAT_U24_BE,
                libspa::sys::SPA_AUDIO_FORMAT_U32_LE,
                libspa::sys::SPA_AUDIO_FORMAT_U32_BE,
            ],
            channels: ChannelSet::Range { min: 1, max: 8 },
            rates: RateSet::Range {
                min: 8_000,
                max: 192_000,
            },
            preferred_rate: Some(48_000),
            rate_tolerance: 0,
            conversion: ConversionPath::Kernel,
            flags: ConfigurationFlags::default(),
        }],
        preferred: 0,
    }
}

static mut DEVICE_TOPIC: libspa::sys::spa_log_topic = libspa::sys::spa_log_topic {
    version: libspa::sys::SPA_VERSION_LOG_TOPIC,
    topic: c"spa.fake.device".as_ptr(),
    level: libspa::sys::SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
static mut MONITOR_TOPIC: libspa::sys::spa_log_topic = libspa::sys::spa_log_topic {
    version: libspa::sys::SPA_VERSION_LOG_TOPIC,
    topic: c"spa.fake.monitor".as_ptr(),
    level: libspa::sys::SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
static mut SINK_TOPIC: libspa::sys::spa_log_topic = libspa::sys::spa_log_topic {
    version: libspa::sys::SPA_VERSION_LOG_TOPIC,
    topic: c"spa.fake.sink".as_ptr(),
    level: libspa::sys::SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};
static mut SOURCE_TOPIC: libspa::sys::spa_log_topic = libspa::sys::spa_log_topic {
    version: libspa::sys::SPA_VERSION_LOG_TOPIC,
    topic: c"spa.fake.source".as_ptr(),
    level: libspa::sys::SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};

impl Backend for FakeBackend {
    type Capture = FakeStream;
    type Playback = FakeStream;
    type Properties = FakeProperties;
    type DeviceInit = FakeDeviceInit;
    type NodeInit = FakeNodeInit;
    type Catalog = FakeCatalog;
    type Hotplug = FakeHotplug;
    type Routes = FakeRoutes;

    const DEVICE_API: &'static str = "fake";
    const DIAGNOSTIC_TAG: &'static str = "fake";
    const REBUILD_THREAD_PREFIX: &'static str = "spa-fake";
    const SOURCE_COMMAND_PREFIX: &'static str = "fake-source: ";
    const STREAM_PATH: &'static str = "fake.stream.path";
    const DEVICE_FACTORY_NAME: &'static std::ffi::CStr = c"fake.device";
    const SINK_FACTORY_NAME: &'static std::ffi::CStr = c"fake.sink";
    const SOURCE_FACTORY_NAME: &'static std::ffi::CStr = c"fake.source";

    fn clock_name(stream_path: &str) -> std::ffi::CString {
        std::ffi::CString::new(stream_path).expect("fake paths contain no NUL")
    }

    fn fallback_caps() -> StreamCaps {
        fake_caps()
    }

    fn hotplug_diagnostic(_kind: HotplugDiagnostic) -> &'static str {
        "fake hotplug event"
    }

    fn probe_caps(_path: &str, _playback: bool) -> Option<StreamCaps> {
        Some(fake_caps())
    }

    fn validate_config(caps: &StreamCaps, config: &StreamConfig) -> Result<(), ChannelMapError> {
        caps.admits(
            config.format.0,
            config.channels,
            (!config.positions.is_empty()).then_some(config.positions.as_slice()),
            config.rate,
        )
        .then_some(())
        .ok_or(ChannelMapError::Unsupported)
    }

    fn device_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut DEVICE_TOPIC).expect("static topic")
    }

    fn monitor_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut MONITOR_TOPIC).expect("static topic")
    }

    fn sink_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut SINK_TOPIC).expect("static topic")
    }

    fn source_log_topic() -> std::ptr::NonNull<libspa::sys::spa_log_topic> {
        std::ptr::NonNull::new(&raw mut SOURCE_TOPIC).expect("static topic")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::test_transport::{drain, fill_pipe, pattern, pipe_pair};

    #[test]
    fn lifecycle_is_deterministic_without_native_audio_calls() {
        let config = StreamConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 4,
        };
        let mut stream = FakeStream::new("fake://duplex");
        let outcome = stream.configure(&config);
        assert_eq!(outcome.actual_config, config);
        assert_eq!(outcome.applied_buffer.capacity_bytes, Some(65_536));

        stream.set_capacity(6);
        stream.set_maximum_io(3);
        assert_eq!(stream.write(&[1, 2, 3, 4]).bytes, 3);
        assert_eq!(stream.write(&[4, 5, 6, 7]).bytes, 3);
        assert!(stream.write(&[7]).would_block());

        stream.set_maximum_io(usize::MAX);
        stream.push_capture(&[9, 8, 7, 6]);
        let mut output = [0; 4];
        assert_eq!(stream.read(&mut output).bytes, 4);
        assert_eq!(output, [9, 8, 7, 6]);

        stream.inject_xruns(2);
        assert_eq!(stream.take_xruns(), 2);
        assert_eq!(stream.take_xruns(), 0);
        assert!(stream.suspend());
        assert!(!stream.is_running());

        stream.detach();
        assert!(stream.is_closed());
        assert_eq!(stream.write(&[1]).status, IoStatus::Disconnected);
        stream.write_silence(4);
        assert!(!stream.is_running());
        assert_eq!(stream.read(&mut output).status, IoStatus::Disconnected);
    }

    #[test]
    fn companded_formats_negotiate_with_format_correct_silence() {
        for (format, silence) in [
            (libspa::sys::SPA_AUDIO_FORMAT_ULAW, 0xff),
            (libspa::sys::SPA_AUDIO_FORMAT_ALAW, 0x55),
        ] {
            assert!(fake_caps().admits(format, 2, None, 8_000));
            let config = StreamConfig {
                format: libspa::param::audio::AudioFormat(format),
                rate: 8_000,
                channels: 2,
                positions: vec![],
                flags: 0,
                stride: 2,
            };
            let mut stream = FakeStream::new("fake://companded");
            assert_eq!(stream.configure(&config).actual_config, config);
            <FakeStream as PlaybackOperations>::write_silence(&mut stream, 8);
            assert_eq!(stream.test_take_playback(), vec![silence; 8]);
        }
    }

    #[test]
    fn additional_raw_formats_negotiate_with_format_correct_silence() {
        const FORMATS: &[(u32, u32, &[u8])] = &[
            (libspa::sys::SPA_AUDIO_FORMAT_S8, 1, &[0x00]),
            (libspa::sys::SPA_AUDIO_FORMAT_U16_LE, 2, &[0x00, 0x80]),
            (libspa::sys::SPA_AUDIO_FORMAT_U16_BE, 2, &[0x80, 0x00]),
            (libspa::sys::SPA_AUDIO_FORMAT_U24_LE, 3, &[0x00, 0x00, 0x80]),
            (libspa::sys::SPA_AUDIO_FORMAT_U24_BE, 3, &[0x80, 0x00, 0x00]),
            (
                libspa::sys::SPA_AUDIO_FORMAT_U32_LE,
                4,
                &[0x00, 0x00, 0x00, 0x80],
            ),
            (
                libspa::sys::SPA_AUDIO_FORMAT_U32_BE,
                4,
                &[0x80, 0x00, 0x00, 0x00],
            ),
        ];

        for &(format, sample_bytes, silence) in FORMATS {
            assert!(fake_caps().admits(format, 2, None, 48_000));
            let config = StreamConfig {
                format: libspa::param::audio::AudioFormat(format),
                rate: 48_000,
                channels: 2,
                positions: vec![],
                flags: 0,
                stride: sample_bytes * 2,
            };
            let mut stream = FakeStream::new("fake://raw-formats");
            assert_eq!(stream.configure(&config).actual_config, config);
            <FakeStream as PlaybackOperations>::write_silence(&mut stream, config.stride * 2);
            assert_eq!(
                stream.test_take_playback(),
                silence
                    .iter()
                    .copied()
                    .cycle()
                    .take((config.stride * 2) as usize)
                    .collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn partial_unsigned_frame_is_repaired_at_the_sample_offset() {
        let config = StreamConfig {
            format: libspa::param::audio::AudioFormat(libspa::sys::SPA_AUDIO_FORMAT_U16_LE),
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 4,
        };
        let mut stream = FakeStream::new("fake://partial-unsigned-frame");
        stream.configure(&config);
        stream.set_maximum_io(1);
        assert_eq!(stream.write(&[0x42, 0x43]).bytes, 1);
        stream.set_maximum_io(usize::MAX);
        assert!(!<FakeStream as PlaybackOperations>::end_buffer_sequence(
            &mut stream
        ));
        assert_eq!(stream.test_take_playback(), [0x42, 0x80, 0x00, 0x80]);
    }

    #[test]
    fn fd_capture_discards_a_torn_frame_tail_before_the_next_read() {
        let (read_fd, write_fd) = pipe_pair(true, true);
        let mut stream = FakeStream::test_on_fd(read_fd, 8);
        let sequence = pattern(2_056, 3);
        assert_eq!(
            unsafe { libc::write(write_fd, sequence.as_ptr().cast(), 2_046) },
            2_046
        );

        let mut output = [0u8; 4_096];
        let first = stream.read(&mut output);
        assert_eq!(first.bytes, 2_040);
        assert_eq!(&output[..first.bytes], &sequence[..2_040]);
        assert_eq!(stream.read_skip, 2);

        assert_eq!(
            unsafe { libc::write(write_fd, sequence.as_ptr().add(2_046).cast(), 10) },
            10
        );
        let second = stream.read(&mut output[..8]);
        assert_eq!(second.bytes, 8);
        assert_eq!(&output[..second.bytes], &sequence[2_048..]);
        assert_eq!(stream.read_skip, 0);
        stream.close();
        unsafe { libc::close(write_fd) };
    }

    #[test]
    fn buffered_capture_discards_a_torn_unsigned_frame_before_later_data() {
        let config = StreamConfig {
            format: libspa::param::audio::AudioFormat(libspa::sys::SPA_AUDIO_FORMAT_U24_LE),
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 6,
        };
        let mut stream = FakeStream::test_buffered(config.stride);
        stream.configure(&config);
        let sequence = pattern(18, 11);
        stream.push_capture(&sequence);
        stream.set_maximum_io(8);

        let mut output = [0u8; 12];
        let first = stream.read(&mut output);
        assert_eq!(first.bytes, 6);
        assert_eq!(&output[..first.bytes], &sequence[..6]);
        assert_eq!(stream.read_skip, 4);

        stream.set_maximum_io(usize::MAX);
        let second = stream.read(&mut output);
        assert_eq!(second.bytes, 6);
        assert_eq!(&output[..second.bytes], &sequence[12..]);
        assert_eq!(stream.read_skip, 0);
    }

    #[test]
    fn buffered_silence_obeys_io_limits_and_retains_u24_frame_phase() {
        let config = StreamConfig {
            format: libspa::param::audio::AudioFormat(libspa::sys::SPA_AUDIO_FORMAT_U24_LE),
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 6,
        };
        let mut stream = FakeStream::test_buffered(config.stride);
        stream.configure(&config);
        stream.set_maximum_io(5);
        <FakeStream as PlaybackOperations>::write_silence(&mut stream, 12);
        assert_eq!(stream.frame_off, 5);
        assert_eq!(stream.test_take_playback(), [0x00, 0x00, 0x80, 0x00, 0x00]);

        stream.set_maximum_io(usize::MAX);
        assert!(!<FakeStream as PlaybackOperations>::end_buffer_sequence(
            &mut stream
        ));
        assert_eq!(stream.frame_off, 0);
        assert_eq!(stream.test_take_playback(), [0x80]);
    }

    #[test]
    fn fd_playback_reports_backpressure_without_assuming_partial_write_size() {
        let (read_fd, write_fd) = pipe_pair(true, true);
        let mut stream = FakeStream::test_on_fd(write_fd, 8);
        assert!(fill_pipe(write_fd) >= 512);

        assert!(stream.write(&[0; 512]).would_block());
        drain(read_fd);

        let data = pattern(512, 7);
        let written = stream.write(&data);
        assert_eq!(written.bytes, data.len());
        assert_eq!(written.status, IoStatus::Progress);
        assert_eq!(drain(read_fd), data);

        stream.close();
        unsafe { libc::close(read_fd) };
    }

    #[test]
    fn clearing_overrun_recovery_keeps_the_observed_counter() {
        let mut stream = FakeStream::new("fake://overrun-observation");
        let _ = <FakeStream as CaptureOperations>::prime_buffer(
            &mut stream,
            CaptureBufferRequest {
                period_bytes: 1024,
                graph_rate: 48_000,
                stride: 4,
                device_rate: 48_000,
            },
            &FakeProperties::new(false),
            &mut [],
            &Log::test_null(),
        );
        stream.inject_xruns(4);
        stream.overrun_observations = 2;

        <FakeStream as CaptureOperations>::clear_overrun_observation(&mut stream);

        assert_eq!(stream.overrun_observations, 0);
        assert_eq!(
            <FakeStream as CaptureOperations>::overruns(&stream).value,
            4
        );
    }

    #[test]
    fn scripted_wakes_and_catalog_detach_are_ordered() {
        let driver = FakeWakeDriver::default();
        let stream = StreamIdentity::new(StreamToken::for_port(0), 3);
        driver.register_stream(stream);
        driver.push(WakeEvent::Stream(StreamWake {
            stream,
            timing: WakeTiming::Readiness,
            ready_bytes: Some(256),
            queue: None,
            clock: None,
            xruns: Some(XrunObservation {
                counter: XrunCounter::PRIMARY,
                value: 1,
                unit: XrunUnit::Events,
                update: CounterUpdate::Delta,
                quality: ObservationQuality::Exact,
            }),
            state: StreamWakeState::Active,
        }));
        driver.push(WakeEvent::Timer);
        assert!(matches!(
            driver.next_event(),
            Ok(Some(WakeEvent::Stream(_)))
        ));
        assert_eq!(driver.next_event().unwrap(), Some(WakeEvent::Timer));

        let second = StreamIdentity::new(StreamToken::for_port(1), 9);
        driver.register_stream(second);
        driver.push(WakeEvent::Stream(StreamWake {
            stream: second,
            timing: WakeTiming::Readiness,
            ready_bytes: Some(64),
            queue: Some(QueueObservation {
                fill_bytes: 4,
                quality: ObservationQuality::Exact,
            }),
            clock: None,
            xruns: None,
            state: StreamWakeState::Active,
        }));
        assert!(matches!(
            driver.next_event(),
            Ok(Some(WakeEvent::Stream(wake))) if wake.stream == second
        ));

        driver.push(WakeEvent::Stream(StreamWake {
            stream: StreamIdentity::new(StreamToken::for_port(0), 2),
            timing: WakeTiming::Readiness,
            ready_bytes: Some(512),
            queue: None,
            clock: None,
            xruns: None,
            state: StreamWakeState::Active,
        }));
        driver.push(WakeEvent::Timer);
        assert_eq!(driver.next_event().unwrap(), Some(WakeEvent::Timer));

        let mut catalog = FakeCatalog::new(vec![CatalogGroupSnapshot {
            key: DeviceKey::qualified("fake", "card"),
            object_id: 7,
            properties: vec![],
        }]);
        assert_eq!(
            catalog.detach("fake:card"),
            Some(CatalogChange::Removed {
                object_id: 7,
                diagnostic: "fake:card".into(),
            })
        );
        assert!(catalog.detach("fake:card").is_none());
    }
}
