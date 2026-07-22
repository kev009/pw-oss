//! Adapter that runs shared node/rebuild code against the in-memory backend.

use std::ffi::c_int;

use libspa::sys::*;

use super::*;
use crate::backend;
use crate::spa::{IoArea, Log, Loop, LoopSource, System};

pub(crate) enum FakeDir {}

#[derive(Default)]
pub(crate) struct FakePortExt {
    pending_buffer: Option<u32>,
    pending_offset: u32,
    target_fill: u32,
}

fn process_fake_port(port: &mut Port<FakeDir>, log: &Log) -> c_int {
    if port.config.is_none() || port.buffers.is_empty() || port.io.is_null() {
        return SPA_STATUS_OK as c_int;
    }
    if port.io.with_ref(|io| io.status) != Some(SPA_STATUS_HAVE_DATA as c_int) {
        return SPA_STATUS_OK as c_int;
    }
    let xruns = port.dsp.take_xruns();
    if xruns > 0 {
        // A deterministic nonzero stamp is enough for generic process() to
        // exercise its collect-then-notify path without a host clock.
        port.pending_xrun = Some(pending_xrun(
            1,
            backend::XrunDelta {
                events: xruns,
                quality: Some(backend::ObservationQuality::Exact),
                ..Default::default()
            },
            port.config.as_ref(),
        ));
    }

    let buffer_id = port.io.with_ref(|io| io.buffer_id).unwrap_or(u32::MAX);
    // SAFETY: tests install the same host-owned MemPtr fixture accepted by the
    // production port contract, and the returned view stays within this call.
    let Some(data) = (unsafe { valid_data_block(port, buffer_id, log) }) else {
        port.io.with(|io| io.status = SPA_STATUS_NEED_DATA as c_int);
        return SPA_STATUS_NEED_DATA as c_int;
    };
    let input = data.input_slice();
    if port.ext.pending_buffer != Some(buffer_id) || port.ext.pending_offset as usize > input.len()
    {
        port.ext.pending_buffer = Some(buffer_id);
        port.ext.pending_offset = 0;
    }
    let offset = port.ext.pending_offset as usize;
    let outcome = port.dsp.write(&input[offset..]);
    latch_rebuild_required(port, outcome.status);

    let accepted = offset.saturating_add(outcome.bytes).min(input.len());
    if accepted < input.len() && outcome.retryable_partial() {
        port.ext.pending_offset = accepted as u32;
        return SPA_STATUS_OK as c_int;
    }

    port.ext.pending_buffer = None;
    port.ext.pending_offset = 0;
    port.io.with(|io| io.status = SPA_STATUS_NEED_DATA as c_int);
    SPA_STATUS_NEED_DATA as c_int
}

impl Direction for FakeDir {
    const DIRECTION: spa_direction = SPA_DIRECTION_INPUT;
    const PLAYBACK: bool = true;
    const MEDIA_CLASS: &'static str = "Audio/Sink";
    const READY_STATUS: i32 = SPA_STATUS_NEED_DATA as i32;
    const CMD_WARN_PREFIX: &'static str = "fake: ";

    type Backend = backend::fake::FakeBackend;
    type Device = backend::fake::FakeStream;
    type DataExt = ();
    type PortExt = FakePortExt;

    fn log_topic() -> std::ptr::NonNull<spa_log_topic> {
        <Self::Backend as backend::Backend>::sink_log_topic()
    }

    fn data_ext(_properties: &backend::fake::FakeProperties) -> Self::DataExt {}

    fn sync_backend_properties(
        _ext: &mut Self::DataExt,
        _properties: &backend::fake::FakeProperties,
    ) {
    }

    fn build_node_param(_state: &mut MainState<Self>, _id: u32, _index: u32) -> ParamBuild {
        ParamBuild::Unknown
    }

    fn reset_props(_state: &mut MainState<Self>, _data: &DataControl<Self>) -> c_int {
        0
    }

    fn try_open_configure(
        stream: &mut Self::Device,
        config: &PortConfig,
        _properties: &backend::fake::FakeProperties,
        _log: &Log,
    ) -> Result<backend::ConfigureOutcome, c_int> {
        Ok(stream.configure(config))
    }

    fn on_device_swapped(_state: &mut DataState<Self>, _port_idx: usize) {}
    fn on_buffers_swapped(_state: &mut DataState<Self>, _port_idx: usize) {}
    fn on_start_loop(_state: &mut DataState<Self>) {}
    fn on_suspend_loop(_state: &mut DataState<Self>) {}
    fn on_role_flip(_state: &mut DataState<Self>) {}
    fn debug_cycle(_state: &DataState<Self>, _now: u64, _nsec: u64) {}
    fn servo_ready(_port: &Port<Self>) -> bool {
        true
    }
    fn servo_fill(port: &mut Port<Self>) -> u32 {
        port.dsp.queued_playback_bytes()
    }
    fn servo_hold(_port: &Port<Self>) -> bool {
        false
    }
    fn servo_err(port: &Port<Self>, fill: u32) -> f64 {
        fill as f64 - port.ext.target_fill as f64
    }
    fn wake_buffer_state(_port: &Port<Self>) -> backend::WakeBufferState {
        backend::WakeBufferState::default()
    }
    fn process_ports(state: &mut DataState<Self>) -> c_int {
        state
            .ports
            .iter_mut()
            .fold(SPA_STATUS_OK as c_int, |result, port| {
                result | process_fake_port(port, &state.log)
            })
    }
}

pub(crate) fn fake_port(path: &str, generation: u64) -> Port<FakeDir> {
    Port {
        config: None,
        buffers: vec![],
        io: IoArea::null(),
        rate_match: IoArea::null(),
        dsp: backend::fake::FakeStream::new(path),
        dll: Default::default(),
        setup_period: 0,
        bw_adapt: Default::default(),
        delivery_quantum_bytes: 0,
        rebuild_pending: false,
        generation,
        stream_token: backend::StreamToken::for_port(0),
        was_matching: false,
        warn_limit: RateLimit::new(),
        pending_xrun: None,
        stream_wake: None,
        rebuild_required: false,
        xrun_tracker: backend::XrunTracker::default(),
        ext: FakePortExt::default(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_state(port: Port<FakeDir>) -> DataState<FakeDir> {
        let events = NodeEvents::<FakeDir>::new();
        let shared = std::sync::Arc::new(NodeShared::<FakeDir>::new());
        let main_events = MainEventTarget::new(&events, shared.alive_token());
        DataState {
            data_loop: Loop::test_null(),
            data_system: System::test_null(),
            log: Log::test_null(),
            clock: IoArea::null(),
            position: IoArea::null(),
            clock_name: c"fake-clock".to_owned(),
            main_loop: None,
            stream_path: "fake://process-cycle".into(),
            timer_fd: None,
            wake_driver: None,
            wake_failed_stream: None,
            wake_source: LoopSource::new(
                spa_source {
                    loop_: std::ptr::null_mut(),
                    func: None,
                    data: std::ptr::null_mut(),
                    fd: -1,
                    mask: 0,
                    rmask: 0,
                    priv_: std::ptr::null_mut(),
                },
                <backend::fake::FakeBackend as backend::Backend>::DIAGNOSTIC_TAG,
            ),
            ready_dispatching: true,
            next_time: 0,
            callbacks: NodeCallbacks::none(),
            ports: [port],
            backend_properties: <backend::fake::FakeProperties as backend::BackendProperties>::new(
                false,
            ),
            shared,
            rebuild_work: std::sync::Arc::new(RebuildWorkSlot::new()),
            deferred_work: None,
            rebuild_takeover: false,
            format_publication: events.format_publication(),
            main_events,
            pending_main_event: None,
            started: true,
            following: false,
            ext: (),
        }
    }

    struct BufferFixture {
        payload: Vec<u8>,
        chunk: Box<spa_chunk>,
        data: Box<spa_data>,
        buffer: Box<spa_buffer>,
        io: spa_io_buffers,
    }

    impl BufferFixture {
        fn new(payload: Vec<u8>) -> Self {
            let mut chunk = Box::new(unsafe { std::mem::zeroed::<spa_chunk>() });
            chunk.size = payload.len() as u32;
            let mut fixture = Self {
                payload,
                chunk,
                data: Box::new(unsafe { std::mem::zeroed() }),
                buffer: Box::new(unsafe { std::mem::zeroed() }),
                io: spa_io_buffers {
                    status: SPA_STATUS_HAVE_DATA as c_int,
                    buffer_id: 0,
                },
            };
            fixture.data.type_ = SPA_DATA_MemPtr;
            fixture.data.data = fixture.payload.as_mut_ptr().cast();
            fixture.data.maxsize = fixture.payload.len() as u32;
            fixture.data.chunk = &mut *fixture.chunk;
            fixture.buffer.n_datas = 1;
            fixture.buffer.datas = &mut *fixture.data;
            fixture
        }

        fn install(&mut self, port: &mut Port<FakeDir>) {
            port.buffers = vec![&mut *self.buffer];
            // SAFETY: this fixture owns the aligned io area for the test and
            // outlives every process_fake_port call below.
            unsafe { port.io.set((&mut self.io as *mut spa_io_buffers).cast()) };
        }
    }

    #[test]
    fn shared_status_and_xrun_latches_accept_fake_stream_outcomes() {
        let mut port = fake_port("fake://node-lifecycle", 1);
        port.dsp.set_maximum_io(2);
        let partial = port.dsp.write(&[1, 2, 3, 4]);
        assert_eq!(partial.bytes, 2);
        assert_eq!(partial.status, backend::IoStatus::Progress);
        latch_rebuild_required(&mut port, partial.status);
        assert!(!port.rebuild_required);

        port.dsp.inject_xruns(3);
        let total = backend::XrunObservation::resetting_events(port.dsp.take_xruns());
        assert_eq!(take_polled_xruns(&mut port, total).events, 3);
        let next_total = backend::XrunObservation::resetting_events(port.dsp.take_xruns());
        assert_eq!(take_polled_xruns(&mut port, next_total).events, 0);

        port.dsp.detach();
        let detached = port.dsp.write(&[5]);
        latch_rebuild_required(&mut port, detached.status);
        assert!(port.rebuild_required);
    }

    #[test]
    fn fake_process_retains_partial_input_and_latches_detach() {
        let mut port = fake_port("fake://process", 2);
        let config = PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 4,
        };
        let outcome = port.dsp.configure(&config);
        port.config = Some(outcome.actual_config);
        port.dsp.set_maximum_io(3);
        port.dsp.inject_xruns(1);
        port.ext.target_fill = 8;

        let mut fixture = BufferFixture::new(vec![1, 2, 3, 4, 5]);
        fixture.install(&mut port);
        assert_eq!(process_fake_port(&mut port, &Log::test_null()), 0);
        assert_eq!(port.ext.pending_offset, 3);
        assert_eq!(fixture.io.status, SPA_STATUS_HAVE_DATA as c_int);
        assert_eq!(port.pending_xrun.map(|report| report.trigger_us), Some(1));
        assert_eq!(FakeDir::servo_fill(&mut port), 3);
        assert_eq!(FakeDir::servo_err(&port, 3), -5.0);

        assert_eq!(
            process_fake_port(&mut port, &Log::test_null()),
            SPA_STATUS_NEED_DATA as c_int
        );
        assert_eq!(fixture.io.status, SPA_STATUS_NEED_DATA as c_int);
        assert_eq!(port.dsp.queued_playback_bytes(), 5);

        fixture.io.status = SPA_STATUS_HAVE_DATA as c_int;
        port.dsp.detach();
        assert_eq!(
            process_fake_port(&mut port, &Log::test_null()),
            SPA_STATUS_NEED_DATA as c_int
        );
        assert!(port.rebuild_required);
    }

    #[test]
    fn shared_process_cycle_handles_fake_partial_xrun_and_detach_lifecycle() {
        let mut port = fake_port("fake://shared-process", 3);
        let config = PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 4,
        };
        let outcome = port.dsp.configure(&config);
        port.config = Some(outcome.actual_config);
        port.dsp.set_maximum_io(3);
        port.dsp.inject_xruns(1);
        port.stream_wake = Some(backend::StreamWake {
            stream: port.stream_identity(),
            timing: backend::WakeTiming::Readiness,
            ready_bytes: Some(0),
            queue: Some(backend::QueueObservation {
                fill_bytes: 0,
                quality: backend::ObservationQuality::Exact,
            }),
            clock: None,
            xruns: Some(backend::XrunObservation::cumulative_events(1)),
            state: backend::StreamWakeState::Active,
        });

        let mut fixture = BufferFixture::new(vec![1, 2, 3, 4, 5]);
        fixture.install(&mut port);
        let mut state = test_state(port);
        let mut position = unsafe { std::mem::zeroed::<spa_io_position>() };
        // SAFETY: position is aligned and outlives every process_data_cycle
        // call in this test.
        unsafe {
            state
                .position
                .set((&mut position as *mut spa_io_position).cast());
        };

        let (result, xrun, main_event) =
            super::super::process::process_data_cycle(&mut state).expect("started fake cycle");
        assert_eq!(result, SPA_STATUS_OK as c_int);
        assert!(matches!(
            xrun,
            Some((report, None)) if report.trigger_us == 1
                && report.quality == Some(backend::ObservationQuality::Exact)
        ));
        assert!(main_event.is_none());
        assert_eq!(state.ports[0].ext.pending_offset, 3);
        assert!(state.ports[0].stream_wake.is_none());

        let (result, xrun, main_event) = super::super::process::process_data_cycle(&mut state)
            .expect("retained-tail fake cycle");
        assert_eq!(result, SPA_STATUS_NEED_DATA as c_int);
        assert!(xrun.is_none());
        assert!(main_event.is_none());
        assert_eq!(state.ports[0].dsp.queued_playback_bytes(), 5);

        fixture.io.status = SPA_STATUS_HAVE_DATA as c_int;
        state.ports[0].dsp.detach();
        let (result, _, _) =
            super::super::process::process_data_cycle(&mut state).expect("detached fake cycle");
        assert_eq!(result, SPA_STATUS_NEED_DATA as c_int);
        assert!(state.ports[0].rebuild_required);
        assert!(state.ports[0].rebuild_pending);
    }

    #[test]
    fn shared_wake_cycle_publishes_the_fake_backend_timestamp() {
        let mut port = fake_port("fake://clock", 5);
        let config = PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48_000,
            channels: 2,
            positions: vec![],
            flags: 0,
            stride: 4,
        };
        let outcome = port.dsp.configure(&config);
        port.config = Some(outcome.actual_config);
        port.setup_period = 1_920;
        assert_eq!(port.dsp.write(&[0; 4]).status, backend::IoStatus::Progress);

        let identity = port.stream_identity();
        let driver = backend::fake::FakeWakeDriver::default();
        driver.register_stream(identity);
        driver.push(backend::WakeEvent::Stream(backend::StreamWake {
            stream: identity,
            timing: backend::WakeTiming::ObservedTime,
            ready_bytes: None,
            queue: Some(backend::QueueObservation {
                fill_bytes: 4,
                quality: backend::ObservationQuality::Exact,
            }),
            clock: Some(backend::ClockObservation {
                position: Some(backend::PositionObservation {
                    frames: 1,
                    scope: backend::ClockScope::Stream,
                    quality: backend::ObservationQuality::Exact,
                }),
                timestamp: Some(backend::TimestampObservation {
                    monotonic_ns: 1_999_000,
                    accuracy_ns: Some(1_000),
                    quality: backend::ObservationQuality::Exact,
                }),
            }),
            xruns: None,
            state: backend::StreamWakeState::Active,
        }));

        let mut state = test_state(port);
        state.data_system = System::test_clock(2_000_000);
        state.next_time = 2_000_000;
        state.wake_driver = Some(driver);

        let mut position = unsafe { std::mem::zeroed::<spa_io_position>() };
        position.clock.target_duration = 480;
        position.clock.target_rate = spa_fraction {
            num: 1,
            denom: 48_000,
        };
        let mut clock = unsafe { std::mem::zeroed::<spa_io_clock>() };
        clock.target_rate = position.clock.target_rate;
        unsafe {
            state
                .position
                .set((&mut position as *mut spa_io_position).cast());
            state.clock.set((&mut clock as *mut spa_io_clock).cast());
        }

        assert!(matches!(
            super::super::timing::wake_cycle(&mut state),
            Some(None)
        ));
        assert_eq!(clock.nsec, 1_999_000);
        assert_eq!(clock.duration, 480);
        assert!(state.next_time > 2_000_000);
    }
}
