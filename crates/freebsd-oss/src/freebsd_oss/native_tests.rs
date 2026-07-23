//! Kernel-backed integration coverage. These tests are ignored by default and
//! run only in the FreeBSD CI VM after loading snd_dummy(4).

mod spa_host;

use std::ffi::c_int;
use std::process::Command;
use std::time::{Duration, Instant};

use nix::errno::Errno;

use super::backend::{channel_positions, configure_capture, configure_playback};
use super::devices::{
    DeviceCatalog, device_snapshot, open_failure_diagnostic, snd_dummy_unit, sndstat_dsp_info,
};
use super::identity::DEVICE_INDEXES;
use super::{
    Dsp, DspWriter, HotplugMonitor, OssNodeProperties, OssWakeDriver, RouteController,
    enriched_sound_kqueue_available, probe_caps,
};
use crate::backend::{
    CaptureBufferRequest, CatalogChange, ConversionPath, CounterUpdate, IoStatus,
    ObservationQuality, PlaybackBufferRequest, PlaybackRetune, RateSet, StreamCaps, StreamConfig,
    StreamIdentity, StreamToken, StreamWakeState, WakeBufferState, WakeDriver, WakeEvent, XrunUnit,
};
use crate::spa::Log;
use libspa::sys::{
    SPA_AUDIO_FORMAT_ALAW, SPA_AUDIO_FORMAT_F32_LE, SPA_AUDIO_FORMAT_S8, SPA_AUDIO_FORMAT_S16_LE,
    SPA_AUDIO_FORMAT_S24_LE, SPA_AUDIO_FORMAT_S32_LE, SPA_AUDIO_FORMAT_U16_BE,
    SPA_AUDIO_FORMAT_U16_LE, SPA_AUDIO_FORMAT_U24_BE, SPA_AUDIO_FORMAT_U24_LE,
    SPA_AUDIO_FORMAT_U32_BE, SPA_AUDIO_FORMAT_U32_LE, SPA_AUDIO_FORMAT_ULAW,
};

fn config(format: u32, rate: u32, stride: u32) -> StreamConfig {
    StreamConfig {
        format: libspa::param::audio::AudioFormat(format),
        rate,
        channels: 2,
        positions: channel_positions(2)
            .expect("stereo has a native layout")
            .to_vec(),
        flags: 0,
        stride,
    }
}

fn default_config() -> StreamConfig {
    config(SPA_AUDIO_FORMAT_S16_LE, 48_000, 4)
}

fn contains_only_silence(config: &StreamConfig, bytes: &[u8]) -> bool {
    match config.format.0 {
        // G.711 has positive/negative zero codes, and FreeBSD's table-based
        // vchan conversion can emit the adjacent +/-8 code for signed zero.
        SPA_AUDIO_FORMAT_ULAW => bytes
            .iter()
            .all(|byte| matches!(byte, 0x7e | 0x7f | 0xfe | 0xff)),
        SPA_AUDIO_FORMAT_ALAW => bytes.iter().all(|byte| matches!(byte, 0x55 | 0xd5)),
        _ => {
            let mut expected = vec![0; bytes.len()];
            config.silence_pattern().fill(&mut expected);
            bytes == expected
        }
    }
}

fn wait_until<T>(timeout: Duration, mut observe: impl FnMut() -> Option<T>) -> Option<T> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(value) = observe() {
            return Some(value);
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn assert_dummy_available() -> (u32, String) {
    assert!(
        std::path::Path::new("/dev/dsp.dummy").exists(),
        "snd_dummy must create /dev/dsp.dummy"
    );
    let unit = snd_dummy_unit().expect("snd_dummy must appear in sndstat and dev.pcm sysctls");
    (unit, format!("/dev/dsp{unit}"))
}

fn command_succeeds(program: &str, args: &[&str]) {
    let status = Command::new(program)
        .args(args)
        .status()
        .unwrap_or_else(|error| panic!("failed to execute {program}: {error}"));
    assert!(status.success(), "{program} {args:?} failed: {status}");
}

fn catalog_contains_unit(catalog: &DeviceCatalog, unit: u32) -> bool {
    let unit = unit.to_string();
    catalog.snapshots().iter().any(|snapshot| {
        snapshot.properties.iter().any(|(key, value)| {
            key == DEVICE_INDEXES && value.split(',').any(|candidate| candidate == unit)
        })
    })
}

fn poll_catalog(
    monitor: &mut HotplugMonitor,
    catalog: &mut DeviceCatalog,
    timeout: Duration,
    mut accept: impl FnMut(&[CatalogChange]) -> bool,
) {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        assert!(
            !remaining.is_zero(),
            "timed out waiting for a devd catalog change"
        );
        let mut pollfd = libc::pollfd {
            fd: monitor.fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let timeout_ms = remaining.as_millis().min(250) as c_int;
        let ready = unsafe { libc::poll(&mut pollfd, 1, timeout_ms) };
        assert!(ready >= 0, "polling the hotplug kqueue failed");
        if ready == 0 {
            continue;
        }
        let (alive, rescan) = monitor.read_catalog_rescan(catalog);
        assert!(alive, "the stable hotplug kqueue died");
        if rescan.is_some_and(|rescan| {
            assert!(rescan.error.is_none(), "catalog refresh failed: {rescan:?}");
            accept(&rescan.changes)
        }) {
            return;
        }
    }
}

struct DummyModuleGuard;

impl Drop for DummyModuleGuard {
    fn drop(&mut self) {
        if !std::path::Path::new("/dev/dsp.dummy").exists() {
            let _ = Command::new("/sbin/kldload").arg("snd_dummy").status();
        }
    }
}

fn read_sysctl(key: &str) -> String {
    let output = Command::new("/sbin/sysctl")
        .args(["-n", key])
        .output()
        .unwrap_or_else(|error| panic!("failed to read {key}: {error}"));
    assert!(
        output.status.success(),
        "reading {key} failed: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );
    String::from_utf8(output.stdout)
        .expect("numeric sysctl output must be UTF-8")
        .trim()
        .to_string()
}

fn write_sysctl(key: &str, value: &str) -> Result<(), String> {
    let assignment = format!("{key}={value}");
    let output = Command::new("/sbin/sysctl")
        .arg(assignment)
        .output()
        .map_err(|error| format!("executing sysctl: {error}"))?;
    if output.status.success() {
        Ok(())
    } else {
        Err(String::from_utf8_lossy(&output.stderr).trim().to_string())
    }
}

struct PcmModeGuard {
    original: Vec<(String, String)>,
}

impl PcmModeGuard {
    fn new(unit: u32) -> Self {
        let original = [
            format!("dev.pcm.{unit}.play.vchans"),
            format!("dev.pcm.{unit}.rec.vchans"),
            format!("dev.pcm.{unit}.bitperfect"),
        ]
        .into_iter()
        .map(|key| {
            let value = read_sysctl(&key);
            (key, value)
        })
        .collect();
        Self { original }
    }

    fn set(&self, key_suffix: &str, value: &str) {
        let (key, _) = self
            .original
            .iter()
            .find(|(key, _)| key.ends_with(key_suffix))
            .unwrap_or_else(|| panic!("missing saved sysctl ending in {key_suffix}"));
        write_sysctl(key, value)
            .unwrap_or_else(|error| panic!("setting {key}={value} failed: {error}"));
    }

    fn enable_direct_bitperfect(&self) {
        self.set("play.vchans", "0");
        self.set("rec.vchans", "0");
        self.set("bitperfect", "1");
    }
}

impl Drop for PcmModeGuard {
    fn drop(&mut self) {
        // Restore bitperfect before rebuilding the original vchan topology.
        for (key, value) in self.original.iter().rev() {
            if let Err(error) = write_sysctl(key, value) {
                eprintln!("restoring {key}={value} failed: {error}");
            }
        }
    }
}

fn assert_bitperfect_caps(caps: &StreamCaps) {
    let positions = channel_positions(2).expect("stereo has a native layout");
    let configuration = caps
        .preferred_configuration()
        .expect("bitperfect snd_dummy must have a preferred configuration");
    assert_eq!(configuration.conversion, ConversionPath::None);
    let RateSet::Discrete(rates) = &configuration.rates else {
        panic!("an exclusive endpoint must not advertise a dense rate range")
    };
    assert!(rates.contains(&8_000));
    assert!(rates.contains(&96_000));
    for format in [
        SPA_AUDIO_FORMAT_S16_LE,
        SPA_AUDIO_FORMAT_S24_LE,
        SPA_AUDIO_FORMAT_S32_LE,
    ] {
        assert!(caps.admits(format, 2, Some(positions), 8_000));
        assert!(caps.admits(format, 2, Some(positions), 96_000));
        assert_eq!(
            caps.conversion_for(format, 2, 8_000),
            Some(ConversionPath::None)
        );
    }
    assert!(!caps.admits(SPA_AUDIO_FORMAT_F32_LE, 2, Some(positions), 8_000));
}

fn wait_for_event(driver: &OssWakeDriver, timeout: Duration) -> WakeEvent {
    let mut pollfd = libc::pollfd {
        fd: driver.notification_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    let timeout_ms = timeout.as_millis().min(c_int::MAX as u128) as c_int;
    let ready = unsafe { libc::poll(&mut pollfd, 1, timeout_ms) };
    assert_eq!(ready, 1, "timed out waiting for the OSS wake driver");
    assert_ne!(pollfd.revents & libc::POLLIN, 0);
    driver
        .next_event()
        .expect("the native wake event must decode")
        .expect("the ready kqueue must contain an event")
}

fn assert_expected_wake_mode() -> bool {
    let enriched = enriched_sound_kqueue_available();
    if let Ok(expected) = std::env::var("PW_OSS_EXPECT_ENRICHED_KQUEUE") {
        let expected = match expected.as_str() {
            "0" => false,
            "1" => true,
            _ => panic!("PW_OSS_EXPECT_ENRICHED_KQUEUE must be 0 or 1"),
        };
        assert_eq!(
            enriched, expected,
            "the selected FreeBSD VM does not exercise its intended wake path"
        );
    }
    enriched
}

#[test]
#[ignore = "requires the FreeBSD snd_dummy kernel module"]
fn native_snd_dummy_kernel_pcm_lifecycle() {
    let (unit, path) = assert_dummy_available();
    let config = default_config();
    let log = Log::test_null();

    let snapshot = device_snapshot(&[unit]).expect("snd_dummy must appear in sndstat");
    assert_eq!(snapshot.endpoints.len(), 2);
    assert!(snapshot.endpoints.iter().any(|endpoint| endpoint.direction
        == crate::backend::StreamDirection::Playback
        && endpoint.locator.value == path));
    assert!(snapshot.endpoints.iter().any(|endpoint| endpoint.direction
        == crate::backend::StreamDirection::Capture
        && endpoint.locator.value == path));

    // Route probing is read-only. Do not change even the dummy mixer's volume:
    // this test also documents the no-live-mixer-write CI contract.
    let (_controller, routes) = RouteController::probe(&snapshot);
    assert!(
        routes
            .iter()
            .any(|route| route.direction == crate::backend::StreamDirection::Playback)
    );
    assert!(
        routes
            .iter()
            .any(|route| route.direction == crate::backend::StreamDirection::Capture)
    );

    for playback in [false, true] {
        let caps = probe_caps(&path, playback).expect("snd_dummy capabilities must probe");
        assert!(caps.admits(
            SPA_AUDIO_FORMAT_S16_LE,
            2,
            Some(channel_positions(2).expect("stereo layout")),
            48_000,
        ));
    }

    let mut playback = DspWriter::new(&path);
    let applied = configure_playback(&mut playback, &config, &log)
        .expect("snd_dummy playback must configure");
    assert_eq!(applied.actual_config, config);
    let playback_request = PlaybackBufferRequest {
        period_bytes: 4_096,
        graph_rate: 48_000,
        stride: 4,
        device_rate: applied.actual_config.rate,
        write_bytes: 4_096,
        maximum_write_bytes: 4_096,
    };
    let playback_geometry =
        playback.prime_buffer(playback_request, &OssNodeProperties::new(true), &log);
    assert!(playback.is_running());
    assert!(playback_geometry.capacity_bytes >= playback_geometry.quantum_bytes);
    assert!(playback_geometry.quantum_bytes > 0);

    let busy = open_failure_diagnostic(&path, true, Errno::EBUSY)
        .expect("a busy playback channel must have a diagnostic");
    assert!(
        busy.contains(&format!("pid {}", std::process::id())),
        "ENGINEINFO did not identify the test process: {busy}"
    );

    let enriched = assert_expected_wake_mode();
    let mut wake = OssWakeDriver::new().expect("kqueue creation must succeed");
    if enriched {
        // Start with queued fill above the target, which puts free write space
        // below the EVFILT_WRITE low-water threshold, then let snd_dummy drain
        // across it after registration. This proves a real sound-buffer
        // transition wins the race rather than relying on EV_ADD to publish an
        // already-ready level before the watchdog.
        playback.write_silence(playback_geometry.capacity_bytes);
        let seeded_fill = playback.queued_bytes();
        assert!(
            seeded_fill > playback_geometry.target_fill_bytes,
            "could not seed an enriched-wake transition: queued {seeded_fill}, target {}",
            playback_geometry.target_fill_bytes
        );
        let stream = StreamIdentity::new(StreamToken::for_port(0), 1);
        playback
            .register_wake(
                &mut wake,
                stream,
                WakeBufferState {
                    frame_stride: config.stride,
                    period_bytes: playback_request.period_bytes,
                    quantum_bytes: playback_geometry.quantum_bytes,
                    capacity_bytes: playback_geometry.capacity_bytes,
                    target_fill_bytes: playback_geometry.target_fill_bytes,
                },
            )
            .expect("snd_dummy must register an enriched sound knote");
        wake.arm_timer(2_000_000_000)
            .expect("the wake watchdog must arm");
        let WakeEvent::Stream(event) = wait_for_event(&wake, Duration::from_secs(5)) else {
            panic!("the watchdog fired before an enriched sound event")
        };
        assert_eq!(event.stream, stream);
        assert_eq!(event.state, StreamWakeState::Active);
        assert_eq!(
            event
                .queue
                .expect("enriched wakes carry queue fill")
                .quality,
            ObservationQuality::Exact
        );
        wake.unregister_stream()
            .expect("the native stream knote must unregister");
        wake.arm_timer(0).expect("the watchdog must disarm");
    } else {
        wake.arm_timer(1_000_000)
            .expect("the timer fallback must arm");
        assert_eq!(
            wait_for_event(&wake, Duration::from_secs(5)),
            WakeEvent::Timer
        );
    }

    playback.pause().expect("dummy playback must pause");
    playback.resume().expect("dummy playback must resume");
    assert!(playback.suspend());
    assert!(!playback.is_running());
    let reprime = playback.prime_buffer(playback_request, &OssNodeProperties::new(true), &log);
    assert!(reprime.capacity_bytes > 0);
    assert!(playback.is_running());
    playback.close();

    let mut capture = Dsp::new(&path);
    let applied = configure_capture(&mut capture, &config, &OssNodeProperties::new(false), &log)
        .expect("snd_dummy capture must configure");
    assert_eq!(applied.actual_config, config);
    let capture_request = CaptureBufferRequest {
        period_bytes: 4_096,
        graph_rate: 48_000,
        stride: 4,
        device_rate: applied.actual_config.rate,
    };
    let mut scratch = [0u8; 65_536];
    let capture_geometry = capture.prime_buffer(
        capture_request,
        &OssNodeProperties::new(false),
        &mut scratch,
        &log,
    );
    assert!(!capture_geometry.device_lost);
    assert!(capture_geometry.capacity_bytes >= capture_geometry.quantum_bytes);
    assert!(capture_geometry.quantum_bytes > 0);
    assert!(capture.ready_for_reading(2_000));

    let mut input = [0xa5; 4_096];
    let read = capture.read(&mut input);
    assert!(read.bytes > 0);
    assert_eq!(read.bytes % config.stride as usize, 0);
    assert_eq!(read.status, IoStatus::Progress);
    assert!(input[..read.bytes].iter().all(|byte| *byte == 0));
    assert!(capture.suspend());
    assert!(!capture.is_running());
    let reprime = capture.prime_buffer(
        capture_request,
        &OssNodeProperties::new(false),
        &mut scratch,
        &log,
    );
    assert!(!reprime.device_lost);
    assert!(capture.is_running());
    capture.close();
}

#[test]
#[ignore = "requires root and the FreeBSD snd_dummy kernel module"]
fn native_snd_dummy_bitperfect_policy_and_exclusive_open() {
    let (unit, path) = assert_dummy_available();
    let mode = PcmModeGuard::new(unit);
    let original_sysctls = mode.original.clone();
    mode.enable_direct_bitperfect();

    let devnode = path.trim_start_matches("/dev/");
    wait_until(Duration::from_secs(2), || {
        let playback = sndstat_dsp_info(devnode, true)?;
        let capture = sndstat_dsp_info(devnode, false)?;
        (playback.bitperfect
            && capture.bitperfect
            && playback.exclusive == Some(true)
            && capture.exclusive == Some(true))
        .then_some(())
    })
    .expect("sndstat did not publish direct bitperfect state");

    let playback_caps = probe_caps(&path, true).expect("bitperfect playback caps must probe");
    let capture_caps = probe_caps(&path, false).expect("bitperfect capture caps must probe");
    assert_bitperfect_caps(&playback_caps);
    assert_bitperfect_caps(&capture_caps);

    let log = Log::test_null();
    let playback_config = config(SPA_AUDIO_FORMAT_S24_LE, 8_000, 6);
    let mut playback = DspWriter::new(&path);
    let applied = configure_playback(&mut playback, &playback_config, &log)
        .expect("direct bitperfect playback must configure");
    assert_eq!(applied.actual_config, playback_config);
    assert!(!playback.is_virtual_channel());

    // Capability discovery for an exclusive device must use sndstat and the
    // mixer control descriptor, not attempt another open of the claimed PCM.
    assert_eq!(
        probe_caps(&path, true).expect("busy bitperfect playback caps must still probe"),
        playback_caps
    );
    let mut playback_contender = DspWriter::new(&path);
    assert_eq!(playback_contender.open(), Err(Errno::EBUSY));
    let diagnostic = open_failure_diagnostic(&path, true, Errno::EBUSY)
        .expect("exclusive playback must produce a busy diagnostic");
    assert!(diagnostic.contains("playback channel is busy"));

    let capture_config = config(SPA_AUDIO_FORMAT_S32_LE, 96_000, 8);
    let mut capture = Dsp::new(&path);
    let applied = configure_capture(
        &mut capture,
        &capture_config,
        &OssNodeProperties::new(false),
        &log,
    )
    .expect("direct bitperfect capture must configure alongside playback");
    assert_eq!(applied.actual_config, capture_config);
    assert!(!capture.is_virtual_channel());
    assert_eq!(
        probe_caps(&path, false).expect("busy bitperfect capture caps must still probe"),
        capture_caps
    );
    let mut capture_contender = Dsp::new(&path);
    assert_eq!(capture_contender.open(), Err(Errno::EBUSY));
    let diagnostic = open_failure_diagnostic(&path, false, Errno::EBUSY)
        .expect("exclusive capture must produce a busy diagnostic");
    assert!(diagnostic.contains("capture channel is busy"));

    capture.close();
    playback.close();
    drop(mode);
    for (key, expected) in original_sysctls {
        assert_eq!(read_sysctl(&key), expected, "{key} was not restored");
    }
}

#[test]
#[ignore = "requires the FreeBSD snd_dummy kernel module"]
fn native_snd_dummy_format_geometry_and_duplex_matrix() {
    let (_, path) = assert_dummy_available();
    let log = Log::test_null();

    for config in [
        config(SPA_AUDIO_FORMAT_ULAW, 8_000, 2),
        config(SPA_AUDIO_FORMAT_ALAW, 8_000, 2),
        config(SPA_AUDIO_FORMAT_S8, 8_000, 2),
        config(SPA_AUDIO_FORMAT_U16_LE, 48_000, 4),
        config(SPA_AUDIO_FORMAT_U16_BE, 48_000, 4),
        config(SPA_AUDIO_FORMAT_U24_LE, 48_000, 6),
        config(SPA_AUDIO_FORMAT_U24_BE, 48_000, 6),
        config(SPA_AUDIO_FORMAT_U32_LE, 48_000, 8),
        config(SPA_AUDIO_FORMAT_U32_BE, 48_000, 8),
        config(SPA_AUDIO_FORMAT_S16_LE, 8_000, 4),
        config(SPA_AUDIO_FORMAT_S24_LE, 48_000, 6),
        config(SPA_AUDIO_FORMAT_S32_LE, 96_000, 8),
    ] {
        for playback in [false, true] {
            let caps = probe_caps(&path, playback).expect("snd_dummy capabilities must probe");
            assert!(caps.admits(
                config.format.0,
                config.channels,
                Some(&config.positions),
                config.rate,
            ));
        }

        // Keep both directions open together: the matrix also verifies the
        // kernel's duplex channel allocation, not merely two serial opens.
        let mut playback = DspWriter::new(&path);
        let playback_applied = configure_playback(&mut playback, &config, &log)
            .expect("snd_dummy playback must configure");
        assert_eq!(playback_applied.actual_config, config);

        let mut capture = Dsp::new(&path);
        let capture_applied =
            configure_capture(&mut capture, &config, &OssNodeProperties::new(false), &log)
                .expect("snd_dummy capture must configure while playback is open");
        assert_eq!(capture_applied.actual_config, config);

        let period_bytes = config.stride * 256;
        let playback_request = PlaybackBufferRequest {
            period_bytes,
            graph_rate: config.rate,
            stride: config.stride,
            device_rate: config.rate,
            write_bytes: period_bytes,
            maximum_write_bytes: period_bytes,
        };
        let playback_geometry =
            playback.prime_buffer(playback_request, &OssNodeProperties::new(true), &log);
        assert!(playback_geometry.capacity_bytes >= playback_geometry.quantum_bytes);
        assert_eq!(playback_geometry.capacity_bytes % config.stride, 0);
        assert_eq!(playback_geometry.quantum_bytes % config.stride, 0);

        let capture_request = CaptureBufferRequest {
            period_bytes,
            graph_rate: config.rate,
            stride: config.stride,
            device_rate: config.rate,
        };
        let mut scratch = vec![0u8; 65_536];
        let capture_geometry = capture.prime_buffer(
            capture_request,
            &OssNodeProperties::new(false),
            &mut scratch,
            &log,
        );
        assert!(!capture_geometry.device_lost);
        assert!(capture_geometry.capacity_bytes >= capture_geometry.quantum_bytes);
        assert_eq!(capture_geometry.capacity_bytes % config.stride, 0);
        assert_eq!(capture_geometry.quantum_bytes % config.stride, 0);
        assert!(capture.ready_for_reading(2_000));

        let mut input = vec![0xa5; period_bytes as usize];
        let read = capture.read(&mut input);
        assert!(read.bytes > 0);
        assert_eq!(read.bytes % config.stride as usize, 0);
        assert!(
            contains_only_silence(&config, &input[..read.bytes]),
            "format {} returned non-silence bytes {:?}",
            config.format.0,
            &input[..read.bytes.min(16)]
        );

        capture.close();
        playback.close();
    }
}

#[test]
#[ignore = "requires the FreeBSD snd_dummy kernel module"]
fn native_snd_dummy_queue_xrun_and_reprime_recovery() {
    let (_, path) = assert_dummy_available();
    let config = default_config();
    let log = Log::test_null();
    let properties = OssNodeProperties::new(true);

    let mut playback = DspWriter::new(&path);
    configure_playback(&mut playback, &config, &log).expect("snd_dummy playback must configure");
    let playback_request = PlaybackBufferRequest {
        period_bytes: 4_096,
        graph_rate: config.rate,
        stride: config.stride,
        device_rate: config.rate,
        write_bytes: 4_096,
        maximum_write_bytes: 4_096,
    };
    let playback_geometry = playback.prime_buffer(playback_request, &properties, &log);
    assert!(playback.queued_bytes() > 0);
    wait_until(Duration::from_secs(5), || {
        (playback.queued_bytes() == 0).then_some(())
    })
    .expect("snd_dummy playback queue did not drain");
    let underrun = wait_until(Duration::from_secs(5), || {
        let observation = playback.underruns();
        (observation.value > 0).then_some(observation)
    })
    .expect("snd_dummy did not report a playback underrun");
    assert_eq!(underrun.unit, XrunUnit::Events);
    assert_eq!(underrun.update, CounterUpdate::ResettingTotal);
    assert_eq!(underrun.quality, ObservationQuality::Exact);

    let oversized_period = playback_geometry.capacity_bytes.saturating_mul(2);
    let oversized = PlaybackBufferRequest {
        period_bytes: oversized_period,
        graph_rate: 0,
        stride: config.stride,
        device_rate: config.rate,
        write_bytes: oversized_period,
        maximum_write_bytes: oversized_period,
    };
    assert_eq!(
        playback.retune_buffer(oversized, 0, 0, &log),
        PlaybackRetune::Pending
    );
    assert_eq!(
        playback.retune_buffer(oversized, 0, 0, &log),
        PlaybackRetune::Reprime
    );
    assert!(!playback.is_running());
    playback.prime_buffer(playback_request, &properties, &log);
    assert!(playback.is_running());
    playback.close();

    let mut capture = Dsp::new(&path);
    configure_capture(&mut capture, &config, &OssNodeProperties::new(false), &log)
        .expect("snd_dummy capture must configure");
    let capture_request = CaptureBufferRequest {
        period_bytes: 4_096,
        graph_rate: config.rate,
        stride: config.stride,
        device_rate: config.rate,
    };
    let mut scratch = [0u8; 65_536];
    capture.prime_buffer(
        capture_request,
        &OssNodeProperties::new(false),
        &mut scratch,
        &log,
    );
    assert!(capture.ready_for_reading(2_000));
    let layout = wait_until(Duration::from_secs(5), || {
        let layout = capture.buffer_layout();
        (layout.capacity_bytes > 0 && layout.queued_bytes == layout.capacity_bytes)
            .then_some(layout)
    })
    .expect("snd_dummy capture queue did not become full");
    // GETERROR consumes pcm_channel::xruns. Clear any fill-time event, then
    // leave one aligned frame of room: FreeBSD's vchan capture mixer only
    // counts an overrun when it can feed a child and still has a hardware
    // remainder. A completely full child is skipped before that accounting,
    // and snd_dummy can fill its ring exactly in one silence delivery.
    let _ = capture.overruns();
    let mut frame = vec![0xa5; config.stride as usize];
    let read = capture.read(&mut frame);
    assert_eq!(read.bytes, frame.len());
    assert!(frame.iter().all(|byte| *byte == 0));
    wait_until(Duration::from_secs(5), || {
        (capture.buffer_layout().queued_bytes == layout.capacity_bytes).then_some(())
    })
    .expect("snd_dummy capture queue did not refill after one frame");
    let overrun = capture.overruns();
    assert!(
        overrun.value > 0,
        "snd_dummy did not report the discarded capture remainder"
    );
    assert_eq!(overrun.unit, XrunUnit::Events);
    assert_eq!(overrun.update, CounterUpdate::ResettingTotal);
    assert_eq!(overrun.quality, ObservationQuality::Exact);

    assert_eq!(
        capture.recover_overrun(1, Some(layout.capacity_bytes), &log),
        None
    );
    assert_eq!(
        capture.recover_overrun(1, Some(layout.capacity_bytes), &log),
        None
    );
    assert_eq!(
        capture.recover_overrun(1, Some(layout.capacity_bytes), &log),
        Some(true)
    );
    assert!(!capture.is_running());
    let reprime = capture.prime_buffer(
        capture_request,
        &OssNodeProperties::new(false),
        &mut scratch,
        &log,
    );
    assert!(!reprime.device_lost);
    assert!(capture.is_running());
    capture.close();
}

#[test]
#[ignore = "requires root, devd, and the FreeBSD snd_dummy kernel module"]
fn native_snd_dummy_hotplug_and_devd_reconnect() {
    let (unit, _) = assert_dummy_available();
    let _restore_dummy = DummyModuleGuard;
    let mut catalog = DeviceCatalog::scan().expect("the initial sndstat catalog must scan");
    assert!(catalog_contains_unit(&catalog, unit));
    let initial_object = catalog
        .snapshots()
        .into_iter()
        .find(|snapshot| {
            snapshot.properties.iter().any(|(key, value)| {
                key == DEVICE_INDEXES
                    && value
                        .split(',')
                        .any(|candidate| candidate == unit.to_string())
            })
        })
        .expect("snd_dummy must have a catalog group")
        .object_id;
    let mut monitor = HotplugMonitor::open().expect("the devd hotplug monitor must open");
    let stable_queue = monitor.fd();

    command_succeeds("/usr/sbin/service", &["devd", "restart"]);
    poll_catalog(&mut monitor, &mut catalog, Duration::from_secs(15), |_| {
        true
    });
    assert_eq!(
        monitor.fd(),
        stable_queue,
        "devd reconnect replaced the kqueue"
    );
    assert!(catalog_contains_unit(&catalog, unit));

    command_succeeds("/sbin/kldunload", &["snd_dummy"]);
    wait_until(Duration::from_secs(5), || {
        (!std::path::Path::new("/dev/dsp.dummy").exists()).then_some(())
    })
    .expect("snd_dummy device node survived module unload");
    poll_catalog(
        &mut monitor,
        &mut catalog,
        Duration::from_secs(15),
        |changes| {
            changes.iter().any(|change| {
                matches!(change, CatalogChange::Removed { object_id, .. } if *object_id == initial_object)
            })
        },
    );
    assert!(!catalog_contains_unit(&catalog, unit));

    command_succeeds("/sbin/kldload", &["snd_dummy"]);
    wait_until(Duration::from_secs(5), || {
        std::path::Path::new("/dev/dsp.dummy")
            .exists()
            .then_some(())
    })
    .expect("snd_dummy device node did not return after module load");
    let reloaded_unit =
        snd_dummy_unit().expect("reloaded snd_dummy must return to sndstat and sysctls");
    poll_catalog(
        &mut monitor,
        &mut catalog,
        Duration::from_secs(15),
        |changes| {
            changes.iter().any(|change| {
                matches!(change, CatalogChange::Added { snapshot, .. } if snapshot.properties.iter().any(|(key, value)| {
                    key == DEVICE_INDEXES
                        && value.split(',').any(|candidate| candidate == reloaded_unit.to_string())
                }))
            })
        },
    );
    assert!(catalog_contains_unit(&catalog, reloaded_unit));
    assert_eq!(monitor.fd(), stable_queue);
}

#[test]
#[ignore = "requires the FreeBSD snd_dummy kernel module"]
fn native_snd_dummy_spa_factory_and_sink_process_smoke() {
    let (_, path) = assert_dummy_available();
    spa_host::run_sink_smoke(&path);
}
