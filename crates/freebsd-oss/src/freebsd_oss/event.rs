use std::cell::Cell;
use std::ffi::c_int;
use std::os::fd::RawFd;

use nix::errno::Errno;

use crate::backend::{
    CatalogRescan, ObservationQuality, QueueObservation, StreamIdentity, StreamWake,
    StreamWakeState, WakeDriver, WakeError, WakeEvent, XrunObservation,
};

use super::devices::DeviceCatalog;
use super::sys::{DevdSocket, LibcFd, SysctlReader};

// The enriched sound kevent payload landed in main while osreldate was
// 1600018 and was merged to stable/15 at 1501501, but both values predate
// the changes on their branches. 15.2-RELEASE and 1600019-CURRENT are the
// first osreldates that unambiguously include ready frames and xrun counts.
const ENRICHED_SOUND_KQUEUE_15_2_OSREL: u32 = 1_502_000;
const FREEBSD_16_BASE_OSREL: u32 = 1_600_000;
const ENRICHED_SOUND_KQUEUE_16_OSREL: u32 = 1_600_019;
const TIMER_IDENT: libc::uintptr_t = 1;

pub(crate) fn enriched_sound_kqueue_available() -> bool {
    SysctlReader::new()
        .read_u32("kern.osreldate")
        .is_ok_and(enriched_sound_kqueue_osrel)
}

fn enriched_sound_kqueue_osrel(version: u32) -> bool {
    (ENRICHED_SOUND_KQUEUE_15_2_OSREL..FREEBSD_16_BASE_OSREL).contains(&version)
        || version >= ENRICHED_SOUND_KQUEUE_16_OSREL
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RegisteredStream {
    fd: c_int,
    filter: i16,
    stream: StreamIdentity,
    frame_stride: u32,
    key: usize,
}

pub(crate) struct OssWakeDriver {
    fd: LibcFd,
    registered: Option<RegisteredStream>,
    next_registration_key: usize,
    // The device knote normally wins the wake race. The one-shot deadline
    // remains armed as a liveness watchdog, and also carries timer fallback
    // when no device knote is registered.
    timer_armed: Cell<bool>,
}

impl OssWakeDriver {
    pub(crate) fn new() -> Result<Self, WakeError> {
        Ok(Self {
            fd: LibcFd::kqueue().map_err(WakeError::new)?,
            registered: None,
            next_registration_key: 0,
            timer_armed: Cell::new(false),
        })
    }

    fn raw(&self) -> c_int {
        self.fd.raw()
    }

    pub(super) fn register_stream(
        &mut self,
        fd: c_int,
        playback: bool,
        stream: StreamIdentity,
        frame_stride: u32,
    ) -> Result<(), WakeError> {
        let filter = if playback {
            libc::EVFILT_WRITE
        } else {
            libc::EVFILT_READ
        };
        if self.registered.is_some_and(|registered| {
            registered.fd == fd
                && registered.filter == filter
                && registered.stream == stream
                && registered.frame_stride == frame_stride
        }) {
            return Ok(());
        }
        self.unregister_native_stream().map_err(WakeError::new)?;
        self.next_registration_key = self.next_registration_key.wrapping_add(1).max(1);
        let registered = RegisteredStream {
            fd,
            filter,
            stream,
            frame_stride: frame_stride.max(1),
            key: self.next_registration_key,
        };
        self.submit_change(kevent(
            fd as libc::uintptr_t,
            filter,
            // A host may defer process() until after ready() returns. Clear
            // the delivered activation so a still-ready level does not make
            // the outer SPA loop spin; the next sound-buffer change can
            // activate it again.
            libc::EV_ADD | libc::EV_CLEAR | libc::EV_RECEIPT,
            0,
            0,
            registered.key,
        ))
        .map_err(WakeError::new)?;
        self.registered = Some(registered);
        Ok(())
    }

    fn unregister_native_stream(&mut self) -> Result<(), Errno> {
        let Some(registered) = self.registered else {
            return Ok(());
        };
        match self.submit_change(kevent(
            registered.fd as libc::uintptr_t,
            registered.filter,
            libc::EV_DELETE | libc::EV_RECEIPT,
            0,
            0,
            registered.key,
        )) {
            // Closing a descriptor removes its knotes. Treat an already-gone
            // registration as the requested final state.
            Ok(()) | Err(Errno::ENOENT | Errno::EBADF) => {
                self.registered = None;
                Ok(())
            }
            Err(err) => Err(err),
        }
    }

    fn arm_native_timer(&self, delay_ns: u64) -> Result<(), Errno> {
        if delay_ns == 0 {
            if !self.timer_armed.get() {
                return Ok(());
            }
            let result = match self.submit_change(kevent(
                TIMER_IDENT,
                libc::EVFILT_TIMER,
                libc::EV_DELETE | libc::EV_RECEIPT,
                0,
                0,
                0,
            )) {
                Err(Errno::ENOENT) => Ok(()),
                result => result,
            };
            if result.is_ok() {
                self.timer_armed.set(false);
            }
            return result;
        }
        // Keep this timer relative. FreeBSD NOTE_ABSTIME interprets `data` as
        // a wall-clock epoch, whereas the graph deadline is monotonic; the
        // shared WakeDriver boundary has already converted it to a delay.
        let result = self.submit_change(kevent(
            TIMER_IDENT,
            libc::EVFILT_TIMER,
            libc::EV_ADD | libc::EV_ONESHOT | libc::EV_RECEIPT,
            libc::NOTE_NSECONDS,
            delay_ns.min(i64::MAX as u64) as i64,
            0,
        ));
        if result.is_ok() {
            self.timer_armed.set(true);
        }
        result
    }

    fn next_native_event(&self) -> Result<Option<WakeEvent>, Errno> {
        // The device edge and its deadline watchdog intentionally share this
        // queue. They can become ready together, so consume both in one read
        // and prefer the enriched device snapshot. Leaving either event
        // queued would make the host start a second graph cycle immediately.
        let mut events = [kevent(0, 0, 0, 0, 0, 0), kevent(0, 0, 0, 0, 0, 0)];
        let timeout = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        let n = unsafe {
            libc::kevent(
                self.raw(),
                std::ptr::null(),
                0,
                events.as_mut_ptr(),
                events.len() as c_int,
                &timeout,
            )
        };
        if n < 0 {
            return Err(Errno::last());
        }
        let (selected, timer_seen) =
            decode_events(events.into_iter().take(n as usize), self.registered)?;
        if timer_seen {
            self.timer_armed.set(false);
        }
        Ok(selected)
    }

    fn submit_change(&self, change: libc::kevent) -> Result<(), Errno> {
        let mut receipt = kevent(0, 0, 0, 0, 0, 0);
        let n = unsafe { libc::kevent(self.raw(), &change, 1, &mut receipt, 1, std::ptr::null()) };
        if n < 0 {
            return Err(Errno::last());
        }
        if n != 1 || receipt.flags & libc::EV_ERROR == 0 {
            return Err(Errno::EIO);
        }
        if receipt.data != 0 {
            return Err(Errno::from_raw(receipt.data as c_int));
        }
        Ok(())
    }
}

impl WakeDriver for OssWakeDriver {
    fn notification_fd(&self) -> RawFd {
        self.raw()
    }

    fn unregister_stream(&mut self) -> Result<(), WakeError> {
        self.unregister_native_stream().map_err(WakeError::new)
    }

    fn arm_timer(&self, delay_ns: u64) -> Result<(), WakeError> {
        self.arm_native_timer(delay_ns).map_err(WakeError::new)
    }

    fn next_event(&self) -> Result<Option<WakeEvent>, WakeError> {
        self.next_native_event().map_err(WakeError::new)
    }
}

fn decode_events(
    events: impl IntoIterator<Item = libc::kevent>,
    registered: Option<RegisteredStream>,
) -> Result<(Option<WakeEvent>, bool), Errno> {
    let mut selected = None;
    let mut first_error = None;
    let mut timer_seen = false;
    for event in events {
        match decode_event(event, registered) {
            Ok(Some(WakeEvent::Timer)) => {
                timer_seen = true;
                selected.get_or_insert(WakeEvent::Timer);
            }
            Ok(Some(stream @ WakeEvent::Stream(_))) => selected = Some(stream),
            Ok(None) => {}
            Err(err) => {
                first_error.get_or_insert(err);
            }
        }
    }
    if selected.is_some() {
        Ok((selected, timer_seen))
    } else if let Some(err) = first_error {
        Err(err)
    } else {
        Ok((None, timer_seen))
    }
}

fn decode_event(
    event: libc::kevent,
    registered: Option<RegisteredStream>,
) -> Result<Option<WakeEvent>, Errno> {
    if event.flags & libc::EV_ERROR != 0 {
        let errno = event.data as c_int;
        return Err(if errno == 0 {
            Errno::EIO
        } else {
            Errno::from_raw(errno)
        });
    }
    if event.filter == libc::EVFILT_TIMER {
        return Ok(Some(WakeEvent::Timer));
    }
    if event.filter != libc::EVFILT_READ && event.filter != libc::EVFILT_WRITE {
        return Ok(None);
    }
    let Some(registered) = registered else {
        return Ok(None);
    };
    if event.ident != registered.fd as libc::uintptr_t
        || event.filter != registered.filter
        || event.udata as usize != registered.key
    {
        return Ok(None);
    }
    let ready_bytes = event.data.max(0).min(u32::MAX as i64) as u32;
    let fill_bytes = if registered.filter == libc::EVFILT_READ {
        u64::from(ready_bytes)
    } else {
        event.ext[0].saturating_mul(u64::from(registered.frame_stride))
    };
    Ok(Some(WakeEvent::Stream(StreamWake {
        stream: registered.stream,
        timing: crate::backend::WakeTiming::NotificationTime,
        ready_bytes: Some(ready_bytes),
        queue: Some(QueueObservation {
            fill_bytes,
            quality: ObservationQuality::Exact,
        }),
        clock: None,
        xruns: Some(XrunObservation::wrapping_events_u32(
            event.ext[1].min(u64::from(u32::MAX)) as u32,
        )),
        state: if event.flags & libc::EV_EOF != 0 {
            StreamWakeState::Disconnected
        } else {
            StreamWakeState::Active
        },
    })))
}

fn kevent(
    ident: libc::uintptr_t,
    filter: i16,
    flags: u16,
    fflags: u32,
    data: i64,
    key: usize,
) -> libc::kevent {
    libc::kevent {
        ident,
        filter,
        flags,
        fflags,
        data,
        udata: std::ptr::without_provenance_mut(key),
        ext: [0; 4],
    }
}

#[derive(Debug, Eq, PartialEq)]
enum CatalogSignal {
    Attached,
    Detached(String),
}

fn decode_catalog_signal(line: &str) -> Option<CatalogSignal> {
    static RE: std::sync::LazyLock<regex::Regex> =
        std::sync::LazyLock::new(|| regex::Regex::new(r"^([\+-])((?:pcm|uaudio)\d+)").unwrap());

    let groups = RE.captures(line)?;
    if groups.get(1)?.as_str() == "-" {
        Some(CatalogSignal::Detached(groups.get(2)?.as_str().to_string()))
    } else {
        Some(CatalogSignal::Attached)
    }
}

// devd's SND CONN payload identifies a PCM unit but does not carry usable
// jack state. sound.c emits the same event when the default unit changes;
// hdaa emits it only for a subset of pin-sense changes, and connect/disconnect
// are indistinguishable. The mixer policy therefore treats it solely as an
// immediate route-poll nudge; route availability remains unchanged.
fn decode_mixer_event(line: &str) -> Option<u32> {
    static RE: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
        regex::Regex::new(r"^!system=SND subsystem=CONN type=(?:IN|OUT) cdev=dsp([0-9]+)").unwrap()
    });

    RE.captures(line)
        .and_then(|groups| groups[1].parse::<u32>().ok())
}

/// devd connection that exposes decoded sound-device events.
pub(crate) struct HotplugMonitor(DevdSocket);

impl HotplugMonitor {
    pub(crate) fn open() -> Result<Self, std::io::Error> {
        DevdSocket::open().map(Self)
    }

    pub(crate) fn fd(&self) -> RawFd {
        self.0.fd()
    }

    /// Consume one relevant native event and apply its replacement semantics
    /// to the concrete catalog. The shared monitor sees only the resulting
    /// neutral diff, never a daemon payload or native device name.
    pub(crate) fn read_catalog_rescan(
        &mut self,
        catalog: &mut DeviceCatalog,
    ) -> (bool, Option<CatalogRescan>) {
        let mut signal = None;
        let alive = self.0.read_event(|line| {
            signal = decode_catalog_signal(line);
        });
        let rescan = match signal {
            Some(CatalogSignal::Attached) => Some(catalog.rescan(&[])),
            Some(CatalogSignal::Detached(subject)) => {
                Some(catalog.rescan(std::slice::from_ref(&subject)))
            }
            None => None,
        };
        (alive, rescan)
    }

    /// Returns connection liveness plus the PCM unit named by a sound event.
    pub(super) fn read_mixer_event(&mut self) -> (bool, Option<u32>) {
        let mut unit = None;
        let alive = self.0.read_event(|line| {
            unit = decode_mixer_event(line);
        });
        (alive, unit)
    }
}

impl crate::backend::HotplugMonitor<super::devices::DeviceCatalog> for HotplugMonitor {
    fn open() -> Result<Self, std::io::Error> {
        Self::open()
    }

    fn fd(&self) -> RawFd {
        self.fd()
    }

    fn read_catalog_rescan(
        &mut self,
        catalog: &mut super::devices::DeviceCatalog,
    ) -> (bool, Option<CatalogRescan>) {
        self.read_catalog_rescan(catalog)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::{StreamToken, test_transport};

    const TEST_STREAM: StreamIdentity = StreamIdentity::new(StreamToken::for_port(0), 7);

    fn registered(fd: c_int, filter: i16, key: usize) -> RegisteredStream {
        RegisteredStream {
            fd,
            filter,
            stream: TEST_STREAM,
            frame_stride: 8,
            key,
        }
    }

    #[test]
    fn hotplug_payloads_are_decoded() {
        assert_eq!(
            decode_catalog_signal("-uaudio3 at uhub2"),
            Some(CatalogSignal::Detached("uaudio3".into()))
        );
        assert_eq!(
            decode_catalog_signal("+pcm7 at uaudio3"),
            Some(CatalogSignal::Attached)
        );
        assert_eq!(decode_catalog_signal("!system=USB"), None);

        assert_eq!(
            decode_mixer_event("!system=SND subsystem=CONN type=OUT cdev=dsp12"),
            Some(12)
        );
        assert_eq!(
            decode_mixer_event("!system=SND subsystem=CONN type=NODEV"),
            None
        );
    }

    #[test]
    fn one_shot_timer_wakes_the_queue() {
        let driver = OssWakeDriver::new().unwrap();
        driver.arm_timer(1).unwrap();
        let mut event = None;
        for _ in 0..100 {
            event = driver.next_event().unwrap();
            if event.is_some() {
                break;
            }
            std::thread::yield_now();
        }
        assert_eq!(event, Some(WakeEvent::Timer));
        assert_eq!(driver.next_event().unwrap(), None);
    }

    #[test]
    fn deleting_an_unarmed_timer_is_idempotent() {
        let driver = OssWakeDriver::new().unwrap();
        driver.arm_timer(0).unwrap();
        driver.arm_timer(0).unwrap();
    }

    #[test]
    fn kqueue_descriptor_is_pollable_by_the_host_loop() {
        let driver = OssWakeDriver::new().unwrap();
        driver.arm_timer(1).unwrap();
        let mut pfd = libc::pollfd {
            fd: driver.notification_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        assert_eq!(unsafe { libc::poll(&mut pfd, 1, 100) }, 1);
        assert_ne!(pfd.revents & libc::POLLIN, 0);
        assert_eq!(driver.next_event().unwrap(), Some(WakeEvent::Timer));
    }

    #[test]
    fn device_registration_delivers_through_the_nested_queue() {
        let (read_fd, write_fd) = test_transport::pipe_pair(true, true);
        let mut driver = OssWakeDriver::new().unwrap();
        driver
            .register_stream(write_fd, true, TEST_STREAM, 8)
            .unwrap();

        let mut pfd = libc::pollfd {
            fd: driver.notification_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        assert_eq!(unsafe { libc::poll(&mut pfd, 1, 100) }, 1);
        let Some(WakeEvent::Stream(event)) = driver.next_event().unwrap() else {
            panic!("the writable descriptor should produce a device event");
        };
        assert_eq!(event.stream, TEST_STREAM);
        assert_eq!(driver.next_event().unwrap(), None);

        // EV_CLEAR does not re-deliver a still-writable level until the
        // underlying object changes, and registering the same knote is
        // intentionally idempotent. The deadline timer must remain able to
        // wake the driver in that no-new-edge state.
        driver
            .register_stream(write_fd, true, TEST_STREAM, 8)
            .unwrap();
        assert_eq!(driver.next_event().unwrap(), None);
        driver.arm_timer(1).unwrap();
        let mut watchdog = None;
        for _ in 0..100 {
            watchdog = driver.next_event().unwrap();
            if watchdog.is_some() {
                break;
            }
            std::thread::yield_now();
        }
        assert_eq!(watchdog, Some(WakeEvent::Timer));

        driver.unregister_stream().unwrap();
        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }

    #[test]
    fn simultaneous_device_and_watchdog_wake_once() {
        let (read_fd, write_fd) = test_transport::pipe_pair(true, true);
        let mut driver = OssWakeDriver::new().unwrap();
        driver
            .register_stream(write_fd, true, TEST_STREAM, 8)
            .unwrap();
        driver.arm_timer(1).unwrap();

        // Keep both knotes pending until one kevent read can coalesce them.
        std::thread::sleep(std::time::Duration::from_millis(1));
        let Some(WakeEvent::Stream(event)) = driver.next_event().unwrap() else {
            panic!("the device snapshot should win the watchdog race");
        };
        assert_eq!(event.stream, TEST_STREAM);
        assert!(!driver.timer_armed.get());
        assert_eq!(driver.next_event().unwrap(), None);

        driver.unregister_stream().unwrap();
        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }

    #[test]
    fn device_snapshot_survives_a_sibling_decode_error() {
        let registration = registered(42, libc::EVFILT_WRITE, 9);
        let mut device = kevent(42, libc::EVFILT_WRITE, 0, 0, 8192, registration.key);
        device.ext[0] = 768;
        device.ext[1] = 3;
        let error = kevent(
            0,
            libc::EVFILT_TIMER,
            libc::EV_ERROR,
            0,
            libc::EIO as i64,
            0,
        );

        for events in [[device, error], [error, device]] {
            let (selected, timer_seen) = decode_events(events, Some(registration)).unwrap();
            assert!(!timer_seen);
            assert_eq!(
                selected,
                Some(WakeEvent::Stream(StreamWake {
                    stream: TEST_STREAM,
                    timing: crate::backend::WakeTiming::NotificationTime,
                    ready_bytes: Some(8192),
                    queue: Some(QueueObservation {
                        fill_bytes: 6144,
                        quality: ObservationQuality::Exact,
                    }),
                    clock: None,
                    xruns: Some(XrunObservation::wrapping_events_u32(3)),
                    state: StreamWakeState::Active,
                }))
            );
        }
    }

    #[test]
    fn batch_decode_reports_an_error_without_a_usable_event() {
        let error = kevent(
            0,
            libc::EVFILT_TIMER,
            libc::EV_ERROR,
            0,
            libc::EIO as i64,
            0,
        );
        assert_eq!(decode_events([error], None), Err(Errno::EIO));
    }

    #[test]
    fn unregister_tolerates_a_device_closed_by_teardown() {
        let (read_fd, write_fd) = test_transport::pipe_pair(true, true);
        let mut driver = OssWakeDriver::new().unwrap();
        driver
            .register_stream(write_fd, true, TEST_STREAM, 8)
            .unwrap();
        unsafe {
            libc::close(write_fd);
        }
        driver.unregister_stream().unwrap();
        unsafe {
            libc::close(read_fd);
        }
    }

    #[test]
    fn enriched_device_fields_decode_as_one_snapshot() {
        let registration = registered(42, libc::EVFILT_WRITE, 11);
        let mut raw = kevent(
            42,
            libc::EVFILT_WRITE,
            libc::EV_EOF,
            0,
            8192,
            registration.key,
        );
        raw.ext[0] = 768;
        raw.ext[1] = 3;
        assert_eq!(
            decode_event(raw, Some(registration)).unwrap(),
            Some(WakeEvent::Stream(StreamWake {
                stream: TEST_STREAM,
                timing: crate::backend::WakeTiming::NotificationTime,
                ready_bytes: Some(8192),
                queue: Some(QueueObservation {
                    fill_bytes: 6144,
                    quality: ObservationQuality::Exact,
                }),
                clock: None,
                xruns: Some(XrunObservation::wrapping_events_u32(3)),
                state: StreamWakeState::Disconnected,
            }))
        );
    }

    #[test]
    fn stale_registration_key_is_not_attributed_to_a_reused_descriptor() {
        let current = registered(42, libc::EVFILT_WRITE, 13);
        let stale = kevent(42, libc::EVFILT_WRITE, 0, 0, 8192, 12);
        assert_eq!(decode_event(stale, Some(current)), Ok(None));
    }

    #[test]
    fn enriched_events_require_an_unambiguous_osrel() {
        assert!(!enriched_sound_kqueue_osrel(1_501_501));
        assert!(enriched_sound_kqueue_osrel(1_502_000));
        assert!(enriched_sound_kqueue_osrel(1_599_999));
        assert!(!enriched_sound_kqueue_osrel(1_600_018));
        assert!(enriched_sound_kqueue_osrel(1_600_019));
    }
}
