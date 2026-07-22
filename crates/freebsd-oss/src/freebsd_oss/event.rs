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
const DEVD_RETRY_TIMER_IDENT: libc::uintptr_t = 2;
const DEVD_RETRY_MIN_MS: u32 = 1_000;
const DEVD_RETRY_MAX_MS: u32 = 30_000;

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
        submit_change(self.raw(), change)
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

fn submit_change(queue_fd: c_int, change: libc::kevent) -> Result<(), Errno> {
    let mut receipt = kevent(0, 0, 0, 0, 0, 0);
    let n = unsafe { libc::kevent(queue_fd, &change, 1, &mut receipt, 1, std::ptr::null()) };
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

#[derive(Debug, Eq, PartialEq)]
enum CatalogSignal {
    Attached,
    Detached(String),
}

#[derive(Debug, Eq, PartialEq)]
enum CatalogRefresh {
    Full,
    Detached(String),
}

fn catalog_refresh(signal: Option<CatalogSignal>, reconnected: bool) -> Option<CatalogRefresh> {
    if reconnected {
        Some(CatalogRefresh::Full)
    } else {
        match signal {
            Some(CatalogSignal::Attached) => Some(CatalogRefresh::Full),
            Some(CatalogSignal::Detached(subject)) => Some(CatalogRefresh::Detached(subject)),
            None => None,
        }
    }
}

fn merge_catalog_signal(
    current: Option<CatalogSignal>,
    next: Option<CatalogSignal>,
) -> Option<CatalogSignal> {
    match (current, next) {
        (current, None) => current,
        (None, next) => next,
        (Some(CatalogSignal::Detached(a)), Some(CatalogSignal::Detached(b))) if a == b => {
            Some(CatalogSignal::Detached(a))
        }
        // One targeted rescan cannot represent two different detachments.
        // Promote the batch to a full refresh rather than lose either one.
        (Some(CatalogSignal::Attached), _)
        | (_, Some(CatalogSignal::Attached))
        | (Some(CatalogSignal::Detached(_)), Some(CatalogSignal::Detached(_))) => {
            Some(CatalogSignal::Attached)
        }
    }
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

fn take_retry_delay(delay_ms: &mut u32) -> u32 {
    let delay = *delay_ms;
    *delay_ms = delay.saturating_mul(2).min(DEVD_RETRY_MAX_MS);
    delay
}

fn kevent_read_error_is_fatal(error: Errno) -> bool {
    // EINTR and resource-pressure failures do not invalidate a kqueue. EBADF
    // is the one read-side error that proves SPA's stable descriptor is gone.
    error == Errno::EBADF
}

/// Stable event queue around a replaceable devd connection. A devd restart
/// closes existing clients; keep the kqueue watched by SPA alive and use its
/// timer filter to reconnect without spinning the main loop.
pub(crate) struct HotplugMonitor {
    queue: LibcFd,
    socket: Option<DevdSocket>,
    socket_key: usize,
    retry_delay_ms: u32,
}

impl HotplugMonitor {
    pub(crate) fn open() -> Result<Self, std::io::Error> {
        let queue =
            LibcFd::kqueue().map_err(|error| std::io::Error::from_raw_os_error(error as c_int))?;
        let mut monitor = Self {
            queue,
            socket: None,
            socket_key: 0,
            retry_delay_ms: DEVD_RETRY_MIN_MS,
        };
        let installed = match DevdSocket::open() {
            Ok(socket) => monitor.install_socket(socket).is_ok(),
            Err(_) => false,
        };
        if !installed {
            monitor
                .arm_retry()
                .map_err(|error| std::io::Error::from_raw_os_error(error as c_int))?;
        }
        Ok(monitor)
    }

    pub(crate) fn fd(&self) -> RawFd {
        self.queue.raw()
    }

    #[cfg(test)]
    fn test_with_socket(socket: DevdSocket) -> Self {
        let queue = LibcFd::kqueue().unwrap();
        let mut monitor = Self {
            queue,
            socket: None,
            socket_key: 0,
            retry_delay_ms: DEVD_RETRY_MIN_MS,
        };
        monitor.install_socket(socket).unwrap();
        monitor
    }

    fn install_socket(&mut self, socket: DevdSocket) -> Result<(), Errno> {
        self.socket_key = self.socket_key.wrapping_add(1).max(1);
        submit_change(
            self.queue.raw(),
            kevent(
                socket.fd() as libc::uintptr_t,
                libc::EVFILT_READ,
                // Keep this level-triggered: devd uses one seqpacket per
                // event and the callback consumes one packet at a time.
                libc::EV_ADD | libc::EV_RECEIPT,
                0,
                0,
                self.socket_key,
            ),
        )?;
        self.socket = Some(socket);
        self.retry_delay_ms = DEVD_RETRY_MIN_MS;
        Ok(())
    }

    fn arm_retry(&mut self) -> Result<(), Errno> {
        let delay_ms = take_retry_delay(&mut self.retry_delay_ms);
        submit_change(
            self.queue.raw(),
            kevent(
                DEVD_RETRY_TIMER_IDENT,
                libc::EVFILT_TIMER,
                libc::EV_ADD | libc::EV_ONESHOT | libc::EV_RECEIPT,
                libc::NOTE_MSECONDS,
                i64::from(delay_ms),
                0,
            ),
        )
    }

    fn reconnect(&mut self) -> Result<bool, Errno> {
        let Ok(socket) = DevdSocket::open() else {
            self.arm_retry()?;
            return Ok(false);
        };
        if self.install_socket(socket).is_err() {
            self.socket = None;
            self.arm_retry()?;
            return Ok(false);
        }
        Ok(true)
    }

    fn handle_socket_loss(&mut self) -> bool {
        self.socket = None;
        self.arm_retry().is_ok()
    }

    /// Drain one kqueue batch. `alive` describes the stable queue itself;
    /// losing only devd schedules a retry and deliberately remains alive.
    fn read_event(&mut self, mut apply: impl FnMut(&str)) -> (bool, bool) {
        let mut events = [kevent(0, 0, 0, 0, 0, 0); 4];
        let timeout = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        let n = unsafe {
            libc::kevent(
                self.queue.raw(),
                std::ptr::null(),
                0,
                events.as_mut_ptr(),
                events.len() as c_int,
                &timeout,
            )
        };
        if n < 0 {
            return (!kevent_read_error_is_fatal(Errno::last()), false);
        }

        let mut reconnected = false;
        for event in events.into_iter().take(n as usize) {
            let matches_retry_timer =
                event.filter == libc::EVFILT_TIMER && event.ident == DEVD_RETRY_TIMER_IDENT;
            let matches_socket = self.socket.as_ref().is_some_and(|socket| {
                event.filter == libc::EVFILT_READ
                    && event.ident == socket.fd() as libc::uintptr_t
                    && event.udata as usize == self.socket_key
            });
            if event.flags & libc::EV_ERROR != 0 {
                if matches_socket {
                    if !self.handle_socket_loss() {
                        return (false, reconnected);
                    }
                } else if matches_retry_timer && self.arm_retry().is_err() {
                    return (false, reconnected);
                }
                continue;
            }
            if matches_retry_timer {
                match self.reconnect() {
                    Ok(restored) => reconnected |= restored,
                    Err(_) => return (false, reconnected),
                }
                continue;
            }

            if !matches_socket {
                continue;
            }
            let socket_alive = self
                .socket
                .as_mut()
                .is_some_and(|socket| socket.read_event(&mut apply));
            if !socket_alive && !self.handle_socket_loss() {
                return (false, reconnected);
            }
        }
        (true, reconnected)
    }

    /// Consume one relevant native event and apply its replacement semantics
    /// to the concrete catalog. The shared monitor sees only the resulting
    /// neutral diff, never a daemon payload or native device name.
    pub(crate) fn read_catalog_rescan(
        &mut self,
        catalog: &mut DeviceCatalog,
    ) -> (bool, Option<CatalogRescan>) {
        let mut signal = None;
        let (alive, reconnected) = self.read_event(|line| {
            signal = merge_catalog_signal(signal.take(), decode_catalog_signal(line));
        });
        let rescan = match catalog_refresh(signal, reconnected) {
            // Events can be lost while devd is down; reconnect maps to Full
            // and reconciles the catalog before trusting the new connection.
            Some(CatalogRefresh::Full) => Some(catalog.rescan(&[])),
            Some(CatalogRefresh::Detached(subject)) => {
                Some(catalog.rescan(std::slice::from_ref(&subject)))
            }
            None => None,
        };
        (alive, rescan)
    }

    /// Returns connection liveness plus the PCM unit named by a sound event.
    pub(super) fn read_mixer_event(&mut self) -> (bool, Option<u32>, bool) {
        let mut unit = None;
        let (alive, reconnected) = self.read_event(|line| {
            if let Some(decoded) = decode_mixer_event(line) {
                unit = Some(decoded);
            }
        });
        (alive, unit, reconnected)
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
    fn reconnect_forces_a_full_catalog_reconciliation() {
        assert_eq!(
            catalog_refresh(Some(CatalogSignal::Detached("uaudio3".into())), true),
            Some(CatalogRefresh::Full)
        );
        assert_eq!(
            catalog_refresh(Some(CatalogSignal::Detached("uaudio3".into())), false),
            Some(CatalogRefresh::Detached("uaudio3".into()))
        );
    }

    #[test]
    fn batch_decode_preserves_relevant_signals() {
        assert_eq!(
            merge_catalog_signal(Some(CatalogSignal::Attached), None),
            Some(CatalogSignal::Attached)
        );
        assert_eq!(
            merge_catalog_signal(
                Some(CatalogSignal::Detached("pcm3".into())),
                Some(CatalogSignal::Detached("pcm4".into())),
            ),
            Some(CatalogSignal::Attached)
        );
    }

    #[test]
    fn only_a_dead_kqueue_ends_the_stable_monitor() {
        assert!(!kevent_read_error_is_fatal(Errno::EINTR));
        assert!(!kevent_read_error_is_fatal(Errno::ENOMEM));
        assert!(kevent_read_error_is_fatal(Errno::EBADF));
    }

    #[test]
    fn socket_filter_failure_uses_the_eof_retry_path() {
        let (socket, _peer) = DevdSocket::test_pair();
        let mut monitor = HotplugMonitor::test_with_socket(socket);
        monitor.retry_delay_ms = 1;

        assert!(monitor.handle_socket_loss());
        assert!(monitor.socket.is_none());
        assert_eq!(monitor.retry_delay_ms, 2);
    }

    #[test]
    fn devd_eof_keeps_the_stable_queue_alive_and_arms_retry() {
        let (socket, peer) = DevdSocket::test_pair();
        let mut monitor = HotplugMonitor::test_with_socket(socket);
        let mut pollfd = libc::pollfd {
            fd: monitor.fd(),
            events: libc::POLLIN,
            revents: 0,
        };

        peer.send(b"+pcm7 at uaudio3").unwrap();
        assert_eq!(unsafe { libc::poll(&mut pollfd, 1, 100) }, 1);
        let mut signal = None;
        assert_eq!(
            monitor.read_event(|line| signal = decode_catalog_signal(line)),
            (true, false)
        );
        assert_eq!(signal, Some(CatalogSignal::Attached));

        monitor.retry_delay_ms = 1;
        drop(peer);
        pollfd.revents = 0;
        assert_eq!(unsafe { libc::poll(&mut pollfd, 1, 100) }, 1);
        assert_eq!(monitor.read_event(|_| {}), (true, false));
        assert!(monitor.socket.is_none());
        assert_eq!(monitor.retry_delay_ms, 2);
        assert!(monitor.fd() >= 0);

        pollfd.revents = 0;
        assert_eq!(unsafe { libc::poll(&mut pollfd, 1, 100) }, 1);
        assert_ne!(pollfd.revents & libc::POLLIN, 0);
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

    #[test]
    fn devd_retry_backoff_is_exponential_and_bounded() {
        let mut delay = DEVD_RETRY_MIN_MS;
        assert_eq!(take_retry_delay(&mut delay), 1_000);
        assert_eq!(take_retry_delay(&mut delay), 2_000);
        assert_eq!(take_retry_delay(&mut delay), 4_000);
        assert_eq!(take_retry_delay(&mut delay), 8_000);
        assert_eq!(take_retry_delay(&mut delay), 16_000);
        assert_eq!(take_retry_delay(&mut delay), 30_000);
        assert_eq!(take_retry_delay(&mut delay), 30_000);
    }
}
