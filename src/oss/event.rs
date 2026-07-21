use std::cell::Cell;
use std::ffi::c_int;

use nix::errno::Errno;

use crate::freebsd::{LibcFd, SysctlReader};

// The enriched sound kevent payload landed in main while osreldate was
// 1600018 and was merged to stable/15 at 1501501, but both values predate
// the changes on their branches. 15.2-RELEASE and 1600019-CURRENT are the
// first osreldates that unambiguously include ready frames and xrun counts.
const ENRICHED_SOUND_KQUEUE_15_2_OSREL: u32 = 1_502_000;
const FREEBSD_16_BASE_OSREL: u32 = 1_600_000;
const ENRICHED_SOUND_KQUEUE_16_OSREL: u32 = 1_600_019;
const TIMER_IDENT: libc::uintptr_t = 1;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct DeviceEvent {
    pub(crate) fd: c_int,
    pub(crate) available_bytes: u32,
    pub(crate) ready_frames: u64,
    pub(crate) xruns: u32,
    pub(crate) eof: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum WakeEvent {
    Timer,
    Device(DeviceEvent),
}

pub(crate) fn enriched_sound_kqueue_available() -> bool {
    SysctlReader::new()
        .read_u32("kern.osreldate")
        .is_ok_and(enriched_sound_kqueue_osrel)
}

fn enriched_sound_kqueue_osrel(version: u32) -> bool {
    (ENRICHED_SOUND_KQUEUE_15_2_OSREL..FREEBSD_16_BASE_OSREL).contains(&version)
        || version >= ENRICHED_SOUND_KQUEUE_16_OSREL
}

pub(crate) struct SoundKqueue {
    fd: LibcFd,
    registered: Option<(c_int, i16)>,
    // The device knote normally wins the wake race. The one-shot deadline
    // remains armed as a liveness watchdog, and also carries timer fallback
    // when no device knote is registered.
    timer_armed: Cell<bool>,
}

impl SoundKqueue {
    pub(crate) fn new() -> Result<Self, Errno> {
        let fd = unsafe { libc::kqueue() };
        if fd < 0 {
            return Err(Errno::last());
        }
        // kqueue() predates kqueuex(KQUEUE_CLOEXEC); setting the descriptor
        // flag works on every supported FreeBSD release.
        if unsafe { libc::fcntl(fd, libc::F_SETFD, libc::FD_CLOEXEC) } < 0 {
            let err = Errno::last();
            unsafe {
                libc::close(fd);
            }
            return Err(err);
        }
        Ok(Self {
            // SAFETY: kqueue returned a fresh descriptor whose ownership is
            // transferred here.
            fd: unsafe { LibcFd::from_raw(fd) },
            registered: None,
            timer_armed: Cell::new(false),
        })
    }

    pub(crate) fn raw(&self) -> c_int {
        self.fd.raw()
    }

    pub(crate) fn register_device(&mut self, fd: c_int, playback: bool) -> Result<(), Errno> {
        let filter = if playback {
            libc::EVFILT_WRITE
        } else {
            libc::EVFILT_READ
        };
        if self.registered == Some((fd, filter)) {
            return Ok(());
        }
        self.unregister_device()?;
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
        ))?;
        self.registered = Some((fd, filter));
        Ok(())
    }

    pub(crate) fn unregister_device(&mut self) -> Result<(), Errno> {
        let Some((fd, filter)) = self.registered else {
            return Ok(());
        };
        match self.submit_change(kevent(
            fd as libc::uintptr_t,
            filter,
            libc::EV_DELETE | libc::EV_RECEIPT,
            0,
            0,
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

    pub(crate) fn arm_timer(&self, delay_ns: u64) -> Result<(), Errno> {
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
            )) {
                Err(Errno::ENOENT) => Ok(()),
                result => result,
            };
            if result.is_ok() {
                self.timer_armed.set(false);
            }
            return result;
        }
        let result = self.submit_change(kevent(
            TIMER_IDENT,
            libc::EVFILT_TIMER,
            libc::EV_ADD | libc::EV_ONESHOT | libc::EV_RECEIPT,
            libc::NOTE_NSECONDS,
            delay_ns.min(i64::MAX as u64) as i64,
        ));
        if result.is_ok() {
            self.timer_armed.set(true);
        }
        result
    }

    pub(crate) fn next_event(&self) -> Result<Option<WakeEvent>, Errno> {
        // The device edge and its deadline watchdog intentionally share this
        // queue. They can become ready together, so consume both in one read
        // and prefer the enriched device snapshot. Leaving either event
        // queued would make the host start a second graph cycle immediately.
        let mut events = [kevent(0, 0, 0, 0, 0), kevent(0, 0, 0, 0, 0)];
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
        let (selected, timer_seen) = decode_events(events.into_iter().take(n as usize))?;
        if timer_seen {
            self.timer_armed.set(false);
        }
        Ok(selected)
    }

    fn submit_change(&self, change: libc::kevent) -> Result<(), Errno> {
        let mut receipt = kevent(0, 0, 0, 0, 0);
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

fn decode_events(
    events: impl IntoIterator<Item = libc::kevent>,
) -> Result<(Option<WakeEvent>, bool), Errno> {
    let mut selected = None;
    let mut first_error = None;
    let mut timer_seen = false;
    for event in events {
        match decode_event(event) {
            Ok(Some(WakeEvent::Timer)) => {
                timer_seen = true;
                selected.get_or_insert(WakeEvent::Timer);
            }
            Ok(Some(device @ WakeEvent::Device(_))) => selected = Some(device),
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

fn decode_event(event: libc::kevent) -> Result<Option<WakeEvent>, Errno> {
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
    let fd = c_int::try_from(event.ident).map_err(|_| Errno::EOVERFLOW)?;
    Ok(Some(WakeEvent::Device(DeviceEvent {
        fd,
        available_bytes: event.data.max(0).min(u32::MAX as i64) as u32,
        ready_frames: event.ext[0],
        xruns: event.ext[1].min(u32::MAX as u64) as u32,
        eof: event.flags & libc::EV_EOF != 0,
    })))
}

fn kevent(ident: libc::uintptr_t, filter: i16, flags: u16, fflags: u32, data: i64) -> libc::kevent {
    libc::kevent {
        ident,
        filter,
        flags,
        fflags,
        data,
        udata: std::ptr::null_mut(),
        ext: [0; 4],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_shot_timer_wakes_the_queue() {
        let queue = SoundKqueue::new().unwrap();
        queue.arm_timer(1).unwrap();
        let mut event = None;
        for _ in 0..100 {
            event = queue.next_event().unwrap();
            if event.is_some() {
                break;
            }
            std::thread::yield_now();
        }
        assert_eq!(event, Some(WakeEvent::Timer));
        assert_eq!(queue.next_event().unwrap(), None);
    }

    #[test]
    fn deleting_an_unarmed_timer_is_idempotent() {
        let queue = SoundKqueue::new().unwrap();
        queue.arm_timer(0).unwrap();
        queue.arm_timer(0).unwrap();
    }

    #[test]
    fn kqueue_descriptor_is_pollable_by_the_host_loop() {
        let queue = SoundKqueue::new().unwrap();
        queue.arm_timer(1).unwrap();
        let mut pfd = libc::pollfd {
            fd: queue.raw(),
            events: libc::POLLIN,
            revents: 0,
        };
        assert_eq!(unsafe { libc::poll(&mut pfd, 1, 100) }, 1);
        assert_ne!(pfd.revents & libc::POLLIN, 0);
        assert_eq!(queue.next_event().unwrap(), Some(WakeEvent::Timer));
    }

    #[test]
    fn device_registration_delivers_through_the_nested_queue() {
        let (read_fd, write_fd) = crate::oss::test_util::pipe_pair(true, true);
        let mut queue = SoundKqueue::new().unwrap();
        queue.register_device(write_fd, true).unwrap();

        let mut pfd = libc::pollfd {
            fd: queue.raw(),
            events: libc::POLLIN,
            revents: 0,
        };
        assert_eq!(unsafe { libc::poll(&mut pfd, 1, 100) }, 1);
        let Some(WakeEvent::Device(event)) = queue.next_event().unwrap() else {
            panic!("the writable descriptor should produce a device event");
        };
        assert_eq!(event.fd, write_fd);
        assert_eq!(queue.next_event().unwrap(), None);

        // EV_CLEAR does not re-deliver a still-writable level until the
        // underlying object changes, and registering the same knote is
        // intentionally idempotent. The deadline timer must remain able to
        // wake the driver in that no-new-edge state.
        queue.register_device(write_fd, true).unwrap();
        assert_eq!(queue.next_event().unwrap(), None);
        queue.arm_timer(1).unwrap();
        let mut watchdog = None;
        for _ in 0..100 {
            watchdog = queue.next_event().unwrap();
            if watchdog.is_some() {
                break;
            }
            std::thread::yield_now();
        }
        assert_eq!(watchdog, Some(WakeEvent::Timer));

        queue.unregister_device().unwrap();
        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }

    #[test]
    fn simultaneous_device_and_watchdog_wake_once() {
        let (read_fd, write_fd) = crate::oss::test_util::pipe_pair(true, true);
        let mut queue = SoundKqueue::new().unwrap();
        queue.register_device(write_fd, true).unwrap();
        queue.arm_timer(1).unwrap();

        // Keep both knotes pending until one kevent read can coalesce them.
        std::thread::sleep(std::time::Duration::from_millis(1));
        let Some(WakeEvent::Device(event)) = queue.next_event().unwrap() else {
            panic!("the device snapshot should win the watchdog race");
        };
        assert_eq!(event.fd, write_fd);
        assert!(!queue.timer_armed.get());
        assert_eq!(queue.next_event().unwrap(), None);

        queue.unregister_device().unwrap();
        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }

    #[test]
    fn device_snapshot_survives_a_sibling_decode_error() {
        let mut device = kevent(42, libc::EVFILT_WRITE, 0, 0, 8192);
        device.ext[0] = 768;
        device.ext[1] = 3;
        let error = kevent(0, libc::EVFILT_TIMER, libc::EV_ERROR, 0, libc::EIO as i64);

        for events in [[device, error], [error, device]] {
            let (selected, timer_seen) = decode_events(events).unwrap();
            assert!(!timer_seen);
            assert_eq!(
                selected,
                Some(WakeEvent::Device(DeviceEvent {
                    fd: 42,
                    available_bytes: 8192,
                    ready_frames: 768,
                    xruns: 3,
                    eof: false,
                }))
            );
        }
    }

    #[test]
    fn batch_decode_reports_an_error_without_a_usable_event() {
        let error = kevent(0, libc::EVFILT_TIMER, libc::EV_ERROR, 0, libc::EIO as i64);
        assert_eq!(decode_events([error]), Err(Errno::EIO));
    }

    #[test]
    fn unregister_tolerates_a_device_closed_by_teardown() {
        let (read_fd, write_fd) = crate::oss::test_util::pipe_pair(true, true);
        let mut queue = SoundKqueue::new().unwrap();
        queue.register_device(write_fd, true).unwrap();
        unsafe {
            libc::close(write_fd);
        }
        queue.unregister_device().unwrap();
        unsafe {
            libc::close(read_fd);
        }
    }

    #[test]
    fn enriched_device_fields_decode_as_one_snapshot() {
        let mut raw = kevent(42, libc::EVFILT_WRITE, libc::EV_EOF, 0, 8192);
        raw.ext[0] = 768;
        raw.ext[1] = 3;
        assert_eq!(
            decode_event(raw).unwrap(),
            Some(WakeEvent::Device(DeviceEvent {
                fd: 42,
                available_bytes: 8192,
                ready_frames: 768,
                xruns: 3,
                eof: true,
            }))
        );
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
