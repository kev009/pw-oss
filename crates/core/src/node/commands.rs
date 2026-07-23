pub(crate) fn spa_command_to_str(body: &libspa::sys::spa_pod_object_body) -> &'static str {
    use libspa::sys::*;
    #[expect(non_upper_case_globals)]
    match (body.type_, body.id) {
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start) => "SPA_NODE_COMMAND_Start",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => "SPA_NODE_COMMAND_Suspend",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => "SPA_NODE_COMMAND_Pause",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin) => "SPA_NODE_COMMAND_ParamBegin",
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamEnd) => "SPA_NODE_COMMAND_ParamEnd",
        (SPA_TYPE_COMMAND_Node, _) => "SPA_NODE_COMMAND_???",
        _ => "???",
    }
}

use super::ports::publish_format_state;
use super::*;
use crate::backend::StreamLifecycle as _;
use crate::spa::SendWrap;

// the io areas set_io accepts, with the geometry a full deref needs
pub(super) const NODE_IO_AREAS: [(u32, usize, usize); 2] = [
    (
        SPA_IO_Clock,
        size_of::<spa_io_clock>(),
        align_of::<spa_io_clock>(),
    ),
    (
        SPA_IO_Position,
        size_of::<spa_io_position>(),
        align_of::<spa_io_position>(),
    ),
];

// ditto for port_set_io
pub(super) const PORT_IO_AREAS: [(u32, usize, usize); 2] = [
    (
        SPA_IO_Buffers,
        size_of::<spa_io_buffers>(),
        align_of::<spa_io_buffers>(),
    ),
    (
        SPA_IO_RateMatch,
        size_of::<spa_io_rate_match>(),
        align_of::<spa_io_rate_match>(),
    ),
];

// The io-area admission shared by set_io and port_set_io: an id outside the
// caller's table is -ENOENT; NULL/0 clears the area; a non-empty area must
// admit a full deref of the struct. A short one is -ENOSPC - the installed
// header specifies it for both entry points ("-ENOSPC when \a size is too
// small", spa/node/node.h set_io and port_set_io) - while a misaligned one
// stays the generic invalid-argument -EINVAL (no closer errno is specified).
pub(super) fn io_area_ok(
    table: &[(u32, usize, usize)],
    id: u32,
    data: *const c_void,
    size: usize,
) -> c_int {
    let Some(&(_, min_size, align)) = table.iter().find(|(t, _, _)| *t == id) else {
        return -libc::ENOENT;
    };
    if !data.is_null() {
        if size < min_size {
            return -libc::ENOSPC;
        }
        if !data.addr().is_multiple_of(align) {
            return -libc::EINVAL;
        }
    }
    0
}

pub(super) unsafe extern "C" fn set_io<D: Direction>(
    object: *mut c_void,
    id: u32,
    data: *mut c_void,
    size: usize,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null());

    let res = io_area_ok(&NODE_IO_AREAS, id, data, size);
    if res != 0 {
        return res;
    }

    // clock/position are read on the data loop; apply the change there.
    // SAFETY: the host keeps the io areas valid while set (set_io contract)
    let data = unsafe { SendWrap::new(data) };
    let control = unsafe { DataControl::from_raw(state) };
    let applied = control.invoke(move |state| {
        let data = data.into_inner();
        let was_armed = !state.clock.is_null() && !state.position.is_null();

        #[expect(non_upper_case_globals)]
        match id {
            SPA_IO_Clock => {
                // SAFETY: size/alignment validated above; the host keeps
                // the area valid while set (the set_io contract)
                unsafe { state.clock.set(data) }; // null clears

                // identify our clock so same-device followers can skip rate matching
                state.clock.with(|c| set_clock_name(c, &state.clock_name));
            }
            // SAFETY: as above
            SPA_IO_Position => unsafe { state.position.set(data) }, // null clears
            _ => (),                                                // filtered above
        };

        if state.started {
            let armed = !state.clock.is_null() && !state.position.is_null();
            let following = state.node_is_follower();
            let flipped = state.following != following;
            if flipped {
                state.following = following;
                D::on_role_flip(state);
            }
            // rearm/park only on a real transition (io presence or role); resetting
            // the timer phase on every re-point causes cycle bunching
            if flipped || was_armed != armed {
                update_driver_wake(state);
            }
        }
    });
    if !applied {
        return -libc::EIO;
    }

    0
}

// Suspend is stronger than Pause: SPA requires it to remove all negotiated
// formats and close the devices. Do the pointer-bearing teardown on the data
// loop while swapping the live descriptor behind a closed placeholder. The
// caller owns and closes the returned device on the non-RT main thread.
pub(super) fn take_suspended_device<D: Direction>(
    port: &mut Port<D>,
    placeholder: D::Device,
) -> D::Device {
    let retired = std::mem::replace(&mut port.dsp, placeholder);
    reset_stream_epoch(port);
    port.buffers.clear();
    port.config = None;
    port.setup_period = 0;
    port.delivery_quantum_bytes = 0;
    port.dll.init();
    port.bw_adapt.reset();
    port.was_matching = false;
    port.pending_xrun = None;
    port.generation = port.generation.wrapping_add(1);
    port.rebuild_pending = true;
    retired
}

pub(super) fn restore_started_if_stop_unobserved(
    started: &std::sync::atomic::AtomicBool,
    was_started: bool,
    data_stopped: &std::sync::atomic::AtomicBool,
) {
    if !data_stopped.load(std::sync::atomic::Ordering::Acquire) {
        started.store(was_started, std::sync::atomic::Ordering::Release);
    }
}

pub(super) unsafe extern "C" fn send_command<D: Direction>(
    object: *mut c_void,
    command: *const spa_command,
) -> c_int {
    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");
    let control = unsafe { DataControl::from_raw(state) };
    let (log, shared, rebuild_work, events) = {
        let main = unsafe { main_ref(state) };
        (
            main.log.clone(),
            main.shared.clone(),
            main.rebuild_worker.endpoint(),
            main.events.clone(),
        )
    };

    assert!(!command.is_null());
    let body = unsafe { (*command).body.body };

    crate::debug!(log, "received command: {}", spa_command_to_str(&body));

    #[expect(non_upper_case_globals)]
    match (body.type_, body.id) {
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Start) => {
            let started = control.query(|state| {
                if state
                    .ports
                    .iter()
                    .any(|p| p.config.is_none() || p.buffers.is_empty())
                {
                    return false;
                }
                // sane clock delay/rate_diff until process() publishes measured values
                state.clock.with(|c| {
                    c.delay = 0;
                    c.rate_diff = 1.0;
                });
                D::on_start_loop(state);
                state.started = true;
                state.following = state.node_is_follower();
                update_driver_wake(state);
                true
            });
            match started {
                Some(true) => (),
                Some(false) => {
                    crate::warn!(log, "can't start: ports are not negotiated");
                    return -libc::EIO;
                }
                None => {
                    crate::warn!(log, "can't start: data loop did not accept the command");
                    return -libc::EIO;
                }
            };
            // Publish only after DataState is fully started. The worker
            // pairs this Release with its pre/post-open Acquire checks.
            shared
                .started
                .store(true, std::sync::atomic::Ordering::Release);
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Pause) => {
            // Publish the stop before the blocking data-loop handoff. A
            // worker open finishing in that window must retire its result,
            // not hand an already-paused node a fresh exclusive fd.
            let was_started = shared
                .started
                .swap(false, std::sync::atomic::Ordering::AcqRel);
            let data_stopped = std::sync::atomic::AtomicBool::new(false);
            let Some(deferred) = control.query(|state| {
                let data_was_started = state.started;
                state.started = false;
                if data_was_started {
                    D::on_pause_loop(state);
                }
                data_stopped.store(true, std::sync::atomic::Ordering::Release);
                update_driver_wake(state);
                state.rebuild_takeover = true;
                let deferred = state.deferred_work.take();
                for port in &mut state.ports {
                    port.rebuild_pending = true;
                    port.generation = port.generation.wrapping_add(1);
                    state
                        .shared
                        .generation
                        .store(port.generation, std::sync::atomic::Ordering::Release);
                }
                deferred
            }) else {
                restore_started_if_stop_unobserved(&shared.started, was_started, &data_stopped);
                return -libc::EIO;
            };
            drop(deferred);
            // Catch both a completion deposited before the fence and one
            // from a worker that passed its final check just before it.
            shared.discard_swap();
            if !rebuild_work.wait_idle() {
                release_rebuild_takeover(&control, 0);
                return -libc::EIO;
            }
            shared.discard_swap();
            if !release_rebuild_takeover(&control, 0) {
                return -libc::EIO;
            }
            0
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_Suspend) => {
            // As with Pause, stop wins before waiting for the data loop.
            let was_started = shared
                .started
                .swap(false, std::sync::atomic::Ordering::AcqRel);
            let data_stopped = std::sync::atomic::AtomicBool::new(false);
            let data_stopped_ref = &data_stopped;
            // Device::new may perform discovery, so build every closed
            // placeholder here on main rather than inside DataControl.
            let stream_path = unsafe { &main_ref(state).stream_path };
            let placeholders: [D::Device; MAX_PORTS] =
                std::array::from_fn(|_| D::Device::new(stream_path));
            // Quiesce, unconfigure, and transfer device ownership out of
            // DataState. Potentially sleeping closes then run on this thread
            // while the data loop sees only closed placeholders.
            let Some((devices, deferred)) = control.query(move |state| {
                state.started = false;
                data_stopped_ref.store(true, std::sync::atomic::Ordering::Release);
                super::timing::invalidate_device_wake(state);
                update_driver_wake(state);
                D::on_suspend_loop(state);
                state.rebuild_takeover = true;
                let deferred = state.deferred_work.take();
                let mut placeholders = placeholders.into_iter();
                let devices: [D::Device; MAX_PORTS] = std::array::from_fn(|index| {
                    let placeholder = placeholders
                        .next()
                        .expect("one prebuilt placeholder per port");
                    let retired = take_suspended_device(&mut state.ports[index], placeholder);
                    state.shared.generation.store(
                        state.ports[index].generation,
                        std::sync::atomic::Ordering::Release,
                    );
                    D::on_device_swapped(state, index);
                    retired
                });
                (devices, deferred)
            }) else {
                restore_started_if_stop_unobserved(&shared.started, was_started, &data_stopped);
                return -libc::EIO;
            };
            drop(deferred);
            // a deposited-but-unconsumed rebuild would hold an open
            // (possibly exclusive) device across the whole suspended stretch
            // (nothing polls while stopped); close it now, off the RT path.
            shared.discard_swap();
            let worker_idle = rebuild_work.wait_idle();
            shared.discard_swap();
            for mut dsp in devices {
                // SPA Suspend closes the device; a native trigger stop is
                // only the weaker Pause operation.
                if !dsp.is_closed() {
                    dsp.close();
                }
            }
            let takeover_released = release_rebuild_takeover(&control, 0);
            // Advertise the transition only after the old descriptor is gone.
            // Keep the callback boundary last: listeners may synchronously
            // re-enter or destroy the node.
            {
                let main = unsafe { main_ref(state) };
                publish_format_state(main, None);
            }
            unsafe { events.flush() };
            if worker_idle && takeover_released {
                0
            } else {
                -libc::EIO
            }
        }
        (SPA_TYPE_COMMAND_Node, SPA_NODE_COMMAND_ParamBegin | SPA_NODE_COMMAND_ParamEnd) => 0, // we don't care
        (cmd_type, cmd_id) => {
            crate::warn!(
                log,
                "{}unknown command: {}, {}",
                D::CMD_WARN_PREFIX,
                cmd_type,
                cmd_id
            );
            -libc::ENOTSUP
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::sink::SinkDir as GenericSinkDir;
    use super::*;
    use crate::backend::{self, fake::FakeBackend};
    use crate::spa::IoArea;

    type SinkDir = GenericSinkDir<FakeBackend>;
    // an aligned backing store for the admission tests (every io struct's
    // alignment divides 16)
    #[repr(align(16))]
    struct Aligned([u8; 4096]);

    #[test]
    fn io_area_admission_null_short_exact_misaligned() {
        let mut area = Aligned([0; 4096]);
        let p = area.0.as_mut_ptr().cast::<c_void>();
        let full = size_of::<spa_io_clock>();

        // NULL/0 clears whatever the size says
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, std::ptr::null(), 0),
            0
        );
        // exact and oversized areas are admitted
        assert_eq!(io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full), 0);
        assert_eq!(io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full + 8), 0);
        // a non-empty-but-short area is -ENOSPC (the header's "size is too
        // small" errno for set_io/port_set_io)
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, p, full - 1),
            -libc::ENOSPC
        );
        // a misaligned one is the generic -EINVAL
        let off = unsafe { p.cast::<u8>().add(1) }.cast::<c_void>();
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Clock, off, full),
            -libc::EINVAL
        );
        // ids outside the caller's table are -ENOENT (set_io does not take
        // the port areas and vice versa)
        assert_eq!(
            io_area_ok(&NODE_IO_AREAS, SPA_IO_Buffers, p, full),
            -libc::ENOENT
        );
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_Clock, p, full),
            -libc::ENOENT
        );
        // the port table's own areas admit the same policy
        let bsize = size_of::<spa_io_buffers>();
        assert_eq!(io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, p, bsize), 0);
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, p, bsize - 1),
            -libc::ENOSPC
        );
        // a short AND misaligned area reports the size problem first: the
        // host's remedy (grow the area) subsumes re-placing it
        let off = unsafe { p.cast::<u8>().add(1) }.cast::<c_void>();
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_Buffers, off, bsize - 1),
            -libc::ENOSPC
        );
        assert_eq!(
            io_area_ok(&PORT_IO_AREAS, SPA_IO_RateMatch, std::ptr::null(), 0),
            0
        );
    }
    fn test_port(fd: c_int) -> Port<SinkDir> {
        Port {
            config: None,
            buffers: vec![],
            io: IoArea::null(),
            rate_match: IoArea::null(),
            dsp: backend::fake::FakeStream::test_on_fd(fd, 8),
            dll: Default::default(),
            setup_period: 0,
            bw_adapt: Default::default(),
            delivery_quantum_bytes: 0,
            rebuild_pending: false,
            generation: 0,
            stream_token: backend::StreamToken::for_port(0),
            was_matching: false,
            warn_limit: RateLimit::new(),
            pending_xrun: None,
            stream_wake: None,
            rebuild_required: false,
            xrun_tracker: backend::XrunTracker::default(),
            ext: Default::default(),
        }
    }

    #[test]
    fn failed_stop_handoff_restores_the_worker_gate_until_acknowledged() {
        use std::sync::atomic::{AtomicBool, Ordering};

        let worker_started = AtomicBool::new(false);
        let data_stopped = AtomicBool::new(false);
        restore_started_if_stop_unobserved(&worker_started, true, &data_stopped);
        assert!(worker_started.load(Ordering::Acquire));

        worker_started.store(false, Ordering::Release);
        data_stopped.store(true, Ordering::Release);
        restore_started_if_stop_unobserved(&worker_started, true, &data_stopped);
        assert!(
            !worker_started.load(Ordering::Acquire),
            "an acknowledged data-loop stop must remain published"
        );
    }

    #[test]
    fn suspend_port_teardown_clears_negotiation_and_returns_live_device() {
        let (old_read, old_write) = backend::test_transport::pipe_pair(true, true);
        let mut port = test_port(old_write);
        port.config = Some(backend::StreamConfig {
            format: libspa::param::audio::AudioFormat(SPA_AUDIO_FORMAT_S16_LE),
            rate: 48_000,
            channels: 2,
            positions: vec![SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR],
            flags: 0,
            stride: 4,
        });
        port.buffers.push(std::ptr::dangling_mut());
        port.setup_period = 1_920;
        port.delivery_quantum_bytes = 1_024;
        port.was_matching = true;
        port.pending_xrun = Some(PendingXrun {
            trigger_us: 1,
            delay_us: 2,
            quality: None,
        });

        let placeholder = backend::fake::FakeStream::new("closed");
        let mut retired = take_suspended_device(&mut port, placeholder);

        assert!(port.config.is_none());
        assert!(port.buffers.is_empty());
        assert_eq!(port.setup_period, 0);
        assert_eq!(port.delivery_quantum_bytes, 0);
        assert!(!port.was_matching);
        assert!(port.pending_xrun.is_none());
        assert!(port.rebuild_pending);
        assert_eq!(port.generation, 1);
        assert_eq!(retired.write(&[1; 8]).bytes, 8);
        assert_eq!(backend::test_transport::drain(old_read), [1; 8]);
        retired.close();

        unsafe {
            libc::close(old_read);
        }
    }
}
