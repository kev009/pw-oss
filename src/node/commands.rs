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

use super::*;

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
    let data = unsafe { crate::spa::SendWrap::new(data) };
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
                state
                    .clock
                    .with(|c| crate::node::set_clock_name(c, &state.clock_name));
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

pub(super) type ExtractedDevices<D> = [(usize, <D as Direction>::Device); MAX_PORTS];

pub(super) fn replace_port_devices<D: Direction>(
    ports: &mut [Port<D>; MAX_PORTS],
    devices: ExtractedDevices<D>,
) -> [D::Device; MAX_PORTS] {
    devices.map(|(index, device)| {
        ports[index].rebuild_pending = false;
        crate::node::reset_device_event(&mut ports[index]);
        std::mem::replace(&mut ports[index].dsp, device)
    })
}

// Return devices extracted by Suspend without transferring their ownership
// into the loop closure. If the invoke cannot run, the caller still owns them
// and can retry or release them only after the loop is known unavailable.
pub(super) fn restore_extracted_devices<D: Direction>(
    control: &DataControl<D>,
    devices: &mut Option<ExtractedDevices<D>>,
) -> Option<[D::Device; MAX_PORTS]> {
    devices.as_ref()?;
    control.query(|state| {
        let devices = devices
            .take()
            .expect("the caller retains extracted devices until this invoke runs");
        let placeholders = replace_port_devices(&mut state.ports, devices);
        state.rebuild_takeover = false;
        placeholders
    })
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
    let (log, shared, rebuild_work) = {
        let main = unsafe { main_ref(state) };
        (
            main.log.clone(),
            main.shared.clone(),
            main.rebuild_worker.endpoint(),
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
            // Device::new may probe sndstat (the sink does), so build every
            // closed placeholder here on main rather than inside DataControl.
            let dsp_path = unsafe { &main_ref(state).dsp_path };
            let placeholders: [D::Device; MAX_PORTS] =
                std::array::from_fn(|_| D::Device::new(dsp_path));
            // Quiesce and transfer device ownership out of DataState. Potentially
            // sleeping SETTRIGGER/close operations then run on this thread while
            // the data loop sees only closed placeholders.
            let Some((devices, deferred)) = control.query(move |state| {
                state.started = false;
                data_stopped_ref.store(true, std::sync::atomic::Ordering::Release);
                update_driver_wake(state);
                D::on_suspend_loop(state);
                state.rebuild_takeover = true;
                let deferred = state.deferred_work.take();
                let mut placeholders = placeholders.into_iter();
                let devices: [(usize, D::Device); MAX_PORTS] = std::array::from_fn(|index| {
                    let port = &mut state.ports[index];
                    port.rebuild_pending = true;
                    crate::node::reset_device_event(port);
                    port.generation = port.generation.wrapping_add(1);
                    state
                        .shared
                        .generation
                        .store(port.generation, std::sync::atomic::Ordering::Release);
                    let placeholder = placeholders
                        .next()
                        .expect("one prebuilt placeholder per port");
                    (index, std::mem::replace(&mut port.dsp, placeholder))
                });
                (devices, deferred)
            }) else {
                restore_started_if_stop_unobserved(&shared.started, was_started, &data_stopped);
                return -libc::EIO;
            };
            let mut devices = Some(devices);
            drop(deferred);
            // a deposited-but-unconsumed rebuild would hold an open
            // (possibly exclusive) device across the whole suspended stretch
            // (nothing polls while stopped); close it now, off the RT path.
            shared.discard_swap();
            if !rebuild_work.wait_idle() {
                let mut placeholders = restore_extracted_devices(&control, &mut devices);
                if placeholders.is_none() && devices.is_some() {
                    placeholders = restore_extracted_devices(&control, &mut devices);
                }
                if placeholders.is_none() && devices.is_some() {
                    crate::warn!(
                        log,
                        "can't restore devices after rebuild worker shutdown: data loop is unavailable"
                    );
                }
                drop(placeholders);
                return -libc::EIO;
            }
            shared.discard_swap();
            for (_, dsp) in devices
                .as_mut()
                .expect("Suspend retains the extracted devices until restoration")
            {
                if !dsp.is_closed() && !dsp.suspend() {
                    dsp.close();
                }
            }
            let placeholders = restore_extracted_devices(&control, &mut devices);
            let Some(placeholders) = placeholders else {
                // The first invoke retained ownership on failure. Retry once
                // so a transient handoff error does not release live or
                // suspended descriptors while placeholders remain installed.
                let restored = devices.is_none()
                    || restore_extracted_devices(&control, &mut devices).is_some();
                if !restored {
                    crate::warn!(
                        log,
                        "can't restore suspended devices: data loop is unavailable"
                    );
                }
                return -libc::EIO;
            };
            // Closed placeholders still own heap fields; destroy them on main.
            drop(placeholders);
            0
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
    use super::*;
    use crate::node::sink::SinkDir;
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
            io: crate::spa::IoArea::null(),
            rate_match: crate::spa::IoArea::null(),
            dsp: crate::oss::DspWriter::test_on_fd(fd, 8),
            dll: Default::default(),
            setup_period: 0,
            bw_adapt: Default::default(),
            setup_blocksize: 0,
            rebuild_pending: false,
            generation: 0,
            was_matching: false,
            warn_limit: crate::node::RateLimit::new(),
            pending_xrun: None,
            device_event: None,
            device_eof: false,
            event_xruns_seen: 0,
            wake_threshold: 0,
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
    fn extracted_devices_restore_ports_and_clear_pending_claims() {
        let (old_read, old_write) = crate::oss::test_util::pipe_pair(true, true);
        let (new_read, new_write) = crate::oss::test_util::pipe_pair(true, true);
        let mut ports = [test_port(old_write)];
        ports[0].rebuild_pending = true;

        let devices = [(0, crate::oss::DspWriter::test_on_fd(new_write, 8))];
        let mut placeholders = replace_port_devices(&mut ports, devices);

        assert!(!ports[0].rebuild_pending);
        assert_eq!(ports[0].dsp.write(&[2; 8]).bytes, 8);
        assert_eq!(placeholders[0].write(&[1; 8]).bytes, 8);
        assert_eq!(crate::oss::test_util::drain(new_read), [2; 8]);
        assert_eq!(crate::oss::test_util::drain(old_read), [1; 8]);

        unsafe {
            libc::close(old_read);
            libc::close(new_read);
        }
    }
}
