use super::commands::{PORT_IO_AREAS, io_area_ok};
use super::*;
use crate::spa::{self, SendWrap};

// PipeWire may drive process() from a different loop than the DataLoop that
// owns the timer and marshaled state. Refuse processing when their thread
// identities differ; users can pin node.loop.name to keep them together.
pub(super) fn check_loop_identity(gate: &DataThreadGate) -> bool {
    use std::sync::atomic::Ordering;
    let tid = unsafe { libc::pthread_self() } as usize;
    // Seed the expected id from a closure run on the data loop at init,
    // not claimed by whoever calls first: a pure follower never runs
    // on_wake, so a process() arriving on a divergent host loop would
    // otherwise install itself as the expected thread and undo the
    // block_on_loop serialization.
    let seen = gate.thread.load(Ordering::Acquire);
    if seen == tid {
        return true;
    }
    if seen != usize::MAX && gate.thread.swap(usize::MAX, Ordering::Relaxed) != usize::MAX {
        crate::warn!(
            gate.log,
            "process() and our data loop run on different threads \
      (multi-data-loop config?); pin node.loop.name for this node. Disabling processing."
        );
    }
    false
}

// Submit a dead-channel rebuild before device I/O. wake_cycle calls this as
// soon as EV_EOF arrives. The sticky EOF survives timer wakes, and process()
// retries whenever an earlier submission did not become pending; an in-flight
// rebuild owns the pending bit and must never be duplicated.
pub(super) fn queue_device_eof_rebuilds<D: Direction>(state: &mut DataState<D>) {
    for port_idx in 0..state.ports.len() {
        let port = &state.ports[port_idx];
        if port.device_eof && !port.rebuild_pending {
            queue_rebuild(state, port_idx);
        }
    }
}

fn discard_device_snapshots<D: Direction>(state: &mut DataState<D>) {
    for port in &mut state.ports {
        port.device_event = None;
    }
}

pub(super) unsafe extern "C" fn process<D: Direction>(object: *mut c_void) -> c_int {
    let root: *mut State<D> = object.cast();
    assert!(!root.is_null(), "object is not supposed to be null");
    // Reject a divergent process loop before projecting or borrowing DataState.
    let gate = unsafe { gate_ref(root) };
    if !check_loop_identity(gate) {
        return SPA_STATUS_OK as i32;
    }

    // Phase 1, under a scoped borrow: the data path. Xrun notifications are
    // collected only (detect_underrun/recover_overrun deposit them on the
    // port) so the C callback below runs with no DataState borrow live.
    // SAFETY: object is our State shell (the spa_interface data contract); the
    // borrow ends before any callback is invoked.
    let phase = unsafe {
        with_data_mut(root, |state| {
            // a cycle that was already signaled when we paused can still land here;
            // drop it instead of assert!()ing, which aborts the daemon across
            // extern "C"
            if !state.started || state.position.is_null() {
                // Keep sticky EOF/rebuild state, but never carry this wake's
                // fill/xrun snapshot into a later Start or IO configuration.
                discard_device_snapshots(state);
                return None;
            }

            // Usually already queued by wake_cycle; retry here if its earlier
            // submission did not become pending.
            queue_device_eof_rebuilds(state);

            let result = D::process_ports(state);
            // Timer-driven I/O can discover a dead descriptor without an
            // enriched EV_EOF event. Semantic stream outcomes latch the same
            // ownership transition during process_ports; submit it now.
            queue_device_eof_rebuilds(state);
            // The event is one pre-I/O snapshot shared by the servo and this
            // process pass. Never let a host-initiated second process() reuse
            // it after the device has moved on.
            discard_device_snapshots(state);
            // process() normally runs inline from ready(), whose return path
            // also selects the next wake. Do it here as well for hosts that
            // defer process(), including the fallback-timer arm when this
            // cycle suspended or replaced the registered device.
            if !state.ready_dispatching {
                super::timing::select_next_wake(state);
            }
            // collect-then-notify: drain the deposited xrun stamp with the hook copy
            let pending = state.ports.iter_mut().find_map(|p| p.pending_xrun.take());
            let main_event = state.pending_main_event.take().map(|event| {
                (
                    state.main_loop,
                    state.main_events.clone(),
                    state.log.clone(),
                    event,
                )
            });
            Some((
                result,
                pending.map(|t| (t, state.callbacks.hook())),
                main_event,
            ))
        })
    };
    let Some((result, xrun, main_event)) = phase else {
        return SPA_STATUS_OK as i32;
    };

    if let Some((trigger_us, Some((cb, data)))) = xrun
        && let Some(xrun_fun) = cb.xrun
    {
        // the xrun event for pw-top's counter; the length isn't known at
        // detection, so 0 delay. No State borrow is live here; sound per
        // NodeCallbacks::hook (validated copy, data valid while set).
        unsafe { xrun_fun(data, trigger_us, 0, std::ptr::null_mut()) };
    }

    if let Some((main_loop, target, log, event)) = main_event {
        // queue_task may execute inline. No DataState reference is live here, so
        // listener reentry through the endpoint is sound. Deliver after the
        // copied xrun hook: a listener may replace callbacks and invalidate
        // the old callback data pointer.
        queue_main_event(main_loop, target, log, event);
    }

    result
}

pub(super) unsafe extern "C" fn port_use_buffers<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    flags: u32,
    buffers: *mut *mut spa_buffer,
    n_buffers: u32,
) -> c_int {
    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");

    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }
    let _ = flags;

    let n_buffers = n_buffers as usize;
    if !spa::raw_slice_len_ok::<*mut spa_buffer>(n_buffers) {
        return -libc::EINVAL;
    }
    let new_buffers = if !buffers.is_null() && n_buffers > 0 {
        // the host passes n_buffers valid pointers; copied before the loop swap
        unsafe { std::slice::from_raw_parts(buffers, n_buffers) }.to_vec()
    } else {
        vec![]
    };

    // process() walks this vec on the data loop; swap it there.
    // SAFETY: the host keeps the buffer pointers valid until the next
    // use_buffers call (the port_use_buffers contract)
    let port_idx = port_id as usize;
    let new_buffers = unsafe { SendWrap::new(new_buffers) };
    let control = unsafe { DataControl::from_raw(state) };
    if !control.invoke(move |state| {
        state.ports[port_idx].buffers = new_buffers.into_inner();
        D::on_buffers_swapped(state, port_idx);
    }) {
        return -libc::EIO; // keeping stale host buffer pointers would be a UAF
    }

    0
}

pub(super) unsafe extern "C" fn port_set_io<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    data: *mut c_void,
    size: usize,
) -> c_int {
    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }

    let res = io_area_ok(&PORT_IO_AREAS, id, data, size);
    if res != 0 {
        return res;
    }

    let state = object.cast::<State<D>>();
    assert!(!state.is_null(), "object is not supposed to be null");

    // these pointers are dereferenced by process() on the data loop.
    // SAFETY: the host keeps the io areas valid while set (port_set_io
    // contract)
    let data = unsafe { SendWrap::new(data) };
    let control = unsafe { DataControl::from_raw(state) };
    let applied = control.invoke(move |state| {
        let data = data.into_inner();
        // SAFETY (both arms): size/alignment validated above; the host
        // keeps the area valid while set (the port_set_io contract)
        #[expect(non_upper_case_globals)]
        match id {
            SPA_IO_Buffers => unsafe { state.ports[port_id as usize].io.set(data) }, // null clears
            // ACTIVE is managed per cycle in process() and set only while
            // rate matching.
            SPA_IO_RateMatch => unsafe { state.ports[port_id as usize].rate_match.set(data) },
            _ => (),
        }
    });
    if !applied {
        return -libc::EIO;
    }

    0
}

pub(super) unsafe extern "C" fn port_reuse_buffer(
    _object: *mut c_void,
    _port_id: u32,
    _buffer_id: u32,
) -> c_int {
    -libc::ENOTSUP // buffers are recycled through io.buffer_id
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spa::Log;
    #[test]
    fn unseeded_data_loop_gate_never_falls_back_to_first_process_caller() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let unseeded = DataThreadGate {
            thread: AtomicUsize::new(0),
            log: Log::test_null(),
        };
        assert!(!check_loop_identity(&unseeded));
        assert_eq!(
            unseeded.thread.load(Ordering::Acquire),
            usize::MAX,
            "an unseeded gate is permanently disabled"
        );

        let current = unsafe { libc::pthread_self() } as usize;
        let seeded = DataThreadGate {
            thread: AtomicUsize::new(current),
            log: Log::test_null(),
        };
        assert!(check_loop_identity(&seeded));
    }
}
