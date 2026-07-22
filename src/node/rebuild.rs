use super::events::emit_node_info;
use super::*;

mod worker;

use worker::{flush_deferred_work, submit_or_defer};

pub(in crate::node) use worker::{
    MainEvent, RebuildWork, RebuildWorkSlot, RebuildWorker, SwapOutcome,
};
pub(crate) use worker::{NodeShared, queue_rebuild};

// The oss.* tunable live re-apply path: store the new loop-owned value on the
// data loop (the prime paths read it there), then rebuild any running port
// from this (main) thread so the next cycle re-primes with the new layout.
// Synchronous contract (see install_device): main-thread entry, blocking
// frame-bounded invokes only.
pub(crate) fn store_and_rebuild<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    store: impl FnOnce(&mut DataState<D>) + Send,
) -> c_int {
    let configs: Option<[Option<PortConfig>; MAX_PORTS]> = data.query(move |data| {
        store(data);
        std::array::from_fn(|i| {
            data.ports[i]
                .dsp
                .is_running()
                .then(|| data.ports[i].config.clone())
                .flatten()
        })
    });
    let Some(configs) = configs else {
        return -libc::EIO;
    };
    for (port_idx, config) in configs.into_iter().enumerate() {
        if let Some(config) = config
            && install_device(state, data, port_idx, config) != 0
        {
            // the host didn't initiate this rebuild; without a re-announce it
            // keeps believing a format is set on a dead port
            emit_format_lost(state);
        }
    }
    0
}

// announce a Props change (so readback stays fresh), then apply it through
// store_and_rebuild; shared by the oss.* prop appliers of both directions
pub(crate) fn apply_props_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    store: impl FnOnce(&mut DataState<D>) + Send,
) -> c_int {
    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Props);
    });
    emit_node_info(state);
    store_and_rebuild(state, data, store)
}

// Release the synchronous rebuild lease after an error. A dead loop cannot
// observe the retained flag, so failure remains best-effort.
pub(in crate::node) fn release_rebuild_takeover<D: Direction>(
    data: &DataControl<D>,
    port_idx: usize,
) -> bool {
    data.invoke(move |state| {
        state.rebuild_takeover = false;
        state.ports[port_idx].rebuild_pending = false;
    })
}

// Open and configure on the main thread because device operations may block,
// then swap the device on the data loop. The takeover fence invalidates
// asynchronous rebuilds while the synchronous install is active. On EBUSY,
// retire the current exclusive device and retry; other failures leave the
// port cleared.
pub(in crate::node) fn install_device<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
    config: PortConfig,
) -> c_int {
    // Acquire the sole-producer takeover on the data loop before waiting.
    // The generation bump makes both queued and active rebuilds stale;
    // rebuild_takeover makes later cycles skip without consuming or
    // submitting until the final swap releases the lease.
    let Some(deferred) = data.query(move |data| {
        debug_assert!(!data.rebuild_takeover, "synchronous installs serialize");
        data.rebuild_takeover = true;
        let deferred = data.deferred_work.take();
        let port = &mut data.ports[port_idx];
        port.rebuild_pending = true;
        port.generation = port.generation.wrapping_add(1);
        data.shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        if data.started {
            update_driver_wake(data);
        }
        deferred
    }) else {
        return -libc::EIO;
    };
    // Any retained RetireAndRetry/device ownership now dies here, never on
    // the data loop.
    drop(deferred);

    // Close a completion that predates the fence, then wait until an active
    // command observes the generation change and finishes. The second drain
    // catches the completion it may have deposited before becoming idle.
    state.shared.discard_swap();
    if !state.rebuild_worker.wait_idle() {
        release_rebuild_takeover(data, port_idx);
        return -libc::EIO;
    }
    state.shared.discard_swap();

    let mut new_dsp = D::Device::new(&state.stream_path);
    // fragment_bytes only mutates from main-thread calls, serialized with us
    let mut res = D::try_open_configure(&mut new_dsp, &config, state.fragment_bytes, &state.log);

    if res == -libc::EBUSY {
        let closed = D::Device::new(&state.stream_path);
        let Some(retired) =
            data.query(move |state| std::mem::replace(&mut state.ports[port_idx].dsp, closed))
        else {
            release_rebuild_takeover(data, port_idx);
            return -libc::EIO;
        };
        drop(retired); // closes the old fd here, off the RT path
        res = D::try_open_configure(&mut new_dsp, &config, state.fragment_bytes, &state.log);
    }

    let ok = res == 0;
    let cap_config = config.clone();
    let old_dsp = data.query(move |state| {
        crate::node::timing::invalidate_device_wake(state);
        let port = &mut state.ports[port_idx];
        // new_dsp is a closed writer/reader when negotiation failed above
        let old = std::mem::replace(&mut port.dsp, new_dsp);
        port.config = if ok { Some(config) } else { None };
        // Retire any in-flight background rebuild.
        port.generation = port.generation.wrapping_add(1);
        state
            .shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        port.rebuild_pending = false;
        port.was_matching = false; // force a relock when matching resumes
        D::on_device_swapped(state, port_idx);
        if state.started {
            update_driver_wake(state);
        }
        state.rebuild_takeover = false;
        old
    });
    let swapped = old_dsp.is_some();
    drop(old_dsp); // ditto

    if !swapped {
        release_rebuild_takeover(data, port_idx);
        return -libc::EIO; // the swap never ran; the port keeps its old state
    }
    if res == 0 {
        publish_ring_quantum_cap(state, &cap_config); // stride is known now
    }
    res
}

// FreeBSD caps every soft ring at CHN_2NDBUFMAXSIZE (131 KiB); at fat strides
// (a 20-channel S32 interface is 80 bytes/frame) the ring holds only ~1.6
// periods at quantum 1024 and both directions glitch structurally - the
// capture side has no room for arrival jitter, the playback side can't hold
// two quanta plus the delay target. Publish node.max-latency once the stride
// is known so the graph never negotiates a quantum the kernel ring can't hold
// four of (pw_impl_node parses the fraction into max_latency, which caps the
// driver quantum). Emitted only when the cap bites below the common
// 2048-frame default in TIME, at a conservative 44.1 kHz reference -
// clock.rate is unknown here and an over-published cap is inert (node::format
// advertised_quantum_cap_frames); published once -
// the props dict is append-only, and a stride change without a node rebuild
// is not worth a duplicate entry.
fn publish_ring_quantum_cap<D: Direction>(state: &mut MainState<D>, config: &PortConfig) {
    let stride = config.stride.max(1);
    let rate = config.rate;
    // the shared ring policy (node::format); the published fraction is time-based
    // (frames/device rate), so it needs no graph-rate scaling
    let Some(frames) = crate::oss::advertised_quantum_cap_frames(stride, rate) else {
        return;
    };
    if state.ring_cap_published {
        return;
    }
    state.ring_cap_published = true;
    crate::info!(
        state.log,
        "kernel ring ({} bytes) at stride {} holds 4 periods only up to \
    quantum {}; publishing node.max-latency",
        crate::oss::buffer_capacity_limit(stride, rate),
        stride,
        frames
    );
    state.events.with_node_info(|info| {
        let _ = info.replace_change_mask(0);
        info.add_prop("node.max-latency", format!("{frames}/{rate}"));
    });
    emit_node_info(state);
}

// Announce a failed background rebuild so the session manager renegotiates
// the cleared format.
fn emit_format_lost<D: Direction>(state: &MainState<D>) {
    state.events.record_current_format_lost();
}

// Asynchronous rebuilds carry owned requests from the data loop to the
// blocking-I/O worker. The worker never accesses State: it returns an owned
// DeviceSwap through NodeShared, and the data loop accepts it only when the
// port generation still matches. Retired devices also move to the worker so
// potentially blocking closes stay off the real-time path.

// The completion mailbox has one slot; multi-port support requires one slot
// per port.

// Deliver endpoint-only work after process() has ended its State phase. The
// queued closure carries MainEventTarget and an owned message; no State pointer
// crosses loops. A non-blocking invoke may execute inline on a single-loop host,
// so callers collect the event first and call here only after dropping every
// State reference.
pub(in crate::node) fn queue_main_event<D: Direction>(
    main_loop: Option<crate::spa::Loop>,
    target: MainEventTarget<D>,
    log: crate::spa::Log,
    event: MainEvent,
) {
    let Some(main_loop) = main_loop else {
        return;
    };
    // SAFETY: host loops outlive the queued item (queue_task's contract)
    let queued = unsafe {
        crate::spa::queue_task(&main_loop, move || {
            // SAFETY: queue_task invokes this closure through `main_loop`.
            target.deliver_on_main(event);
        })
    };
    if !queued {
        // emission lost: the node stays format-less until the host
        // renegotiates on its own; nothing dangles
        crate::warn!(
            log,
            "can't deliver a deferred node event (main loop unavailable)"
        );
    }
}

// At the start of each data-loop cycle, apply
// a deposited rebuild completion. A matching generation applies it; a stale
// one (superseded by install/release/Suspend) is retired to the worker for
// closing. Returns whether the cycle must skip the port (rebuild still
// in flight, or this cycle consumed a non-install outcome).
#[inline]
pub(crate) fn poll_rebuild<D: Direction>(state: &mut DataState<D>, port_idx: usize) -> bool {
    if state.rebuild_takeover {
        return true;
    }
    if !flush_deferred_work(state) {
        return true;
    }
    // The completion slot is empty on virtually every audio cycle. Check it
    // with a plain load so the steady-state path does not issue a locked CAS.
    // A concurrent deposit missed here is consumed next cycle, just as when
    // take_swap loses its CAS to a writer holding SLOT_BUSY.
    if !state.shared.swap_ready() {
        return state.ports[port_idx].rebuild_pending;
    }
    poll_rebuild_completion(state, port_idx)
}

// Keep the large owned completion variants and their drop paths out of the
// ordinary audio-cycle frame.
#[cold]
#[inline(never)]
fn poll_rebuild_completion<D: Direction>(state: &mut DataState<D>, port_idx: usize) -> bool {
    let Some(swap) = state.shared.take_swap() else {
        return state.ports[port_idx].rebuild_pending;
    };
    debug_assert_eq!(
        swap.port_idx, port_idx,
        "single mailbox slot: MAX_PORTS == 1"
    );
    if swap.generation != state.ports[port_idx].generation {
        // superseded; the payload may hold an open device - transfer the
        // whole owned message to the blocking-I/O worker.
        submit_or_defer(state, RebuildWork::RetireSwap(swap));
        return state.ports[port_idx].rebuild_pending;
    }
    match swap.outcome {
        SwapOutcome::Installed { dsp, config } => {
            crate::node::timing::invalidate_device_wake(state);
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, dsp);
            port.config = Some(config);
            port.generation = port.generation.wrapping_add(1);
            state
                .shared
                .generation
                .store(port.generation, std::sync::atomic::Ordering::Release);
            port.rebuild_pending = false;
            port.was_matching = false; // force a relock when matching resumes
            D::on_device_swapped(state, port_idx);
            if state.started {
                update_driver_wake(state);
            }
            crate::info!(
                state.log,
                "{}: background device rebuild applied",
                state.stream_path
            );
            submit_or_defer(state, RebuildWork::RetireDevice(old));
            false // the cycle continues on the fresh device (prime re-runs)
        }
        SwapOutcome::Aborted => {
            // stopped when the task ran; drop the claim so the next cycle
            // (running again, or it wouldn't poll) can re-queue
            state.ports[port_idx].rebuild_pending = false;
            true
        }
        SwapOutcome::RetireAndRetry {
            mut request,
            placeholder,
        } => {
            // swap the dying fd out behind a closed placeholder so the
            // retry's open can succeed on an exclusive device; it rides the
            // request as retire_first, so close-then-retry runs as one
            // worker command (ordering holds) under the task's unwind guard
            let port = &mut state.ports[port_idx];
            crate::node::reset_device_event(port);
            let old = std::mem::replace(&mut port.dsp, placeholder);
            request.retire_first = Some(old);
            if state.started {
                update_driver_wake(state);
            }
            submit_or_defer(state, RebuildWork::Rebuild(request));
            true
        }
        SwapOutcome::Failed { placeholder } => {
            // mirror install_device's failure shape: closed device, cleared
            // config, and a re-announce so the host renegotiates instead of
            // believing a format is set on a dead port
            crate::node::timing::invalidate_device_wake(state);
            let port = &mut state.ports[port_idx];
            let old = std::mem::replace(&mut port.dsp, placeholder);
            port.config = None;
            port.generation = port.generation.wrapping_add(1);
            state
                .shared
                .generation
                .store(port.generation, std::sync::atomic::Ordering::Release);
            port.rebuild_pending = false;
            port.was_matching = false;
            D::on_device_swapped(state, port_idx);
            if state.started {
                update_driver_wake(state);
            }
            submit_or_defer(state, RebuildWork::RetireDevice(old));
            // process() extracts and queues this only after its &mut DataState
            // phase ends. The endpoint epoch prevents an old loss from
            // overwriting a newer successful format publication.
            state.pending_main_event = Some(MainEvent::FormatLost {
                expected_publication_epoch: state.format_publication.epoch(),
            });
            true
        }
    }
}
