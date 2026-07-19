use super::*;

pub(super) unsafe extern "C" fn get_interface<D: Direction>(
    handle: *mut spa_handle,
    type_: *const c_char,
    interface: *mut *mut c_void,
) -> c_int {
    let state = handle.cast::<State<D>>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    assert!(!interface.is_null());
    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Node.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe {
            *interface = std::ptr::addr_of_mut!((*state).node).cast::<c_void>();
        }
    } else {
        return -libc::ENOENT;
    }
    0
}

pub(super) unsafe extern "C" fn clear<D: Direction>(handle: *mut spa_handle) -> c_int {
    let state: *mut State<D> = handle.cast();
    assert!(!state.is_null());

    // Queued tasks own only messages and MainEventTarget, never State. close()
    // revokes deliveries before State drops its Rc<NodeEvents>; a delivery
    // already running holds its own main-thread Rc through listener callbacks.
    // The host must still stop driving the node before clear (Suspend/Pause and
    // io teardown precede it in the SPA lifecycle). A host that calls
    // process()/on_timeout() afterward frees the ground under the data loop;
    // timer detachment below is our side of the contract.
    {
        let main = unsafe { main_mut(state) };
        // Win every open/configure race before asking the worker to stop.
        // stop() drains device-bearing commands on that thread and joins it,
        // so no blocking device destructor remains concurrent with teardown.
        main.shared
            .started
            .store(false, std::sync::atomic::Ordering::Release);
        main.shared.close();
        main.rebuild_worker.stop();
        // A final worker completion may own a device; destroy it here on the
        // main thread, after the worker can no longer deposit another one.
        main.shared.discard_swap();
    }

    // the data loop still holds the timer source; detach it there before the
    // state is freed, then close the timerfd
    let control = unsafe { DataControl::from_raw(state) };
    let detached = control.query(|state| {
        // SAFETY: this closure runs on the source's registered data loop.
        let err = unsafe { state.timer_source.unregister() };
        if err >= 0 {
            drop(state.timer_fd.take());
            state.timer_source.set_fd(-1);
        }
        err
    });
    if !matches!(detached, Some(err) if err >= 0) {
        // freeing the state now would leave the loop a dangling source; a clean
        // abort beats a use-after-free on the next timer tick
        eprintln!("freebsd-oss: can't detach the timer source; aborting");
        std::process::abort();
    }
    // the host frees the memory after clear; drop the fields exactly once here
    unsafe { std::ptr::drop_in_place(state) };
    0
}

pub(crate) extern "C" fn get_size<D: Direction>(
    _factory: *const spa_handle_factory,
    _params: *const spa_dict,
) -> usize {
    std::mem::size_of::<State<D>>()
}

// the init-dict node properties: the device path, the shared oss.fragment
// default and whatever direction-specific keys D::info_item consumes
pub(super) unsafe fn parse_init_dict<D: Direction>(
    info: *const spa_dict,
) -> (Option<String>, u32, D::MainExt) {
    let mut dsp_path = None;
    let mut oss_fragment = 0u32; // automatic (today's layout) unless the dict says otherwise
    let mut ext = D::MainExt::default();

    if let Some(info) = unsafe { info.as_ref() } {
        #[cfg(debug_assertions)]
        unsafe {
            crate::spa::dump_spa_dict(info);
        }

        unsafe {
            crate::spa::for_each_dict_item(info, |key, value| {
                if key == crate::keys::OSS_DSP_PATH {
                    dsp_path = Some(value.to_string());
                } else if key == crate::keys::OSS_FRAGMENT {
                    // direction-shared per-device default, e.g. from a wireplumber node
                    // rule; stored normalized so readback reports the effective value
                    if let Ok(v) = value.parse::<u32>() {
                        oss_fragment = normalize_fragment(v);
                    }
                } else {
                    D::info_item(&mut ext, key, value);
                }
            });
        }
    }
    D::ext_ready(&mut ext);

    (dsp_path, oss_fragment, ext)
}

// the static node/port info published at init: flags, props and the param
// directory (the readable/writable flags flip later in port_set_param)
pub(super) fn publish_static_info<D: Direction>(state: &MainState<D>) {
    state.events.with_info(|node, port| {
        // NodeEvents is now at its final Rc allocation, so weave the inline
        // params arrays' self-pointers only after State construction.
        node.fix_pointers();
        port.fix_pointers();

        if D::DIRECTION == SPA_DIRECTION_INPUT {
            node.set_max_input_ports(1);
        } else {
            node.set_max_output_ports(1);
        }
        // The RT flag declares process() safe on the realtime data loop.
        node.set_flags(SPA_NODE_FLAG_RT as u64);
        node.add_prop(crate::spa::key(SPA_KEY_MEDIA_CLASS), D::MEDIA_CLASS);
        node.add_prop(crate::spa::key(SPA_KEY_NODE_DRIVER), "true");

        // No EnumPortConfig/PortConfig (or node-level IO/EnumFormat): dead
        // surface on a follower, see build_port_format_info.
        node.add_param(SPA_PARAM_PropInfo, SPA_PARAM_INFO_READ);
        node.add_param(SPA_PARAM_Props, SPA_PARAM_INFO_READWRITE);
        node.add_param(SPA_PARAM_ProcessLatency, SPA_PARAM_INFO_READWRITE);

        port.set_flags((SPA_PORT_FLAG_PHYSICAL | SPA_PORT_FLAG_TERMINAL) as u64);
        // 1/48000 is the pre-negotiation placeholder.
        port.set_rate(spa_fraction {
            num: 1,
            denom: 48000,
        });
        port.add_param(SPA_PARAM_EnumFormat, SPA_PARAM_INFO_READ);
        port.add_param(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
        port.add_param(SPA_PARAM_Buffers, 0);
        port.add_param(SPA_PARAM_Latency, SPA_PARAM_INFO_READWRITE);
    });
}

pub(crate) unsafe extern "C" fn init<D: Direction>(
    _factory: *const spa_handle_factory,
    handle: *mut spa_handle,
    info: *const spa_dict,
    support: *const spa_support,
    n_support: u32,
) -> c_int {
    // the support array is the host's init contract: n_support valid entries
    let log =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Log.as_ptr().cast()) }
            as *mut spa_log;
    let log = unsafe { crate::spa::Log::wrap(log, Some(D::log_topic())) };

    let data_loop = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_DataLoop.as_ptr().cast(),
        )
    } as *mut spa_loop;
    let data_system = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_DataSystem.as_ptr().cast(),
        )
    } as *mut spa_system;
    let main_loop =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast()) }
            as *mut spa_loop;

    if data_loop.is_null() || data_system.is_null() {
        return -libc::EINVAL;
    }

    let data_loop = unsafe { crate::spa::Loop::wrap(data_loop) };
    let data_system = unsafe { crate::spa::System::wrap(data_system) };

    let timer_fd = match data_system.timerfd_create(
        libc::CLOCK_MONOTONIC,
        (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as i32,
    ) {
        Ok(timer_fd) => timer_fd,
        Err(err) => return err, // fd exhaustion fails node creation, not the daemon
    };
    let timer_fd_raw = timer_fd.raw();

    let (dsp_path, oss_fragment, ext) = unsafe { parse_init_dict::<D>(info) };

    let Some(dsp_path) = dsp_path else {
        crate::error!(
            log,
            "{} missing from the node properties",
            crate::keys::OSS_DSP_PATH
        );
        return -libc::EINVAL;
    };

    let mut caps_fallback = false;
    let caps = crate::oss::probe_caps(&dsp_path, D::PLAYBACK).unwrap_or_else(|| {
        crate::warn!(log, "{}: can't probe device caps; using fallback", dsp_path);
        caps_fallback = true;
        crate::oss::DspCaps::fallback()
    });
    crate::debug!(log, "{}: {:?}", dsp_path, caps);

    let state = handle.cast::<State<D>>();
    assert!(!state.is_null(), "handle is not supposed to be null");

    let node_methods: &'static spa_node_methods = &D::NODE_METHODS;
    let events = NodeEvents::<D>::new();
    let shared = std::sync::Arc::new(NodeShared::new());
    let main_events = MainEventTarget::new(&events, shared.alive_token());
    let format_publication = events.format_publication();
    let rebuild_worker = match RebuildWorker::<D>::start() {
        Ok(worker) => worker,
        Err(err) => {
            crate::error!(log, "can't start the device rebuild worker: {}", err);
            return -libc::EIO;
        }
    };
    let rebuild_work = rebuild_worker.endpoint();
    let data_ext = D::data_ext(&ext);
    let main_loop = if main_loop.is_null() {
        None
    } else {
        Some(unsafe { crate::spa::Loop::wrap(main_loop) })
    };

    // the host hands us uninitialized memory of get_size() bytes; write the
    // whole State without dropping the garbage "old" value
    unsafe {
        std::ptr::write(
            state,
            State {
                handle: spa_handle {
                    version: SPA_VERSION_HANDLE,
                    get_interface: Some(get_interface::<D>),
                    clear: Some(clear::<D>),
                },

                node: spa_node {
                    iface: spa_interface {
                        type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
                        version: SPA_VERSION_NODE,
                        cb: spa_callbacks {
                            funcs: node_methods as *const _ as *const c_void,
                            data: state as *mut _ as *mut c_void,
                        },
                    },
                },

                gate: DataThreadGate {
                    thread: std::sync::atomic::AtomicUsize::new(0),
                    log: log.clone(),
                },
                main: MainState {
                    events: events.clone(),
                    data_loop,
                    log: log.clone(),
                    dsp_path: dsp_path.clone(),
                    caps,
                    caps_fallback,
                    oss_fragment,
                    oss_fragment_default: oss_fragment,
                    latency: [
                        crate::spa::latency_info_default(SPA_DIRECTION_INPUT),
                        crate::spa::latency_info_default(SPA_DIRECTION_OUTPUT),
                    ],
                    process_latency: crate::spa::process_latency_default(),
                    shared: shared.clone(),
                    rebuild_worker,
                    ring_cap_published: false,
                    ext,
                },
                data: DataState {
                    data_loop,
                    data_system,
                    log,
                    clock: crate::spa::IoArea::null(),
                    position: crate::spa::IoArea::null(),
                    clock_name: std::ffi::CString::new(format!(
                        "freebsd-oss.{}",
                        dsp_path.trim_start_matches("/dev/")
                    ))
                    .unwrap_or_default(),
                    main_loop,
                    dsp_path: dsp_path.clone(),
                    timer_fd: Some(timer_fd),
                    timer_source: crate::spa::LoopSource::new(spa_source {
                        loop_: std::ptr::null_mut(),
                        func: Some(on_timeout::<D>),
                        data: state.cast::<c_void>(),
                        fd: timer_fd_raw,
                        mask: SPA_IO_IN,
                        rmask: 0,
                        priv_: std::ptr::null_mut(),
                    }),
                    next_time: 0,
                    callbacks: NodeCallbacks::none(),
                    ports: [Port {
                        config: None,
                        buffers: vec![],
                        io: crate::spa::IoArea::null(),
                        rate_match: crate::spa::IoArea::null(),
                        dsp: D::Device::new(&dsp_path),
                        dll: std::default::Default::default(),
                        setup_period: 0,
                        bw_adapt: std::default::Default::default(),
                        setup_blocksize: 0,
                        rebuild_pending: false,
                        generation: 0,
                        was_matching: false,
                        warn_limit: crate::node::RateLimit::new(),
                        pending_xrun: None,
                        ext: std::default::Default::default(),
                    }; MAX_PORTS],
                    oss_fragment,
                    shared,
                    rebuild_work,
                    deferred_work: None,
                    rebuild_takeover: false,
                    format_publication,
                    main_events,
                    pending_main_event: None,
                    started: false,
                    following: false,
                    ext: data_ext,
                },
            },
        );
    }

    let main = unsafe { main_ref(state) };
    publish_static_info(main);

    let err = unsafe {
        with_data_mut(state, |data| {
            let data_loop = data.data_loop;
            // SAFETY: init performs registration on the data loop endpoint; the
            // pinned source and its State data pointer live until clear.
            data.timer_source.register(&data_loop)
        })
    };
    if err < 0 {
        unsafe {
            with_data_mut(state, |data| {
                drop(data.timer_fd.take());
                data.timer_source.set_fd(-1);
            });
        };
        // the host won't call clear() after a failed init; free what we built
        unsafe { std::ptr::drop_in_place(state) };
        return err;
    }

    // learn the data loop's thread identity from the loop itself (see
    // check_loop_identity); pw's data loops run before any node loads, so
    // this executes on the loop thread, not inline
    let control = unsafe { DataControl::from_raw(state) };
    let gate = unsafe { gate_ref(state) };
    let thread = std::ptr::addr_of!(gate.thread);
    let loop_thread = unsafe { crate::spa::SendWrap::new(thread.cast_mut()) };
    let seeded = control.invoke(move |_data| {
        let thread = loop_thread.into_inner();
        let tid = unsafe { libc::pthread_self() } as usize;
        // A process call cannot legitimately precede successful init, but
        // preserve a gate that was already disabled rather than reviving it.
        let _ = unsafe { &*thread }.compare_exchange(
            0,
            tid,
            std::sync::atomic::Ordering::Release,
            std::sync::atomic::Ordering::Relaxed,
        );
    });
    if !seeded {
        unsafe { &*thread }.store(usize::MAX, std::sync::atomic::Ordering::Release);
        crate::warn!(
            gate.log,
            "can't seed the data-loop thread identity; disabling processing"
        );
    }

    0
}

pub(super) const INTERFACE_INFO: [spa_interface_info; 1] = [spa_interface_info {
    type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
}];

pub(crate) unsafe extern "C" fn enum_interface_info(
    _factory: *const spa_handle_factory,
    info: *mut *const spa_interface_info,
    index: *mut u32,
) -> c_int {
    assert!(!info.is_null());
    assert!(!index.is_null());
    // non-null asserted above; the caller contract makes both valid and writable
    unsafe {
        match *index {
            0 => {
                *info = &INTERFACE_INFO[0];
                *index += 1;
                1
            }
            _ => 0,
        }
    }
}
