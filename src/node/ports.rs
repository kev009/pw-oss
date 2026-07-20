use super::events::{emit_param_result, emit_port_info};
use super::*;

pub(super) unsafe extern "C" fn add_port(
    _object: *mut c_void,
    _direction: spa_direction,
    _port_id: u32,
    _props: *const spa_dict,
) -> c_int {
    -libc::ENOTSUP // the ports are static
}

pub(super) unsafe extern "C" fn remove_port(
    _object: *mut c_void,
    _direction: spa_direction,
    _port_id: u32,
) -> c_int {
    -libc::ENOTSUP // the ports are static
}

// No EnumPortConfig/PortConfig params here, on purpose: a follower's
// PortConfig surface is dead code under the adapter. audioadapter answers
// both params itself in passthrough and from its convert node otherwise
// (audioadapter.c:221) and only mirrors PropInfo/Props/ProcessLatency from
// the follower's node info (follower_info, audioadapter.c:1312); WirePlumber
// never reads them either - it probes EnumFormat and writes PortConfig on
// the adapter (module-si-audio-adapter.c si_audio_adapter_find_format /
// set_ports_format). Passthrough mode is carried entirely by the port
// params below: reconfigure_mode sets our Format with the NEAREST flag
// (audioadapter.c:758) and the graph link then negotiates buffers against
// the port directly (negotiate_buffers/negotiate_format short-circuit when
// follower == target, audioadapter.c:445, :995).

// replays the negotiated format exactly, for port_enum_params(Format);
// kept on the C spa_format_audio_raw_build FFI (unlike the Value-tree
// builders in node::format) so the pod stays byte-identical to the C helper
pub(super) fn build_port_format_info(config: &PortConfig, id: u32) -> Vec<u8> {
    let mut position = [0u32; 64];
    for (slot, &p) in position.iter_mut().zip(config.positions.iter()) {
        *slot = p;
    }

    let raw = spa_audio_info_raw {
        format: config.format.0,
        flags: config.flags,
        rate: config.rate,
        channels: config.channels,
        position,
    };

    let mut buffer = vec![];
    let builder = libspa::pod::builder::Builder::new(&mut buffer);
    // the raw struct is fully initialized above; output goes into the builder
    unsafe { spa_format_audio_raw_build(builder.as_raw_ptr(), id, &raw) };
    drop(builder);
    buffer
}

pub(super) unsafe extern "C" fn port_enum_params<D: Direction>(
    object: *mut c_void,
    seq: c_int,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");

    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }
    let events = unsafe { main_ref(state).events.clone() };
    let main = unsafe { main_ptr(state) };
    let control = unsafe { DataControl::from_raw(state) };

    unsafe {
        crate::spa::enum_params_loop(
            main,
            (start, max),
            filter,
            |state, index| {
                use crate::spa::ParamStep;
                #[allow(non_upper_case_globals)]
                match (id, index) {
                    (SPA_PARAM_EnumFormat, i) => {
                        if state.caps_fallback {
                            // the init-time probe hit a busy device and baked in fallback
                            // caps; retry now (main thread, transient open)
                            if let Some(caps) = crate::oss::probe_caps(&state.dsp_path, D::PLAYBACK)
                            {
                                crate::info!(state.log, "re-probed caps: {:?}", caps);
                                state.caps = caps;
                                state.caps_fallback = false;
                            }
                        }
                        match crate::node::build_enum_format_info(&state.caps, i) {
                            Some(pod) => ParamStep::Built(pod),
                            None => ParamStep::Stop(0),
                        }
                    }
                    (SPA_PARAM_Format, 0) => {
                        match control.query(move |data| data.ports[port_id as usize].config.clone())
                        {
                            Some(Some(cfg)) => {
                                ParamStep::Built(build_port_format_info(&cfg, SPA_PARAM_Format))
                            }
                            Some(None) => ParamStep::Stop(-libc::ENOENT),
                            None => ParamStep::Stop(-libc::EIO),
                        }
                    }
                    (SPA_PARAM_Buffers, 0) => {
                        match control.query(move |data| data.ports[port_id as usize].config.clone())
                        {
                            Some(Some(cfg)) => {
                                ParamStep::Built(crate::node::build_buffers_info(cfg.stride))
                            }
                            Some(None) => ParamStep::Stop(-libc::ENOENT),
                            None => ParamStep::Stop(-libc::EIO),
                        }
                    }
                    (SPA_PARAM_Latency, 0 | 1) => {
                        let mut info = state.latency[index as usize];
                        // the process latency shifts what we report toward the peer (upstream
                        // for the sink, downstream for the source)
                        if info.direction == D::DIRECTION {
                            crate::spa::process_latency_info_add(&state.process_latency, &mut info);
                        }
                        ParamStep::Built(crate::spa::build_latency_info(&info))
                    }
                    // a known id whose indices are exhausted ends the enumeration
                    (SPA_PARAM_Format | SPA_PARAM_Buffers | SPA_PARAM_Latency, _) => {
                        ParamStep::Stop(0)
                    }
                    _ => ParamStep::Stop(-libc::ENOENT), // unknown param id (ALSA convention)
                }
            },
            |index, param| emit_param_result(&events, seq, id, index, param),
        )
    }
}

// port_set_param(Format): validate the raw format against the format map and
// build the shared config (the stride falls out of the map's bytes/sample)
pub(super) fn parse_config<D: Direction>(
    state: &MainState<D>,
    raw: &spa_audio_info_raw,
) -> Result<PortConfig, c_int> {
    let format = libspa::param::audio::AudioFormat(raw.format);

    // only formats from our EnumFormat are expected; reject the rest
    let Some((_, bytes_per_sample)) = crate::node::oss_format_info(raw.format) else {
        crate::warn!(state.log, "rejecting unsupported format {:?}", format);
        return Err(-libc::ENOTSUP);
    };

    let config = PortConfig {
        format,
        rate: raw.rate,
        channels: raw.channels,
        positions: raw.position[..raw.channels as usize].to_vec(),
        flags: raw.flags,
        stride: bytes_per_sample * raw.channels, // bytes per interleaved frame
    };

    match config.oss_channel_order() {
        Err(_) => {
            crate::warn!(
                state.log,
                "rejecting unsupported channel map: {:?}",
                config.positions
            );
            return Err(-libc::EINVAL);
        }
        Ok(Some(_)) if state.caps.convertless => {
            // Bitperfect skips the matrix feeder: SET_CHNORDER would update
            // only the reported matrix while hardware kept its native order.
            crate::warn!(
                state.log,
                "rejecting channel reorder on a bitperfect device: {:?}",
                config.positions
            );
            return Err(-libc::EINVAL);
        }
        _ => (),
    }

    crate::debug!(state.log, "reconfiguring with {:?}", config);

    Ok(config)
}

// A validated Format request. The channel map occupies
// raw.position[..raw.channels]; no pod data is retained.
pub(crate) struct RequestedFormat {
    pub raw: spa_audio_info_raw,
}

// Decode and validate a raw-audio Format pod. Non-raw media returns -ENOENT;
// malformed or degenerate formats return -EINVAL.
//
// # Safety
// `param` must point at a valid, complete spa_pod (the port_set_param
// contract). This is the only raw-pod consumer on the Format path.
pub(super) unsafe fn decode_format(
    param: *const spa_pod,
    log: &crate::spa::Log,
) -> Result<RequestedFormat, c_int> {
    use libspa::param::audio::AudioInfoRaw;
    use libspa::param::format::{MediaSubtype, MediaType};
    use libspa::param::format_utils::parse_format;

    let pod = unsafe { libspa::pod::Pod::from_raw(param) };
    match parse_format(pod) {
        Ok((MediaType::Audio, MediaSubtype::Raw)) => (),
        Ok((t, st)) => {
            crate::warn!(log, "unknown media type combination: {:?}, {:?}", t, st);
            return Err(-libc::ENOENT);
        }
        Err(err) => {
            crate::warn!(log, "parse_format failed: {}", err);
            return Err(-libc::EINVAL);
        }
    }

    // AudioInfoRaw starts every optional field from its defined defaults,
    // then contains the raw C parser behind its safe result-returning API.
    let mut info = AudioInfoRaw::new();
    if let Err(err) = info.parse(pod) {
        crate::warn!(log, "audio format parse failed: {}", err);
        return Err(-libc::EINVAL);
    }
    let raw = info.as_raw();

    // format flags are stored but unused, OSS writes interleaved frames
    if raw.rate == 0 || raw.channels == 0 || raw.channels > SPA_AUDIO_MAX_CHANNELS {
        crate::warn!(
            log,
            "rejecting format: rate={} channels={}",
            raw.rate,
            raw.channels
        );
        return Err(-libc::EINVAL);
    }

    Ok(RequestedFormat { raw })
}

// Apply a validated Format request. NEAREST may snap unsupported values to
// the advertised capabilities. Ok(1) tells the adapter to read back the
// adjusted format; validation errors return without emitting port info.
pub(super) fn set_format_param<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
    flags: u32,
    requested: RequestedFormat,
) -> Result<c_int, c_int> {
    let mut raw = requested.raw;

    // audioadapter always sets the follower format with NEAREST
    // (audioadapter.c:758, :1059); snap only what the exact path
    // below would reject, so in-caps requests stay untouched
    let admitted = |caps: &crate::oss::DspCaps, raw: &spa_audio_info_raw| {
        crate::node::oss_format_info(raw.format)
            .is_some_and(|(m, _)| caps.admits(m, raw.channels, raw.rate))
    };
    let mut snapped = false;
    if flags & crate::spa::SPA_NODE_PARAM_FLAG_NEAREST != 0 && !admitted(&state.caps, &raw) {
        snapped = crate::node::snap_raw_to_caps(&state.caps, &mut raw);
        if snapped {
            crate::info!(
                state.log,
                "snapped requested format to caps: format={} rate={} channels={}",
                raw.format,
                raw.rate,
                raw.channels
            );
        }
    }

    let config = parse_config(state, &raw)?;

    // Validate against the advertised caps first: an out-of-caps
    // request on an exclusive device would EBUSY-retire the WORKING
    // fd and then fail configure, killing the stream for nothing.
    // configure() stays as the backstop for stale caps (a rejection
    // there re-probes and re-announces).
    if !state
        .caps
        .admits(config.oss_format(), raw.channels, raw.rate)
    {
        crate::warn!(
            state.log,
            "rejecting out-of-caps format: rate={} channels={}",
            raw.rate,
            raw.channels
        );
        return Err(-libc::EINVAL);
    }

    let mut res = install_device(state, data, port_idx, config);
    if res == 0 && snapped {
        res = 1;
    }
    if res == -libc::EINVAL || res == -libc::ENOTSUP {
        // the device rejected caps-derived values: the snapshot may be
        // stale (vchans/bitperfect toggled at runtime); re-probe and
        // re-announce EnumFormat so the host renegotiates from reality
        if let Some(caps) = crate::oss::probe_caps(&state.dsp_path, D::PLAYBACK) {
            state.caps_fallback = false;
            // bump only on a real change: the serial flip re-triggers the
            // adapter's negotiation, and an unchanged snapshot would loop
            // it against the same rejection
            if caps != state.caps {
                crate::info!(state.log, "re-probed caps after rejection: {:?}", caps);
                state.caps = caps;
                state
                    .events
                    .with_port_info(|info| info.bump_param(SPA_PARAM_EnumFormat));
            }
        }
    }
    Ok(res)
}

// port_set_param(Format) with a NULL pod: release the format. Swap a closed
// placeholder and drop the buffers on the data loop, then destroy the old
// device back on the calling main thread (close can sleep).
pub(super) fn release_format<D: Direction>(
    state: &MainState<D>,
    data: &DataControl<D>,
    port_idx: usize,
) -> c_int {
    let placeholder = D::Device::new(&state.dsp_path);
    let Some((retired, deferred)) = data.query(move |state| {
        debug_assert!(!state.rebuild_takeover, "format releases serialize");
        state.rebuild_takeover = true;
        crate::node::timing::invalidate_device_wake(state);
        let deferred = state.deferred_work.take();
        let port = &mut state.ports[port_idx];
        let retired = std::mem::replace(&mut port.dsp, placeholder);
        crate::node::reset_device_event(port);
        port.buffers.clear();
        port.config = None;
        // retire any in-flight background rebuild, and drop its pending
        // claim with it - a released port must not keep skipping cycles
        // for a completion the bump just retired
        port.generation = port.generation.wrapping_add(1);
        state
            .shared
            .generation
            .store(port.generation, std::sync::atomic::Ordering::Release);
        port.rebuild_pending = true;
        if state.started {
            update_driver_wake(state);
        }
        (retired, deferred)
    }) else {
        return -libc::EIO; // the loop still holds the buffers; freeing them would dangle
    };
    drop(retired);
    drop(deferred);
    // Nothing polls a released port. Quiesce the invalidated worker command
    // and drain both sides of the wait so a late Installed deposit cannot
    // retain an exclusive fd indefinitely.
    state.shared.discard_swap();
    if !state.rebuild_worker.wait_idle() {
        release_rebuild_takeover(data, port_idx);
        return -libc::EIO;
    }
    state.shared.discard_swap();
    if release_rebuild_takeover(data, port_idx) {
        0
    } else {
        -libc::EIO
    }
}

// update the port rate and flip Format/Buffers flags to reflect whether a
// format is negotiated, then re-emit so the host re-reads them (PipeWire
// ALSA sink/source pattern)
pub(super) fn publish_format_state<D: Direction>(state: &MainState<D>, rate: Option<u32>) {
    state.events.with_port_info(|info| {
        let _ = info.replace_change_mask(0);
        if let Some(rate) = rate {
            info.set_rate(spa_fraction {
                num: 1,
                denom: rate,
            });
            info.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_READWRITE);
            info.set_param_flags(SPA_PARAM_Buffers, SPA_PARAM_INFO_READ);
        } else {
            info.set_param_flags(SPA_PARAM_Format, SPA_PARAM_INFO_WRITE);
            info.set_param_flags(SPA_PARAM_Buffers, 0);
        }
    });
    emit_port_info(state);
    // This is the ordering token for deferred FormatLost delivery. Advance
    // after the matching owned snapshot has been queued but before callbacks
    // can run at the extern wrapper's flush boundary.
    state.events.advance_format_publication_epoch();
}

// A validated Latency request. The host supplies the opposite direction
// (downstream for a sink, upstream for a source); NULL resets that direction
// to its default. Invalid or same-direction values return -EINVAL.
pub(crate) struct LatencyRequest {
    pub(super) info: spa_latency_info,
}

pub(super) fn decode_latency_request(
    direction: spa_direction,
    value: Option<&libspa::pod::Value>,
) -> Result<LatencyRequest, c_int> {
    let other = direction ^ 1;
    let info = match value {
        None => crate::spa::latency_info_default(other),
        Some(v) => match crate::spa::parse_latency_info(Some(v)) {
            Some(info) if info.direction == other => info,
            _ => return Err(-libc::EINVAL),
        },
    };
    Ok(LatencyRequest { info })
}

// Store the latency and re-emit it through the graph.
pub(super) fn set_latency_param<D: Direction>(
    state: &mut MainState<D>,
    request: LatencyRequest,
) -> c_int {
    let info = request.info;
    state.latency[info.direction as usize] = info;

    state.events.with_port_info(|port| {
        let _ = port.replace_change_mask(0);
        port.bump_param(SPA_PARAM_Latency);
    });
    emit_port_info(state);

    0
}

pub(super) unsafe extern "C" fn port_set_param<D: Direction>(
    object: *mut c_void,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    flags: u32,
    param: *const spa_pod,
) -> c_int {
    let state: *mut State<D> = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let control = unsafe { DataControl::from_raw(state) };
    let main = unsafe { main_ptr(state) };
    let events = unsafe { (*main).events.clone() };
    // SAFETY: the host keeps param valid for this method call. The inner
    // phase queues owned snapshots and invokes no listeners.
    let result =
        unsafe { port_set_param_inner(&mut *main, &control, direction, port_id, id, flags, param) };
    // SAFETY: port_set_param_inner returned, ending its State borrow.
    unsafe { events.flush() };
    result
}

pub(super) unsafe fn port_set_param_inner<D: Direction>(
    state: &mut MainState<D>,
    data: &DataControl<D>,
    direction: spa_direction,
    port_id: u32,
    id: u32,
    flags: u32,
    param: *const spa_pod,
) -> c_int {
    if direction != D::DIRECTION || (port_id as usize) >= MAX_PORTS {
        return -libc::EINVAL;
    }

    #[allow(non_upper_case_globals)]
    match id {
        SPA_PARAM_Format => {
            let res = if !param.is_null() {
                // decode to owned data at the boundary; the set is safe code
                let requested = match unsafe { decode_format(param, &state.log) } {
                    Ok(requested) => requested,
                    Err(err) => return err,
                };
                match set_format_param(state, data, port_id as usize, flags, requested) {
                    Ok(res) => res,
                    Err(err) => return err,
                }
            } else {
                match release_format(state, data, port_id as usize) {
                    0 => 0,
                    err => return err,
                }
            };
            // emit even on failure: the flags derive from the (now cleared) config
            let rate = match data.query(move |data| {
                data.ports[port_id as usize]
                    .config
                    .as_ref()
                    .map(|config| config.rate)
            }) {
                Some(rate) => rate,
                None => return -libc::EIO,
            };
            publish_format_state(state, rate);
            res
        }
        SPA_PARAM_Latency => {
            // deserialize at the FFI boundary (None = NULL pod, the reset),
            // decode to the owned request there too; the apply is safe code
            let value = if param.is_null() {
                None
            } else {
                match unsafe { crate::spa::deserialize_pod(param) } {
                    Some(value) => Some(value),
                    None => return -libc::EINVAL,
                }
            };
            match decode_latency_request(direction, value.as_ref()) {
                Ok(request) => set_latency_param(state, request),
                Err(err) => err,
            }
        }
        SPA_PARAM_Tag => 0,
        id => {
            crate::warn!(state.log, "port_set_param: unknown param {}", id);
            -libc::ENOENT
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Latency requests reset on NULL and accept only the opposite direction.
    #[test]
    fn latency_requests_decode_direction_gated() {
        let dir = |d, v: Option<&libspa::pod::Value>| {
            decode_latency_request(d, v).map(|r| r.info.direction)
        };
        assert_eq!(dir(SPA_DIRECTION_INPUT, None), Ok(SPA_DIRECTION_OUTPUT));
        assert_eq!(dir(SPA_DIRECTION_OUTPUT, None), Ok(SPA_DIRECTION_INPUT));

        let info = crate::spa::latency_info_default(SPA_DIRECTION_OUTPUT);
        let value = crate::spa::parse_back(&crate::spa::build_latency_info(&info));
        assert_eq!(
            dir(SPA_DIRECTION_INPUT, Some(&value)),
            Ok(SPA_DIRECTION_OUTPUT)
        );
        // same-direction info and non-latency pods are rejected
        assert_eq!(dir(SPA_DIRECTION_OUTPUT, Some(&value)), Err(-libc::EINVAL));
        assert_eq!(
            dir(SPA_DIRECTION_INPUT, Some(&libspa::pod::Value::Int(1))),
            Err(-libc::EINVAL)
        );
    }
    // Format decoding accepts readback pods and rejects degenerate values.
    #[test]
    fn decode_format_roundtrips_and_rejects_degenerate_values() {
        let log = crate::spa::Log::test_null();
        let config = PortConfig {
            format: libspa::param::audio::AudioFormat::S16LE,
            rate: 48000,
            channels: 2,
            positions: vec![SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR],
            flags: 0,
            stride: 4,
        };
        // the builder returns bytes; the C parser needs a pod-aligned buffer
        let aligned = |bytes: &[u8]| {
            let mut buf = vec![0u64; bytes.len().div_ceil(8)];
            unsafe {
                std::ptr::copy_nonoverlapping(
                    bytes.as_ptr(),
                    buf.as_mut_ptr().cast::<u8>(),
                    bytes.len(),
                );
            };
            buf
        };

        let pod = build_port_format_info(&config, SPA_PARAM_Format);
        let buf = aligned(&pod);
        let requested = unsafe { decode_format(buf.as_ptr().cast(), &log) }
            .expect("our own Format pod must decode");
        assert_eq!(requested.raw.format, SPA_AUDIO_FORMAT_S16_LE);
        assert_eq!(requested.raw.rate, 48000);
        assert_eq!(requested.raw.channels, 2);
        assert_eq!(
            &requested.raw.position[..2],
            &[SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR]
        );

        // A zero rate is structurally valid but semantically invalid.
        let zero_rate = PortConfig { rate: 0, ..config };
        let pod = build_port_format_info(&zero_rate, SPA_PARAM_Format);
        let buf = aligned(&pod);
        assert_eq!(
            unsafe { decode_format(buf.as_ptr().cast(), &log) }
                .err()
                .expect("rate 0 must be rejected"),
            -libc::EINVAL
        );
    }
}
