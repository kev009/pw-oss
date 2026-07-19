use libspa::sys::*;
use std::os::raw::{c_char, c_int, c_uint, c_void};

mod params;

use params::set_param;

#[cfg(test)]
use params::{
    ProfileRequest, RouteProp, RouteProps, RouteRequest, decode_profile_request,
    decode_route_request,
};

// One hardware route per (pcm device, direction) that has a usable mixer
// control - except capture with a multi-source RECMASK, which gets one
// selectable route per source (the acp port model). The shadow fields mirror
// the kernel mixer state; the poll timer and set_param keep them in sync so
// re-emissions never report placeholders.
struct RouteState {
    node_id: u32, // our node object id (index * 2 + rec)
    rec: bool,
    name: String, // stable, never localized: WirePlumber's persistence key
    description: String,
    priority: i32,
    mixer: usize,            // index into State::mixers
    control: Option<c_uint>, // mixer level control; None = no volume props
    follows_recsrc: bool,    // control derives from RECSRC; re-resolve on change
    source: Option<c_uint>,  // the RECSRC bit this route selects (multi-source)
    active: bool,            // currently routed to its node; only active routes emit Route pods
    levels: (u32, u32),      // shadow OSS levels, 0-100 each
    mute: bool,
    save: bool, // echoed back in the Route pod, never interpreted
}

struct MixerHandle {
    mixer: crate::mixer::Mixer,
    counter: c_int, // modify_counter baseline for external-change detection
    recsrc: u32,    // RECSRC shadow; polled by value (the counter never ticks for it)
}

// Listener and info state lives outside State so a C callback may re-enter a
// device method without overlapping a Rust reference to the containing State.
// All device methods run on the main loop; Rc/RefCell express that ownership
// without claiming cross-thread access.
struct DeviceEvents {
    hooks: crate::spa::ListenerList<spa_device_events>,
    info: std::cell::RefCell<crate::spa::DeviceInfo>,
    pending: crate::spa::LocalNotificationQueue<DeviceNotification>,
}

enum DeviceObjectEvent {
    Added {
        id: u32,
        rec: bool,
        description: String,
        route_count: usize,
    },
    Removed {
        id: u32,
    },
}

enum DeviceNotification {
    Info(Box<crate::spa::DeviceInfo>),
    Object(DeviceObjectEvent),
    Event(Vec<u8>),
    Done(c_int),
    ActivateListeners(std::rc::Rc<crate::spa::ListenerList<spa_device_events>>),
}

impl DeviceEvents {
    fn new() -> Self {
        Self {
            hooks: crate::spa::ListenerList::new(),
            info: std::cell::RefCell::new(crate::spa::DeviceInfo::new()),
            pending: crate::spa::LocalNotificationQueue::new(),
        }
    }

    fn with_info<R>(&self, apply: impl FnOnce(&mut crate::spa::DeviceInfo) -> R) -> R {
        apply(&mut self.info.borrow_mut())
    }

    fn initial_info(&self) -> Box<crate::spa::DeviceInfo> {
        let mut snapshot = self.info.borrow().snapshot();
        let _ = snapshot.replace_change_mask(crate::spa::SPA_DEVICE_CHANGE_MASK_ALL as u64);
        snapshot
    }

    fn take_info(&self) -> Box<crate::spa::DeviceInfo> {
        let mut info = self.info.borrow_mut();
        let snapshot = info.snapshot();
        let _ = info.replace_change_mask(0);
        snapshot
    }

    // SAFETY: no reference into the associated State may be live; listener
    // code may synchronously re-enter any device method.
    unsafe fn emit_info(&self, snapshot: &crate::spa::DeviceInfo) {
        unsafe { self.emit_info_on(&self.hooks, snapshot) };
    }

    // SAFETY: as emit_info(); `hooks` is either the active list or one
    // isolated activation batch with the same event-table type.
    unsafe fn emit_info_on(
        &self,
        hooks: &crate::spa::ListenerList<spa_device_events>,
        snapshot: &crate::spa::DeviceInfo,
    ) {
        hooks.emit(|f, data| {
            if let Some(info) = f.info {
                // through the C listener vtable (the add_listener contract)
                unsafe { info(data, snapshot.raw()) };
            }
        });
    }

    // SAFETY: as emit_info().
    unsafe fn emit_done(&self, seq: c_int) {
        self.hooks.emit(|f, data| {
            if let Some(result) = f.result {
                // through the C listener vtable (the add_listener contract)
                unsafe { result(data, seq, 0, 0, std::ptr::null()) };
            }
        });
    }

    // SAFETY: as emit_info().
    unsafe fn emit_result(&self, seq: c_int, result: &spa_result_device_params) {
        crate::spa::dev_emit_result(&self.hooks, seq, 0, SPA_RESULT_TYPE_DEVICE_PARAMS, result);
    }

    // SAFETY: as emit_info().
    unsafe fn emit_object(&self, event: &DeviceObjectEvent) {
        unsafe { self.emit_object_on(&self.hooks, event) };
    }

    // SAFETY: as emit_info_on().
    unsafe fn emit_object_on(
        &self,
        hooks: &crate::spa::ListenerList<spa_device_events>,
        event: &DeviceObjectEvent,
    ) {
        match event {
            DeviceObjectEvent::Removed { id } => hooks.emit(|f, data| {
                if let Some(object_info) = f.object_info {
                    unsafe { object_info(data, *id, std::ptr::null()) };
                }
            }),
            DeviceObjectEvent::Added {
                id,
                rec,
                description,
                route_count,
            } => {
                let index = *id / 2;
                let mut dict = crate::spa::Dictionary::new();
                dict.add_item(
                    crate::spa::key(SPA_KEY_NODE_NAME),
                    format!("pcm{index}.{}", if *rec { "rec" } else { "play" }),
                );
                dict.add_item(
                    crate::spa::key(SPA_KEY_NODE_DESCRIPTION),
                    description.as_str(),
                );
                dict.add_item(crate::keys::OSS_DSP_PATH, format!("/dev/dsp{index}"));
                if *route_count > 0 {
                    dict.add_item("card.profile.device", format!("{id}"));
                    dict.add_item("device.routes", format!("{route_count}"));
                }
                let info = spa_device_object_info {
                    version: SPA_VERSION_DEVICE_OBJECT_INFO,
                    type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
                    factory_name: if *rec {
                        c"freebsd-oss.source".as_ptr()
                    } else {
                        c"freebsd-oss.sink".as_ptr()
                    },
                    change_mask: crate::spa::SPA_DEVICE_OBJECT_CHANGE_MASK_ALL as u64,
                    flags: 0,
                    props: dict.raw(),
                };
                // Keep the dictionary beside the callback payload for the
                // entire traversal.
                hooks.emit(|f, data| {
                    if let Some(object_info) = f.object_info {
                        unsafe { object_info(data, *id, &info) };
                    }
                });
            }
        }
    }

    // SAFETY: as emit_info().
    unsafe fn dispatch(&self, notification: &DeviceNotification) {
        match notification {
            DeviceNotification::Info(info) => unsafe { self.emit_info(info) },
            DeviceNotification::Object(object) => unsafe { self.emit_object(object) },
            DeviceNotification::Event(buffer) => {
                self.hooks.emit(|f, data| {
                    if let Some(event) = f.event {
                        unsafe { event(data, buffer.as_ptr().cast()) };
                    }
                });
            }
            DeviceNotification::Done(seq) => unsafe { self.emit_done(*seq) },
            DeviceNotification::ActivateListeners(hooks) => {
                // SAFETY: FIFO barriers run between traversals, after the
                // isolated batch's synchronous initial callbacks returned.
                unsafe { self.hooks.append_from(hooks) };
            }
        }
    }

    // See NodeEvents::with_new_listener: the barrier is queued before initial
    // callbacks whenever older FIFO work exists, so the listener skips only
    // those entries and is active for every notification the callbacks append.
    unsafe fn with_new_listener<R>(
        &self,
        listener: *mut spa_hook,
        events: *const spa_device_events,
        data: *mut c_void,
        initial: impl FnOnce(&crate::spa::ListenerList<spa_device_events>) -> R,
    ) -> R {
        let deferred = self.pending.defer_when_busy(|| {
            let hooks = std::rc::Rc::new(crate::spa::ListenerList::new());
            (DeviceNotification::ActivateListeners(hooks.clone()), hooks)
        });
        let hooks = deferred.as_deref().unwrap_or(&self.hooks);
        unsafe { hooks.with_isolated_listener(listener, events, data, || initial(hooks)) }
    }

    // Claim the main-loop endpoint's dispatch turn. Reentrant methods append
    // complete transactions to the FIFO and return; the outer owner drains
    // them after finishing its current transaction.
    fn begin_dispatch(&self) -> Option<crate::spa::LocalDispatchGuard<'_, DeviceNotification>> {
        self.pending.begin_dispatch()
    }

    // SAFETY: as emit_info(); only the begin_dispatch() owner may call this.
    // RefCell borrows end before every callback, so listener reentry can append.
    unsafe fn drain(&self, guard: crate::spa::LocalDispatchGuard<'_, DeviceNotification>) {
        self.pending.drain(guard, |notification| unsafe {
            self.dispatch(&notification);
        });
    }

    // SAFETY: as emit_info(). The entire input vector is enqueued atomically
    // before dispatch starts, preserving transaction order under reentry.
    unsafe fn dispatch_all(&self, notifications: Vec<DeviceNotification>) {
        self.pending
            .dispatch_all(notifications, |notification| unsafe {
                self.dispatch(&notification);
            });
    }
}

// repr(C): the host casts spa_handle* to State*, so `handle` must stay
// the first field at offset 0
#[repr(C)]
struct State {
    handle: spa_handle,
    device: spa_device,
    runtime: Runtime,
}

struct Runtime {
    events: std::rc::Rc<DeviceEvents>,
    pcm_devices: Vec<crate::sound::PcmDevice>,
    description: String,
    profile: u32,       // 0 = off, 1 = default
    profile_save: bool, // echoed back in the Profile pod
    routes: Vec<RouteState>,
    mixers: Vec<MixerHandle>,
    main_loop: Option<crate::spa::Loop>, // for the mixer poll timer
    system: Option<crate::spa::System>,  // ditto
    timer_fd: Option<crate::spa::TimerFd>, // owns the LoopSource fd mirror
    timer_source: crate::spa::LoopSource,
    devd_socket: Option<crate::utils::DevdSocket>, // jack/default-unit nudges; None = poll only
    devd_source: crate::spa::LoopSource,
    log: crate::spa::Log,
}

// Project only the mutable runtime payload. The host-visible handle and
// interface stay outside every callback borrow, so listener reentry cannot
// overlap a broad &mut State.
unsafe fn with_runtime_mut<R>(
    state: *mut State,
    apply: impl for<'a> FnOnce(&'a mut Runtime) -> R,
) -> R {
    assert!(!state.is_null(), "state is not supposed to be null");
    let runtime = unsafe { &mut *std::ptr::addr_of_mut!((*state).runtime) };
    apply(runtime)
}

unsafe fn with_runtime_ref<R>(
    state: *const State,
    apply: impl for<'a> FnOnce(&'a Runtime) -> R,
) -> R {
    assert!(!state.is_null(), "state is not supposed to be null");
    let runtime = unsafe { &*std::ptr::addr_of!((*state).runtime) };
    apply(runtime)
}

// OSS levels are a 0-100 slider scale, so map them through the cubic curve
// like ALSA devices without a dB scale (acp channel_map.c); a 1:1 linear map
// would make the volume keys feel wrong at the bottom of the range.
fn linear_to_oss(v: f32) -> u32 {
    if v.is_nan() || v <= 0.0 {
        // hostile pods included
        return 0;
    }
    (v.min(1.0).cbrt() * 100.0).round() as u32
}

// report the quantized readback, never the request, so the session manager
// converges on values the hardware can actually hold
fn oss_to_linear(l: u32) -> f32 {
    let x = l.min(100) as f32 / 100.0;
    x * x * x
}

// the mixer is stereo everywhere (STEREODEVS is the devmask, mixer.c:1094),
// so routes carry fixed FL/FR maps whatever width the node negotiates
const ROUTE_CHANNELS: u32 = 2;
const ROUTE_MAP: [u32; ROUTE_CHANNELS as usize] = [SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR];

// Build owned add/remove events while State is borrowed. Dispatch happens
// afterward, one traversal per object, with no State reference alive.
fn object_events(
    pcm_devices: &[crate::sound::PcmDevice],
    routes: &[RouteState],
    description: &str,
    present: bool,
) -> Vec<DeviceObjectEvent> {
    let mut events = Vec::new();
    for device in pcm_devices {
        for (rec, enabled) in [(false, device.play), (true, device.rec)] {
            if !enabled {
                continue;
            }

            let id = device.index * 2 + rec as u32;

            if !present {
                events.push(DeviceObjectEvent::Removed { id });
                continue;
            }
            let object_description = if device.desc == description && !device.location.is_empty() {
                format!("{} @ {}", device.desc, device.location)
            } else {
                device.desc.clone()
            };

            // Only nodes with a hardware route get linked to it; the rest (no
            // mixer, or no usable control - the bitperfect-purist case included)
            // keep the session manager's node softvol as their only volume.
            let route_count = routes.iter().filter(|r| r.node_id == id).count();
            events.push(DeviceObjectEvent::Added {
                id,
                rec,
                description: object_description,
                route_count,
            });
        }
    }
    events
}

fn build_profile_info(
    id: u32,
    index: u32,
    pcm_devices: &[crate::sound::PcmDevice],
    profile_save: bool,
    current: bool,
) -> Vec<u8> {
    use libspa::pod::{Object, Value, ValueArray};
    use libspa::utils::Id;

    let (name, description, priority) = if index == 0 {
        ("off", "Off", 0)
    } else {
        ("default", "Default", 100)
    };

    // The classes struct is what WirePlumber's select-routes walks to map
    // nodes to this profile; without it no route is ever applied. Every node
    // is listed, routed or not (pod shape: alsa-acp-device.c:326-384).
    let mut capture: Vec<i32> = vec![];
    let mut playback: Vec<i32> = vec![];
    for device in pcm_devices {
        if device.play {
            playback.push((device.index * 2) as i32);
        }
        if device.rec {
            capture.push((device.index * 2 + 1) as i32);
        }
    }

    let classes: [(&str, &Vec<i32>); 2] = [("Audio/Source", &capture), ("Audio/Sink", &playback)];
    let n_classes = if index == 0 {
        0
    } else {
        classes.iter().filter(|(_, ids)| !ids.is_empty()).count()
    };

    let mut class_fields = vec![Value::Int(n_classes as i32)];
    if index != 0 {
        for (class, ids) in classes {
            if ids.is_empty() {
                continue;
            }
            class_fields.push(Value::Struct(vec![
                Value::String(class.to_string()),
                Value::Int(ids.len() as i32),
                Value::String("card.profile.devices".to_string()),
                Value::ValueArray(ValueArray::Int(ids.clone())),
            ]));
        }
    }

    let mut properties = vec![
        crate::utils::pod_prop(SPA_PARAM_PROFILE_index, Value::Int(index as i32)),
        crate::utils::pod_prop(SPA_PARAM_PROFILE_name, Value::String(name.to_string())),
        crate::utils::pod_prop(
            SPA_PARAM_PROFILE_description,
            Value::String(description.to_string()),
        ),
        crate::utils::pod_prop(SPA_PARAM_PROFILE_priority, Value::Int(priority)),
        crate::utils::pod_prop(
            SPA_PARAM_PROFILE_available,
            Value::Id(Id(SPA_PARAM_AVAILABILITY_yes)),
        ),
        crate::utils::pod_prop(SPA_PARAM_PROFILE_classes, Value::Struct(class_fields)),
    ];

    if current {
        properties.push(crate::utils::pod_prop(
            SPA_PARAM_PROFILE_save,
            Value::Bool(profile_save),
        ));
    }

    crate::utils::serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamProfile,
        id,
        properties,
    }))
}

// EnumRoute (full = false) carries the static description only; Route
// (full = true) adds device/profile/save and the volume props object
// (pod shape: alsa-acp-device.c build_route)
fn build_route_info(id: u32, route: &RouteState, pos: usize, profile: u32, full: bool) -> Vec<u8> {
    use libspa::pod::{Object, Property, PropertyFlags, Value, ValueArray};
    use libspa::utils::Id;

    let mut properties = vec![
        crate::utils::pod_prop(SPA_PARAM_ROUTE_index, Value::Int(pos as i32)),
        // note: PLAYBACK maps to OUTPUT here (the route points out of the graph)
        crate::utils::pod_prop(
            SPA_PARAM_ROUTE_direction,
            Value::Id(Id(if route.rec {
                SPA_DIRECTION_INPUT
            } else {
                SPA_DIRECTION_OUTPUT
            })),
        ),
        crate::utils::pod_prop(SPA_PARAM_ROUTE_name, Value::String(route.name.clone())),
        crate::utils::pod_prop(
            SPA_PARAM_ROUTE_description,
            Value::String(route.description.clone()),
        ),
        crate::utils::pod_prop(SPA_PARAM_ROUTE_priority, Value::Int(route.priority)),
        // Constant yes: FreeBSD exposes no per-jack state userland can read (the
        // SND CONN devctl names a preferred device, not a jack - see
        // on_devd_event), and "no" would make WirePlumber's find-best-routes skip
        // the route and state-routes refuse to save its volume. acp would say
        // "unknown" where detection is absent, but flipping v1's "yes" carries no
        // information and only churns session-manager behavior.
        crate::utils::pod_prop(
            SPA_PARAM_ROUTE_available,
            Value::Id(Id(SPA_PARAM_AVAILABILITY_yes)),
        ),
        crate::utils::pod_prop(
            SPA_PARAM_ROUTE_profiles,
            Value::ValueArray(ValueArray::Int(vec![1])),
        ),
        crate::utils::pod_prop(
            SPA_PARAM_ROUTE_devices,
            Value::ValueArray(ValueArray::Int(vec![route.node_id as i32])),
        ),
    ];

    if full {
        properties.push(crate::utils::pod_prop(
            SPA_PARAM_ROUTE_device,
            Value::Int(route.node_id as i32),
        ));

        // Volume writers (pulse, the session manager) direct volume at the card
        // whenever an ACTIVE Route exists, regardless of props presence
        // (pulse-server.c:3004-3010 gates on active_port) - so even a source
        // with no level control must carry props, backed by a soft shadow that
        // audioconvert applies (the acp softvol model). The HARDWARE flag and
        // unity softVolumes apply only when a real control exists.
        let hw = route.control.is_some();
        let flag = if hw {
            PropertyFlags::HARDWARE
        } else {
            PropertyFlags::empty()
        };
        let volumes = vec![oss_to_linear(route.levels.0), oss_to_linear(route.levels.1)];
        // with hardware attenuation the node's software volume must stay at
        // unity or the signal is attenuated twice; a soft route IS the
        // software volume, so it mirrors the levels
        let soft = if hw {
            vec![1.0; ROUTE_CHANNELS as usize]
        } else {
            volumes.clone()
        };
        properties.push(crate::utils::pod_prop(
            SPA_PARAM_ROUTE_props,
            Value::Object(Object {
                type_: SPA_TYPE_OBJECT_Props,
                id,
                properties: vec![
                    Property {
                        key: SPA_PROP_mute,
                        flags: flag,
                        value: Value::Bool(route.mute),
                    },
                    Property {
                        key: SPA_PROP_channelVolumes,
                        flags: flag,
                        value: Value::ValueArray(ValueArray::Float(volumes)),
                    },
                    Property {
                        key: SPA_PROP_volumeBase,
                        flags: PropertyFlags::READONLY,
                        value: Value::Float(1.0),
                    },
                    Property {
                        key: SPA_PROP_volumeStep,
                        flags: PropertyFlags::READONLY,
                        value: Value::Float(1.0 / 101.0),
                    },
                    crate::utils::pod_prop(
                        SPA_PROP_channelMap,
                        Value::ValueArray(ValueArray::Id(
                            ROUTE_MAP.iter().map(|&c| Id(c)).collect(),
                        )),
                    ),
                    crate::utils::pod_prop(
                        SPA_PROP_softVolumes,
                        Value::ValueArray(ValueArray::Float(soft)),
                    ),
                ],
            }),
        ));

        properties.push(crate::utils::pod_prop(
            SPA_PARAM_ROUTE_profile,
            Value::Int(profile as i32),
        ));
        properties.push(crate::utils::pod_prop(
            SPA_PARAM_ROUTE_save,
            Value::Bool(route.save),
        ));
    }

    crate::utils::serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_OBJECT_ParamRoute,
        id,
        properties,
    }))
}

fn build_object_config(
    node_id: u32,
    volume: Option<((u32, u32), bool)>,
    mute: Option<bool>,
) -> Vec<u8> {
    use libspa::pod::{Object, Value, ValueArray};
    use libspa::utils::Id;

    let mut props = vec![];

    if let Some(((left, right), hw)) = volume {
        let volumes = vec![oss_to_linear(left), oss_to_linear(right)];
        // hardware attenuation keeps the node at unity; a soft route IS the
        // node's software volume, so audioconvert applies the levels
        let soft = if hw { vec![1.0; 2] } else { volumes.clone() };
        props.push(crate::utils::pod_prop(
            SPA_PROP_channelVolumes,
            Value::ValueArray(ValueArray::Float(volumes)),
        ));
        props.push(crate::utils::pod_prop(
            SPA_PROP_channelMap,
            Value::ValueArray(ValueArray::Id(ROUTE_MAP.iter().map(|&c| Id(c)).collect())),
        ));
        props.push(crate::utils::pod_prop(
            SPA_PROP_softVolumes,
            Value::ValueArray(ValueArray::Float(soft)),
        ));
    }

    if let Some(mute) = mute {
        props.push(crate::utils::pod_prop(SPA_PROP_mute, Value::Bool(mute)));
        props.push(crate::utils::pod_prop(SPA_PROP_softMute, Value::Bool(mute)));
    }

    crate::utils::serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_EVENT_Device,
        id: SPA_DEVICE_EVENT_ObjectConfig,
        properties: vec![
            crate::utils::pod_prop(SPA_EVENT_DEVICE_Object, Value::Int(node_id as i32)),
            crate::utils::pod_prop(
                SPA_EVENT_DEVICE_Props,
                Value::Object(Object {
                    type_: SPA_TYPE_OBJECT_Props,
                    id: SPA_EVENT_DEVICE_Props,
                    properties: props,
                }),
            ),
        ],
    }))
}

// Tell the session manager to push the new hardware state into the child
// node's Props (channelVolumes/softVolumes or mute/softMute), keeping
// audioconvert at unity - the anti-double-attenuation mechanism
// (pod shape: alsa-acp-device.c:1015-1084).
fn queue_object_config(
    state: &Runtime,
    pos: usize,
    volume: bool,
    notifications: &mut Vec<DeviceNotification>,
) {
    let route = &state.routes[pos];
    let (node_id, levels, mute) = (route.node_id, route.levels, route.mute);
    let hw = route.control.is_some();

    let buffer = if volume {
        build_object_config(node_id, Some((levels, hw)), None)
    } else {
        build_object_config(node_id, None, Some(mute))
    };

    notifications.push(DeviceNotification::Event(buffer));
}

// announce a Route change: flip the serial so consumers re-read the param
fn queue_route_change(state: &Runtime, notifications: &mut Vec<DeviceNotification>) {
    state.events.with_info(|info| {
        let _ = info.replace_change_mask(0);
        info.bump_param(SPA_PARAM_Route);
    });
    notifications.push(DeviceNotification::Info(state.events.take_info()));
}

// The ~1 Hz external-change poll: on a modify_counter tick, value-diff the
// levels and mute against the shadow and re-emit only on a real change. The
// counter is only a hint (it misses RECSRC changes and writes-to-muted); the
// value diff is what prevents spurious re-emissions either way.
// re-resolve a recsrc-derived capture control (RECSRC changes never tick the
// modify counter, and the write path must not adjust the OLD source)
fn resolve_recsrc(state: &mut Runtime, pos: usize) {
    if !state.routes[pos].follows_recsrc {
        return;
    }
    let mi = state.routes[pos].mixer;
    if let Some((control, true)) = state.mixers[mi].mixer.input_control() {
        state.routes[pos].control = Some(control);
    }
}

// pull the hardware state into a route's shadow (no emissions)
fn refresh_route_shadow(state: &mut Runtime, pos: usize) {
    resolve_recsrc(state, pos);
    let mi = state.routes[pos].mixer;
    let Some(control) = state.routes[pos].control else {
        return; // nothing to shadow for a control-less source route
    };
    if let Some(levels) = state.mixers[mi].mixer.level(control) {
        state.routes[pos].levels = levels;
    }
    if let Some(mute) = state.mixers[mi].mixer.muted(control) {
        state.routes[pos].mute = mute;
    }
}

// Value-poll RECSRC and move the active flag to the route backing the
// current source; the kernel never ticks modify_counter for RECSRC writes
// (mixer_setrecsrc, mixer.c:334-361), so external mixer(8) changes are only
// visible this way. Multiple set bits collapse to the lowest (the v1
// single-route convention). Returns the newly active route when it moved.
fn sync_recsrc(state: &mut Runtime, mi: usize) -> Option<usize> {
    if !state
        .routes
        .iter()
        .any(|r| r.mixer == mi && r.source.is_some())
    {
        return None;
    }
    let recsrc = state.mixers[mi].mixer.recsrc()?;
    if recsrc == state.mixers[mi].recsrc {
        return None;
    }
    state.mixers[mi].recsrc = recsrc;
    let masked = recsrc & state.mixers[mi].mixer.recmask();
    if masked == 0 {
        return None; // keep the current selection rather than guessing
    }
    let bit = masked.trailing_zeros();
    let pos = state
        .routes
        .iter()
        .position(|r| r.mixer == mi && r.source == Some(bit))?;
    if state.routes[pos].active {
        return None; // an extra bit appeared; the winning source is unchanged
    }
    for route in state.routes.iter_mut() {
        if route.mixer == mi && route.source.is_some() {
            route.active = route.source == Some(bit);
        }
    }
    refresh_route_shadow(state, pos);
    Some(pos)
}

fn poll_mixers(state: &mut Runtime) -> Vec<DeviceNotification> {
    let mut notifications = Vec::new();
    if state.profile == 0 {
        return notifications; // nodes are retracted under Off
    }

    let mut changed: Vec<(usize, bool, bool)> = vec![]; // (route, volume, mute)
    let mut switched: Vec<usize> = vec![];

    for mi in 0..state.mixers.len() {
        let Some(counter) = state.mixers[mi].mixer.modify_counter() else {
            continue; // the device may be mid-detach; the node teardown handles it
        };
        // Diff by VALUE every tick, not only when the counter moved: the kernel
        // doesn't bump it for writes to a muted control (mixer.c early-returns
        // into level_muted), and an external write landing inside our own
        // write-then-refresh window is swallowed by the baseline. The counter is
        // still tracked for log/debug value.
        state.mixers[mi].counter = counter;

        // recsrc first: it refreshes the new active route's shadow, so the
        // value diff below won't double-report the same movement
        if let Some(pos) = sync_recsrc(state, mi) {
            crate::info!(
                state.log,
                "recording source changed externally: route {}",
                state.routes[pos].name
            );
            switched.push(pos);
        }

        for pos in 0..state.routes.len() {
            if state.routes[pos].mixer != mi {
                continue;
            }
            resolve_recsrc(state, pos);
            let Some(control) = state.routes[pos].control else {
                continue; // control-less source routes carry no volume state
            };
            let mut vol_changed = false;
            let mut mute_changed = false;
            if let Some(levels) = state.mixers[mi].mixer.level(control) {
                if levels != state.routes[pos].levels {
                    state.routes[pos].levels = levels;
                    vol_changed = true;
                }
            }
            if let Some(mute) = state.mixers[mi].mixer.muted(control) {
                if mute != state.routes[pos].mute {
                    state.routes[pos].mute = mute;
                    mute_changed = true;
                }
            }
            // inactive routes still track the hardware (their level shows again on
            // the next switch), but a change there is observable in no pod
            if (vol_changed || mute_changed) && state.routes[pos].active {
                crate::info!(
                    state.log,
                    "route {} changed externally: levels {:?}, mute {}",
                    state.routes[pos].name,
                    state.routes[pos].levels,
                    state.routes[pos].mute
                );
                changed.push((pos, vol_changed, mute_changed));
            }
        }
    }

    if changed.is_empty() && switched.is_empty() {
        return notifications;
    }

    queue_route_change(state, &mut notifications);

    for pos in switched {
        // the node's effective input volume is the new source's control now
        if state.routes[pos].control.is_some() {
            queue_object_config(state, pos, true, &mut notifications);
            queue_object_config(state, pos, false, &mut notifications);
        }
    }

    for (pos, vol_changed, mute_changed) in changed {
        if vol_changed {
            queue_object_config(state, pos, true, &mut notifications);
        }
        if mute_changed {
            queue_object_config(state, pos, false, &mut notifications);
        }
    }
    notifications
}

unsafe extern "C" fn on_mixer_timeout(source: *mut spa_source) {
    let state: *mut State = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        // Scoped runtime borrow: all mixer mutations and payload construction
        // finish before arbitrary listener code runs below.
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let timer_fd = state.timer_fd.as_ref()?;
                let mut expirations = 0;
                (timer_fd.read(&mut expirations) >= 0)
                    .then(|| (state.events.clone(), poll_mixers(state)))
            })
        }) else {
            return;
        };
        result
    };
    // SAFETY: the scoped State borrow ended above.
    unsafe { events.dispatch_all(notifications) };
}

// devd "SND CONN" watcher. What the kernel actually emits (verified against
// 14.4+ /usr/src) is "!system=SND subsystem=CONN type={IN,OUT} cdev=dspN"
// (type=NODEV without a cdev when the last device goes):
//  - sound.c:81-97 (pcm_hotswap) fires it when hw.snd.default_unit moves -
//    not jack state at all;
//  - hdaa.c:566-592 (hdaa_presence_handler) fires it on a pin-sense change,
//    but only when the codec owns the default unit, never for headphone
//    redirect associations (hdaa.c:572 returns first - the common laptop
//    jack), and cdev names the device the kernel now PREFERS: the plugged
//    association on connect, the first enabled same-direction association
//    on disconnect. Connect and disconnect messages are indistinguishable.
// No other sound driver emits it. The payload therefore identifies a pcm
// unit but carries no jack state, so per-route available yes/no cannot be
// derived and availability stays a constant "yes" (see build_route_info).
// What a jack event DOES change kernel-side is the recording source
// (hdaa_autorecsrc_handler, hdaa.c:562) and pin mutes, so the one sound
// reaction is nudging the mixer poll instead of waiting out the 1 Hz tick.
unsafe extern "C" fn on_devd_event(source: *mut spa_source) {
    let state: *mut State = unsafe { (*source).data.cast() };
    assert!(
        !state.is_null(),
        "(*source).data is not supposed to be null"
    );

    let (events, notifications) = {
        let Some(result) = (unsafe {
            with_runtime_mut(state, |state| {
                let devd_socket = state.devd_socket.as_mut()?;

                let pcm_devices = &state.pcm_devices;
                let mut nudged = false;
                let alive = devd_socket.read_event(|line| {
                    static RE: std::sync::OnceLock<regex::Regex> = std::sync::OnceLock::new();
                    let re = RE.get_or_init(|| {
                        regex::Regex::new(
                            r"^!system=SND subsystem=CONN type=(?:IN|OUT) cdev=dsp([0-9]+)",
                        )
                        .unwrap()
                    });
                    if let Some(groups) = re.captures(line) {
                        if let Ok(unit) = groups[1].parse::<u32>() {
                            nudged |= pcm_devices.iter().any(|d| d.index == unit);
                        }
                    }
                });

                let notifications = if nudged {
                    crate::debug!(state.log, "SND CONN event; re-polling the mixers");
                    poll_mixers(state)
                } else {
                    Vec::new()
                };

                if !alive {
                    // devd restarted or dropped us; deregister or the level-triggered fd
                    // spins the main loop forever. The 1 Hz poll still covers changes.
                    crate::warn!(
                        state.log,
                        "devd connection lost; falling back to the mixer poll alone"
                    );
                    // SAFETY: this callback runs on the registered main loop.
                    if state.devd_source.unregister() < 0 {
                        eprintln!("freebsd-oss: can't detach the devd source; aborting");
                        std::process::abort();
                    }
                    state.devd_socket = None;
                    state.devd_source.set_fd(-1);
                }
                Some((state.events.clone(), notifications))
            })
        }) else {
            return;
        };
        result
    };
    // SAFETY: the scoped State borrow ended above.
    unsafe { events.dispatch_all(notifications) };
}

unsafe extern "C" fn add_listener(
    object: *mut c_void,
    listener: *mut spa_hook,
    events: *const spa_device_events,
    data: *mut c_void,
) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let (device_events, objects) = {
        unsafe {
            with_runtime_ref(state, |state| {
                (
                    state.events.clone(),
                    object_events(
                        &state.pcm_devices,
                        &state.routes,
                        &state.description,
                        state.profile != 0,
                    ),
                )
            })
        }
    };

    let initial = |hooks: &crate::spa::ListenerList<spa_device_events>| {
        // The initial emissions only reach the newly added listener (the list
        // is isolated). One method per traversal, mirroring C's
        // spa_hook_list_call: a listener that removes and frees its hook
        // inside a callback must not be read for the next method.
        let info = device_events.initial_info();
        let dispatch_guard = device_events.begin_dispatch();
        // SAFETY: all State-backed object data was copied above.
        unsafe { device_events.emit_info_on(hooks, &info) };
        for object in &objects {
            unsafe { device_events.emit_object_on(hooks, object) };
        }
        dispatch_guard
    };
    let dispatch_guard =
        unsafe { device_events.with_new_listener(listener, events, data, initial) };
    if let Some(guard) = dispatch_guard {
        // Nested profile/route changes queued during the initial transaction
        // are delivered only after every initial snapshot, and after the full
        // listener list has been restored.
        unsafe { device_events.drain(guard) };
    }
    0
}

unsafe extern "C" fn sync(object: *mut c_void, seq: c_int) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { with_runtime_ref(state, |state| state.events.clone()) };
    // SAFETY: only the independent endpoint remains borrowed. Done joins the
    // same FIFO as info/object transactions, so reentrant sync cannot overtake
    // already-produced state notifications.
    unsafe { events.dispatch_all(vec![DeviceNotification::Done(seq)]) };

    0
}

unsafe extern "C" fn enum_params(
    object: *mut c_void,
    seq: c_int,
    id: u32,
    start: u32,
    max: u32,
    filter: *const spa_pod,
) -> c_int {
    let state: *mut State = object.cast();
    assert!(!state.is_null(), "object is not supposed to be null");
    let events = unsafe { with_runtime_ref(state, |state| state.events.clone()) };
    let runtime = unsafe { std::ptr::addr_of_mut!((*state).runtime) };

    unsafe {
        crate::spa::enum_params_loop(
            runtime,
            (start, max),
            filter,
            |state, index| {
                use crate::spa::ParamStep;
                // only the active route becomes a Route pod; inactive selectable sources
                // exist as EnumRoute only (acp emits one Route per device with the
                // active port's index, alsa-acp-device.c:582-600)
                if id == SPA_PARAM_Route
                    && (index as usize) < state.routes.len()
                    && !state.routes[index as usize].active
                {
                    return ParamStep::Skip;
                }

                #[allow(non_upper_case_globals)]
                match (id, index) {
                    (SPA_PARAM_EnumProfile, 0 | 1) => ParamStep::Built(build_profile_info(
                        SPA_PARAM_EnumProfile,
                        index,
                        &state.pcm_devices,
                        state.profile_save,
                        false,
                    )),
                    (SPA_PARAM_Profile, 0) => ParamStep::Built(build_profile_info(
                        SPA_PARAM_Profile,
                        state.profile,
                        &state.pcm_devices,
                        state.profile_save,
                        true,
                    )),
                    (SPA_PARAM_EnumRoute, i) if (i as usize) < state.routes.len() => {
                        ParamStep::Built(build_route_info(
                            SPA_PARAM_EnumRoute,
                            &state.routes[i as usize],
                            i as usize,
                            state.profile,
                            false,
                        ))
                    }
                    // no Route pods while Off is active: there is nothing routed
                    (SPA_PARAM_Route, i)
                        if state.profile != 0 && (i as usize) < state.routes.len() =>
                    {
                        ParamStep::Built(build_route_info(
                            SPA_PARAM_Route,
                            &state.routes[i as usize],
                            i as usize,
                            state.profile,
                            true,
                        ))
                    }
                    // a known id whose indices are exhausted ends the enumeration
                    (
                        SPA_PARAM_EnumProfile
                        | SPA_PARAM_Profile
                        | SPA_PARAM_EnumRoute
                        | SPA_PARAM_Route,
                        _,
                    ) => ParamStep::Stop(0),
                    _ => ParamStep::Stop(-libc::ENOENT),
                }
            },
            |index, param| {
                let result = spa_result_device_params {
                    id,
                    index,
                    next: index + 1,
                    param,
                };
                // SAFETY: enum_params_loop ended its per-step runtime borrow
                // before invoking this closure.
                events.emit_result(seq, &result);
            },
        )
    }
}

const DEVICE_IMPL: spa_device_methods = spa_device_methods {
    version: SPA_VERSION_DEVICE_METHODS,
    add_listener: Some(add_listener),
    sync: Some(sync),
    enum_params: Some(enum_params),
    set_param: Some(set_param),
};

unsafe extern "C" fn get_interface(
    handle: *mut spa_handle,
    type_: *const c_char,
    interface: *mut *mut c_void,
) -> c_int {
    let state = handle.cast::<State>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    assert!(!interface.is_null());
    if unsafe { spa_streq(type_, SPA_TYPE_INTERFACE_Device.as_ptr().cast()) } {
        // interface is non-null (asserted above) and writable per the contract
        unsafe {
            *interface = std::ptr::addr_of_mut!((*state).device).cast::<c_void>();
        }
    } else {
        return -libc::ENOENT;
    }
    0
}

unsafe extern "C" fn clear(handle: *mut spa_handle) -> c_int {
    let state = handle.cast::<State>();
    assert!(!state.is_null(), "handle is not supposed to be null");
    unsafe {
        with_runtime_mut(state, |runtime| {
            // clear runs on the main loop's thread, so detach both sources there.
            if runtime.timer_source.is_registered() {
                if runtime.timer_source.unregister() < 0 {
                    eprintln!("freebsd-oss: can't detach the mixer timer source; aborting");
                    std::process::abort();
                }
                drop(runtime.timer_fd.take());
                runtime.timer_source.set_fd(-1);
            }
            if runtime.devd_source.is_registered() && runtime.devd_source.unregister() < 0 {
                eprintln!("freebsd-oss: can't detach the devd source; aborting");
                std::process::abort();
            }
            if !runtime.devd_source.is_registered() {
                runtime.devd_source.set_fd(-1);
            }
        });
    }
    // the host frees the memory after clear; drop the fields exactly once here
    unsafe { std::ptr::drop_in_place(state) };
    0
}

extern "C" fn get_size(_factory: *const spa_handle_factory, _params: *const spa_dict) -> usize {
    std::mem::size_of::<State>()
}

// loosely mirror acp's analog input ordering: mic on top, then line, then
// the rest in a stable bit-derived order
fn source_priority(dev: c_uint) -> i32 {
    match dev {
        crate::mixer::SOUND_MIXER_MIC => 100,
        crate::mixer::SOUND_MIXER_LINE => 90,
        _ => 80 - dev as i32,
    }
}

fn capitalize(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
        None => String::new(),
    }
}

// Discover the usable hardware controls and read their ACTUAL state before
// anything is emitted: reporting 1.0 placeholders and correcting later is a
// classic volume-jump source.
fn probe_routes(pcm_devices: &[crate::sound::PcmDevice]) -> (Vec<RouteState>, Vec<MixerHandle>) {
    let mut routes: Vec<RouteState> = vec![];
    let mut mixers: Vec<MixerHandle> = vec![];
    let mut n_out = 0;
    let mut n_in = 0;
    let device_count = pcm_devices.len();

    for device in pcm_devices {
        let Some(mixer) = crate::mixer::Mixer::open(device.index) else {
            continue; // no mixer device: the node keeps its softvol
        };

        // one read shared by the route active flags and the poll shadow: a
        // RECSRC change between two reads would mismark the active route
        let probe_recsrc = mixer.recsrc().unwrap_or(0);

        let mixer_index = mixers.len();
        let mut used = false;

        for (rec, enabled) in [(false, device.play), (true, device.rec)] {
            if !enabled {
                continue;
            }

            // A multi-source RECMASK becomes one selectable route per source (the
            // acp port model). Single-source and no-recmask devices keep the v1
            // single route below - its name is WirePlumber's persistence key and
            // must not churn.
            if rec && mixer.recmask().count_ones() >= 2 {
                let recmask = mixer.recmask();
                let recsrc = probe_recsrc & recmask;
                // multiple set bits: the lowest wins, matching the v1 convention
                let current = if recsrc != 0 {
                    recsrc.trailing_zeros()
                } else {
                    recmask.trailing_zeros()
                };
                for dev_bit in 0..crate::mixer::SOUND_MIXER_NRDEVICES {
                    if recmask & (1 << dev_bit) == 0 {
                        continue;
                    }
                    let control = mixer.source_volume_control(dev_bit);
                    let levels = control.and_then(|c| mixer.level(c));
                    let control = control.filter(|_| levels.is_some());
                    let mute = control.and_then(|c| mixer.muted(c)).unwrap_or(false);
                    let src = crate::mixer::SOUND_DEVICE_NAMES[dev_bit as usize];
                    let (name, description) = if device_count == 1 {
                        (format!("oss-input-{src}"), capitalize(src))
                    } else {
                        (
                            format!("oss-input-pcm{}-{}", device.index, src),
                            format!("{} (pcm{})", capitalize(src), device.index),
                        )
                    };
                    routes.push(RouteState {
                        node_id: device.index * 2 + 1,
                        rec: true,
                        name,
                        description,
                        priority: source_priority(dev_bit),
                        mixer: mixer_index,
                        control,
                        follows_recsrc: false,
                        source: Some(dev_bit),
                        active: dev_bit == current,
                        levels: levels.unwrap_or((100, 100)), // soft shadow starts at unity
                        mute,
                        save: false,
                    });
                    used = true;
                }
                continue;
            }

            let picked = if rec {
                mixer.input_control()
            } else {
                mixer.output_control().map(|c| (c, false))
            };
            let Some((control, follows_recsrc)) = picked else {
                continue; // no usable control for this direction
            };
            let Some(levels) = mixer.level(control) else {
                continue;
            };
            let mute = mixer.muted(control).unwrap_or(false);

            // Names are the session manager's persistence key: stable, no locale.
            // Derived from the pcm unit, not an ordinal - an ordinal shifts every
            // sibling when one unit's mixer fails to probe (attach-order race) or
            // the unit set changes, restoring saved volumes onto the wrong output.
            let (name, description) = if rec {
                n_in += 1;
                if n_in == 1 && device_count == 1 {
                    ("oss-input".to_string(), "Input".to_string())
                } else {
                    (
                        format!("oss-input-pcm{}", device.index),
                        format!("Input (pcm{})", device.index),
                    )
                }
            } else {
                n_out += 1;
                if n_out == 1 && device_count == 1 {
                    ("oss-output".to_string(), "Output".to_string())
                } else {
                    (
                        format!("oss-output-pcm{}", device.index),
                        format!("Output (pcm{})", device.index),
                    )
                }
            };

            routes.push(RouteState {
                node_id: device.index * 2 + rec as u32,
                rec,
                name,
                description,
                priority: 100,
                mixer: mixer_index,
                control: Some(control),
                follows_recsrc,
                source: None,
                active: true,
                levels,
                mute,
                save: false,
            });
            used = true;
        }

        if used {
            let counter = mixer.modify_counter().unwrap_or(0);
            mixers.push(MixerHandle {
                mixer,
                counter,
                recsrc: probe_recsrc,
            });
        }
    }

    (routes, mixers)
}

// the init-dict device properties: the parent device name (for
// SPA_KEY_DEVICE_NAME) and the pcm unit indexes this device aggregates
unsafe fn parse_device_dict(info: *const spa_dict) -> (Option<String>, Vec<u32>) {
    let mut pcm_parent_device = None;
    let mut pcm_device_indexes = vec![];

    if let Some(info) = unsafe { info.as_ref() } {
        #[cfg(debug_assertions)]
        unsafe {
            crate::spa::dump_spa_dict(info);
        }

        unsafe {
            crate::spa::for_each_dict_item(info, |key, value| match key {
                crate::keys::PCM_PARENT_DEVICE => {
                    pcm_parent_device = Some(value.to_string());
                }
                crate::keys::PCM_DEVICE_INDEXES => {
                    for part in value.split(',') {
                        if let Ok(index) = part.parse::<u32>() {
                            pcm_device_indexes.push(index);
                        }
                    }
                }
                _ => (),
            });
        }
    }

    (pcm_parent_device, pcm_device_indexes)
}

// the device description shared by every aggregated pcm unit: the longest
// common prefix of their descriptions, trimmed of a dangling " (" tail.
// `pcm_devices` must be non-empty (init rejects an empty list first).
fn common_description(pcm_devices: &[crate::sound::PcmDevice]) -> String {
    let mut common_desc = pcm_devices[0].desc.clone();
    for pcm_device in &pcm_devices[1..] {
        let mut count = 0;

        for (a, b) in common_desc.bytes().zip(pcm_device.desc.bytes()) {
            if a == b {
                count += 1;
            } else {
                break;
            }
        }

        common_desc.truncate(count);
    }

    while common_desc.ends_with(' ') || common_desc.ends_with('(') {
        common_desc.truncate(common_desc.len() - 1);
    }

    common_desc
}

// arm the external-change watchers: the ~1 Hz mixer poll timer and the devd
// socket (jack sense / recording-source flips). Both are best-effort - a
// failure only costs noticing external changes - and only worth arming
// when something is routed.
unsafe fn arm_mixer_watch(state: &mut Runtime) {
    if state.routes.is_empty() {
        return;
    }

    if let (Some(main_loop), Some(system)) = (&state.main_loop, &state.system) {
        match system.timerfd_create(
            libc::CLOCK_MONOTONIC,
            (SPA_FD_CLOEXEC | SPA_FD_NONBLOCK) as c_int,
        ) {
            Err(_) => {
                crate::warn!(
                    state.log,
                    "can't create the mixer poll timer; external volume changes won't be noticed"
                );
            }
            Ok(timer_fd) => {
                let timerspec = itimerspec {
                    it_value: timespec {
                        tv_sec: 1,
                        tv_nsec: 0,
                    },
                    it_interval: timespec {
                        tv_sec: 1,
                        tv_nsec: 0,
                    },
                };
                if timer_fd.settime(0, &timerspec) < 0 {
                    crate::warn!(state.log, "can't arm the mixer poll timer");
                }
                state.timer_source.set_fd(timer_fd.raw());
                state.timer_fd = Some(timer_fd);
                // SAFETY: init runs in the host context accepted by add_source;
                // the pinned source remains alive until clear unregisters it.
                if unsafe { state.timer_source.register(main_loop) } < 0 {
                    crate::warn!(
                        state.log,
                        "can't watch the mixer; external volume changes won't be noticed"
                    );
                    drop(state.timer_fd.take());
                    state.timer_source.set_fd(-1);
                }
            }
        }
    }

    // devd's SND CONN notifications (jack sense, default-unit moves) nudge
    // the same poll so kernel-side recording-source flips show up right
    // away; losing devd only costs that immediacy (jails, minimal systems)
    if let Some(main_loop) = &state.main_loop {
        match crate::utils::DevdSocket::open() {
            Ok(socket) => {
                state.devd_source.set_fd(socket.fd());
                state.devd_socket = Some(socket);
                // SAFETY: as for the mixer source above.
                if unsafe { state.devd_source.register(main_loop) } < 0 {
                    crate::warn!(
                        state.log,
                        "can't watch devd; jack events will wait for the mixer poll"
                    );
                    state.devd_socket = None;
                    state.devd_source.set_fd(-1);
                }
            }
            Err(err) => {
                crate::info!(
                    state.log,
                    "no devd connection ({}); jack events will wait for the mixer poll",
                    err
                );
            }
        }
    }
}

unsafe extern "C" fn init(
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
    let log =
        unsafe { crate::spa::Log::wrap(log, std::ptr::NonNull::new(&raw mut OSS_DEVICE_TOPIC)) };

    // the main loop and system drive the mixer poll timer; both are optional -
    // without them external mixer changes just go unnoticed
    let main_loop =
        unsafe { spa_support_find(support, n_support, SPA_TYPE_INTERFACE_Loop.as_ptr().cast()) }
            as *mut spa_loop;
    let system = unsafe {
        spa_support_find(
            support,
            n_support,
            SPA_TYPE_INTERFACE_System.as_ptr().cast(),
        )
    } as *mut spa_system;
    let main_loop = if main_loop.is_null() {
        None
    } else {
        Some(unsafe { crate::spa::Loop::wrap(main_loop) })
    };
    let system = if system.is_null() {
        None
    } else {
        Some(unsafe { crate::spa::System::wrap(system) })
    };

    let state = handle.cast::<State>();
    assert!(!state.is_null(), "handle is not supposed to be null");

    let (pcm_parent_device, pcm_device_indexes) = unsafe { parse_device_dict(info) };

    if pcm_device_indexes.is_empty() {
        crate::error!(
            log,
            "{} should contain pcm device indexes",
            crate::keys::PCM_DEVICE_INDEXES
        );
        return -libc::EINVAL;
    }

    let pcm_devices = crate::sound::list_pcm_devices(&pcm_device_indexes);

    if pcm_devices.is_empty() {
        crate::error!(log, "can't retrieve pcm device information");
        return -libc::EINVAL;
    }

    let (routes, mixers) = probe_routes(&pcm_devices);
    let common_desc = common_description(&pcm_devices);
    let events = std::rc::Rc::new(DeviceEvents::new());

    // the host hands us uninitialized memory of get_size() bytes; write the
    // whole State without dropping the garbage "old" value
    unsafe {
        std::ptr::write(
            state,
            State {
                handle: spa_handle {
                    version: SPA_VERSION_HANDLE,
                    get_interface: Some(get_interface),
                    clear: Some(clear),
                },

                device: spa_device {
                    iface: spa_interface {
                        type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
                        version: SPA_VERSION_DEVICE,
                        cb: spa_callbacks {
                            funcs: &DEVICE_IMPL as *const _ as *const c_void,
                            data: state as *mut _ as *mut c_void,
                        },
                    },
                },

                runtime: Runtime {
                    events,

                    pcm_devices,
                    description: common_desc,
                    profile: 1, // default on until a session manager decides otherwise
                    profile_save: false,

                    routes,
                    mixers,

                    main_loop,
                    system,

                    timer_fd: None,
                    timer_source: crate::spa::LoopSource::new(spa_source {
                        loop_: std::ptr::null_mut(),
                        func: Some(on_mixer_timeout),
                        data: state.cast::<c_void>(),
                        fd: -1,
                        mask: SPA_IO_IN,
                        rmask: 0,
                        priv_: std::ptr::null_mut(),
                    }),

                    devd_socket: None,
                    devd_source: crate::spa::LoopSource::new(spa_source {
                        loop_: std::ptr::null_mut(),
                        func: Some(on_devd_event),
                        data: state.cast::<c_void>(),
                        fd: -1,
                        mask: SPA_IO_IN,
                        rmask: 0,
                        priv_: std::ptr::null_mut(),
                    }),

                    log,
                },
            },
        );
    }

    unsafe {
        with_runtime_mut(state, |state| {
            let description = state.description.clone();
            state.events.with_info(|info| {
                info.fix_pointers();
                info.add_prop(crate::spa::key(SPA_KEY_DEVICE_API), "freebsd-oss");
                info.add_prop(crate::spa::key(SPA_KEY_MEDIA_CLASS), "Audio/Device");
                if let Some(pcm_parent_device) = pcm_parent_device {
                    info.add_prop(crate::spa::key(SPA_KEY_DEVICE_NAME), pcm_parent_device);
                }
                info.add_prop(
                    crate::spa::key(SPA_KEY_DEVICE_DESCRIPTION),
                    description.as_str(),
                );
                info.add_param(SPA_PARAM_EnumProfile, SPA_PARAM_INFO_READ);
                info.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READWRITE);
                info.add_param(SPA_PARAM_EnumRoute, SPA_PARAM_INFO_READ);
                info.add_param(SPA_PARAM_Route, SPA_PARAM_INFO_READWRITE);
            });

            arm_mixer_watch(state);
        });
    }

    0
}

const INTERFACE_INFO: [spa_interface_info; 1] = [spa_interface_info {
    type_: SPA_TYPE_INTERFACE_Device.as_ptr().cast(),
}];

unsafe extern "C" fn enum_interface_info(
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

const OSS_DEVICE_FACTORY_INFO: spa_dict = spa_dict {
    flags: 0,
    n_items: 0,
    items: std::ptr::null(),
};

pub(crate) const OSS_DEVICE_FACTORY: spa_handle_factory = spa_handle_factory {
    version: SPA_VERSION_HANDLE_FACTORY,
    name: c"freebsd-oss.device".as_ptr(),
    info: &OSS_DEVICE_FACTORY_INFO,
    get_size: Some(get_size),
    init: Some(init),
    enum_interface_info: Some(enum_interface_info),
};

// mut: the host logger writes level/has_custom_level back after registration
pub(crate) static mut OSS_DEVICE_TOPIC: spa_log_topic = spa_log_topic {
    version: SPA_VERSION_LOG_TOPIC,
    topic: c"spa.oss.device".as_ptr(),
    level: SPA_LOG_LEVEL_NONE,
    has_custom_level: false,
};

#[cfg(test)]
mod tests;
