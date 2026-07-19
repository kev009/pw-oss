use super::*;

struct ReentrantDeviceInfoContext {
    events: *const DeviceEvents,
    seen: Vec<u32>,
}

unsafe extern "C" fn reentrant_device_info(data: *mut c_void, info: *const spa_device_info) {
    let context = unsafe { &mut *data.cast::<ReentrantDeviceInfoContext>() };
    let info = unsafe { &*info };
    let params = unsafe { std::slice::from_raw_parts(info.params, info.n_params as usize) };
    context.seen.push(
        params
            .iter()
            .find(|param| param.id == SPA_PARAM_Profile)
            .expect("Profile is published")
            .flags,
    );
    if context.seen.len() == 1 {
        let events = unsafe { &*context.events };
        events.with_info(|info| info.bump_param(SPA_PARAM_Profile));
        let nested = DeviceNotification::Info(events.take_info());
        // SAFETY: the test owns no State; the endpoint queues this behind
        // the remaining outer notification.
        unsafe { events.dispatch_all(vec![nested]) };
    }
}

#[test]
fn device_notifications_preserve_fifo_order_under_reentry() {
    let events = DeviceEvents::new();
    events.with_info(|info| {
        info.fix_pointers();
        info.add_param(SPA_PARAM_Profile, SPA_PARAM_INFO_READ);
    });

    let mut context = ReentrantDeviceInfoContext {
        events: &events,
        seen: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.info = Some(reentrant_device_info);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || {
        events.with_info(|info| info.bump_param(SPA_PARAM_Profile));
        let first = DeviceNotification::Info(events.take_info());
        events.with_info(|info| info.bump_param(SPA_PARAM_Profile));
        let second = DeviceNotification::Info(events.take_info());
        unsafe { events.dispatch_all(vec![first, second]) };
    };
    unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        );
    }

    assert_eq!(
        context.seen,
        [
            SPA_PARAM_INFO_READ | SPA_PARAM_INFO_SERIAL,
            SPA_PARAM_INFO_READ,
            SPA_PARAM_INFO_READ | SPA_PARAM_INFO_SERIAL,
        ]
    );
}

struct InitialDeviceContext {
    events: *const DeviceEvents,
    sequence: Vec<&'static str>,
}

unsafe extern "C" fn initial_device_info(data: *mut c_void, _info: *const spa_device_info) {
    let context = unsafe { &mut *data.cast::<InitialDeviceContext>() };
    context.sequence.push("info");
    let events = unsafe { &*context.events };
    unsafe {
        events.dispatch_all(vec![DeviceNotification::Object(
            DeviceObjectEvent::Removed { id: 2 },
        )]);
    }
}

unsafe extern "C" fn initial_device_object(
    data: *mut c_void,
    _id: u32,
    info: *const spa_device_object_info,
) {
    let context = unsafe { &mut *data.cast::<InitialDeviceContext>() };
    context
        .sequence
        .push(if info.is_null() { "removed" } else { "added" });
}

#[test]
fn initial_device_transaction_finishes_before_reentrant_changes() {
    let events = DeviceEvents::new();
    events.with_info(|info| info.fix_pointers());
    let info = events.initial_info();
    let initial_object = DeviceObjectEvent::Added {
        id: 2,
        rec: false,
        description: "Playback".into(),
        route_count: 0,
    };
    let mut context = InitialDeviceContext {
        events: &events,
        sequence: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.info = Some(initial_device_info);
    table.object_info = Some(initial_device_object);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || {
        let dispatch_guard = events.begin_dispatch().expect("the test owns dispatch");
        unsafe {
            events.emit_info(&info);
            events.emit_object(&initial_object);
        }
        dispatch_guard
    };
    let dispatch_guard = unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        )
    };
    unsafe {
        events.drain(dispatch_guard);
    }
    assert_eq!(context.sequence, ["info", "added", "removed"]);
}

struct LateDeviceListener {
    seen: Vec<u32>,
}

unsafe extern "C" fn record_late_device_object(
    data: *mut c_void,
    id: u32,
    _info: *const spa_device_object_info,
) {
    unsafe { &mut *data.cast::<LateDeviceListener>() }
        .seen
        .push(id);
}

struct AddDeviceListenerContext {
    events: *const DeviceEvents,
    late_hook: *mut spa_hook,
    late_table: *const spa_device_events,
    late_data: *mut c_void,
    seen: Vec<u32>,
}

unsafe extern "C" fn add_device_listener_during_dispatch(
    data: *mut c_void,
    id: u32,
    _info: *const spa_device_object_info,
) {
    let context = unsafe { &mut *data.cast::<AddDeviceListenerContext>() };
    context.seen.push(id);
    if context.seen.len() != 1 {
        return;
    }
    let events = unsafe { &*context.events };
    let initial = |hooks: &crate::spa::ListenerList<spa_device_events>| unsafe {
        events.emit_object_on(hooks, &DeviceObjectEvent::Removed { id: 3 });
    };
    unsafe {
        events.with_new_listener(
            context.late_hook,
            context.late_table,
            context.late_data,
            initial,
        );
        events.dispatch_all(vec![DeviceNotification::Object(
            DeviceObjectEvent::Removed { id: 4 },
        )]);
    }
}

#[test]
fn device_listener_added_during_dispatch_starts_at_its_barrier() {
    let events = DeviceEvents::new();
    let mut late = LateDeviceListener { seen: Vec::new() };
    let mut late_table: spa_device_events = unsafe { std::mem::zeroed() };
    late_table.version = SPA_VERSION_DEVICE_EVENTS;
    late_table.object_info = Some(record_late_device_object);
    let mut late_hook: spa_hook = unsafe { std::mem::zeroed() };
    let mut context = AddDeviceListenerContext {
        events: &events,
        late_hook: &mut late_hook,
        late_table: &late_table,
        late_data: (&raw mut late).cast(),
        seen: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.object_info = Some(add_device_listener_during_dispatch);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    unsafe {
        events.with_new_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            |_hooks| {},
        );
        events.dispatch_all(vec![
            DeviceNotification::Object(DeviceObjectEvent::Removed { id: 1 }),
            DeviceNotification::Object(DeviceObjectEvent::Removed { id: 2 }),
        ]);
        events.dispatch_all(vec![DeviceNotification::Object(
            DeviceObjectEvent::Removed { id: 5 },
        )]);
    }

    assert_eq!(context.seen, [1, 2, 4, 5]);
    assert_eq!(late.seen, [3, 4, 5]);
}

struct DoneBarrierContext {
    events: *const DeviceEvents,
    sequence: Vec<&'static str>,
}

unsafe extern "C" fn done_barrier_info(data: *mut c_void, _info: *const spa_device_info) {
    let context = unsafe { &mut *data.cast::<DoneBarrierContext>() };
    context.sequence.push("info");
    if context.sequence.len() == 1 {
        let events = unsafe { &*context.events };
        unsafe { events.dispatch_all(vec![DeviceNotification::Done(7)]) };
    }
}

unsafe extern "C" fn done_barrier_result(
    data: *mut c_void,
    seq: c_int,
    _res: c_int,
    _type: u32,
    _result: *const c_void,
) {
    assert_eq!(seq, 7);
    unsafe { &mut *data.cast::<DoneBarrierContext>() }
        .sequence
        .push("done");
}

#[test]
fn device_done_does_not_overtake_an_active_transaction() {
    let events = DeviceEvents::new();
    events.with_info(|info| info.fix_pointers());
    let first = DeviceNotification::Info(events.take_info());
    let second = DeviceNotification::Info(events.take_info());
    let mut context = DoneBarrierContext {
        events: &events,
        sequence: Vec::new(),
    };
    let mut table: spa_device_events = unsafe { std::mem::zeroed() };
    table.version = SPA_VERSION_DEVICE_EVENTS;
    table.info = Some(done_barrier_info);
    table.result = Some(done_barrier_result);
    let mut hook: spa_hook = unsafe { std::mem::zeroed() };
    let initial = || unsafe { events.dispatch_all(vec![first, second]) };
    unsafe {
        events.hooks.with_isolated_listener(
            &mut hook,
            &raw const table,
            (&raw mut context).cast(),
            initial,
        );
    }
    assert_eq!(context.sequence, ["info", "info", "done"]);
}

fn pcm_device(index: u32, play: bool, rec: bool) -> crate::sound::PcmDevice {
    crate::sound::PcmDevice {
        index,
        desc: format!("pcm{index}"),
        location: "hdac0".to_string(),
        play,
        rec,
    }
}

fn route(control: Option<u32>, rec: bool) -> RouteState {
    RouteState {
        node_id: if rec { 3 } else { 2 },
        rec,
        name: "analog-output".to_string(),
        description: "Analog Output".to_string(),
        priority: 100,
        mixer: 0,
        control,
        follows_recsrc: false,
        source: None,
        active: true,
        levels: (75, 50),
        mute: rec,
        save: !rec,
    }
}

// Parse-back semantics (see the note on the utils tests): run the
// Profile and Route pods back through the same libspa PodDeserializer
// WirePlumber uses and pin the parsed content it depends on - keys,
// values, the nested classes/props shape and especially the
// per-property HARDWARE/READONLY flags no other test covers.
#[test]
fn profile_parses_back_with_classes_struct() {
    use libspa::pod::{Object, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::Id;

    // the current default profile of a duplex card: strings, nested
    // structs, an int array and the save bool
    let pod = super::build_profile_info(
        SPA_PARAM_Profile,
        1,
        &[pcm_device(0, true, true)],
        true,
        true,
    );
    let prop = crate::utils::pod_prop;
    assert_eq!(
        crate::utils::parse_back(&pod),
        Value::Object(Object {
            type_: SPA_TYPE_OBJECT_ParamProfile,
            id: SPA_PARAM_Profile,
            properties: vec![
                prop(SPA_PARAM_PROFILE_index, Value::Int(1)),
                prop(SPA_PARAM_PROFILE_name, Value::String("default".to_string())),
                prop(
                    SPA_PARAM_PROFILE_description,
                    Value::String("Default".to_string()),
                ),
                prop(SPA_PARAM_PROFILE_priority, Value::Int(100)),
                prop(
                    SPA_PARAM_PROFILE_available,
                    Value::Id(Id(SPA_PARAM_AVAILABILITY_yes)),
                ),
                // the classes struct select-routes walks: a count, then
                // one class struct with the node ids (capture odd,
                // playback even)
                prop(
                    SPA_PARAM_PROFILE_classes,
                    Value::Struct(vec![
                        Value::Int(2),
                        Value::Struct(vec![
                            Value::String("Audio/Source".to_string()),
                            Value::Int(1),
                            Value::String("card.profile.devices".to_string()),
                            Value::ValueArray(ValueArray::Int(vec![1])),
                        ]),
                        Value::Struct(vec![
                            Value::String("Audio/Sink".to_string()),
                            Value::Int(1),
                            Value::String("card.profile.devices".to_string()),
                            Value::ValueArray(ValueArray::Int(vec![0])),
                        ]),
                    ]),
                ),
                prop(SPA_PARAM_PROFILE_save, Value::Bool(true)),
            ],
        })
    );
}

#[test]
fn route_parses_back_with_hardware_volume_flags() {
    use libspa::pod::{Object, Property, PropertyFlags, Value, ValueArray};
    use libspa::sys::*;
    use libspa::utils::Id;

    // a full playback route with a hardware control: the nested Props
    // object, float/id arrays and the HARDWARE/READONLY prop flags
    let pod = super::build_route_info(SPA_PARAM_Route, &route(Some(0), false), 1, 1, true);
    let prop = crate::utils::pod_prop;
    assert_eq!(
        crate::utils::parse_back(&pod),
        Value::Object(Object {
            type_: SPA_TYPE_OBJECT_ParamRoute,
            id: SPA_PARAM_Route,
            properties: vec![
                prop(SPA_PARAM_ROUTE_index, Value::Int(1)),
                prop(
                    SPA_PARAM_ROUTE_direction,
                    Value::Id(Id(SPA_DIRECTION_OUTPUT)),
                ),
                prop(
                    SPA_PARAM_ROUTE_name,
                    Value::String("analog-output".to_string()),
                ),
                prop(
                    SPA_PARAM_ROUTE_description,
                    Value::String("Analog Output".to_string()),
                ),
                prop(SPA_PARAM_ROUTE_priority, Value::Int(100)),
                prop(
                    SPA_PARAM_ROUTE_available,
                    Value::Id(Id(SPA_PARAM_AVAILABILITY_yes)),
                ),
                prop(
                    SPA_PARAM_ROUTE_profiles,
                    Value::ValueArray(ValueArray::Int(vec![1])),
                ),
                prop(
                    SPA_PARAM_ROUTE_devices,
                    Value::ValueArray(ValueArray::Int(vec![2])),
                ),
                prop(SPA_PARAM_ROUTE_device, Value::Int(2)),
                prop(
                    SPA_PARAM_ROUTE_props,
                    Value::Object(Object {
                        type_: SPA_TYPE_OBJECT_Props,
                        id: SPA_PARAM_Route,
                        properties: vec![
                            // HARDWARE on mute and channelVolumes: the
                            // mixer control owns them, so pulse and the
                            // session manager write them at the card
                            Property {
                                key: SPA_PROP_mute,
                                flags: PropertyFlags::HARDWARE,
                                value: Value::Bool(false),
                            },
                            // the cubic taper: (75/100)^3, (50/100)^3
                            Property {
                                key: SPA_PROP_channelVolumes,
                                flags: PropertyFlags::HARDWARE,
                                value: Value::ValueArray(ValueArray::Float(vec![0.421875, 0.125,])),
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
                            prop(
                                SPA_PROP_channelMap,
                                Value::ValueArray(ValueArray::Id(vec![
                                    Id(SPA_AUDIO_CHANNEL_FL),
                                    Id(SPA_AUDIO_CHANNEL_FR),
                                ])),
                            ),
                            // unity soft volume: the hardware control
                            // attenuates, audioconvert must not double it
                            prop(
                                SPA_PROP_softVolumes,
                                Value::ValueArray(ValueArray::Float(vec![1.0, 1.0])),
                            ),
                        ],
                    }),
                ),
                prop(SPA_PARAM_ROUTE_profile, Value::Int(1)),
                prop(SPA_PARAM_ROUTE_save, Value::Bool(true)),
            ],
        })
    );
}

// Profile requests accept NULL reset, valid indexes, and durable names.
#[test]
fn profile_requests_decode_and_validate() {
    use libspa::pod::{Object, Value};
    use libspa::sys::*;
    let prop = crate::utils::pod_prop;

    assert_eq!(
        super::decode_profile_request(None),
        Ok(super::ProfileRequest {
            index: 1,
            save: false
        })
    );
    let pod = |props| {
        Some(Value::Object(Object {
            type_: SPA_TYPE_OBJECT_ParamProfile,
            id: SPA_PARAM_Profile,
            properties: props,
        }))
    };
    assert_eq!(
        super::decode_profile_request(pod(vec![
            prop(SPA_PARAM_PROFILE_index, Value::Int(0)),
            prop(SPA_PARAM_PROFILE_save, Value::Bool(true)),
        ])),
        Ok(super::ProfileRequest {
            index: 0,
            save: true
        })
    );
    // Resolve durable profile names.
    assert_eq!(
        super::decode_profile_request(pod(vec![prop(
            SPA_PARAM_PROFILE_name,
            Value::String("off".into())
        )])),
        Ok(super::ProfileRequest {
            index: 0,
            save: false
        })
    );
    // out-of-range index is ignored, an unknown name resolves nothing
    assert_eq!(
        super::decode_profile_request(pod(vec![
            prop(SPA_PARAM_PROFILE_index, Value::Int(7)),
            prop(SPA_PARAM_PROFILE_name, Value::String("bogus".into())),
        ])),
        Err(-libc::EINVAL)
    );
    // a non-Profile object is rejected whole
    assert_eq!(
        super::decode_profile_request(Some(Value::Int(1))),
        Err(-libc::EINVAL)
    );
}

// Route requests require a device and ignore unsupported properties.
#[test]
fn route_requests_decode_with_typed_props() {
    use libspa::pod::{Object, Value, ValueArray};
    use libspa::sys::*;
    let prop = crate::utils::pod_prop;

    let pod = |props| {
        Some(Value::Object(Object {
            type_: SPA_TYPE_OBJECT_ParamRoute,
            id: SPA_PARAM_Route,
            properties: props,
        }))
    };
    assert_eq!(
        super::decode_route_request(pod(vec![
            prop(SPA_PARAM_ROUTE_index, Value::Int(1)),
            prop(SPA_PARAM_ROUTE_name, Value::String("oss-output".into())),
            prop(SPA_PARAM_ROUTE_device, Value::Int(2)),
            prop(SPA_PARAM_ROUTE_save, Value::Bool(true)),
            prop(
                SPA_PARAM_ROUTE_props,
                Value::Object(Object {
                    type_: SPA_TYPE_OBJECT_Props,
                    id: SPA_PARAM_Route,
                    properties: vec![
                        prop(
                            SPA_PROP_channelVolumes,
                            Value::ValueArray(ValueArray::Float(vec![0.5, 0.25])),
                        ),
                        prop(SPA_PROP_mute, Value::Bool(true)),
                        // ignored at decode: softVolumes ride along
                        prop(
                            SPA_PROP_softVolumes,
                            Value::ValueArray(ValueArray::Float(vec![1.0, 1.0])),
                        ),
                    ],
                }),
            ),
        ])),
        Ok(super::RouteRequest {
            index: Some(1),
            name: Some("oss-output".into()),
            device: 2,
            save: true,
            props: Some(super::RouteProps(vec![
                super::RouteProp::ChannelVolumes(vec![0.5, 0.25]),
                super::RouteProp::Mute(true),
            ])),
        })
    );
    // an empty volume array is dropped before it can become a mixer write
    assert_eq!(
        super::decode_route_request(pod(vec![
            prop(SPA_PARAM_ROUTE_device, Value::Int(2)),
            prop(
                SPA_PARAM_ROUTE_props,
                Value::Object(Object {
                    type_: SPA_TYPE_OBJECT_Props,
                    id: SPA_PARAM_Route,
                    properties: vec![prop(
                        SPA_PROP_channelVolumes,
                        Value::ValueArray(ValueArray::Float(vec![])),
                    )],
                }),
            ),
        ])),
        Ok(super::RouteRequest {
            index: None,
            name: None,
            device: 2,
            save: false,
            props: Some(super::RouteProps::default()),
        })
    );
    // no device: unaddressable; no pod: nothing to reset
    assert_eq!(
        super::decode_route_request(pod(vec![prop(SPA_PARAM_ROUTE_index, Value::Int(0))])),
        Err(-libc::EINVAL)
    );
    assert_eq!(super::decode_route_request(None), Err(-libc::EINVAL));
}
