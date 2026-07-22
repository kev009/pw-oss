use super::*;
use crate::backend;
use crate::spa::{
    self, DeviceInfo, Dictionary, ListenerList, LocalDispatchGuard, LocalListenerTarget,
    LocalNotificationQueue, SPA_DEVICE_CHANGE_MASK_ALL, SPA_DEVICE_OBJECT_CHANGE_MASK_ALL, key,
    pod_prop,
};

#[cfg(test)]
mod tests;

// Listener and info state lives outside State so a C callback may re-enter a
// device method without overlapping a Rust reference to the containing State.
// All device methods run on the main loop; Rc/RefCell express that ownership
// without claiming cross-thread access.
pub(super) struct DeviceEvents<B: backend::Backend> {
    pub(super) hooks: ListenerList<spa_device_events>,
    pub(super) info: std::cell::RefCell<DeviceInfo>,
    pub(super) pending: LocalNotificationQueue<DeviceNotification>,
    result_target: LocalListenerTarget<spa_device_events>,
    backend: std::marker::PhantomData<B>,
}

pub(super) enum DeviceObjectEvent {
    Added {
        endpoint: backend::EndpointSnapshot,
        route_count: usize,
    },
    Removed {
        id: u32,
    },
}

pub(super) enum DeviceNotification {
    Info(Box<DeviceInfo>),
    Object(DeviceObjectEvent),
    Event(Vec<u8>),
    Done(c_int),
    ActivateListeners(std::rc::Rc<ListenerList<spa_device_events>>),
}

impl<B: backend::Backend> DeviceEvents<B> {
    pub(super) fn new() -> Self {
        Self {
            hooks: ListenerList::new(),
            info: std::cell::RefCell::new(DeviceInfo::new()),
            pending: LocalNotificationQueue::new(),
            result_target: LocalListenerTarget::new(),
            backend: std::marker::PhantomData,
        }
    }

    pub(super) fn with_info<R>(&self, apply: impl FnOnce(&mut DeviceInfo) -> R) -> R {
        apply(&mut self.info.borrow_mut())
    }

    pub(super) fn initial_info(&self) -> Box<DeviceInfo> {
        let mut snapshot = self.info.borrow().snapshot();
        let _ = snapshot.replace_change_mask(SPA_DEVICE_CHANGE_MASK_ALL as u64);
        snapshot
    }

    pub(super) fn take_info(&self) -> Box<DeviceInfo> {
        let mut info = self.info.borrow_mut();
        let snapshot = info.snapshot();
        let _ = info.replace_change_mask(0);
        snapshot
    }

    // SAFETY: no reference into the associated State may be live; listener
    // code may synchronously re-enter any device method.
    pub(super) unsafe fn emit_info(&self, snapshot: &DeviceInfo) {
        unsafe { self.emit_info_on(&self.hooks, snapshot) };
    }

    // SAFETY: as emit_info(); `hooks` is either the active list or one
    // isolated activation batch with the same event-table type.
    pub(super) unsafe fn emit_info_on(
        &self,
        hooks: &ListenerList<spa_device_events>,
        snapshot: &DeviceInfo,
    ) {
        hooks.emit(|f, data| {
            if let Some(info) = f.info {
                // through the C listener vtable (the add_listener contract)
                unsafe { info(data, snapshot.raw()) };
            }
        });
    }

    // SAFETY: as emit_info().
    pub(super) unsafe fn emit_done(&self, seq: c_int) {
        self.hooks.emit(|f, data| {
            if let Some(result) = f.result {
                // through the C listener vtable (the add_listener contract)
                unsafe { result(data, seq, 0, 0, std::ptr::null()) };
            }
        });
    }

    // SAFETY: as emit_info().
    pub(super) unsafe fn emit_result(&self, seq: c_int, result: &spa_result_device_params) {
        self.result_target.with_current(&self.hooks, |hooks| {
            spa::dev_emit_result(hooks, seq, 0, SPA_RESULT_TYPE_DEVICE_PARAMS, result);
        });
    }

    // SAFETY: as emit_info().
    pub(super) unsafe fn emit_object(&self, event: &DeviceObjectEvent) {
        unsafe { self.emit_object_on(&self.hooks, event) };
    }

    // SAFETY: as emit_info_on().
    pub(super) unsafe fn emit_object_on(
        &self,
        hooks: &ListenerList<spa_device_events>,
        event: &DeviceObjectEvent,
    ) {
        match event {
            DeviceObjectEvent::Removed { id } => hooks.emit(|f, data| {
                if let Some(object_info) = f.object_info {
                    unsafe { object_info(data, *id, std::ptr::null()) };
                }
            }),
            DeviceObjectEvent::Added {
                endpoint,
                route_count,
            } => {
                let id = endpoint.object_id;
                let rec = endpoint.direction == backend::StreamDirection::Capture;
                let mut dict = Dictionary::new();
                dict.add_item(key(SPA_KEY_NODE_NAME), endpoint.name.as_str());
                dict.add_item(key(SPA_KEY_NODE_DESCRIPTION), endpoint.description.as_str());
                dict.add_item(B::STREAM_PATH, endpoint.locator.value.as_str());
                if *route_count > 0 {
                    dict.add_item("card.profile.device", format!("{id}"));
                    dict.add_item("device.routes", format!("{route_count}"));
                }
                let info = spa_device_object_info {
                    version: SPA_VERSION_DEVICE_OBJECT_INFO,
                    type_: SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
                    factory_name: if rec {
                        B::SOURCE_FACTORY_NAME.as_ptr()
                    } else {
                        B::SINK_FACTORY_NAME.as_ptr()
                    },
                    change_mask: SPA_DEVICE_OBJECT_CHANGE_MASK_ALL as u64,
                    flags: 0,
                    props: dict.raw(),
                };
                // Keep the dictionary beside the callback payload for the
                // entire traversal.
                hooks.emit(|f, data| {
                    if let Some(object_info) = f.object_info {
                        unsafe { object_info(data, id, &info) };
                    }
                });
            }
        }
    }

    // SAFETY: as emit_info().
    pub(super) unsafe fn dispatch(&self, notification: &DeviceNotification) {
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
    pub(super) unsafe fn with_new_listener<R>(
        &self,
        listener: *mut spa_hook,
        events: *const spa_device_events,
        data: *mut c_void,
        initial: impl FnOnce(&ListenerList<spa_device_events>) -> R,
    ) -> R {
        let deferred = self.pending.defer_when_busy(|| {
            let hooks = std::rc::Rc::new(ListenerList::new());
            (DeviceNotification::ActivateListeners(hooks.clone()), hooks)
        });
        let hooks = deferred.as_deref().unwrap_or(&self.hooks);
        unsafe {
            hooks.with_isolated_listener(listener, events, data, || {
                self.result_target.scoped(hooks, || initial(hooks))
            })
        }
    }

    // Claim the main-loop endpoint's dispatch turn. Reentrant methods append
    // complete transactions to the FIFO and return; the outer owner drains
    // them after finishing its current transaction.
    pub(super) fn begin_dispatch(&self) -> Option<LocalDispatchGuard<'_, DeviceNotification>> {
        self.pending.begin_dispatch()
    }

    // SAFETY: as emit_info(); only the begin_dispatch() owner may call this.
    // RefCell borrows end before every callback, so listener reentry can append.
    pub(super) unsafe fn drain(&self, guard: LocalDispatchGuard<'_, DeviceNotification>) {
        self.pending.drain(guard, |notification| unsafe {
            self.dispatch(&notification);
        });
    }

    // SAFETY: as emit_info(). The entire input vector is enqueued atomically
    // before dispatch starts, preserving transaction order under reentry.
    pub(super) unsafe fn dispatch_all(&self, notifications: Vec<DeviceNotification>) {
        self.pending
            .dispatch_all(notifications, |notification| unsafe {
                self.dispatch(&notification);
            });
    }
}

// Build owned add/remove events while State is borrowed. Dispatch happens
// afterward, one traversal per object, with no State reference alive.
pub(super) fn object_events(
    snapshot: &backend::DeviceSnapshot,
    routes: &[RouteState],
    present: bool,
) -> Vec<DeviceObjectEvent> {
    let mut events = Vec::new();
    for endpoint in &snapshot.endpoints {
        let id = endpoint.object_id;
        if !present {
            events.push(DeviceObjectEvent::Removed { id });
            continue;
        }
        let route_count = routes.iter().filter(|r| r.node_id == id).count();
        events.push(DeviceObjectEvent::Added {
            endpoint: endpoint.clone(),
            route_count,
        });
    }
    events
}
pub(super) fn build_object_config(
    node_id: u32,
    volume: Option<(&[f32], bool, &[u32])>,
    mute: Option<bool>,
) -> Vec<u8> {
    use libspa::pod::{Object, Value, ValueArray};
    use libspa::utils::Id;

    let mut props = vec![];

    if let Some((volumes, hw, channels)) = volume {
        let volumes = volumes.to_vec();
        // hardware attenuation keeps the node at unity; a soft route IS the
        // node's software volume, so audioconvert applies the levels
        let soft = if hw {
            vec![1.0; volumes.len()]
        } else {
            volumes.clone()
        };
        props.push(pod_prop(
            SPA_PROP_channelVolumes,
            Value::ValueArray(ValueArray::Float(volumes)),
        ));
        props.push(pod_prop(
            SPA_PROP_channelMap,
            Value::ValueArray(ValueArray::Id(channels.iter().map(|&c| Id(c)).collect())),
        ));
        props.push(pod_prop(
            SPA_PROP_softVolumes,
            Value::ValueArray(ValueArray::Float(soft)),
        ));
    }

    if let Some(mute) = mute {
        props.push(pod_prop(SPA_PROP_mute, Value::Bool(mute)));
        props.push(pod_prop(SPA_PROP_softMute, Value::Bool(mute)));
    }

    spa::serialize_pod(&Value::Object(Object {
        type_: SPA_TYPE_EVENT_Device,
        id: SPA_DEVICE_EVENT_ObjectConfig,
        properties: vec![
            pod_prop(SPA_EVENT_DEVICE_Object, Value::Int(node_id as i32)),
            pod_prop(
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
