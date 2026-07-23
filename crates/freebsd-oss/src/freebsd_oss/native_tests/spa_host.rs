use std::ffi::{CStr, CString, c_int, c_void};
use std::mem::{MaybeUninit, size_of};

use libspa::pod::{Object, Property, Value, ValueArray};
use libspa::sys::*;
use libspa::utils::Id;

use crate::freebsd_oss::DspWriter;

struct TestLoopState {
    loop_: *mut spa_loop,
    source: *mut spa_source,
}

struct TestLoop {
    _methods: Box<spa_loop_methods>,
    state: Box<TestLoopState>,
    interface: Box<spa_loop>,
}

unsafe extern "C" fn add_source(object: *mut c_void, source: *mut spa_source) -> c_int {
    let state = unsafe { &mut *object.cast::<TestLoopState>() };
    assert!(state.source.is_null(), "the test loop supports one source");
    state.source = source;
    unsafe {
        (*source).loop_ = state.loop_;
    }
    0
}

unsafe extern "C" fn update_source(_object: *mut c_void, _source: *mut spa_source) -> c_int {
    0
}

unsafe extern "C" fn remove_source(object: *mut c_void, source: *mut spa_source) -> c_int {
    let state = unsafe { &mut *object.cast::<TestLoopState>() };
    assert_eq!(state.source, source);
    state.source = std::ptr::null_mut();
    unsafe {
        (*source).loop_ = std::ptr::null_mut();
    }
    0
}

unsafe extern "C" fn invoke(
    object: *mut c_void,
    func: spa_invoke_func_t,
    seq: u32,
    data: *const c_void,
    size: usize,
    _block: bool,
    user_data: *mut c_void,
) -> c_int {
    let state = unsafe { &mut *object.cast::<TestLoopState>() };
    unsafe {
        func.expect("SPA loop invoke requires a callback")(
            state.loop_,
            false,
            seq,
            data,
            size,
            user_data,
        )
    }
}

impl TestLoop {
    fn new() -> Self {
        let mut methods = Box::new(unsafe { std::mem::zeroed::<spa_loop_methods>() });
        methods.version = SPA_VERSION_LOOP_METHODS;
        methods.add_source = Some(add_source);
        methods.update_source = Some(update_source);
        methods.remove_source = Some(remove_source);
        methods.invoke = Some(invoke);

        let mut state = Box::new(TestLoopState {
            loop_: std::ptr::null_mut(),
            source: std::ptr::null_mut(),
        });
        let mut interface = Box::new(spa_loop {
            iface: spa_interface {
                type_: SPA_TYPE_INTERFACE_DataLoop.as_ptr().cast(),
                version: SPA_VERSION_LOOP,
                cb: spa_callbacks {
                    funcs: std::ptr::from_ref(methods.as_ref()).cast(),
                    data: (&raw mut *state).cast(),
                },
            },
        });
        state.loop_ = &raw mut *interface;
        Self {
            _methods: methods,
            state,
            interface,
        }
    }

    fn raw(&mut self) -> *mut spa_loop {
        &raw mut *self.interface
    }
}

impl Drop for TestLoop {
    fn drop(&mut self) {
        assert!(
            self.state.source.is_null(),
            "the SPA handle must unregister its loop source before host teardown"
        );
    }
}

unsafe extern "C" fn system_close(_object: *mut c_void, fd: c_int) -> c_int {
    unsafe { libc::close(fd) }
}

unsafe extern "C" fn system_clock_gettime(
    _object: *mut c_void,
    clock_id: c_int,
    value: *mut timespec,
) -> c_int {
    unsafe { libc::clock_gettime(clock_id, value.cast()) }
}

// The smoke test forces timer mode but advances the graph by calling process()
// directly; it never dispatches this source. A kqueue supplies a valid owned
// descriptor, while arm/read are deliberately inert rather than a timerfd
// emulation. Real timer behavior is covered by the native wake-driver tests.
unsafe extern "C" fn timerfd_create(
    _object: *mut c_void,
    _clock_id: c_int,
    _flags: c_int,
) -> c_int {
    unsafe { libc::kqueue() }
}

unsafe extern "C" fn timerfd_settime(
    _object: *mut c_void,
    _fd: c_int,
    _flags: c_int,
    _new_value: *const itimerspec,
    old_value: *mut itimerspec,
) -> c_int {
    if !old_value.is_null() {
        unsafe {
            old_value.write(std::mem::zeroed());
        }
    }
    0
}

unsafe extern "C" fn timerfd_read(
    _object: *mut c_void,
    _fd: c_int,
    expirations: *mut u64,
) -> c_int {
    if !expirations.is_null() {
        unsafe {
            expirations.write(0);
        }
    }
    -libc::EAGAIN
}

struct TestSystem {
    _methods: Box<spa_system_methods>,
    interface: Box<spa_system>,
}

impl TestSystem {
    fn new() -> Self {
        let mut methods = Box::new(unsafe { std::mem::zeroed::<spa_system_methods>() });
        methods.version = SPA_VERSION_SYSTEM_METHODS;
        methods.close = Some(system_close);
        methods.clock_gettime = Some(system_clock_gettime);
        methods.timerfd_create = Some(timerfd_create);
        methods.timerfd_settime = Some(timerfd_settime);
        methods.timerfd_read = Some(timerfd_read);
        let interface = Box::new(spa_system {
            iface: spa_interface {
                type_: SPA_TYPE_INTERFACE_DataSystem.as_ptr().cast(),
                version: SPA_VERSION_SYSTEM,
                cb: spa_callbacks {
                    funcs: std::ptr::from_ref(methods.as_ref()).cast(),
                    data: std::ptr::null_mut(),
                },
            },
        });
        Self {
            _methods: methods,
            interface,
        }
    }

    fn raw(&mut self) -> *mut spa_system {
        &raw mut *self.interface
    }
}

struct TestLog {
    _methods: Box<spa_log_methods>,
    interface: Box<spa_log>,
}

impl TestLog {
    fn new() -> Self {
        let mut methods = Box::new(unsafe { std::mem::zeroed::<spa_log_methods>() });
        methods.version = SPA_VERSION_LOG_METHODS;
        let interface = Box::new(spa_log {
            iface: spa_interface {
                type_: SPA_TYPE_INTERFACE_Log.as_ptr().cast(),
                version: SPA_VERSION_LOG,
                cb: spa_callbacks {
                    funcs: std::ptr::from_ref(methods.as_ref()).cast(),
                    data: std::ptr::null_mut(),
                },
            },
            level: SPA_LOG_LEVEL_NONE,
        });
        Self {
            _methods: methods,
            interface,
        }
    }

    fn raw(&mut self) -> *mut spa_log {
        &raw mut *self.interface
    }
}

struct TestHost {
    loop_: TestLoop,
    system: TestSystem,
    log: TestLog,
}

impl TestHost {
    fn new() -> Self {
        Self {
            loop_: TestLoop::new(),
            system: TestSystem::new(),
            log: TestLog::new(),
        }
    }

    fn support(&mut self) -> [spa_support; 4] {
        let loop_ = self.loop_.raw().cast();
        [
            spa_support {
                type_: SPA_TYPE_INTERFACE_Log.as_ptr().cast(),
                data: self.log.raw().cast(),
            },
            spa_support {
                type_: SPA_TYPE_INTERFACE_DataLoop.as_ptr().cast(),
                data: loop_,
            },
            spa_support {
                type_: SPA_TYPE_INTERFACE_DataSystem.as_ptr().cast(),
                data: self.system.raw().cast(),
            },
            spa_support {
                type_: SPA_TYPE_INTERFACE_Loop.as_ptr().cast(),
                data: loop_,
            },
        ]
    }
}

struct FactoryHandle {
    storage: Vec<MaybeUninit<u128>>,
    initialized: bool,
}

impl FactoryHandle {
    unsafe fn new(
        factory: *const spa_handle_factory,
        info: &spa_dict,
        support: &[spa_support],
    ) -> Self {
        let get_size = unsafe { (*factory).get_size }.expect("factory must publish get_size");
        let size = unsafe { get_size(factory, info) };
        let words = size.div_ceil(size_of::<u128>()).max(1);
        let mut this = Self {
            storage: vec![MaybeUninit::uninit(); words],
            initialized: false,
        };
        let init = unsafe { (*factory).init }.expect("factory must publish init");
        let result = unsafe {
            init(
                factory,
                this.raw(),
                info,
                support.as_ptr(),
                support.len() as u32,
            )
        };
        assert_eq!(result, 0, "SPA factory initialization failed");
        this.initialized = true;
        this
    }

    fn raw(&mut self) -> *mut spa_handle {
        self.storage.as_mut_ptr().cast()
    }

    unsafe fn node(&mut self) -> *mut spa_node {
        let handle = self.raw();
        let get_interface = unsafe { (*handle).get_interface }
            .expect("initialized handle must publish get_interface");
        let mut interface = std::ptr::null_mut();
        let result = unsafe {
            get_interface(
                handle,
                SPA_TYPE_INTERFACE_Node.as_ptr().cast(),
                &raw mut interface,
            )
        };
        assert_eq!(result, 0, "sink factory did not provide spa_node");
        interface.cast()
    }
}

impl Drop for FactoryHandle {
    fn drop(&mut self) {
        if self.initialized {
            let handle = self.raw();
            let clear = unsafe { (*handle).clear }.expect("initialized handle must publish clear");
            assert_eq!(unsafe { clear(handle) }, 0, "clearing SPA handle failed");
        }
    }
}

struct AlignedPod {
    words: Vec<MaybeUninit<u64>>,
}

impl AlignedPod {
    fn format(rate: i32) -> Self {
        use libspa::pod::serialize::PodSerializer;

        let value = Value::Object(Object {
            type_: SPA_TYPE_OBJECT_Format,
            id: SPA_PARAM_Format,
            properties: vec![
                Property::new(SPA_FORMAT_mediaType, Value::Id(Id(SPA_MEDIA_TYPE_audio))),
                Property::new(
                    SPA_FORMAT_mediaSubtype,
                    Value::Id(Id(SPA_MEDIA_SUBTYPE_raw)),
                ),
                Property::new(
                    SPA_FORMAT_AUDIO_format,
                    Value::Id(Id(SPA_AUDIO_FORMAT_S16_LE)),
                ),
                Property::new(SPA_FORMAT_AUDIO_rate, Value::Int(rate)),
                Property::new(SPA_FORMAT_AUDIO_channels, Value::Int(2)),
                Property::new(
                    SPA_FORMAT_AUDIO_position,
                    Value::ValueArray(ValueArray::Id(vec![
                        Id(SPA_AUDIO_CHANNEL_FL),
                        Id(SPA_AUDIO_CHANNEL_FR),
                    ])),
                ),
            ],
        });
        let bytes = PodSerializer::serialize(std::io::Cursor::new(Vec::new()), &value)
            .expect("the fixed format pod must serialize")
            .0
            .into_inner();
        let mut words = vec![MaybeUninit::zeroed(); bytes.len().div_ceil(size_of::<u64>())];
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                words.as_mut_ptr().cast::<u8>(),
                bytes.len(),
            );
        }
        Self { words }
    }

    fn as_pod(&self) -> *const spa_pod {
        self.words.as_ptr().cast()
    }
}

struct BufferFixture {
    // Fields drop in declaration order: holders precede every pointee so the
    // raw SPA pointer chain is valid throughout each holder's destruction.
    buffer: Box<spa_buffer>,
    data: Box<spa_data>,
    chunk: Box<spa_chunk>,
    io: Box<spa_io_buffers>,
    payload: Vec<u8>,
}

impl BufferFixture {
    fn new() -> Self {
        let mut payload = vec![0x5a; 8_192];
        let mut chunk = Box::new(spa_chunk {
            offset: 0,
            size: 4_096,
            stride: 4,
            flags: 0,
        });
        let mut data = Box::new(spa_data {
            type_: SPA_DATA_MemPtr,
            flags: 0,
            fd: -1,
            mapoffset: 0,
            maxsize: payload.len() as u32,
            data: payload.as_mut_ptr().cast(),
            chunk: &raw mut *chunk,
        });
        let buffer = Box::new(spa_buffer {
            n_metas: 0,
            n_datas: 1,
            metas: std::ptr::null_mut(),
            datas: &raw mut *data,
        });
        let io = Box::new(spa_io_buffers {
            status: SPA_STATUS_HAVE_DATA as i32,
            buffer_id: 0,
        });
        Self {
            buffer,
            data,
            chunk,
            io,
            payload,
        }
    }

    fn buffer_ptr(&mut self) -> *mut spa_buffer {
        &raw mut *self.buffer
    }

    fn io_ptr(&mut self) -> *mut c_void {
        (&raw mut *self.io).cast()
    }

    fn requeue(&mut self) {
        self.io.status = SPA_STATUS_HAVE_DATA as i32;
        self.io.buffer_id = 0;
        self.chunk.offset = 0;
        self.chunk.size = 4_096;
        self.data.data = self.payload.as_mut_ptr().cast();
    }
}

fn command(id: u32) -> spa_command {
    spa_command {
        pod: spa_pod {
            size: size_of::<spa_command_body>() as u32,
            type_: SPA_TYPE_Object,
        },
        body: spa_command_body {
            body: spa_pod_object_body {
                type_: SPA_TYPE_COMMAND_Node,
                id,
            },
        },
    }
}

fn monotonic_ns() -> u64 {
    let mut now = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    assert_eq!(
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &raw mut now) },
        0
    );
    now.tv_sec as u64 * 1_000_000_000 + now.tv_nsec as u64
}

pub(super) fn run_sink_smoke(path: &str) {
    run_sink_smoke_at(path, 48_000);
}

pub(super) fn run_sink_smoke_at(path: &str, rate: u32) {
    let mut index = 0;
    let mut sink_factory = std::ptr::null();
    let mut names = Vec::new();
    loop {
        let mut factory = std::ptr::null();
        let result = unsafe { crate::spa_handle_factory_enum(&raw mut factory, &raw mut index) };
        if result == 0 {
            break;
        }
        assert_eq!(result, 1);
        let name = unsafe { CStr::from_ptr((*factory).name) };
        names.push(name.to_string_lossy().into_owned());
        if name == c"freebsd-oss.sink" {
            sink_factory = factory;
        }
    }
    assert_eq!(
        names,
        [
            "freebsd-oss.monitor",
            "freebsd-oss.device",
            "freebsd-oss.sink",
            "freebsd-oss.source",
        ]
    );
    assert!(!sink_factory.is_null());

    let path = CString::new(path).expect("the device path cannot contain NUL");
    let items = [
        spa_dict_item {
            key: c"api.freebsd-oss.dsp-path".as_ptr(),
            value: path.as_ptr(),
        },
        spa_dict_item {
            key: c"api.freebsd-oss.force-timer".as_ptr(),
            value: c"true".as_ptr(),
        },
    ];
    let info = spa_dict {
        flags: 0,
        n_items: items.len() as u32,
        items: items.as_ptr(),
    };
    let mut host = TestHost::new();
    let support = host.support();
    let mut handle = unsafe { FactoryHandle::new(sink_factory, &info, &support) };
    let node = unsafe { handle.node() };
    let methods = unsafe { (*node).iface.cb.funcs.cast::<spa_node_methods>() };
    let object = unsafe { (*node).iface.cb.data };

    let format = AlignedPod::format(rate as i32);
    let set_format = unsafe { (*methods).port_set_param }.expect("node must set port params");
    assert_eq!(
        unsafe {
            set_format(
                object,
                SPA_DIRECTION_INPUT,
                0,
                SPA_PARAM_Format,
                0,
                format.as_pod(),
            )
        },
        0
    );

    let mut fixture = BufferFixture::new();
    let mut buffers = [fixture.buffer_ptr()];
    let use_buffers = unsafe { (*methods).port_use_buffers }.expect("node must accept buffers");
    assert_eq!(
        unsafe {
            use_buffers(
                object,
                SPA_DIRECTION_INPUT,
                0,
                0,
                buffers.as_mut_ptr(),
                buffers.len() as u32,
            )
        },
        0
    );
    let set_port_io = unsafe { (*methods).port_set_io }.expect("node must accept port IO");
    assert_eq!(
        unsafe {
            set_port_io(
                object,
                SPA_DIRECTION_INPUT,
                0,
                SPA_IO_Buffers,
                fixture.io_ptr(),
                size_of::<spa_io_buffers>(),
            )
        },
        0
    );

    let mut clock = Box::new(unsafe { std::mem::zeroed::<spa_io_clock>() });
    clock.id = 7;
    clock.rate = spa_fraction {
        num: 1,
        denom: rate,
    };
    clock.target_rate = clock.rate;
    clock.target_duration = 1_024;
    let mut position = Box::new(unsafe { std::mem::zeroed::<spa_io_position>() });
    position.clock = *clock;
    position.clock.nsec = monotonic_ns();
    position.state = SPA_IO_POSITION_STATE_RUNNING;
    let set_io = unsafe { (*methods).set_io }.expect("node must accept node IO");
    assert_eq!(
        unsafe {
            set_io(
                object,
                SPA_IO_Clock,
                (&raw mut *clock).cast(),
                size_of::<spa_io_clock>(),
            )
        },
        0
    );
    assert_eq!(
        unsafe {
            set_io(
                object,
                SPA_IO_Position,
                (&raw mut *position).cast(),
                size_of::<spa_io_position>(),
            )
        },
        0
    );

    let send_command = unsafe { (*methods).send_command }.expect("node must accept commands");
    let process = unsafe { (*methods).process }.expect("node must process buffers");
    let start = command(SPA_NODE_COMMAND_Start);
    let pause = command(SPA_NODE_COMMAND_Pause);
    let suspend = command(SPA_NODE_COMMAND_Suspend);
    assert_eq!(unsafe { send_command(object, &raw const start) }, 0);
    let status = unsafe { process(object) };
    assert_ne!(status & SPA_STATUS_NEED_DATA as i32, 0);
    assert_eq!(fixture.io.status, SPA_STATUS_NEED_DATA as i32);
    assert!(
        clock.name[0] != 0,
        "the sink did not publish its clock name"
    );

    assert_eq!(unsafe { send_command(object, &raw const pause) }, 0);
    fixture.requeue();
    position.clock.nsec = monotonic_ns();
    assert_eq!(unsafe { send_command(object, &raw const start) }, 0);
    let status = unsafe { process(object) };
    assert_ne!(status & SPA_STATUS_NEED_DATA as i32, 0);
    assert_eq!(fixture.io.status, SPA_STATUS_NEED_DATA as i32);

    assert_eq!(unsafe { send_command(object, &raw const suspend) }, 0);

    // Suspend is not Pause: it releases the native descriptor and removes
    // both Format and Buffers. The open also becomes an exclusive-release
    // assertion when this smoke test runs in direct bitperfect mode.
    let mut contender = DspWriter::new(
        path.as_c_str()
            .to_str()
            .expect("the test device path must be UTF-8"),
    );
    contender
        .open()
        .expect("SPA Suspend must release the playback descriptor");
    contender.close();
    assert_eq!(
        unsafe { send_command(object, &raw const start) },
        -libc::EIO,
        "Suspend must remove the negotiated format and buffers"
    );

    assert_eq!(
        unsafe {
            set_format(
                object,
                SPA_DIRECTION_INPUT,
                0,
                SPA_PARAM_Format,
                0,
                format.as_pod(),
            )
        },
        0
    );
    assert_eq!(
        unsafe { send_command(object, &raw const start) },
        -libc::EIO,
        "renegotiating Format alone must not resurrect released buffers"
    );
    assert_eq!(
        unsafe {
            use_buffers(
                object,
                SPA_DIRECTION_INPUT,
                0,
                0,
                buffers.as_mut_ptr(),
                buffers.len() as u32,
            )
        },
        0
    );
    fixture.requeue();
    position.clock.nsec = monotonic_ns();
    assert_eq!(unsafe { send_command(object, &raw const start) }, 0);
    let status = unsafe { process(object) };
    assert_ne!(status & SPA_STATUS_NEED_DATA as i32, 0);
    assert_eq!(unsafe { send_command(object, &raw const suspend) }, 0);
    drop(handle);
}
