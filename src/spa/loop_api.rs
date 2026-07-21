use super::*;

#[derive(Clone, Copy)]
pub(crate) struct Loop {
    // Keep the host-owned spa_loop as a raw pointer because the host may
    // mutate it. wrap() validates and copies the method slots once; data is
    // read through the raw interface for each call.
    loop_: std::ptr::NonNull<spa_loop>,
    add_source_fn: unsafe extern "C" fn(*mut c_void, *mut spa_source) -> c_int,
    remove_source_fn: unsafe extern "C" fn(*mut c_void, *mut spa_source) -> c_int,
    invoke_fn: unsafe extern "C" fn(
        *mut c_void,
        spa_invoke_func_t,
        u32,
        *const c_void,
        usize,
        bool,
        *mut c_void,
    ) -> c_int,
}

impl Loop {
    /// # Safety
    /// `loop_` must point at a live, initialized spa_loop from the host's
    /// support array, and the host keeps it (and its methods vtable) valid
    /// for the plugin's lifetime (the spa_support contract).
    pub(crate) unsafe fn wrap(loop_: *mut spa_loop) -> Self {
        let loop_ = std::ptr::NonNull::new(loop_).expect("loop should be initialized");
        // Validate the version and every required slot during initialization.
        let funcs = unsafe { (*loop_.as_ptr()).iface.cb.funcs };
        let methods = std::ptr::NonNull::new(funcs.cast::<spa_loop_methods>().cast_mut())
            .expect("loop methods should be initialized")
            .as_ptr();
        assert!(version_ok(
            unsafe { (*methods).version },
            SPA_VERSION_LOOP_METHODS
        ));
        Self {
            loop_,
            add_source_fn: unsafe { (*methods).add_source }
                .expect("add_source should be initialized"),
            remove_source_fn: unsafe { (*methods).remove_source }
                .expect("remove_source should be initialized"),
            invoke_fn: unsafe { (*methods).invoke }.expect("invoke should be initialized"),
        }
    }

    fn data(&self) -> *mut c_void {
        // per-call raw read (see the struct comment); valid per wrap()'s contract
        unsafe { (*self.loop_.as_ptr()).iface.cb.data }
    }

    /// # Safety
    /// `source` must stay valid (and pinned) until it is removed from the
    /// loop again; the loop stores the pointer.
    pub(crate) unsafe fn add_source(&self, source: *mut spa_source) -> c_int {
        unsafe { (self.add_source_fn)(self.data(), source) }
    }

    /// # Safety
    /// Must be called from the loop thread (or through an invoke); `source`
    /// must be the pointer previously registered with add_source.
    pub(crate) unsafe fn remove_source(&self, source: *mut spa_source) -> c_int {
        unsafe { (self.remove_source_fn)(self.data(), source) }
    }

    /// # Safety
    /// The marshaling contract: `func` must treat `data`/`user_data`
    /// according to `block` (a non-blocking invoke copies `data` and runs
    /// after this call returns, so pointees must outlive the run); see
    /// block_on_loop / queue_task below.
    pub(crate) unsafe fn invoke(
        &self,
        func: spa_invoke_func_t,
        seq: u32,
        data: *const c_void,
        size: usize,
        block: bool,
        user_data: *mut c_void,
    ) -> c_int {
        unsafe { (self.invoke_fn)(self.data(), func, seq, data, size, block, user_data) }
    }
}

// A spa_source pinned at a stable address for the entire interval in which a
// host loop retains its pointer. Registration is explicit because removal has
// to run on the owning loop; Drop aborts rather than free a still-linked source.
pub(crate) struct LoopSource {
    source: std::pin::Pin<Box<spa_source>>,
    loop_: Option<Loop>,
}

impl LoopSource {
    pub(crate) fn new(source: spa_source) -> Self {
        Self {
            source: Box::pin(source),
            loop_: None,
        }
    }

    pub(crate) fn is_registered(&self) -> bool {
        self.loop_.is_some()
    }

    pub(crate) fn set_fd(&mut self, fd: c_int) {
        assert!(
            !self.is_registered(),
            "a registered loop source cannot change its fd"
        );
        // Pin protects the allocation address, not field mutation.
        unsafe { self.source.as_mut().get_unchecked_mut() }.fd = fd;
    }

    fn as_mut_ptr(&mut self) -> *mut spa_source {
        // The Box allocation stays pinned when LoopSource itself moves.
        unsafe { self.source.as_mut().get_unchecked_mut() }
    }

    /// Register this pinned source on `loop_`.
    ///
    /// # Safety
    /// The caller must use a host context in which `add_source` is valid. The
    /// source callback and data pointer must satisfy SPA's lifetime contract
    /// until `unregister` succeeds.
    pub(crate) unsafe fn register(&mut self, loop_: &Loop) -> c_int {
        assert!(!self.is_registered(), "loop source is already registered");
        let err = unsafe { loop_.add_source(self.as_mut_ptr()) };
        if err >= 0 {
            self.loop_ = Some(*loop_);
        }
        err
    }

    /// Remove this source from its owning loop.
    ///
    /// # Safety
    /// Must run on the registered loop thread. On failure the source remains
    /// registered and must stay alive.
    pub(crate) unsafe fn unregister(&mut self) -> c_int {
        let Some(loop_) = self.loop_ else {
            return 0;
        };
        let err = unsafe { loop_.remove_source(self.as_mut_ptr()) };
        if err >= 0 {
            self.loop_ = None;
        }
        err
    }
}

impl Drop for LoopSource {
    fn drop(&mut self) {
        if self.is_registered() {
            eprintln!("freebsd-oss: dropping a registered SPA loop source; aborting");
            std::process::abort();
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct System {
    // Keep the host-owned spa_system as a raw pointer and copy its validated
    // method slots. Safe wrappers pass only scalars and call-scoped references.
    system: std::ptr::NonNull<spa_system>,
    close_fn: unsafe extern "C" fn(*mut c_void, c_int) -> c_int,
    clock_gettime_fn: unsafe extern "C" fn(*mut c_void, c_int, *mut timespec) -> c_int,
    timerfd_create_fn: unsafe extern "C" fn(*mut c_void, c_int, c_int) -> c_int,
    timerfd_read_fn: unsafe extern "C" fn(*mut c_void, c_int, *mut u64) -> c_int,
    timerfd_settime_fn: unsafe extern "C" fn(
        *mut c_void,
        c_int,
        c_int,
        *const itimerspec,
        *mut itimerspec,
    ) -> c_int,
}

impl System {
    /// # Safety
    /// `system` must point at a live, initialized spa_system from the host's
    /// support array, and the host keeps it (and its methods vtable) valid
    /// for the plugin's lifetime (the spa_support contract). That contract
    /// is what makes the safe methods below sound.
    pub(crate) unsafe fn wrap(system: *mut spa_system) -> Self {
        let system = std::ptr::NonNull::new(system).expect("system should be initialized");
        // the whole vtable is validated once here (non-null, version, every
        // slot the methods below call - see Loop::wrap)
        let funcs = unsafe { (*system.as_ptr()).iface.cb.funcs };
        let methods = std::ptr::NonNull::new(funcs.cast::<spa_system_methods>().cast_mut())
            .expect("system methods should be initialized")
            .as_ptr();
        assert!(version_ok(
            unsafe { (*methods).version },
            SPA_VERSION_SYSTEM_METHODS
        ));
        Self {
            system,
            close_fn: unsafe { (*methods).close }.expect("close should be initialized"),
            clock_gettime_fn: unsafe { (*methods).clock_gettime }
                .expect("clock_gettime should be initialized"),
            timerfd_create_fn: unsafe { (*methods).timerfd_create }
                .expect("timerfd_create should be initialized"),
            timerfd_read_fn: unsafe { (*methods).timerfd_read }
                .expect("timerfd_read should be initialized"),
            timerfd_settime_fn: unsafe { (*methods).timerfd_settime }
                .expect("timerfd_settime should be initialized"),
        }
    }

    fn data(&self) -> *mut c_void {
        // per-call raw read (see Loop::data); valid per wrap()'s contract
        unsafe { (*self.system.as_ptr()).iface.cb.data }
    }

    pub(crate) fn clock_gettime(&self, clock_id: c_int, value: &mut timespec) -> c_int {
        // sound per wrap()'s contract; `value` is a live &mut for the call
        unsafe { (self.clock_gettime_fn)(self.data(), clock_id, value) }
    }

    pub(crate) fn timerfd_create(&self, clock_id: c_int, flags: c_int) -> Result<TimerFd, c_int> {
        let fd = unsafe { (self.timerfd_create_fn)(self.data(), clock_id, flags) };
        if fd < 0 {
            Err(fd)
        } else {
            Ok(TimerFd { system: *self, fd })
        }
    }
}

pub(crate) struct TimerFd {
    system: System,
    fd: c_int,
}

impl TimerFd {
    /// Borrow the descriptor for host registration. `TimerFd` remains its
    /// sole owner, and callers must invalidate any stored mirror after drop.
    pub(crate) fn raw(&self) -> c_int {
        self.fd
    }

    pub(crate) fn read(&self, expirations: &mut u64) -> c_int {
        // sound per wrap()'s contract; `expirations` is a live &mut for the call
        unsafe { (self.system.timerfd_read_fn)(self.system.data(), self.fd, expirations) }
    }

    // Callers do not request the previous timer value.
    pub(crate) fn settime(&self, flags: c_int, new_value: &itimerspec) -> c_int {
        // sound per wrap()'s contract; `new_value` is a live shared reference
        unsafe {
            (self.system.timerfd_settime_fn)(
                self.system.data(),
                self.fd,
                flags,
                new_value,
                std::ptr::null_mut(),
            )
        }
    }
}

impl Drop for TimerFd {
    fn drop(&mut self) {
        unsafe {
            (self.system.close_fn)(self.system.data(), self.fd);
        }
    }
}
// Marks a captured value as allowed to cross onto the loop thread inside an
// invoke closure. For host pointers (io areas, the callback table, buffer
// arrays) whose validity the SPA contract ties to the node's lifetime rather
// than to a Rust Send impl; the loop invoke is the serialization point.
// The field is private on purpose: closures capture fields precisely, so a
// public .0 (or a destructuring pattern of it) would be captured
// field-by-field, skipping the wrapper's Send; into_inner takes self whole,
// forcing whole-value capture.
pub(crate) struct SendWrap<T>(T);
unsafe impl<T> Send for SendWrap<T> {}
impl<T> SendWrap<T> {
    /// Allow `v` to cross onto the loop thread.
    ///
    /// # Safety
    /// The caller asserts that this particular value stays valid and usable
    /// from the loop thread for as long as it is used there. For host
    /// pointers that is the SPA lifetime contract (the host keeps callback
    /// tables, io areas and buffer arrays valid while they are set); the
    /// blocking loop invoke is the serialization point.
    pub(crate) unsafe fn new(v: T) -> Self {
        SendWrap(v)
    }
    pub(crate) fn into_inner(self) -> T {
        self.0
    }
}

// Run `f` on the data loop and wait for it; serializes main-thread
// reconfiguration against process()/on_wake() (runs inline when already on
// the loop thread). The closure and target cross a thread boundary; callers
// only capture raw pointers and plain data (F: Send; the blocking call keeps
// stack borrows sound, so no 'static is needed). Returns false when the
// invoke failed or the closure panicked - the closure then may not have run.
pub(crate) unsafe fn block_on_loop<T, F: FnOnce(&mut T) + Send>(
    loop_: &crate::spa::Loop,
    target: *mut T,
    f: F,
) -> bool {
    struct Ctx<T, F> {
        target: *mut T,
        f: Option<F>,
    }

    unsafe extern "C" fn trampoline<T, F: FnOnce(&mut T) + Send>(
        _loop: *mut libspa::sys::spa_loop,
        _async: bool,
        _seq: u32,
        _data: *const c_void,
        _size: usize,
        user_data: *mut c_void,
    ) -> c_int {
        // user_data is the &mut Ctx the blocking invoke below keeps alive
        let ctx = unsafe { user_data.cast::<Ctx<T, F>>().as_mut() }
            .expect("user_data is not supposed to be null");
        let f = ctx.f.take().expect("the invoked function only runs once");
        let target = ctx.target;
        // a panic must not unwind into the C loop (that aborts the daemon)
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            // target validity is the caller's contract on block_on_loop
            f(unsafe { target.as_mut() }.expect("target is not supposed to be null"));
        }));
        if ok.is_err() { -libc::ECANCELED } else { 0 }
    }

    // blocking, so `ctx` outlives the call
    let mut ctx = Ctx { target, f: Some(f) };
    let err = unsafe {
        loop_.invoke(
            Some(trampoline::<T, F>),
            0,
            std::ptr::null(),
            0,
            true,
            (&raw mut ctx).cast(),
        )
    };
    err >= 0
}

// Queue an owned closure on the target loop. spa_loop.invoke may execute it
// inline when called from that loop, so callers must release reentrant state
// borrows first and real-time callers must not enqueue blocking work. A false
// return means the closure and its payload were dropped on the calling thread.
//
// # Safety
// The loop must outlive the queued item's execution: host loops come from
// the spa_support array and live for the plugin host's lifetime.
pub(crate) unsafe fn queue_task<F: FnOnce() + Send + 'static>(
    loop_: &crate::spa::Loop,
    f: F,
) -> bool {
    unsafe extern "C" fn trampoline<F: FnOnce() + Send + 'static>(
        _loop: *mut libspa::sys::spa_loop,
        _async: bool,
        _seq: u32,
        _data: *const c_void,
        _size: usize,
        user_data: *mut c_void,
    ) -> c_int {
        // user_data is the Box::into_raw'd closure below; the loop runs each
        // queued item exactly once, so this is the sole owner
        let f = unsafe { Box::from_raw(user_data.cast::<F>()) };
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
        if ok.is_err() {
            eprintln!("freebsd-oss: panic in a queued loop task (swallowed)");
        }
        // never negative: an inline flush returns this value to the caller,
        // and a negative would make it free the closure a second time
        0
    }

    let ctx = Box::into_raw(Box::new(f));
    let err = unsafe {
        loop_.invoke(
            Some(trampoline::<F>),
            0,
            std::ptr::null(),
            0,
            false,
            ctx.cast(),
        )
    };
    if err < 0 {
        // a negative here uniquely means the item was never queued (the
        // trampoline never ran, so this is still the sole owner)
        drop(unsafe { Box::from_raw(ctx) });
        return false;
    }
    true
}
