use super::*;

#[derive(Clone)]
pub struct Log {
    // Raw pointers end to end, never long-lived references: the host logger
    // mutates the spa_log pointee (level) and the registered topic
    // (level/has_custom_level) at runtime, so holding a &'static over
    // host-mutated memory would be unsound. The methods vtable gets the
    // same treatment - nothing guarantees the host never touches it, and a
    // per-call raw read costs nothing.
    logger: std::ptr::NonNull<spa_log>,
    methods: std::ptr::NonNull<spa_log_methods>,
    // the module's registered topic (see the lib.rs section entries)
    topic: Option<std::ptr::NonNull<spa_log_topic>>,
}

// The host logger is thread-safe and outlives every node, so cloned handles
// may travel with cross-loop messages.
unsafe impl Send for Log {}

impl Log {
    #[allow(dead_code)]
    pub(crate) unsafe fn wrap(
        log: *mut spa_log,
        topic: Option<std::ptr::NonNull<spa_log_topic>>,
    ) -> Option<Self> {
        let logger = std::ptr::NonNull::new(log)?;
        // the vtable pointer is read once here; the vtable fields are read
        // per call through the raw pointer
        let funcs = unsafe { (*log).iface.cb.funcs };
        let methods = std::ptr::NonNull::new(funcs.cast::<spa_log_methods>().cast_mut())?;
        // no minimum-version assert: version 0 (predating the logt slot) is
        // accepted - log() gates every logt read on the vtable being v1+,
        // and the v0 `log` method covers the rest
        Some(Self {
            logger,
            methods,
            topic,
        })
    }

    pub fn log_level(&self) -> spa_log_level {
        use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
        // The host logger rewrites the registered topic's level (and the
        // logger's own level) from its own threads on runtime log-level
        // changes (inherent to the C API). Per-call atomic views over the
        // C-layout fields remove our side's contribution to the race, but
        // the writer side stays plain C stores, so the mixed access
        // formally remains a data race - UB, with no bounded worst case to
        // appeal to. The host writes directly into the structs we handed
        // it, so no plugin-side synchronization can intercept the stores,
        // and every C plugin shares the pattern; the atomics merely narrow
        // the practical exposure (aligned single-word loads, level-check
        // only). The only sound alternative (an FFI shim serializing every
        // read with the logger) would put a call on every log-level check.
        if let Some(topic) = self.topic {
            let topic = topic.as_ptr();
            unsafe {
                let custom = AtomicBool::from_ptr(&raw mut (*topic).has_custom_level);
                if custom.load(Ordering::Relaxed) {
                    return AtomicU32::from_ptr(&raw mut (*topic).level).load(Ordering::Relaxed);
                }
            }
        }
        let logger = self.logger.as_ptr();
        unsafe { AtomicU32::from_ptr(&raw mut (*logger).level).load(Ordering::Relaxed) }
    }

    pub fn log(&self, level: spa_log_level, file: &str, line: c_int, func: &str, msg: &str) {
        let file = CString::new(file).unwrap(); // ours, no interior NULs
        let func = CString::new(func).unwrap(); // ditto

        // the message can carry host-derived strings; don't abort on an
        // interior NUL
        let msg = CString::new(msg).unwrap_or_else(|_| c"<message contained NUL>".to_owned());
        let topic = self
            .topic
            .map_or(std::ptr::null(), |topic| topic.as_ptr().cast_const());
        let methods = self.methods.as_ptr();
        let data = unsafe { (*self.logger.as_ptr()).iface.cb.data };
        // a v0 vtable has no logt slot at all (the C struct is shorter), so
        // the field may only be read behind the version gate; a missing logt
        // on a v1+ logger falls back the same way
        let logt = if version_ok(unsafe { (*methods).version }, SPA_VERSION_LOG_METHODS) {
            unsafe { (*methods).logt }
        } else {
            None
        };
        unsafe {
            if let Some(logt) = logt {
                logt(
                    data,
                    level,
                    topic,
                    file.as_ptr(),
                    line,
                    func.as_ptr(),
                    c"%s".as_ptr(),
                    msg.as_ptr(),
                );
            } else {
                let log = (*methods).log.expect("log should be initialized");
                log(
                    data,
                    level,
                    file.as_ptr(),
                    line,
                    func.as_ptr(),
                    c"%s".as_ptr(),
                    msg.as_ptr(),
                );
            }
        }
    }
}

// a Log that never emits (level NONE, methods never reached): phase-function
// tests need a Log without a live host logger. Safe because the log! macros
// gate every method call on log_level(), which reads NONE here.
#[cfg(any(test, feature = "test-support"))]
impl Log {
    pub fn test_null() -> Self {
        Self {
            logger: std::ptr::NonNull::from(Box::leak(Box::new(unsafe {
                std::mem::zeroed::<spa_log>()
            }))),
            methods: std::ptr::NonNull::from(Box::leak(Box::new(unsafe {
                std::mem::zeroed::<spa_log_methods>()
            }))),
            topic: None,
        }
    }
}

#[macro_export]
macro_rules! log {
    ($log:expr, $log_level:expr, $($arg:tt)*) => {
        if $log.log_level() >= $log_level {
            let file = file!();
            let line = line!();
            let func = ""; // no cheap function-name source; file:line suffices
            $log.log(
                $log_level,
                file,
                line as ::std::ffi::c_int,
                func,
                &format!($($arg)*),
            );
        }
    };
}

#[macro_export]
macro_rules! error {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, ::libspa::sys::SPA_LOG_LEVEL_ERROR, $($arg)*)
    };
}

#[macro_export]
macro_rules! warn {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, ::libspa::sys::SPA_LOG_LEVEL_WARN, $($arg)*)
    };
}

#[macro_export]
macro_rules! info {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, ::libspa::sys::SPA_LOG_LEVEL_INFO, $($arg)*)
    };
}

#[macro_export]
macro_rules! debug {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, ::libspa::sys::SPA_LOG_LEVEL_DEBUG, $($arg)*)
    };
}

#[macro_export]
macro_rules! trace {
    ($log:expr, $($arg:tt)*) => {
        $crate::log!($log, ::libspa::sys::SPA_LOG_LEVEL_TRACE, $($arg)*)
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_or_incomplete_log_support_is_rejected() {
        assert!(unsafe { Log::wrap(std::ptr::null_mut(), None) }.is_none());

        let mut logger = unsafe { std::mem::zeroed::<spa_log>() };
        assert!(unsafe { Log::wrap(&raw mut logger, None) }.is_none());
    }
}
