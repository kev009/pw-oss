use libc::sysctlbyname;
use nix::errno::Errno;
use std::ffi::{CStr, CString, c_int, c_ulong, c_void};

mod nv;

pub(crate) use nv::{NvList, NvRef};

/// An owned libc descriptor closed with `libc::close`.
pub(crate) struct LibcFd(c_int);

impl LibcFd {
    pub(crate) fn open(path: &CStr, flags: c_int) -> Option<Self> {
        let fd = unsafe { libc::open(path.as_ptr(), flags | libc::O_CLOEXEC) };
        (fd != -1).then(|| Self(fd))
    }

    /// Take ownership of an existing descriptor.
    ///
    /// # Safety
    /// `fd` must be open and exclusively transferred to the returned owner.
    pub(crate) unsafe fn from_raw(fd: c_int) -> Self {
        assert!(fd >= 0);
        Self(fd)
    }

    pub(crate) fn raw(&self) -> c_int {
        self.0
    }
}

impl Drop for LibcFd {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.0);
        }
    }
}

/// A C-compatible plain-data type that an ioctl may initialize byte-for-byte.
///
/// # Safety
/// Implementors must be `Copy`, contain no references or drop state, accept
/// the all-zero value and every bit pattern the kernel can return, and have
/// the exact C layout encoded by the corresponding ioctl request.
pub(crate) unsafe trait IoctlPod: Copy {}

unsafe impl IoctlPod for c_int {}

pub(crate) fn ioctl_zeroed<T: IoctlPod>() -> T {
    // IoctlPod requires the all-zero value to be valid.
    unsafe { std::mem::zeroed() }
}

pub(crate) fn ioctl_int(fd: c_int, req: c_ulong, value: c_int) -> Option<c_int> {
    unsafe { ioctl_value(fd, req, value) }
}

/// Pass an initialized POD value through an ioctl that may update it.
///
/// # Safety
/// `req` must address exactly `T` and may not retain the pointer.
pub(crate) unsafe fn ioctl_value<T: IoctlPod>(fd: c_int, req: c_ulong, mut value: T) -> Option<T> {
    (unsafe { libc::ioctl(fd, req, &mut value) } != -1).then_some(value)
}

/// Read a POD value fully initialized by an ioctl.
///
/// # Safety
/// `req` must address exactly `T`, fully initialize it on success, and not
/// retain the pointer.
pub(crate) unsafe fn ioctl_read<T: IoctlPod>(fd: c_int, req: c_ulong) -> Option<T> {
    let mut value = std::mem::MaybeUninit::<T>::uninit();
    if unsafe { libc::ioctl(fd, req, value.as_mut_ptr()) } == -1 {
        None
    } else {
        Some(unsafe { value.assume_init() })
    }
}

// the shared read-only sysctlbyname shape (no new value): `buf` may be null
// for a size probe, `len` is in/out. Callers pass a `buf` valid for `len`
// bytes (or null).
unsafe fn sysctl_read(name: &CStr, buf: *mut c_void, len: &mut usize) -> Result<(), Errno> {
    if unsafe { sysctlbyname(name.as_ptr(), buf, len, std::ptr::null(), 0) } == -1 {
        return Err(Errno::last());
    }
    Ok(())
}

// a NUL-terminated sysctl name
pub(crate) struct SysctlName(CString);

impl From<&str> for SysctlName {
    fn from(str: &str) -> Self {
        SysctlName(CString::new(str).unwrap())
    }
}

impl From<String> for SysctlName {
    fn from(str: String) -> Self {
        SysctlName(CString::new(str).unwrap())
    }
}

pub(crate) struct SysctlReader {
    scratch_buffer: Vec<u8>,
}

impl SysctlReader {
    pub(crate) fn new() -> Self {
        Self {
            scratch_buffer: Vec::with_capacity(32),
        }
    }

    pub(crate) fn read_string<T: Into<SysctlName>>(
        &mut self,
        name: T,
        max_len: usize,
    ) -> Result<String, Errno> {
        let SysctlName(name) = name.into();

        let mut len = 0;
        unsafe { sysctl_read(&name, std::ptr::null_mut(), &mut len) }?;

        if len > max_len {
            return Err(Errno::ENOMEM);
        }

        if len == 0 {
            return Ok("".to_string());
        }

        self.scratch_buffer.resize(len, 0);
        unsafe { sysctl_read(&name, self.scratch_buffer.as_mut_ptr().cast(), &mut len) }?;

        // classic string sysctls (e.g. kern.ostype) count the terminating NUL
        // in the returned length; device-tree ones don't - trim either way, or
        // the NUL poisons map keys and C-string conversions downstream
        let mut bytes = &self.scratch_buffer[0..len];
        while let [head @ .., 0] = bytes {
            bytes = head;
        }
        Ok(String::from_utf8_lossy(bytes).to_string())
    }

    pub(crate) fn read_u32<T: Into<SysctlName>>(&self, name: T) -> Result<u32, Errno> {
        let SysctlName(name) = name.into();
        let mut value: u32 = 0;
        let mut len = size_of::<u32>();
        unsafe { sysctl_read(&name, std::ptr::from_mut(&mut value).cast(), &mut len) }?;
        Ok(value)
    }
}

use std::os::fd::AsRawFd;
use std::os::fd::RawFd;
use uds::UnixSeqpacketConn;

pub(crate) struct DevdSocket {
    socket: UnixSeqpacketConn,
    buffer: Vec<u8>,
}

impl DevdSocket {
    pub(crate) fn open() -> Result<Self, std::io::Error> {
        let socket = UnixSeqpacketConn::connect("/var/run/devd.seqpacket.pipe")?;
        let buffer = [0; 8192 /* DEVCTL_MAXBUF */].to_vec();
        Ok(Self { socket, buffer })
    }

    pub(crate) fn fd(&self) -> RawFd {
        self.socket.as_raw_fd()
    }

    // false when the connection is dead (EOF or error): the fd stays readable
    // forever then, and the caller must deregister it or the loop busy-spins
    pub(crate) fn read_event(&mut self, mut apply: impl FnMut(&str)) -> bool {
        match self.socket.recv(&mut self.buffer) {
            Ok(0) => false, // EOF: devd went away (e.g. service devd restart)
            Ok(len) => {
                assert!(len <= self.buffer.len());
                // devd events should be ASCII, but don't abort on a stray byte
                apply(&String::from_utf8_lossy(&self.buffer[..len]));
                true
            }
            Err(err) => matches!(
                err.kind(),
                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted
            ),
        }
    }
}
