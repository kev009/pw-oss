//! Deterministic byte transport for stream and lifecycle tests.
//!
//! A nonblocking pipe reproduces the partial-I/O and backpressure properties
//! the node cares about without requiring a sound device or OSS ioctls.

use std::ffi::c_int;

pub(crate) fn set_nonblock(fd: c_int) {
    unsafe {
        let flags = libc::fcntl(fd, libc::F_GETFL);
        assert_ne!(libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK), -1);
    }
}

pub(crate) fn pipe_pair(nonblock_read: bool, nonblock_write: bool) -> (c_int, c_int) {
    let mut fds = [0 as c_int; 2];
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
    if nonblock_read {
        set_nonblock(fds[0]);
    }
    if nonblock_write {
        set_nonblock(fds[1]);
    }
    (fds[0], fds[1])
}

pub(crate) fn drain(fd: c_int) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buffer = [0u8; 16_384];
    loop {
        let count = unsafe { libc::read(fd, buffer.as_mut_ptr().cast(), buffer.len()) };
        if count <= 0 {
            break;
        }
        out.extend_from_slice(&buffer[..count as usize]);
    }
    out
}

pub(crate) fn pattern(len: usize, seed: usize) -> Vec<u8> {
    (0..len)
        .map(|index| ((index * 7 + seed) % 251) as u8)
        .collect()
}

pub(crate) fn fill_pipe(write_fd: c_int) -> usize {
    let fill = vec![0xffu8; 16_384];
    let mut total = 0usize;
    loop {
        let count = unsafe { libc::write(write_fd, fill.as_ptr().cast(), fill.len()) };
        if count <= 0 {
            break;
        }
        total += count as usize;
    }
    total
}

pub(crate) fn free_space(read_fd: c_int, len: usize) {
    let mut buffer = vec![0u8; len];
    let mut freed = 0usize;
    while freed < len {
        let count =
            unsafe { libc::read(read_fd, buffer.as_mut_ptr().add(freed).cast(), len - freed) };
        assert!(count > 0);
        freed += count as usize;
    }
}
