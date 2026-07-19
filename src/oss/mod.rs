mod abi;
mod devices;
mod dsp;
mod mixer;

pub(crate) use abi::{
    AFMT_F32_BE, AFMT_F32_LE, AFMT_S16_BE, AFMT_S16_LE, AFMT_S24_BE, AFMT_S24_LE, AFMT_S32_BE,
    AFMT_S32_LE, AFMT_U8,
};
pub(crate) use devices::{
    DspCaps, MIN_RING_BYTES, PcmDevice, drain_quantum_ns, group_pcm_devices_by_parent,
    list_pcm_devices, probe_caps, read_sndstat, ring_byte_cap,
};
pub(crate) use dsp::{Dsp, DspWriter};
pub(crate) use mixer::{
    Mixer, SOUND_DEVICE_NAMES, SOUND_MIXER_LINE, SOUND_MIXER_MIC, SOUND_MIXER_NRDEVICES,
};

// Pipe plumbing shared by the oss::dsp alignment tests and the node sink/source
// phase tests (they drive the extracted process phases on pipe fds).
#[cfg(test)]
pub(crate) mod test_util {
    use libc::c_int;

    pub(crate) fn set_nonblock(fd: c_int) {
        unsafe {
            let fl = libc::fcntl(fd, libc::F_GETFL);
            assert_ne!(libc::fcntl(fd, libc::F_SETFL, fl | libc::O_NONBLOCK), -1);
        }
    }

    // (read end, write end); nonblock as requested per end
    pub(crate) fn pipe_pair(nb_read: bool, nb_write: bool) -> (c_int, c_int) {
        let mut fds = [0 as c_int; 2];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        if nb_read {
            set_nonblock(fds[0]);
        }
        if nb_write {
            set_nonblock(fds[1]);
        }
        (fds[0], fds[1])
    }

    pub(crate) fn drain(fd: c_int) -> Vec<u8> {
        let mut out = Vec::new();
        let mut buf = [0u8; 16384];
        loop {
            let n = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
            if n <= 0 {
                break;
            }
            out.extend_from_slice(&buf[..n as usize]);
        }
        out
    }

    pub(crate) fn pattern(len: usize, seed: usize) -> Vec<u8> {
        (0..len).map(|i| ((i * 7 + seed) % 251) as u8).collect()
    }

    // fill a nonblocking pipe to capacity (the "OSS ring" is full); returns
    // the capacity actually taken
    pub(crate) fn fill_pipe(w: c_int) -> usize {
        let fill = vec![0xffu8; 16384];
        let mut total = 0usize;
        loop {
            let n = unsafe { libc::write(w, fill.as_ptr().cast(), fill.len()) };
            if n <= 0 {
                break;
            }
            total += n as usize;
        }
        total
    }

    // free exactly `n` bytes of a full pipe by consuming them from the read end
    pub(crate) fn free_space(r: c_int, n: usize) {
        let mut buf = vec![0u8; n];
        let mut freed = 0usize;
        while freed < n {
            let m = unsafe { libc::read(r, buf.as_mut_ptr().add(freed).cast(), n - freed) };
            assert!(m > 0);
            freed += m as usize;
        }
    }
}
