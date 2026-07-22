use std::os::fd::{AsRawFd, RawFd};

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

    #[cfg(test)]
    pub(crate) fn test_pair() -> (Self, UnixSeqpacketConn) {
        let (socket, peer) = UnixSeqpacketConn::pair().expect("create devd test socket pair");
        (
            Self {
                socket,
                buffer: vec![0; 8192],
            },
            peer,
        )
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
