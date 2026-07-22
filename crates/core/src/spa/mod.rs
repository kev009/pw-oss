use libspa::sys::*;
use std::ffi::{CString, c_int};

mod log;

pub use log::Log;

fn version_ok(version: u32, min: u32) -> bool {
    version >= min
}
