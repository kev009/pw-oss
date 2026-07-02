// Minimal nv(9) bindings for the sndstat(4) nvlist interface. Every getter
// is guarded by a typed exists check: libnv aborts the process on a missing
// or type-mismatched key, which must never happen from host data.

use std::ffi::CStr;
use std::os::raw::c_char;
use std::os::raw::c_int;
use std::os::raw::c_void;

// libnv exports its symbols as FreeBSD_nvlist_* (renamed to avoid clashing
// with ZFS's libnvpair); C callers get the mapping from sys/nv_namespace.h,
// which a raw extern block bypasses - hence the explicit link names.
#[link(name = "nv")]
extern "C" {
  #[link_name = "FreeBSD_nvlist_unpack"]
  fn nvlist_unpack(buf: *const c_void, size: usize, flags: c_int) -> *mut c_void;
  #[link_name = "FreeBSD_nvlist_destroy"]
  fn nvlist_destroy(nvl: *mut c_void);
  #[link_name = "FreeBSD_nvlist_exists_string"]
  fn nvlist_exists_string(nvl: *const c_void, name: *const c_char) -> bool;
  #[link_name = "FreeBSD_nvlist_exists_number"]
  fn nvlist_exists_number(nvl: *const c_void, name: *const c_char) -> bool;
  #[link_name = "FreeBSD_nvlist_exists_bool"]
  fn nvlist_exists_bool(nvl: *const c_void, name: *const c_char) -> bool;
  #[link_name = "FreeBSD_nvlist_exists_nvlist"]
  fn nvlist_exists_nvlist(nvl: *const c_void, name: *const c_char) -> bool;
  #[link_name = "FreeBSD_nvlist_exists_nvlist_array"]
  fn nvlist_exists_nvlist_array(nvl: *const c_void, name: *const c_char) -> bool;
  #[link_name = "FreeBSD_nvlist_get_string"]
  fn nvlist_get_string(nvl: *const c_void, name: *const c_char) -> *const c_char;
  #[link_name = "FreeBSD_nvlist_get_number"]
  fn nvlist_get_number(nvl: *const c_void, name: *const c_char) -> u64;
  #[link_name = "FreeBSD_nvlist_get_bool"]
  fn nvlist_get_bool(nvl: *const c_void, name: *const c_char) -> bool;
  #[link_name = "FreeBSD_nvlist_get_nvlist"]
  fn nvlist_get_nvlist(nvl: *const c_void, name: *const c_char) -> *const c_void;
  #[link_name = "FreeBSD_nvlist_get_nvlist_array"]
  fn nvlist_get_nvlist_array(nvl: *const c_void, name: *const c_char, nitems: *mut usize) -> *const *const c_void;
}

// the owned root of an unpacked nvlist
pub struct NvList {
  ptr: *mut c_void
}

impl NvList {

  pub fn unpack(buf: &[u8]) -> Option<Self> {
    let ptr = unsafe { nvlist_unpack(buf.as_ptr().cast(), buf.len(), 0) };
    if ptr.is_null() { None } else { Some(Self { ptr }) }
  }

  pub fn root(&self) -> NvRef<'_> {
    NvRef { ptr: self.ptr, _owner: std::marker::PhantomData }
  }
}

impl Drop for NvList {

  fn drop(&mut self) {
    unsafe { nvlist_destroy(self.ptr) };
  }
}

// a borrowed (child) nvlist; lives as long as the owning NvList
#[derive(Clone, Copy)]
pub struct NvRef<'a> {
  ptr:    *const c_void,
  _owner: std::marker::PhantomData<&'a NvList>
}

impl<'a> NvRef<'a> {

  pub fn string(&self, name: &CStr) -> Option<&'a str> {
    unsafe {
      if !nvlist_exists_string(self.ptr, name.as_ptr()) {
        return None;
      }
      CStr::from_ptr(nvlist_get_string(self.ptr, name.as_ptr())).to_str().ok()
    }
  }

  pub fn number(&self, name: &CStr) -> Option<u64> {
    unsafe {
      if !nvlist_exists_number(self.ptr, name.as_ptr()) {
        return None;
      }
      Some(nvlist_get_number(self.ptr, name.as_ptr()))
    }
  }

  pub fn boolean(&self, name: &CStr) -> Option<bool> {
    unsafe {
      if !nvlist_exists_bool(self.ptr, name.as_ptr()) {
        return None;
      }
      Some(nvlist_get_bool(self.ptr, name.as_ptr()))
    }
  }

  pub fn nvlist(&self, name: &CStr) -> Option<NvRef<'a>> {
    unsafe {
      if !nvlist_exists_nvlist(self.ptr, name.as_ptr()) {
        return None;
      }
      Some(NvRef { ptr: nvlist_get_nvlist(self.ptr, name.as_ptr()), _owner: std::marker::PhantomData })
    }
  }

  pub fn nvlist_array(&self, name: &CStr) -> Vec<NvRef<'a>> {
    unsafe {
      if !nvlist_exists_nvlist_array(self.ptr, name.as_ptr()) {
        return vec![];
      }
      let mut nitems = 0usize;
      let items = nvlist_get_nvlist_array(self.ptr, name.as_ptr(), &mut nitems);
      if items.is_null() {
        return vec![];
      }
      std::slice::from_raw_parts(items, nitems).iter()
        .map(|p| NvRef { ptr: *p, _owner: std::marker::PhantomData })
        .collect()
    }
  }
}
