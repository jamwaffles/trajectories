#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

extern crate libc;

use libc::size_t;

type Eigen_Vector3d = [f64; 3usize];

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

extern "C" {
    pub fn path_create(items: *const f64, len: size_t, step: f64) -> *mut Path;
}
