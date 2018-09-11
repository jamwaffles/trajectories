#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

type Eigen_Vector3d = [f64; 3usize];

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

extern "C" {
    pub fn path_create(items: *const [f64; 3], len: usize, step: f64) -> *mut Path;
}
