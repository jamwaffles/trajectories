extern crate bindgen;
extern crate cc;

use std::env;
use std::path::PathBuf;

fn main() {
    // Tell cargo to tell rustc to link the system Eigen
    // shared library. Install on macOS with `brew install eigen`
    // println!("cargo:rustc-link-lib=dylib=stdc++");
    // println!("cargo:rustc-link-lib=static=traj");
    // println!("cargo:rustc-link-search=/home/james/Repositories/trajectories/trajectories-sys");
    // // println!("cargo:rustc-link-lib=libeigen3");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // Disabled so CI passes
        .rustfmt_bindings(false)
        // macOS
        .clang_arg("-I/usr/local/Cellar/eigen/3.3.4/include/eigen3")
        // Linux
        .clang_arg("-I/usr/lib/clang/6.0/include")
        .clang_arg("-I/usr/include/eigen3")
        // .enable_cxx_namespaces()
        .whitelist_type("CircularPathSegment")
        .whitelist_type("LinearPathSegment")
        .whitelist_type("Trajectory")
        .whitelist_type("Path")
        .opaque_type("std::.*")
        .opaque_type("Eigen::.*")
        .blacklist_type("Eigen::Vector3f")
        .blacklist_type("Eigen::Vector3d")
        .blacklist_type("Eigen_Vector3d")
        // The input header we would like to generate
        // bindings for.
        .header("wrapper.hpp")
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").expect("No OUT_DIR set"));
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    cc::Build::new()
        .cpp(true)
        .file("Path.cpp")
        .file("Trajectory.cpp")
        .file("CBindings.cpp")
        .flag("-std=c++11")
        .shared_flag(true)
        .opt_level(3)
        .flag("-lm")
        // Linux
        .include("/usr/include/eigen3")
        // macOS
        .include("/usr/local/Cellar/eigen/3.3.4/include/eigen3")
        .compile("traj");
}
