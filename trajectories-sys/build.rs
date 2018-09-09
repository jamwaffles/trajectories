extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    // Tell cargo to tell rustc to link the system Eigen
    // shared library. Install on macOS with `brew install eigen`
    // println!("cargo:rustc-link-lib=libeigen");
    // println!("cargo:rustc-link-lib=libeigen3");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // macOS
        .clang_arg("-I/usr/local/Cellar/eigen/3.3.4/include/eigen3")

        // Linux
        .clang_arg("-I/usr/lib/clang/6.0/include")
        .clang_arg("-I/usr/include/eigen3")

        .enable_cxx_namespaces()

        .whitelist_type("CircularPathSegment")
        .whitelist_type("LinearPathSegment")

        .whitelist_type("Trajectory")

        .opaque_type("std::list")
        .opaque_type("Eigen::.*")
        // .blacklist_type("Eigen::.*")

        .whitelist_type("Eigen::VectorXd")
        // .whitelist_type("std::list")
        // .whitelist_type("std::pair")

        // .emit_ir_graphviz("./bindings.dot")

        // The input header we would like to generate
        // bindings for.
        .header("wrapper.hpp")
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
