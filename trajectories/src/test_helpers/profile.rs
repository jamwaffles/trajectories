#[cfg(feature = "profile")]
extern crate cpuprofiler;

use std::error::Error;
use std::ffi::OsStr;
use std::fs;
use std::fs::OpenOptions;
use std::io;
use std::io::prelude::*;
use std::path::{Path, PathBuf};

#[cfg(feature = "profile")]
#[allow(dead_code)]
/// Begin a profiling capture
pub fn start_profile() {
    use self::cpuprofiler::PROFILER;
    use std::env;
    use std::fs;

    let exe = env::current_exe().unwrap();
    let exe_name = Path::new(&exe).file_name().unwrap();

    fs::create_dir_all("../target/profiles").unwrap();

    let profile_name = format!("../target/profiles/{}.profile", exe_name.to_str().unwrap());

    println!("Profiling into {}", profile_name);

    PROFILER.lock().unwrap().start(profile_name).unwrap();
}

#[cfg(feature = "profile")]
#[allow(dead_code)]
/// End the current profiling capture
pub fn end_profile() {
    use self::cpuprofiler::PROFILER;

    PROFILER.lock().unwrap().stop().unwrap();
}

#[cfg(not(feature = "profile"))]
#[allow(dead_code)]
/// Begin a profiling capture (noop, enable `profile` feature for functionality)
pub fn start_profile() {
    println!("Profile noop");
    // Noop
}

#[cfg(not(feature = "profile"))]
#[allow(dead_code)]
/// End the current profiling capture (noop, enable `profile` feature for functionality)
pub fn end_profile() {
    println!("Profile noop");
    // Noop
}
