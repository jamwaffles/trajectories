//! Compare Rust implementation output with C++ bindings

extern crate pretty_env_logger;

use approx::assert_ulps_eq;
use trajectories::test_helpers::TestCoord3;
use trajectories::{prelude::*, Path, PathOptions};
use trajectories_sys::path_create;

/// Compare Rust impl of Path against C++ version
#[test]
fn compare_path_length_large_deviation() {
    pretty_env_logger::init();

    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(1427.0, 368.0, 690.0),
        TestCoord3::new(1427.0, 368.0, 790.0),
        TestCoord3::new(952.499938964844, 433.0, 1051.0),
        TestCoord3::new(452.5, 533.0, 1051.0),
        TestCoord3::new(452.5, 533.0, 951.0),
    ];

    let rust_path = Path::from_waypoints(
        &waypoints,
        PathOptions {
            max_deviation: 100.0,
        },
    );

    let cpp_path = unsafe {
        path_create(
            vec![
                1427.0,
                368.0,
                690.0, //
                1427.0,
                368.0,
                790.0, //
                952.499938964844,
                433.0,
                1051.0, //
                452.5,
                533.0,
                1051.0, //
                452.5,
                533.0,
                951.0,
            ]
            .as_ptr(),
            waypoints.len() * 3,
            100.0f64,
        )
    };

    assert_ulps_eq!(rust_path.len(), unsafe {
        trajectories_sys::Path_getLength(cpp_path)
    });
}

/// Compare Rust impl of Path against C++ version
#[test]
fn compare_path_length() {
    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(1427.0, 368.0, 690.0),
        TestCoord3::new(1427.0, 368.0, 790.0),
        TestCoord3::new(952.499938964844, 433.0, 1051.0),
        TestCoord3::new(452.5, 533.0, 1051.0),
        TestCoord3::new(452.5, 533.0, 951.0),
    ];

    let rust_path = Path::from_waypoints(
        &waypoints,
        PathOptions {
            max_deviation: 0.001,
        },
    );

    let cpp_path = unsafe {
        path_create(
            vec![
                1427.0,
                368.0,
                690.0, //
                1427.0,
                368.0,
                790.0, //
                952.499938964844,
                433.0,
                1051.0, //
                452.5,
                533.0,
                1051.0, //
                452.5,
                533.0,
                951.0,
            ]
            .as_ptr(),
            waypoints.len() * 3,
            0.001f64,
        )
    };

    assert_ulps_eq!(rust_path.len(), unsafe {
        trajectories_sys::Path_getLength(cpp_path)
    });
}
