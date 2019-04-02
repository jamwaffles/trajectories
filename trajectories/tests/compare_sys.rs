//! This test suite is a port of the non-random tests from trajectories-sys/Test.cpp

extern crate pretty_env_logger;

use trajectories::test_helpers::*;
use trajectories::*;
use trajectories_sys::{path_create, Trajectory as CppTrajectory};

#[test]
fn compare_test_1() {
    pretty_env_logger::init();

    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(1424.0, 984.999694824219, 2126.0),
        TestCoord3::new(1423.0, 985.000244140625, 2126.0),
    ];

    let rust_path = Path::from_waypoints(&waypoints, 0.001);

    let rust_trajectory = Trajectory::new(
        rust_path,
        TestCoord3::new(1.3, 0.67, 0.67),
        TestCoord3::new(0.00249, 0.00249, 0.00249),
        0.000001,
        0.001,
    )
    .unwrap();

    let cpp_path = unsafe {
        path_create(
            vec![
                1424.0,
                984.999694824219,
                2126.0, //
                1423.0,
                985.000244140625,
                2126.0,
            ]
            .as_ptr(),
            waypoints.len() * 3,
            0.001f64,
        )
    };

    let cpp_trajectory = unsafe {
        CppTrajectory::new(
            cpp_path,
            &[1.3, 0.67, 0.67],
            &[0.00249, 0.00249, 0.00249],
            0.001f64,
        )
    };

    assert_eq!(rust_trajectory.duration(), unsafe {
        cpp_trajectory.getDuration()
    });
}
