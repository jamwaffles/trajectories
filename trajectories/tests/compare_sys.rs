//! Compare Rust implementation output with C++ bindings

extern crate pretty_env_logger;

use approx::assert_ulps_eq;
use trajectories::test_helpers::TestCoord3;
use trajectories::test_helpers::*;
use trajectories::{prelude::*, Path, PathOptions, Trajectory, TrajectoryOptions};
use trajectories_sys::{path_create, Trajectory as CppTrajectory};

/// test1 from Test.cpp
#[test]
fn compare_test_1() {
    pretty_env_logger::init();

    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(1424.0, 984.999694824219, 2126.0),
        TestCoord3::new(1423.0, 985.000244140625, 2126.0),
    ];

    let rust_path = Path::from_waypoints(
        &waypoints,
        PathOptions {
            max_deviation: 0.001,
        },
    );

    let rust_trajectory = Trajectory::new(
        &rust_path,
        TrajectoryOptions {
            velocity_limit: TestCoord3::new(1.3, 0.67, 0.67),
            acceleration_limit: TestCoord3::new(0.00249, 0.00249, 0.00249),
            // Same epsilon as C++ hardcoded value
            epsilon: 0.000001,
            timestep: 0.001,
        },
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
            // Max deviation
            0.001f64,
        )
    };

    let cpp_trajectory = unsafe {
        CppTrajectory::new(
            cpp_path,
            &[1.3, 0.67, 0.67],
            &[0.00249, 0.00249, 0.00249],
            // Timestep
            0.001f64,
        )
    };

    assert_eq!(rust_trajectory.duration(), unsafe {
        cpp_trajectory.getDuration()
    });
}

/// test2 from Test.cpp
#[test]
fn compare_test_2() {
    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(1427.0, 368.0, 690.0),
        TestCoord3::new(1427.0, 368.0, 790.0),
        TestCoord3::new(952.499938964844, 433.0, 1051.0),
        TestCoord3::new(452.5, 533.0, 1051.0),
        TestCoord3::new(452.5, 533.0, 951.0),
    ];

    let max_deviation = 100.0;
    let timestep = 10.0;
    // Same epsilon as C++ hardcoded value
    let epsilon = 0.000001;

    let rust_path = Path::from_waypoints(&waypoints, PathOptions { max_deviation });

    let rust_path_len = rust_path.len();

    let rust_trajectory = Trajectory::new(
        &rust_path,
        TrajectoryOptions {
            velocity_limit: TestCoord3::new(1.3, 0.67, 0.67),
            acceleration_limit: TestCoord3::new(0.002, 0.002, 0.002),
            epsilon,
            timestep,
        },
    )
    .unwrap();

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
            max_deviation,
        )
    };

    let cpp_trajectory = unsafe {
        CppTrajectory::new(
            cpp_path,
            &[1.3, 0.67, 0.67],
            &[0.002, 0.002, 0.002],
            timestep,
        )
    };

    let mut cpp_rows = Vec::new();
    let mut rust_rows = Vec::new();

    let step_size = 5;

    for i in (0..unsafe { cpp_trajectory.getDuration() } as usize).step_by(step_size) {
        let time: f64 = i as f64;

        let c_p = unsafe { cpp_trajectory.getPosition(time) };
        let c_v = unsafe { cpp_trajectory.getVelocity(time) };

        let c_pos: TestPoint = c_p.into();
        let c_vel: TestPoint = c_v.into();

        cpp_rows.push(TrajectoryStepRow::from_parts(time, &c_pos, &c_vel));
    }

    for i in (0..rust_trajectory.duration() as usize).step_by(step_size) {
        let time: f64 = i as f64;

        let r_p = rust_trajectory.position(time);
        let r_v = rust_trajectory.velocity(time);

        let r_pos = TestPoint::from([r_p[0], r_p[1], r_p[2]]);
        let r_vel = TestPoint::from([r_v[0], r_v[1], r_v[2]]);

        rust_rows.push(TrajectoryStepRow::from_parts(time, &r_pos, &r_vel));
    }

    write_debug_csv("../target/compare_cpp_output.csv".into(), &cpp_rows);
    write_debug_csv("../target/compare_rust_output.csv".into(), &rust_rows);
    write_debug_csv(
        "../target/compare_rust_steps.csv".into(),
        &rust_trajectory
            .trajectory()
            .iter()
            .map(|step| step.time)
            .collect::<Vec<f64>>(),
    );
    write_debug_csv(
        "../target/compare_rust_switching_points.csv".into(),
        &rust_path
            .switching_points()
            .iter()
            .map(|point| point.position)
            .collect::<Vec<f64>>(),
    );

    assert_eq!(
        unsafe { trajectories_sys::Path_getLength(cpp_path) },
        rust_path_len
    );

    assert_ulps_eq!(
        rust_trajectory.duration(),
        unsafe { cpp_trajectory.getDuration() },
        epsilon = epsilon
    );
}
