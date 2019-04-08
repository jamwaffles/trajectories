//! Profile native Rust

extern crate trajectories;

use approx::assert_ulps_eq;
use trajectories::test_helpers::*;
use trajectories::*;

#[test]
fn profile_native() {
    pretty_env_logger::init();

    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(0.0, 0.2, 1.0),
        TestCoord3::new(0.0, 3.0, 0.5),
        TestCoord3::new(1.1, 2.0, 0.0),
        TestCoord3::new(1.0, 0.0, 0.0),
        TestCoord3::new(0.0, 1.0, 0.0),
        TestCoord3::new(0.0, 0.0, 1.0),
    ];

    // start_profile();

    let p = Path::from_waypoints(
        &waypoints,
        PathOptions {
            max_deviation: 0.001,
        },
    );

    let trajectory = Trajectory::new(
        p,
        TrajectoryOptions {
            velocity_limit: TestCoord3::new(1.0, 1.0, 1.0),
            acceleration_limit: TestCoord3::new(1.0, 1.0, 1.0),
            epsilon: 0.000001,
            timestep: 0.001,
        },
    )
    .unwrap();

    let _point1 = trajectory.position(0.01);
    let _point2 = trajectory.position(5.678);
    let _point3 = trajectory.position(7.89);
    let _point4 = trajectory.position(14.00001);

    assert_ulps_eq!(trajectory.duration(), 14.802832847319943);

    // end_profile();
}
