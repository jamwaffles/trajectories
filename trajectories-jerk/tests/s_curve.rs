use trajectories_jerk::{
    test_helpers::{debug_s_curve_trajectory, TestCoord3},
    LinearTrajectory, Path, TrajectoryOptions, Waypoint,
};

#[test]
#[ignore]
fn s_curve_trajectory() {
    let options = TrajectoryOptions {
        velocity_limit: TestCoord3::repeat(1.0),
        acceleration_limit: TestCoord3::repeat(1.0),
    };

    let velocity = TestCoord3::new(1.0, 1.0, 1.0);

    let p = Path::from_waypoints(&vec![
        Waypoint::new(TestCoord3::new(0.0, 0.0, 0.0), velocity * 0.0),
        Waypoint::new(TestCoord3::new(1.0, 0.0, 0.0), velocity),
        Waypoint::new(TestCoord3::new(1.0, 1.0, 0.0), velocity * 1.5),
        Waypoint::new(TestCoord3::new(1.0, 1.0, 1.0), velocity * 0.0),
    ])
    .expect("Failed to create path");

    let traj = LinearTrajectory::new(&p, options).expect("Could not create trajectory");

    debug_s_curve_trajectory(&traj, "s_curve_trajectory", 0.01);
}
