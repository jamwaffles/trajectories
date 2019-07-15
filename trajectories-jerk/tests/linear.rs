use trajectories_jerk::{
    test_helpers::{debug_linear_trajectory, TestCoord3},
    LinearTrajectory, Path, TrajectoryOptions, Waypoint,
};

#[test]
fn trapezoidal_trajectory() {
    let options = TrajectoryOptions {
        velocity_limit: TestCoord3::repeat(5.0),
        acceleration_limit: TestCoord3::repeat(10.0),
    };

    let velocity = TestCoord3::new(1.0, 1.0, 1.0);

    let p = Path::from_waypoints(&vec![
        Waypoint::new(TestCoord3::new(0.0, 0.0, 0.0), velocity * 0.0),
        Waypoint::new(TestCoord3::new(5.0, 0.0, 0.0), velocity),
        Waypoint::new(TestCoord3::new(10.0, 0.0, 0.0), velocity * 2.0),
        Waypoint::new(TestCoord3::new(15.0, 0.0, 0.0), velocity * 0.0),
    ])
    .expect("Failed to create path");

    let traj = LinearTrajectory::new(&p, options).expect("Could not create trajectory");

    debug_linear_trajectory(&traj, "linear_trajectory", 0.001);
}
