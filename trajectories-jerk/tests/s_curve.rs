use trajectories_jerk::{
    test_helpers::{debug_s_curve_trajectory, TestCoord3},
    LinearTrajectory, Path, TrajectoryOptions,
};

#[test]
fn s_curve_trajectory() {
    let options = TrajectoryOptions {
        velocity_limit: TestCoord3::repeat(1.0),
        acceleration_limit: TestCoord3::repeat(1.0),
    };

    let p = Path::from_waypoints(&vec![
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(1.0, 0.0, 0.0),
        TestCoord3::new(1.0, 1.0, 0.0),
        TestCoord3::new(1.0, 1.0, 1.0),
    ])
    .expect("Failed to create path");

    let traj = LinearTrajectory::new(&p, options).expect("Could not create trajectory");

    debug_s_curve_trajectory(&traj, "s_curve_trajectory", 0.01);
}
