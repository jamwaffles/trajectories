//! Test bindings to C++ code

extern crate trajectories;
extern crate trajectories_sys;

use trajectories::test_helpers::*;
use trajectories_sys::*;

#[test]
fn cpp_bindings() {
    pretty_env_logger::init();

    let waypoints: Vec<f64> = vec![
        0.0, 0.0, 0.0, //
        0.0, 0.2, 1.0, //
        0.0, 3.0, 0.5, //
        1.1, 2.0, 0.0, //
        1.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, //
        0.0, 0.0, 1.0,
    ];

    println!("Len: {}", waypoints.len());

    let path = unsafe { path_create(waypoints.as_ptr(), waypoints.len(), 0.001f64) };

    println!("Expected: {:?}", waypoints);

    let max_velocity = [1.0, 1.0, 1.0];
    let max_acceleration = [1.0, 1.0, 1.0];

    let traj = unsafe { Trajectory::new(path, &max_velocity, &max_acceleration, 0.001f64) };

    let duration = unsafe { traj.getDuration() };

    println!("TRAJ DURATION {}\n", duration);

    unsafe { assert!(traj.isValid(), "Invalid trajectory") };

    let mut t = 0u64;

    let mut rows = Vec::new();

    while t < (duration * 1000.0) as u64 {
        let divided: f64 = t as f64 / 1000.0;

        let p = unsafe { traj.getPosition(divided) };
        let v = unsafe { traj.getVelocity(divided) };

        let pos: TestPoint = p.into();
        let vel: TestPoint = v.into();

        rows.push(TrajectoryStepRow::from_parts(divided, &pos, &vel));

        t += 100;
    }

    let p = unsafe { traj.getPosition(duration) };
    let v = unsafe { traj.getVelocity(duration) };

    let pos: TestPoint = p.into();
    let vel: TestPoint = v.into();

    rows.push(TrajectoryStepRow::from_parts(duration, &pos, &vel));

    write_debug_csv("../target/plot_cpp_bindings.csv".into(), &rows);

    assert_eq!(2 + 2, 4);
}
