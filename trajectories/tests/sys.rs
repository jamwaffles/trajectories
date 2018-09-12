//! Test bindings to C++ code

extern crate trajectories_sys;

use std::fmt;
use trajectories_sys::*;

struct Point {
    x: f64,
    y: f64,
    z: f64,
}

impl From<[f64; 3]> for Point {
    fn from(other: [f64; 3]) -> Self {
        Point {
            x: other[0],
            y: other[1],
            z: other[2],
        }
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}, {}, {}", self.x, self.y, self.z)
    }
}

#[test]
fn it_works() {
    let waypoints: Vec<f64> = vec![
        0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 3.0, 0.5, 1.1, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ];

    println!("Len: {}", waypoints.len());

    let path = unsafe { path_create(waypoints.as_ptr(), waypoints.len(), 0.1f64) };

    println!("Expected: {:?}", waypoints);

    let max_velocity = [1.0, 1.0, 1.0];
    let max_acceleration = [1.0, 1.0, 1.0];

    let traj = unsafe { Trajectory::new(path, &max_velocity, &max_acceleration, 0.001f64) };

    let duration = unsafe { traj.getDuration() };

    println!("TRAJ DURATION {}\n", duration);

    unsafe { assert!(traj.isValid(), "Invalid trajectory") };

    let mut t = 0u64;

    println!("t,px,py,pz,vx,vy,vz");

    while t < (duration * 1000.0) as u64 {
        let divided: f64 = t as f64 / 1000.0;

        let p = unsafe { Trajectory_getPosition(&traj, divided) };
        let v = unsafe { Trajectory_getVelocity(&traj, divided) };

        let pos: Point = p.into();
        let vel: Point = v.into();

        println!("{},{},{}", divided, pos, vel);

        t += 100;
    }

    let p = unsafe { Trajectory_getPosition(&traj, duration) };
    let v = unsafe { Trajectory_getVelocity(&traj, duration) };

    let pos: Point = p.into();
    let vel: Point = v.into();

    println!("{},{},{}", duration, pos, vel);

    assert_eq!(2 + 2, 4);
}
