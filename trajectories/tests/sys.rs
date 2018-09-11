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

    let path = unsafe { path_create(waypoints.as_ptr(), waypoints.len(), 0.1) };

    println!("Expected: {:?}", waypoints);

    let max_velocity = [1.0, 1.0, 1.0];
    let max_acceleration = [1.0, 1.0, 1.0];

    let traj = unsafe { Trajectory::new(path, &max_velocity, &max_acceleration, 0.1) };

    let duration = unsafe { traj.getDuration() };

    println!("TRAJ DURATION {}\n", duration);

    let mut t = 0.0;

    println!("t,px,py,pz,vx,vy,vz");

    while t <= duration {
        println!("Get things for T = {}...", t);

        // let p = unsafe { Trajectory_getPosition(&traj, t) };
        let v = unsafe { Trajectory_getVelocity(&traj, t) };

        // println!("      [[{}]] {:?}", t, v);

        // let pos: Point = p.into();
        // let vel: Point = v.into();

        // println!("{},{},{}", t, pos, vel);

        t += 0.1;
    }

    assert_eq!(2 + 2, 4);
}
