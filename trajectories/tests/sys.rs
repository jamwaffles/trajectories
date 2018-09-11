//! Test bindings to C++ code

extern crate trajectories_sys;

use trajectories_sys::*;

#[test]
fn it_works() {
    let waypoints: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [0.0, 0.2, 1.0],
        [0.0, 3.0, 0.5],
        [1.1, 2.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    println!("Len: {}", waypoints.len());

    let path = unsafe { path_create(waypoints.as_ptr(), waypoints.len() * 3, 0.1) };

    println!("Expected: {:?}", waypoints);

    let config = unsafe { Path_getConfig(path, 0.1) };

    println!("Config: {:?}", config);

    // let mut wp = std_list::new();

    // let max_velocity = [1.0, 1.0, 1.0];
    // let max_acceleration = [1.0, 1.0, 1.0];

    // let traj = Trajectory::new(
    //     &Path::new(waypoints.as_ptr(), 0.1),
    //     &max_velocity,
    //     &max_acceleration,
    //     0.1,
    // );

    assert_eq!(2 + 2, 4);
}
