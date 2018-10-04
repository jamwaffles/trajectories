//! Profile native Rust

#[macro_use]
extern crate trajectories;

use trajectories::test_helpers::*;
use trajectories::*;

#[test]
fn profile_native() {
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

    let p = Path::from_waypoints(&waypoints, 0.001);

    let trajectory = Trajectory::new(
        p,
        TestCoord3::new(1.0, 1.0, 1.0),
        TestCoord3::new(1.0, 1.0, 1.0),
        0.000001,
    );

    // assert_eq!(trajectory.trajectory.len(), 14814);
    assert_eq!(trajectory.get_duration(), 14.80283284731994);

    // end_profile();
}
