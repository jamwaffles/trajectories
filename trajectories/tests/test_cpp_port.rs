//! This test suite is a port of the non-random tests from trajectories-sys/Test.cpp

use trajectories::test_helpers::*;
use trajectories::*;

#[test]
fn test_1() {
    let waypoints: Vec<TestCoord4> = vec![
        TestCoord4::new(1424.0, 984.999694824219, 2126.0, 0.0),
        TestCoord4::new(1423.0, 985.000244140625, 2126.0, 0.0),
    ];

    let p = Path::from_waypoints(&waypoints, 100.0);

    let trajectory = Trajectory::new(
        p,
        TestCoord4::new(1.3, 0.67, 0.67, 0.5),
        TestCoord4::new(0.00249, 0.00249, 0.00249, 0.00249),
        10.0,
    );

    // TODO: Loop through, assert things
}

// TODO: Un-ignore
#[test]
#[ignore]
fn test_2() {
    let waypoints: Vec<TestCoord4> = vec![
        TestCoord4::new(1427.0, 368.0, 690.0, 90.0),
        TestCoord4::new(1427.0, 368.0, 790.0, 90.0),
        TestCoord4::new(952.499938964844, 433.0, 1051.0, 90.0),
        TestCoord4::new(452.5, 533.0, 1051.0, 90.0),
        TestCoord4::new(452.5, 533.0, 951.0, 90.0),
    ];

    let p = Path::from_waypoints(&waypoints, 100.0);

    let trajectory = Trajectory::new(
        p,
        TestCoord4::new(1.3, 0.67, 0.67, 0.5),
        TestCoord4::new(0.002, 0.002, 0.002, 0.002),
        10.0,
    );

    // TODO: Loop through, assert things
}
