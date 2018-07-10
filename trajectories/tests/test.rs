extern crate trajectories;

use trajectories::*;

#[test]
fn it_works() {
    let waypoints = vec![
        Coord::new(0.0, 0.0, 0.0),
        Coord::new(0.0, 0.2, 1.0),
        Coord::new(0.0, 3.0, 0.5),
        Coord::new(1.1, 2.0, 0.0),
        Coord::new(1.0, 0.0, 0.0),
        Coord::new(0.0, 1.0, 0.0),
        Coord::new(0.0, 0.0, 1.0),
    ];

    let maxAcceleration = Coord::repeat(1.0);
    let maxVelocity = Coord::repeat(1.0);

    let path = Path::from_waypoints(waypoints);
}
