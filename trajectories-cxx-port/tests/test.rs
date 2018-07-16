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

    let max_acceleration = Coord::repeat(1.0);
    let max_velocity = Coord::repeat(1.0);

    let path = Path::from_waypoints(&waypoints, 0.1);

    let trajectory = Trajectory::from_path(&path, max_velocity, max_acceleration, 0.1);

    trajectory.output_phase_plane_trajectory();

    if trajectory.is_valid() {
        let duration = trajectory.get_duration();

        println!("Trajectory duration: {}", duration);
        println!("Time      Position                  Velocity");

        let mut t = 0.0;

        while t < duration {
            println!(
                "{}: {:?} {:?}",
                t,
                trajectory.get_position(t),
                trajectory.get_velocity(t)
            );

            t += 0.1;
        }

        println!(
            "{}: {:?} {:?}",
            duration,
            trajectory.get_position(duration),
            trajectory.get_velocity(duration)
        );
    } else {
        panic!("Trajectory gen failed");
    }
}
