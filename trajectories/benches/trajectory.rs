#[macro_use]
extern crate criterion;
extern crate trajectories;
extern crate trajectories_sys;

use criterion::Criterion;
use trajectories::{test_helpers::TestCoord3, Path, Trajectory};

fn get_positions(c: &mut Criterion) {
    c.bench_function("get positions from trajectory", |b| {
        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];

        b.iter(|| {
            let p = Path::from_waypoints(&waypoints, 0.001);

            let trajectory = Trajectory::new(
                p,
                TestCoord3::new(1.0, 1.0, 1.0),
                TestCoord3::new(1.0, 1.0, 1.0),
                0.000001,
                0.001,
            )
            .expect("Failed to create trajectory");

            let _start = trajectory.position(0.0);
            let _mid = trajectory.position(5.1234);
            let _end = trajectory.position(trajectory.duration());
        })
    });
}

criterion_group!(trajectory, get_positions,);
criterion_main!(trajectory);
