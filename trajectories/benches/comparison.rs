#[macro_use]
extern crate criterion;
extern crate trajectories;
extern crate trajectories_sys;

use criterion::Criterion;
use std::time::Duration;
use trajectories::prelude::*;
use trajectories::{Coord, Path, Trajectory};
use trajectories_sys::{path_create, Trajectory as CPPTrajectory};

fn cpp_bindings(c: &mut Criterion) {
    c.bench_function("c++ bindings trajectory create", |b| {
        b.iter(|| {
            let max_velocity = [1.0, 1.0, 1.0];
            let max_acceleration = [1.0, 1.0, 1.0];

            let waypoints: Vec<f64> = vec![
                0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 3.0, 0.5, 1.1, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0, 1.0,
            ];

            let path = unsafe { path_create(waypoints.as_ptr(), waypoints.len(), 0.001f64) };

            let mut traj =
                unsafe { CPPTrajectory::new(path, &max_velocity, &max_acceleration, 0.001f64) };
        })
    });
}

fn rust_native(c: &mut Criterion) {
    c.bench_function("rust native trajectory create", |b| {
        b.iter(|| {
            let waypoints: Vec<Coord> = vec![
                Coord::new(0.0, 0.0, 0.0),
                Coord::new(0.0, 0.2, 1.0),
                Coord::new(0.0, 3.0, 0.5),
                Coord::new(1.1, 2.0, 0.0),
                Coord::new(1.0, 0.0, 0.0),
                Coord::new(0.0, 1.0, 0.0),
                Coord::new(0.0, 0.0, 1.0),
            ];

            let p = Path::from_waypoints(&waypoints, 0.001);

            let trajectory = Trajectory::new(
                p,
                Coord::new(1.0, 1.0, 1.0),
                Coord::new(1.0, 1.0, 1.0),
                0.000001,
            );
        })
    });
}

criterion_group!(
    name = create_trajectory;

    config = Criterion::default()
        .warm_up_time(Duration::from_millis(1000))
        .sample_size(5)
        .measurement_time(Duration::from_millis(4000));

    targets =
        rust_native,
        cpp_bindings
);

criterion_main!(create_trajectory);
