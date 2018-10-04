#[macro_use]
extern crate criterion;
extern crate trajectories;
extern crate trajectories_sys;

use criterion::{Criterion, Fun};
use std::time::Duration;
use trajectories::{test_helpers::TestCoord3, Path, Trajectory};
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

            let _traj =
                unsafe { CPPTrajectory::new(path, &max_velocity, &max_acceleration, 0.001f64) };
        })
    });
}

fn rust_native(c: &mut Criterion) {
    c.bench_function("rust native trajectory create", |b| {
        b.iter(|| {
            let waypoints: Vec<TestCoord3> = vec![
                TestCoord3::new(0.0, 0.0, 0.0),
                TestCoord3::new(0.0, 0.2, 1.0),
                TestCoord3::new(0.0, 3.0, 0.5),
                TestCoord3::new(1.1, 2.0, 0.0),
                TestCoord3::new(1.0, 0.0, 0.0),
                TestCoord3::new(0.0, 1.0, 0.0),
                TestCoord3::new(0.0, 0.0, 1.0),
            ];

            let p = Path::from_waypoints(&waypoints, 0.001);

            let _trajectory = Trajectory::new(
                p,
                TestCoord3::new(1.0, 1.0, 1.0),
                TestCoord3::new(1.0, 1.0, 1.0),
                0.000001,
            );
        })
    });
}

fn compare(c: &mut Criterion) {
    let comp_native = Fun::new("c++ bindings", |b, _| {
        b.iter(|| {
            let waypoints: Vec<TestCoord3> = vec![
                TestCoord3::new(0.0, 0.0, 0.0),
                TestCoord3::new(0.0, 0.2, 1.0),
                TestCoord3::new(0.0, 3.0, 0.5),
                TestCoord3::new(1.1, 2.0, 0.0),
                TestCoord3::new(1.0, 0.0, 0.0),
                TestCoord3::new(0.0, 1.0, 0.0),
                TestCoord3::new(0.0, 0.0, 1.0),
            ];

            let p = Path::from_waypoints(&waypoints, 0.001);

            let _trajectory = Trajectory::new(
                p,
                TestCoord3::new(1.0, 1.0, 1.0),
                TestCoord3::new(1.0, 1.0, 1.0),
                0.000001,
            );
        })
    });
    let comp_bindings = Fun::new("rust native", |b, _| {
        b.iter(|| {
            let max_velocity = [1.0, 1.0, 1.0];
            let max_acceleration = [1.0, 1.0, 1.0];

            let waypoints: Vec<f64> = vec![
                0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 3.0, 0.5, 1.1, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0, 0.0, 1.0,
            ];

            let path = unsafe { path_create(waypoints.as_ptr(), waypoints.len(), 0.001f64) };

            let _traj =
                unsafe { CPPTrajectory::new(path, &max_velocity, &max_acceleration, 0.001f64) };
        })
    });

    let functions = vec![comp_native, comp_bindings];

    c.bench_functions("compare native trajectory create", functions, ());
}

criterion_group!(
    name = create_trajectory;

    config = Criterion::default()
        .warm_up_time(Duration::from_millis(1000))
        .sample_size(30)
        .measurement_time(Duration::from_millis(6000));

    targets =
        rust_native,
        cpp_bindings,
        compare
);

criterion_main!(create_trajectory);
