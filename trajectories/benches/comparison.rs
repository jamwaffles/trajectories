#[macro_use]
extern crate criterion;
extern crate trajectories;
extern crate trajectories_sys;

use criterion::{Criterion, Fun};
use std::time::Duration;
use trajectories::{test_helpers::TestCoord3, Path, PathOptions, Trajectory, TrajectoryOptions};
use trajectories_sys::{path_create, Trajectory as CPPTrajectory};

fn compare(c: &mut Criterion) {
    let comp_native = Fun::new("rust native", |b, _| {
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
            let p = Path::from_waypoints(
                &waypoints,
                PathOptions {
                    max_deviation: 0.001,
                },
            );

            let _trajectory = Trajectory::new(
                &p,
                TrajectoryOptions {
                    velocity_limit: TestCoord3::new(1.0, 1.0, 1.0),
                    acceleration_limit: TestCoord3::new(1.0, 1.0, 1.0),
                    epsilon: 0.000001,
                    timestep: 0.001,
                },
            );
        })
    });
    let comp_bindings = Fun::new("c++ bindings", |b, _| {
        let max_velocity = [1.0, 1.0, 1.0];
        let max_acceleration = [1.0, 1.0, 1.0];

        let waypoints: Vec<f64> = vec![
            0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 3.0, 0.5, 1.1, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 1.0,
        ];

        b.iter(|| {
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
        .warm_up_time(Duration::from_millis(2000))
        .sample_size(25)
        .measurement_time(Duration::from_millis(6000));

    targets =
        compare
);

criterion_main!(create_trajectory);
