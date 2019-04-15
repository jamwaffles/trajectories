extern crate criterion;
extern crate trajectories;

use criterion::*;
use trajectories::prelude::*;
use trajectories::{
    test_helpers::{
        max_acceleration_at, max_velocity_at, LimitType, MinMax, TestCoord3, TrajectoryStep,
    },
    Path, PathOptions, TrajectoryOptions,
};

const DEVIATION: f64 = 0.01;
const NUM_POINTS: usize = 100;

fn waypoints() -> Vec<TestCoord3> {
    vec![
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(4.0, 0.0, 4.0),
        TestCoord3::new(10.897, 00.111, 1.1234),
        TestCoord3::new(1.5, 1.5, 0.0),
        TestCoord3::new(1.0, 2.0, 0.0),
        TestCoord3::new(00.897, 10.111, 2.1234),
        TestCoord3::new(00.897, 10.111, 2.1234),
        TestCoord3::new(4.0, 6.0, 0.0),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(10.897, 20.111, 0.1234),
        TestCoord3::new(0.0, 1.0, 2.0),
        TestCoord3::new(1.5, 0.0, 1.5),
        TestCoord3::new(10.897, 00.111, 2.1234),
        TestCoord3::new(00.897, 10.111, 1.1234),
        TestCoord3::new(40.897, 00.111, 4.1234),
        TestCoord3::new(0.0, 4.0, 6.0),
        TestCoord3::new(1.0, 0.0, 2.0),
        TestCoord3::new(4.0, 4.0, 0.0),
        TestCoord3::new(50.897, 00.111, 5.1234),
        TestCoord3::new(50.897, 50.111, 0.1234),
        TestCoord3::new(4.0, 0.0, 6.0),
        TestCoord3::new(5.0, 5.0, 0.0),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(40.897, 40.111, 0.1234),
        TestCoord3::new(5.0, 5.0, 0.0),
        TestCoord3::new(1.5, 1.5, 0.0),
        TestCoord3::new(50.897, 00.111, 5.1234),
        TestCoord3::new(1.0, 0.0, 2.0),
        TestCoord3::new(40.897, 40.111, 0.1234),
        TestCoord3::new(10.897, 20.111, 0.1234),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(1.5, 0.0, 1.5),
        TestCoord3::new(1.0, 2.0, 0.0),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(0.0, 5.0, 5.0),
        TestCoord3::new(4.0, 6.0, 0.0),
        TestCoord3::new(40.897, 60.111, 0.1234),
        TestCoord3::new(00.897, 30.111, 5.1234),
        TestCoord3::new(4.0, 0.0, 6.0),
        TestCoord3::new(4.0, 0.0, 4.0),
        TestCoord3::new(0.0, 4.0, 4.0),
        TestCoord3::new(00.897, 30.111, 5.1234),
        TestCoord3::new(4.0, 4.0, 0.0),
        TestCoord3::new(3.0, 0.0, 5.0),
        TestCoord3::new(00.897, 40.111, 6.1234),
        TestCoord3::new(0.0, 1.0, 2.0),
        TestCoord3::new(50.897, 50.111, 0.1234),
        TestCoord3::new(0.0, 5.0, 5.0),
        TestCoord3::new(00.897, 10.111, 1.1234),
        TestCoord3::new(00.897, 40.111, 4.1234),
        TestCoord3::new(0.0, 3.0, 5.0),
        TestCoord3::new(30.897, 00.111, 5.1234),
        TestCoord3::new(10.897, 10.111, 0.1234),
        TestCoord3::new(10.897, 00.111, 2.1234),
        TestCoord3::new(0.0, 4.0, 6.0),
        TestCoord3::new(1.5, 0.0, 1.5),
        TestCoord3::new(4.0, 4.0, 0.0),
        TestCoord3::new(1.5, 1.5, 0.0),
        TestCoord3::new(5.0, 5.0, 0.0),
        TestCoord3::new(0.0, 3.0, 5.0),
        TestCoord3::new(30.897, 50.111, 0.1234),
        TestCoord3::new(40.897, 40.111, 0.1234),
        TestCoord3::new(00.897, 50.111, 5.1234),
        TestCoord3::new(40.897, 00.111, 4.1234),
        TestCoord3::new(10.897, 00.111, 1.1234),
        TestCoord3::new(1.0, 0.0, 2.0),
        TestCoord3::new(4.0, 6.0, 0.0),
        TestCoord3::new(40.897, 00.111, 6.1234),
        TestCoord3::new(00.897, 40.111, 4.1234),
        TestCoord3::new(5.0, 0.0, 5.0),
        TestCoord3::new(0.0, 1.5, 1.5),
        TestCoord3::new(00.897, 40.111, 6.1234),
        TestCoord3::new(50.897, 50.111, 0.1234),
        TestCoord3::new(0.0, 4.0, 6.0),
        TestCoord3::new(3.0, 0.0, 5.0),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(5.0, 0.0, 5.0),
        TestCoord3::new(10.897, 10.111, 0.1234),
        TestCoord3::new(00.897, 40.111, 4.1234),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(00.897, 40.111, 6.1234),
        TestCoord3::new(3.0, 5.0, 0.0),
        TestCoord3::new(1.0, 2.0, 0.0),
        TestCoord3::new(40.897, 00.111, 4.1234),
        TestCoord3::new(40.897, 60.111, 0.1234),
        TestCoord3::new(5.0, 0.0, 5.0),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(30.897, 00.111, 5.1234),
        TestCoord3::new(40.897, 00.111, 6.1234),
        TestCoord3::new(30.897, 50.111, 0.1234),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(50.897, 00.111, 5.1234),
        TestCoord3::new(0.0, 4.0, 4.0),
        TestCoord3::new(00.897, 10.111, 2.1234),
        TestCoord3::new(0.0, 5.0, 5.0),
        TestCoord3::new(3.0, 5.0, 0.0),
        TestCoord3::new(10.897, 00.111, 2.1234),
        TestCoord3::new(00.897, 50.111, 5.1234),
        TestCoord3::new(3.0, 5.0, 0.0),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(0.0, 1.5, 1.5),
        TestCoord3::new(00.897, 10.111, 1.1234),
        TestCoord3::new(0.0, 4.0, 4.0),
        TestCoord3::new(00.897, 30.111, 5.1234),
        TestCoord3::new(0.0, 3.0, 5.0),
        TestCoord3::new(0.0, 1.5, 1.5),
        TestCoord3::new(10.897, 10.111, 0.1234),
        TestCoord3::new(4.0, 0.0, 6.0),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(30.897, 00.111, 5.1234),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(30.897, 50.111, 0.1234),
        TestCoord3::new(4.0, 0.0, 4.0),
        TestCoord3::new(0.0, 1.0, 2.0),
        TestCoord3::new(10.897, 00.111, 1.1234),
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(40.897, 00.111, 6.1234),
        TestCoord3::new(00.897, 50.111, 5.1234),
        TestCoord3::new(00.897, 00.111, 0.1234),
        TestCoord3::new(10.897, 20.111, 0.1234),
        TestCoord3::new(40.897, 60.111, 0.1234),
        TestCoord3::new(3.0, 0.0, 5.0),
        TestCoord3::new(0.0, 0.0, 0.0),
    ]
}

fn bench_max_velocity_at_accel_limited(c: &mut Criterion) {
    c.bench_function("max_velocity_at_accel_limited", move |b| {
        let options = TrajectoryOptions {
            acceleration_limit: TestCoord3::repeat(1.0),
            velocity_limit: TestCoord3::repeat(1.0),
            epsilon: 0.000001,
            timestep: 0.01,
        };

        b.iter_with_setup(
            || {
                let p = Path::from_waypoints(
                    &waypoints(),
                    PathOptions {
                        max_deviation: DEVIATION,
                    },
                );
                let len = p.len();
                let step = len / NUM_POINTS as f64;

                (p, len, step)
            },
            |(p, len, step)| {
                let mut i = 0.0;

                while i < len {
                    let _pos =
                        max_velocity_at(&p, i, LimitType::Acceleration(options.acceleration_limit));

                    i += step;
                }
            },
        )
    });
}

fn bench_max_velocity_at_vel_limited(c: &mut Criterion) {
    c.bench_function("max_velocity_at_vel_limited", move |b| {
        let options = TrajectoryOptions {
            acceleration_limit: TestCoord3::repeat(1.0),
            velocity_limit: TestCoord3::repeat(1.0),
            epsilon: 0.000001,
            timestep: 0.01,
        };

        b.iter_with_setup(
            || {
                let p = Path::from_waypoints(
                    &waypoints(),
                    PathOptions {
                        max_deviation: DEVIATION,
                    },
                );
                let len = p.len();
                let step = len / NUM_POINTS as f64;

                (p, len, step)
            },
            |(p, len, step)| {
                let mut i = 0.0;

                while i < len {
                    let _pos = max_velocity_at(&p, i, LimitType::Velocity(options.velocity_limit));

                    i += step;
                }
            },
        )
    });
}

fn bench_max_acceleration_at(c: &mut Criterion) {
    c.bench_function("max_acceleration_at", move |b| {
        let options = TrajectoryOptions {
            acceleration_limit: TestCoord3::repeat(1.0),
            velocity_limit: TestCoord3::repeat(1.0),
            epsilon: 0.000001,
            timestep: 0.01,
        };

        b.iter_with_setup(
            || {
                let p = Path::from_waypoints(
                    &waypoints(),
                    PathOptions {
                        max_deviation: DEVIATION,
                    },
                );
                let len = p.len();
                let step = len / NUM_POINTS as f64;

                (p, len, step)
            },
            |(p, len, step)| {
                let mut i = 0.0;

                while i < len {
                    let _pos = max_acceleration_at(
                        &p,
                        &TrajectoryStep::new(i, 1.0),
                        MinMax::Max,
                        &options,
                    );

                    i += step;
                }
            },
        )
    });
}

criterion_group!(
    limits,
    bench_max_velocity_at_accel_limited,
    bench_max_velocity_at_vel_limited,
    bench_max_acceleration_at
);
criterion_main!(limits);
