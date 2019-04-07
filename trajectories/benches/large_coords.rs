extern crate criterion;
extern crate trajectories;

use criterion::*;
use std::time::Duration;
use trajectories::{test_helpers::TestCoord4, Path, PathOptions, Trajectory, TrajectoryOptions};

fn test_2(c: &mut Criterion) {
    let waypoints: Vec<TestCoord4> = vec![
        TestCoord4::new(1427.0, 368.0, 690.0, 90.0),
        TestCoord4::new(1427.0, 368.0, 790.0, 90.0),
        TestCoord4::new(952.499938964844, 433.0, 1051.0, 90.0),
        TestCoord4::new(452.5, 533.0, 1051.0, 90.0),
        TestCoord4::new(452.5, 533.0, 951.0, 90.0),
    ];

    let len = waypoints.len();

    c.bench(
        "large_coords",
        Benchmark::new("test_2 path with large coordinates", move |b| {
            b.iter(|| {
                let p = Path::from_waypoints(
                    &waypoints,
                    PathOptions {
                        max_deviation: 100.0,
                    },
                );

                let _trajectory = Trajectory::new(
                    p,
                    TrajectoryOptions {
                        velocity_limit: TestCoord4::new(1.3, 0.67, 0.67, 0.5),
                        acceleration_limit: TestCoord4::new(0.002, 0.002, 0.002, 0.002),
                        epsilon: 0.000001,
                        timestep: 10.0,
                    },
                );
            })
        })
        .throughput(Throughput::Elements(len as u32)),
    );
}

criterion_group!(
    name = large_coords;

    config = Criterion::default()
        .warm_up_time(Duration::from_millis(1000))
        .sample_size(10)
        .measurement_time(Duration::from_millis(5000));

    targets =
        test_2,
);

criterion_main!(large_coords);
