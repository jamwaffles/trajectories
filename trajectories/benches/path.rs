#[macro_use]
extern crate criterion;
extern crate trajectories;

use criterion::*;
use trajectories::prelude::*;
use trajectories::{test_helpers::TestCoord3, Path, PathOptions};

const DEVIATION: f64 = 0.01;
const NUM_POINTS: usize = 100;

fn create_path(c: &mut Criterion) {
    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(1.0, 2.0, 0.0),
        TestCoord3::new(1.5, 1.5, 0.0),
        TestCoord3::new(3.0, 5.0, 0.0),
        TestCoord3::new(4.0, 6.0, 0.0),
        TestCoord3::new(5.0, 5.0, 0.0),
        TestCoord3::new(4.0, 4.0, 0.0),
    ];

    let len = waypoints.len();

    c.bench(
        "path",
        Benchmark::new(format!("create path with {} waypoints", 7), move |b| {
            b.iter(|| {
                Path::from_waypoints(
                    &waypoints,
                    PathOptions {
                        max_deviation: DEVIATION,
                    },
                )
            })
        })
        .throughput(Throughput::Elements(len as u32)),
    );
}

fn create_path_and_iterate(c: &mut Criterion) {
    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(1.0, 2.0, 0.0),
        TestCoord3::new(1.5, 1.5, 0.0),
        TestCoord3::new(3.0, 5.0, 0.0),
        TestCoord3::new(4.0, 6.0, 0.0),
        TestCoord3::new(5.0, 5.0, 0.0),
        TestCoord3::new(4.0, 4.0, 0.0),
    ];

    c.bench_function(
        &format!(
            "create path with {} waypoints and get {} positions",
            7, NUM_POINTS
        ),
        move |b| {
            b.iter_with_setup(
                || {
                    Path::from_waypoints(
                        &waypoints,
                        PathOptions {
                            max_deviation: DEVIATION,
                        },
                    )
                },
                |p| {
                    let len = p.len();
                    let step = len / NUM_POINTS as f64;

                    let mut i = 0.0;

                    while i < len {
                        let _pos = p.position(i);

                        i += step;
                    }
                },
            )
        },
    );
}

fn get_position(c: &mut Criterion) {
    let waypoints: Vec<TestCoord3> = vec![
        TestCoord3::new(0.0, 0.0, 0.0),
        TestCoord3::new(1.0, 2.0, 0.0),
        TestCoord3::new(1.5, 1.5, 0.0),
        TestCoord3::new(3.0, 5.0, 0.0),
        TestCoord3::new(4.0, 6.0, 0.0),
        TestCoord3::new(5.0, 5.0, 0.0),
        TestCoord3::new(4.0, 4.0, 0.0),
    ];

    let len = waypoints.len();

    c.bench(
        "path",
        Benchmark::new("get position at point along path", move |b| {
            b.iter_with_setup(
                || {
                    Path::from_waypoints(
                        &waypoints,
                        PathOptions {
                            max_deviation: DEVIATION,
                        },
                    )
                },
                |p| p.position(5.6789),
            )
        })
        .throughput(Throughput::Elements(len as u32)),
    );
}

fn get_tangent(c: &mut Criterion) {
    c.bench_function("get tangent at point along path", |b| {
        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(1.0, 2.0, 0.0),
            TestCoord3::new(1.5, 1.5, 0.0),
            TestCoord3::new(3.0, 5.0, 0.0),
            TestCoord3::new(4.0, 6.0, 0.0),
            TestCoord3::new(5.0, 5.0, 0.0),
            TestCoord3::new(4.0, 4.0, 0.0),
        ];

        let p = Path::from_waypoints(
            &waypoints,
            PathOptions {
                max_deviation: DEVIATION,
            },
        );

        b.iter(|| {
            p.tangent(5.6789);
        })
    });
}

fn get_curvature(c: &mut Criterion) {
    c.bench_function("get curvature at point along path", |b| {
        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(1.0, 2.0, 0.0),
            TestCoord3::new(1.5, 1.5, 0.0),
            TestCoord3::new(3.0, 5.0, 0.0),
            TestCoord3::new(4.0, 6.0, 0.0),
            TestCoord3::new(5.0, 5.0, 0.0),
            TestCoord3::new(4.0, 4.0, 0.0),
        ];

        let p = Path::from_waypoints(
            &waypoints,
            PathOptions {
                max_deviation: DEVIATION,
            },
        );

        b.iter(|| {
            p.tangent(5.6789);
        })
    });
}

fn get_segment_at_position(c: &mut Criterion) {
    c.bench_function("get segment at position", |b| {
        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(1.0, 2.0, 0.0),
            TestCoord3::new(1.5, 1.5, 0.0),
            TestCoord3::new(3.0, 5.0, 0.0),
            TestCoord3::new(4.0, 6.0, 0.0),
            TestCoord3::new(5.0, 5.0, 0.0),
            TestCoord3::new(4.0, 4.0, 0.0),
        ];

        let p = Path::from_waypoints(
            &waypoints,
            PathOptions {
                max_deviation: DEVIATION,
            },
        );

        b.iter(|| {
            p.segment_at_position(5.6789);
        })
    });
}

criterion_group!(
    path,
    create_path,
    create_path_and_iterate,
    get_position,
    get_tangent,
    get_curvature,
    get_segment_at_position
);
criterion_main!(path);
