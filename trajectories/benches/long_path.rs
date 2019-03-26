#[macro_use]
extern crate criterion;
extern crate trajectories;

use criterion::*;
use trajectories::prelude::*;
use trajectories::{test_helpers::TestCoord3, Path};

const DEVIATION: f64 = 0.01;
const NUM_POINTS: usize = 100;

fn long_path_bench(c: &mut Criterion) {
    let waypoints: Vec<TestCoord3> = vec![
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
    ];

    c.bench_function("benchmark long path", move |b| {
        b.iter_with_setup(
            || Path::from_waypoints(&waypoints, DEVIATION),
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
    });
}

criterion_group!(long_path, long_path_bench);
criterion_main!(long_path);
