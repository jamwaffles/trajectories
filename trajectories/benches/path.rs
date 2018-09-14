#[macro_use]
extern crate criterion;
extern crate trajectories;

use criterion::Criterion;
use trajectories::PathItem;
use trajectories::{Coord, Path};

const DEVIATION: f64 = 0.01;
const NUM_POINTS: usize = 100;

fn create_path(c: &mut Criterion) {
    c.bench_function(&format!("create path with {} waypoints", 7), |b| {
        let waypoints: Vec<Coord> = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(1.0, 2.0, 0.0),
            Coord::new(1.5, 1.5, 0.0),
            Coord::new(3.0, 5.0, 0.0),
            Coord::new(4.0, 6.0, 0.0),
            Coord::new(5.0, 5.0, 0.0),
            Coord::new(4.0, 4.0, 0.0),
        ];

        b.iter(|| Path::from_waypoints(&waypoints, DEVIATION))
    });
}

fn create_path_and_iterate(c: &mut Criterion) {
    c.bench_function(
        &format!(
            "create path with {} waypoints and get {} positions",
            7, NUM_POINTS
        ),
        |b| {
            let waypoints: Vec<Coord> = vec![
                Coord::new(0.0, 0.0, 0.0),
                Coord::new(1.0, 2.0, 0.0),
                Coord::new(1.5, 1.5, 0.0),
                Coord::new(3.0, 5.0, 0.0),
                Coord::new(4.0, 6.0, 0.0),
                Coord::new(5.0, 5.0, 0.0),
                Coord::new(4.0, 4.0, 0.0),
            ];

            let p = Path::from_waypoints(&waypoints, DEVIATION);

            b.iter(|| {
                let len = p.get_length();
                let step = len / NUM_POINTS as f64;

                let mut i = 0.0;

                while i < len {
                    let _pos = p.get_position(i);

                    i += step;
                }
            })
        },
    );
}

criterion_group!(path, create_path, create_path_and_iterate);
criterion_main!(path);
