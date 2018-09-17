#[macro_use]
extern crate criterion;
extern crate trajectories;

use criterion::Criterion;
use trajectories::prelude::*;
use trajectories::{test_helpers::CircularPathSegment, Coord, Path};

const DEVIATION: f64 = 0.01;
const NUM_POINTS: usize = 100;

fn get_circular_segment_switching_points(c: &mut Criterion) {
    c.bench_function("circular segment switching points", |b| {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(1.0, 5.0, 0.0);
        let after = Coord::new(5.0, 5.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        b.iter(|| {
            let _thing = blend_circle.get_switching_points();
        })
    });
}

fn get_next_switching_point(c: &mut Criterion) {
    c.bench_function("get next switching point", |b| {
        let waypoints: Vec<Coord> = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(1.0, 2.0, 0.0),
            Coord::new(1.5, 1.5, 0.0),
            Coord::new(3.0, 5.0, 0.0),
            Coord::new(4.0, 6.0, 0.0),
            Coord::new(5.0, 5.0, 0.0),
            Coord::new(4.0, 4.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, DEVIATION);

        b.iter(|| {
            let next_waypoint = path.get_next_switching_point(4.0);
            let next_waypoint = path.get_next_switching_point(13.0);
        })
    });
}

criterion_group!(
    path,
    get_circular_segment_switching_points,
    get_next_switching_point
);
criterion_main!(path);
