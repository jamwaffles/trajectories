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

criterion_group!(path, get_circular_segment_switching_points);
criterion_main!(path);
