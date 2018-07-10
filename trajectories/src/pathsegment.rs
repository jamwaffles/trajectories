use std::cmp;

use super::Coord;

pub trait PathSegment {
    fn get_length(&self) -> f64;

    fn get_config(&self, s: f64) -> Coord;

    fn get_tangent(&self, s: f64) -> Coord;

    fn get_curvature(&self, s: f64) -> Coord;

    fn get_switching_points(&self, s: f64) -> Vec<f64>;
}

#[derive(Copy, Clone, Debug)]
pub struct LinearPathSegment {
    // position: f64,
    length: f64,
    start: Coord,
    end: Coord,
}

impl LinearPathSegment {
    fn new(start: Coord, end: Coord) -> Self {
        LinearPathSegment {
            start,
            end,
            length: (end - start).norm(),
        }
    }
}

impl PathSegment for LinearPathSegment {
    fn get_length(&self) -> f64 {
        self.length
    }

    fn get_config(&self, s: f64) -> Coord {
        let clamped = 0.0f64.max(1.0f64.min(s / self.length));

        (1.0 - clamped) * self.start + clamped * self.end
    }

    fn get_tangent(&self, s: f64) -> Coord {
        (self.end - self.start) / self.length
    }

    fn get_curvature(&self, s: f64) -> Coord {
        Coord::repeat(0.0)
    }

    fn get_switching_points(&self, s: f64) -> Vec<f64> {
        Vec::new()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct CircularPathSegment {
    // position: Option<f64>,
    length: f64,
    radius: f64,
    center: Coord,
    x: Coord,
    y: Coord,
}

impl CircularPathSegment {
    fn new(start: Coord, intersection: Coord, end: Coord, max_deviation: f64) -> Self {
        if (intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001 {
            return Self {
                length: 0.0,
                radius: 1.0,
                center: intersection,
                x: Coord::repeat(0.0),
                y: Coord::repeat(0.0),
            };
        }

        let start_direction = (intersection - start).normalize();
        let end_direction = (end - intersection).normalize();

        if (start_direction - end_direction).norm() < 0.000001 {
            return Self {
                length: 0.0,
                radius: 1.0,
                center: intersection,
                x: Coord::repeat(0.0),
                y: Coord::repeat(0.0),
            };
        }

        // let start_distance = (start - intersection).norm();
        // let end_distance = (end - intersection).norm();

        let mut distance = (start - intersection)
            .norm()
            .min((end - intersection).norm());

        let angle = start_direction.dot(&end_direction).acos();

        distance = distance.min(max_deviation * (0.5 * angle).sin() / (1.0 - (0.5 * angle).cos()));

        let radius = distance / (0.5 * angle).tan();
        let length = angle * radius;

        let center = intersection
            + (end_direction - start_direction).normalize() * radius / (0.5 * angle).cos();
        let x = (intersection - distance * start_direction - center).normalize();
        let y = start_direction;

        Self {
            length,
            radius,
            center,
            x,
            y,
        }
    }
}

impl PathSegment for CircularPathSegment {
    fn get_length(&self) -> f64 {
        self.length
    }

    fn get_config(&self, s: f64) -> Coord {
        unimplemented!()
    }

    fn get_tangent(&self, s: f64) -> Coord {
        unimplemented!()
    }

    fn get_curvature(&self, s: f64) -> Coord {
        unimplemented!()
    }

    fn get_switching_points(&self, s: f64) -> Vec<f64> {
        unimplemented!()
    }
}
