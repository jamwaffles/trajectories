use std::f64;

use super::Coord;

pub trait PathSegment: Copy + Clone {
    fn get_position(&self) -> f64;

    fn get_length(&self) -> f64;

    fn get_config(&self, s: f64) -> Coord;

    fn get_tangent(&self, s: f64) -> Coord;

    fn get_curvature(&self, s: f64) -> Coord;

    fn get_switching_points(&self) -> Vec<f64>;

    fn set_position(&mut self, position: f64);
}

#[derive(Copy, Clone, Debug)]
pub struct LinearPathSegment {
    position: f64,
    length: f64,
    start: Coord,
    end: Coord,
}

impl LinearPathSegment {
    pub fn new(start: Coord, end: Coord) -> Self {
        LinearPathSegment {
            position: 0.0,
            start,
            end,
            length: (end - start).norm(),
        }
    }
}

impl PathSegment for LinearPathSegment {
    fn get_position(&self) -> f64 {
        self.position
    }

    fn set_position(&mut self, position: f64) {
        self.position = position;
    }

    fn get_length(&self) -> f64 {
        self.length
    }

    fn get_config(&self, s: f64) -> Coord {
        let clamped = 0.0f64.max(1.0f64.min(s / self.length));

        (1.0 - clamped) * self.start + clamped * self.end
    }

    fn get_tangent(&self, _s: f64) -> Coord {
        (self.end - self.start) / self.length
    }

    fn get_curvature(&self, _s: f64) -> Coord {
        Coord::zeros()
    }

    fn get_switching_points(&self) -> Vec<f64> {
        Vec::new()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct CircularPathSegment {
    position: f64,
    length: f64,
    radius: f64,
    center: Coord,
    x: Coord,
    y: Coord,
}

impl CircularPathSegment {
    pub fn new(start: Coord, intersection: Coord, end: Coord, max_deviation: f64) -> Self {
        if (intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001 {
            return Self {
                position: 0.0,
                length: 0.0,
                radius: 1.0,
                center: intersection,
                x: Coord::zeros(),
                y: Coord::zeros(),
            };
        }

        let start_direction = (intersection - start).normalize();
        let end_direction = (end - intersection).normalize();

        if (start_direction - end_direction).norm() < 0.000001 {
            return Self {
                position: 0.0,
                length: 0.0,
                radius: 1.0,
                center: intersection,
                x: Coord::zeros(),
                y: Coord::zeros(),
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
            position: 0.0,
            length,
            radius,
            center,
            x,
            y,
        }
    }
}

impl PathSegment for CircularPathSegment {
    fn get_position(&self) -> f64 {
        self.position
    }

    fn set_position(&mut self, position: f64) {
        self.position = position;
    }

    fn get_length(&self) -> f64 {
        self.length
    }

    fn get_config(&self, s: f64) -> Coord {
        let angle = s / self.radius;

        self.center + self.radius * (self.x * angle.cos() + self.y * angle.sin())
    }

    fn get_tangent(&self, s: f64) -> Coord {
        let angle = s / self.radius;

        -self.x * angle.sin() + self.y * angle.cos()
    }

    fn get_curvature(&self, s: f64) -> Coord {
        let angle = s / self.radius;

        -1.0 / self.radius * (self.x * angle.cos() + self.y * angle.sin())
    }

    fn get_switching_points(&self) -> Vec<f64> {
        let mut switching_points = Vec::new();

        let dim = self.x.len();

        for i in 0..dim {
            let mut switching_angle = self.y[i].atan2(self.x[i]);

            if switching_angle < 0.0 {
                switching_angle += f64::consts::PI;
            }

            let switching_point = switching_angle + self.radius;

            if switching_point < self.length {
                switching_points.push(switching_point);
            }
        }

        switching_points.sort_by(|a, b| a.partial_cmp(b).unwrap());

        switching_points
    }
}
