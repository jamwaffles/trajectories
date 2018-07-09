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
    position: f64,
    length: f64,
    radius: f64,
    center: Coord,
    x: Coord,
    y: Coord,
}

impl CircularPathSegment {
    fn new(start: Coord, intersection: Coord, end: Coord, max_deviation: f64) -> Self {
        // if((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001) {
        //     length = 0.0;
        //     radius = 1.0;
        //     center = intersection;
        //     x = Eigen::VectorXd::Zero(start.size());
        //     y = Eigen::VectorXd::Zero(start.size());
        //     return;
        // }

        if (intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001 {
            return Self {
                length: 0.0,
                radius: 1.0,
                center: intersection,
                x: Coord::repeat(0.0),
                y: Coord::repeat(0.0),
            };
        }

        // const Eigen::VectorXd startDirection = (intersection - start).normalized();
        // const Eigen::VectorXd endDirection = (end - intersection).normalized();

        let start_direction = (intersection - start).normalized();
        let end_direction = (end - intersection).normalized();

        // if((startDirection - endDirection).norm() < 0.000001) {
        //     length = 0.0;
        //     radius = 1.0;
        //     center = intersection;
        //     x = Eigen::VectorXd::Zero(start.size());
        //     y = Eigen::VectorXd::Zero(start.size());
        //     return;
        // }

        // const double startDistance = (start - intersection).norm();
        // const double endDistance = (end - intersection).norm();

        // double distance = std::min((start - intersection).norm(), (end - intersection).norm());
        // const double angle = acos(startDirection.dot(endDirection));

        // distance = std::min(distance, maxDeviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));  // enforce max deviation

        // radius = distance / tan(0.5 * angle);
        // length = angle * radius;

        // center = intersection + (endDirection - startDirection).normalized() * radius / cos(0.5 * angle);
        // x = (intersection - distance * startDirection - center).normalized();
        // y = startDirection;

        // CircularPathSegment {
        //     start,
        //     end,
        //     length: (end - start).norm(),
        // }

        Self {
            length: 0.0,
            radius: 1.0,
            center: intersection,
            x: Coord::repeat(0.0),
            y: Coord::repeat(0.0),
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
