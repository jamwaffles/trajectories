use std::cmp;

use super::Coord;

pub trait PathSegment {
    fn get_length(&self) -> f64;

    fn get_config(&self, s: f64) -> Coord;

    fn get_tangent(&self, s: f64) -> Coord;

    fn get_curvature(&self, s: f64) -> Coord;

    fn get_switching_points(&self, s: f64) -> Coord;
}

#[derive(Copy, Clone, Debug)]
pub struct LinearPathSegment {
    position: f64,
    length: f64,
    start: Coord,
    end: Coord,
}

impl PathSegment for LinearPathSegment {
    fn get_length(&self) -> f64 {
        self.length
    }

    fn get_config(&self, s: f64) -> Coord {
        let div = 0.0f64.max(1.0f64.min(s / self.length));

        return (1.0 - div) * self.start + div * self.end;
    }

    fn get_tangent(&self, s: f64) -> Coord {
        unimplemented!()
    }

    fn get_curvature(&self, s: f64) -> Coord {
        unimplemented!()
    }

    fn get_switching_points(&self, s: f64) -> Coord {
        unimplemented!()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct CircularPathSegment {
    position: f64,
    length: f64,
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

    fn get_switching_points(&self, s: f64) -> Coord {
        unimplemented!()
    }
}
