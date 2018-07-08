use super::pathsegment::PathSegment;
use super::Coord;

#[derive(Clone, Debug)]
pub struct Path<'a, P: 'a>
where
    P: PathSegment,
{
    length: f64,
    switching_points: Vec<(f64, bool)>,
    path_segments: &'a Vec<P>,
}

impl<'a, P> Path<'a, P>
where
    P: PathSegment,
{
    pub fn from_segments(path_segments: &'a Vec<P>) -> Self {
        Self {
            length: 0.0,
            switching_points: Vec::new(),
            path_segments,
        }
    }

    pub fn get_length(&self) -> f64 {
        self.length
    }

    pub fn get_config(&self, s: f64) -> Coord {
        unimplemented!()
    }

    pub fn get_tangent(&self, s: f64) -> Coord {
        unimplemented!()
    }

    pub fn get_curvature(&self, s: f64) -> Coord {
        unimplemented!()
    }

    pub fn get_next_switching_point(&self, s: f64, discontinuity: bool) -> Coord {
        unimplemented!()
    }

    pub fn get_switching_points(&self) -> Coord {
        unimplemented!()
    }

    pub fn get_path_segment(&self, s: f64) -> Coord {
        unimplemented!()
    }
}
