use super::pathsegment::PathSegment;
use super::Coord;

pub type SwitchingPoint = (f64, bool);

#[derive(Clone, Debug)]
pub struct Path<'a, P: 'a>
where
    P: PathSegment,
{
    length: f64,
    switching_points: Vec<SwitchingPoint>,
    path_segments: &'a Vec<P>,
}

impl<'a, P> Path<'a, P>
where
    P: PathSegment,
{
    pub fn from_waypoints(path: &'a Vec<P>, max_deviation: Option<f64>) -> Self {
        while let Some(&[prev, curr, next]) = path.windows(3).next() {}

        unimplemented!()
    }

    fn get_path_segment(&self, s: f64) -> P {
        for p in self.path_segments {
            if s >= p.get_position() {
                return p.clone();
            }
        }

        self.path_segments
            .iter()
            .next()
            .expect("No path segments")
            .clone()
    }

    pub fn get_length(&self) -> f64 {
        self.length
    }

    pub fn get_config(&self, s: f64) -> Coord {
        self.get_path_segment(s).get_config(s)
    }

    pub fn get_tangent(&self, s: f64) -> Coord {
        self.get_path_segment(s).get_tangent(s)
    }

    pub fn get_curvature(&self, s: f64) -> Coord {
        self.get_path_segment(s).get_curvature(s)
    }

    pub fn get_next_switching_point(&self, s: f64) -> SwitchingPoint {
        let mut it = self.switching_points.iter();

        let mut current = None;

        while let Some(item) = it.next() {
            if item.0 > s {
                break;
            }

            current = Some(item);
        }

        if let Some(point) = current {
            *point
        } else {
            (self.length, true)
        }
    }

    pub fn get_switching_points(&self) -> &Vec<SwitchingPoint> {
        &self.switching_points
    }
}
