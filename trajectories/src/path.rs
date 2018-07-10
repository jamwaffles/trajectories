use super::pathsegment::{CircularPathSegment, LinearPathSegment, PathSegment};
use super::Coord;

pub type SwitchingPoint = (f64, bool);

#[derive(Debug, Copy, Clone)]
pub enum PathItem {
    CircularPathSegment(CircularPathSegment),
    LinearPathSegment(LinearPathSegment),
}

impl PathSegment for PathItem {
    fn get_position(&self) -> f64 {
        match self {
            PathItem::CircularPathSegment(segment) => segment.get_position(),
            PathItem::LinearPathSegment(segment) => segment.get_position(),
        }
    }

    fn get_length(&self) -> f64 {
        match self {
            PathItem::CircularPathSegment(segment) => segment.get_length(),
            PathItem::LinearPathSegment(segment) => segment.get_length(),
        }
    }

    fn get_config(&self, s: f64) -> Coord {
        match self {
            PathItem::CircularPathSegment(segment) => segment.get_config(s),
            PathItem::LinearPathSegment(segment) => segment.get_config(s),
        }
    }

    fn get_tangent(&self, s: f64) -> Coord {
        match self {
            PathItem::CircularPathSegment(segment) => segment.get_tangent(s),
            PathItem::LinearPathSegment(segment) => segment.get_tangent(s),
        }
    }

    fn get_curvature(&self, s: f64) -> Coord {
        match self {
            PathItem::CircularPathSegment(segment) => segment.get_curvature(s),
            PathItem::LinearPathSegment(segment) => segment.get_curvature(s),
        }
    }

    fn get_switching_points(&self, s: f64) -> Vec<f64> {
        match self {
            PathItem::CircularPathSegment(segment) => segment.get_switching_points(s),
            PathItem::LinearPathSegment(segment) => segment.get_switching_points(s),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Path {
    length: f64,
    switching_points: Vec<SwitchingPoint>,
    path_segments: Vec<PathItem>,
}

impl Path {
    pub fn from_waypoints(path: &Vec<Coord>, max_deviation: f64) -> Self {
        let mut start_config = path.first().expect("Could not get first path item").clone();

        let mut windows = path.windows(3);

        let mut path_segments = Vec::new();

        // TODO: Cater for paths with length 2
        while let Some(&[config1, config2, config3]) = windows.next() {
            if max_deviation > 0.0 {
                let blend_segment = CircularPathSegment::new(
                    0.5 * (config1 + config2),
                    config2,
                    0.5 * (config2 + config3),
                    max_deviation,
                );
                let end_config = blend_segment.get_config(0.0);

                if (end_config - start_config).norm() > 0.000001 {
                    path_segments.push(PathItem::LinearPathSegment(LinearPathSegment::new(
                        start_config,
                        end_config,
                    )));
                }
                path_segments.push(PathItem::CircularPathSegment(blend_segment));

                start_config = blend_segment.get_config(blend_segment.get_length());
            } else {
                path_segments.push(PathItem::LinearPathSegment(LinearPathSegment::new(
                    start_config,
                    config2,
                )));
                start_config = config2.clone();
            }
        }

        // create list of switching point candidates, calculate total path length and absolute positions of path segments
        // for(list<PathSegment*>::iterator segment = pathSegments.begin(); segment != pathSegments.end(); segment++) {
        //     (*segment)->position = length;
        //     list<double> localSwitchingPoints = (*segment)->getSwitchingPoints();
        //     for(list<double>::const_iterator point = localSwitchingPoints.begin(); point != localSwitchingPoints.end(); point++) {
        //         switchingPoints.push_back(make_pair(length + *point, false));
        //     }
        //     length += (*segment)->getLength();
        //     while(!switchingPoints.empty() && switchingPoints.back().first >= length)
        //         switchingPoints.pop_back();
        //     switchingPoints.push_back(make_pair(length, true));
        // }
        // switchingPoints.pop_back();

        Self {
            length: 0.0,
            path_segments,
            switching_points: Vec::new(),
        }
    }

    fn get_path_segment(&self, s: f64) -> PathItem {
        for p in self.path_segments.iter() {
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
