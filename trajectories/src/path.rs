use circular_path_segment::CircularPathSegment;
use linear_path_segment::LinearPathSegment;
use Coord;
use PathItem;

#[derive(Debug, Clone)]
pub enum PathSegment {
    Linear(LinearPathSegment),
    Circular(CircularPathSegment),
}

impl PathItem for PathSegment {
    /// Get length of path
    fn get_length(&self) -> f64 {
        match self {
            PathSegment::Linear(s) => s.get_length(),
            PathSegment::Circular(s) => s.get_length(),
        }
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord {
        match self {
            PathSegment::Linear(s) => s.get_position(distance_along_line),
            PathSegment::Circular(s) => s.get_position(distance_along_line),
        }
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord {
        match self {
            PathSegment::Linear(s) => s.get_tangent(distance_along_line),
            PathSegment::Circular(s) => s.get_tangent(distance_along_line),
        }
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord {
        match self {
            PathSegment::Linear(s) => s.get_curvature(distance_along_line),
            PathSegment::Circular(s) => s.get_curvature(distance_along_line),
        }
    }
}

impl PathSegment {
    /// Clone segment and give it a start offset
    // TODO: Trait
    fn with_start_offset(&self, offset: f64) -> Self {
        match self {
            PathSegment::Linear(s) => PathSegment::Linear(s.with_start_offset(offset)),
            PathSegment::Circular(s) => PathSegment::Circular(s.with_start_offset(offset)),
        }
    }

    /// Get start offset of this segment
    // TODO: Trait
    fn get_start_offset(&self) -> f64 {
        match self {
            PathSegment::Linear(s) => s.start_offset,
            PathSegment::Circular(s) => s.start_offset,
        }
    }
}

/// A path with circular blends between segments
#[derive(Debug)]
pub struct Path {
    /// Linear path segments and circular blends
    pub segments: Vec<PathSegment>,

    /// Total path length
    length: f64,
}

impl Path {
    /// Create a blended path from a set of waypoints
    ///
    /// The path must be differentiable, so small blends are added between linear segments
    pub fn from_waypoints(waypoints: &Vec<Coord>, max_deviation: f64) -> Self {
        // let mut segments: Vec<PathSegment> = Vec::with_capacity(waypoints.len());
        let mut length = 0.0;
        let mut start_offset = 0.0;

        let segments =
            waypoints
                .windows(3)
                .fold(Vec::new(), |mut segments: Vec<PathSegment>, parts| {
                    if let &[prev, curr, next] = parts {
                        let blend_segment =
                            CircularPathSegment::from_waypoints(&prev, &curr, &next, max_deviation);

                        let blend_start = blend_segment.get_position(0.0);
                        let blend_end = blend_segment.get_position(blend_segment.get_length());

                        // Update previous segment with new end point, or create a new one if we're
                        // at the beginning of the path
                        let prev_segment = segments
                            .pop()
                            .map(|segment| match segment {
                                PathSegment::Linear(s) => {
                                    LinearPathSegment::from_waypoints(s.start, blend_start)
                                        .with_start_offset(s.start_offset)
                                }
                                _ => panic!("Invalid path: expected last segment to be linear"),
                            }).unwrap_or(
                                LinearPathSegment::from_waypoints(prev, blend_start)
                                    .with_start_offset(start_offset),
                            );

                        segments.push(PathSegment::Linear(prev_segment));

                        start_offset += prev_segment.get_length();

                        segments.push(PathSegment::Circular(
                            blend_segment.with_start_offset(start_offset),
                        ));

                        start_offset += blend_segment.get_length();

                        let next_segment = LinearPathSegment::from_waypoints(blend_end, next)
                            .with_start_offset(start_offset);

                        segments.push(PathSegment::Linear(next_segment));

                        segments
                    } else {
                        panic!("Linear segments");
                    }
                });

        length = start_offset + segments.last().unwrap().get_length();

        Self { segments, length }
    }

    /// Get a path segment for a position along the entire path
    pub fn get_segment_at_position(&self, position_along_path: f64) -> Option<&PathSegment> {
        self.segments
            .iter()
            .position(|segment| segment.get_start_offset() > position_along_path)
            .and_then(|pos| self.segments.get(pos - 1))
            .or(self.segments.last())
    }
}

impl PathItem for Path {
    /// Get the length of the complete path
    fn get_length(&self) -> f64 {
        self.length
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord {
        self.get_segment_at_position(distance_along_line)
            .map(|segment| segment.get_position(distance_along_line - segment.get_start_offset()))
            .expect(&format!(
                "Could not get position for path offset {}, total length {}",
                distance_along_line,
                self.get_length()
            ))
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord {
        self.get_segment_at_position(distance_along_line)
            .map(|segment| segment.get_tangent(distance_along_line - segment.get_start_offset()))
            .expect(&format!(
                "Could not get derivative for path offset {}, total length {}",
                distance_along_line,
                self.get_length()
            ))
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord {
        self.get_segment_at_position(distance_along_line)
            .map(|segment| segment.get_curvature(distance_along_line - segment.get_start_offset()))
            .expect(&format!(
                "Could not get second derivative for path offset {}, total length {}",
                distance_along_line,
                self.get_length()
            ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_helpers::*;

    #[test]
    fn it_creates_path_with_blends() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(1.0, 2.0, 0.0),
            Coord::new(1.5, 1.5, 0.0),
            Coord::new(3.0, 5.0, 0.0),
            Coord::new(4.0, 6.0, 0.0),
            Coord::new(5.0, 5.0, 0.0),
            Coord::new(4.0, 4.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);

        debug_path("../target/path_with_blends", &path, &waypoints);

        assert!(true);
    }

    #[test]
    fn get_pos_in_first_segment() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(1.0, 1.0, 0.0),
            Coord::new(2.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(0.5);

        debug_path_point(
            "../target/get_pos_in_first_segment",
            &path,
            &waypoints,
            &pos,
        );

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos.x, 0.0);
        assert_near!(pos.y, 0.5);
    }

    #[test]
    fn get_pos_in_last_segment() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(1.0, 1.0, 0.0),
            Coord::new(2.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(path.get_length() - 0.70710678118);

        debug_path_point("../target/get_pos_in_last_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos.x, 1.5);
        assert_near!(pos.y, 1.5);
    }

    #[test]
    fn get_pos_in_last_segment_other() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(1.0, 1.0, 0.0),
            Coord::new(2.5, 0.5, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(path.get_length() - 0.2);

        debug_path_point(
            "../target/get_pos_in_last_segment_other",
            &path,
            &waypoints,
            &pos,
        );

        assert_near!(path.get_length(), 3.4688780239495878);
        assert_near!(pos.x, 2.310263340389897);
        assert_near!(pos.y, 0.5632455532033677);
    }

    #[test]
    fn get_final_position() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(1.0, 1.0, 0.0),
            Coord::new(2.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(path.get_length());

        debug_path_point("../target/get_pos_in_last_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos.x, 2.0);
        assert_near!(pos.y, 2.0);
    }
}
