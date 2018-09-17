mod circular_segment;
mod linear_segment;
mod segment;

pub use self::circular_segment::CircularPathSegment;
pub use self::linear_segment::LinearPathSegment;
pub use self::segment::PathSegment;
use Coord;

/// Helpful methods to get information about a path
pub trait PathItem {
    /// Get length of path
    fn get_length(&self) -> f64;

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord;

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord;

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord;

    /// Get switching points for this path segment
    fn get_switching_points(&self) -> Vec<f64>;
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

        let length = start_offset + segments.last().unwrap().get_length();

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

    /// Get all switching points along this path
    fn get_switching_points(&self) -> Vec<f64> {
        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_helpers::*;

    #[test]
    fn length_limit_blend_size() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(1.0, 1.0, 0.0),
            Coord::new(1.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 1.5);

        debug_path("length_limit_blend_size", &path, &waypoints);

        // TODO: Better assertion than overall length. This test is "tested" by looking at the
        // rendered output. This should be fixed.
        assert_near!(path.get_length(), 2.5707963267948974);
    }

    #[test]
    fn correct_switching_points() {
        // Data from Example.cpp in C++ example code
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 0.2, 1.0),
            Coord::new(0.0, 3.0, 0.5),
            Coord::new(1.1, 2.0, 0.0),
            Coord::new(1.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(0.0, 0.0, 1.0),
        ];

        // Switching generated from waypoints from Example.cpp. Empty Vecs are linear segments
        let expected_switching_points = vec![
            vec![],
            vec![0.0, 0.00343513],
            vec![],
            vec![0.00127369],
            vec![],
            vec![0.00741455],
            vec![],
            vec![0.0, 0.00103794],
            vec![],
            vec![0.0010472, 0.0020944],
            vec![],
        ];

        // Match Example.cpp accuracy
        let accuracy = 0.001;

        let path = Path::from_waypoints(&waypoints, accuracy);

        debug_path("correct_switching_points", &path, &waypoints);

        for (segment, expected_points) in path.segments.iter().zip(expected_switching_points.iter())
        {
            match segment {
                PathSegment::Circular(s) => {
                    let switching_points = s.get_switching_points();

                    for (point, expected) in switching_points.iter().zip(expected_points.iter()) {
                        assert_near!(*point, *expected + s.start_offset);
                    }
                }
                PathSegment::Linear(s) => assert_eq!(s.get_switching_points(), Vec::new()),
            }
        }
    }

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

        debug_path("path_with_blends", &path, &waypoints);

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

        debug_path_point("get_pos_in_first_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos, Coord::new(0.0, 0.5, 0.0));
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

        debug_path_point("get_pos_in_last_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos, Coord::new(1.5, 1.5, 0.0));
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

        debug_path_point("get_pos_in_last_segment_other", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.4688780239495878);
        assert_near!(pos, Coord::new(2.310263340389897, 0.5632455532033677, 0.0));
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

        debug_path_point("get_pos_in_last_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos, Coord::new(2.0, 2.0, 0.0));
    }
}
