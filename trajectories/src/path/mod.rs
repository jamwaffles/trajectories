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
}

/// A switching point
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchingPoint(
    /// Position along the path at which this switching point occurs
    f64,
    /// Whether this switching point is discontinuous or not
    Continuity,
);

/// Continuity flag
#[derive(Debug, Clone, PartialEq)]
pub enum Continuity {
    /// The path at a point is discontinuous
    Discontinuous,

    /// The path at a point is continuous
    Continuous,
}

/// A path with circular blends between segments
#[derive(Debug)]
pub struct Path {
    /// Linear path segments and circular blends
    pub segments: Vec<PathSegment>,

    /// Total path length
    length: f64,

    /// Switching points. Bool denotes whether point is discontinuous (`true`) or not (`false`)
    switching_points: Vec<SwitchingPoint>,
}

impl Path {
    /// Create a blended path from a set of waypoints
    ///
    /// The path must be differentiable, so small blends are added between linear segments
    pub fn from_waypoints(waypoints: &Vec<Coord>, max_deviation: f64) -> Self {
        let mut start_offset = 0.0;
        let mut switching_points = Vec::new();

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

                        start_offset += prev_segment.get_length();

                        // Switching point where linear segment touches blend (discontinuous)
                        // TODO: Get actual list of switching points when support for non-linear
                        // path segments (that aren't blends) is added.
                        switching_points
                            .push(SwitchingPoint(start_offset, Continuity::Discontinuous));

                        let blend_segment = blend_segment.with_start_offset(start_offset);
                        let blend_switching_points = blend_segment.get_switching_points();
                        let blend_end_offset =
                            blend_segment.start_offset + blend_segment.get_length();

                        // Get switching points over the duration of the blend segment
                        switching_points.append(
                            &mut blend_switching_points
                                .iter()
                                .filter_map(|p| {
                                    let p_offset = p + blend_segment.start_offset;

                                    if p_offset < blend_end_offset {
                                        Some(SwitchingPoint(p_offset, Continuity::Continuous))
                                    } else {
                                        None
                                    }
                                }).collect(),
                        );

                        // Add blend segment length to path length total
                        start_offset = blend_end_offset;

                        let next_segment = LinearPathSegment::from_waypoints(blend_end, next)
                            .with_start_offset(start_offset);

                        // Switching point where linear segment touches blend
                        // TODO: Get actual list of switching points when support for non-linear
                        // path segments (that aren't blends) is added.
                        switching_points
                            .push(SwitchingPoint(start_offset, Continuity::Discontinuous));

                        // Add both linear segments with blend in between to overall path
                        segments.append(&mut vec![
                            PathSegment::Linear(prev_segment),
                            PathSegment::Circular(blend_segment),
                            PathSegment::Linear(next_segment),
                        ]);

                        segments
                    } else {
                        panic!("Linear segments");
                    }
                });

        let length = start_offset + segments.last().unwrap().get_length();

        Self {
            switching_points,
            segments,
            length,
        }
    }

    /// Get a path segment for a position along the entire path
    pub fn get_segment_at_position(&self, position_along_path: f64) -> Option<&PathSegment> {
        self.segments
            .iter()
            .position(|segment| segment.get_start_offset() > position_along_path)
            .and_then(|pos| self.segments.get(pos - 1))
            .or(self.segments.last())
    }

    /// Get all switching points along this path
    pub fn get_switching_points(&self) -> &Vec<SwitchingPoint> {
        &self.switching_points
    }

    /// Get position of next switching point after a position along the path
    ///
    /// Returns the end of the path as position if no switching point could be found
    pub fn get_next_switching_point(&self, position_along_path: f64) -> SwitchingPoint {
        self.switching_points
            .iter()
            .cloned()
            .find(|sp| sp.0 > position_along_path)
            .unwrap_or(SwitchingPoint(self.length, Continuity::Discontinuous))
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
    fn get_next_switching_point() {
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

        // Match Example.cpp accuracy
        let accuracy = 0.001;

        let path = Path::from_waypoints(&waypoints, accuracy);

        assert_eq!(
            path.get_next_switching_point(0.0),
            SwitchingPoint(1.0173539279271488, Continuity::Discontinuous)
        );
        assert_eq!(
            path.get_next_switching_point(5.425844),
            SwitchingPoint(5.43325752688998, Continuity::Continuous)
        );
        assert_eq!(
            path.get_next_switching_point(path.get_length() - 0.01),
            SwitchingPoint(path.get_length(), Continuity::Discontinuous),
            "Expected last switching point to be end of path"
        );
    }

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
    fn correct_path_switching_points() {
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

        // Switching generated from waypoints from Example.cpp
        let expected_switching_points = vec![
            (1.0173539279271488, Continuity::Discontinuous),
            (1.0173539279271488, Continuity::Continuous),
            (1.02079, Continuity::Continuous),
            (1.0212310438858092, Continuity::Discontinuous),
            (3.8614234182834446, Continuity::Discontinuous),
            (3.8626971078471364, Continuity::Continuous),
            (3.8633, Continuity::Discontinuous),
            (5.425842981796586, Continuity::Discontinuous),
            (5.43325752688998, Continuity::Continuous),
            (5.43372148747555, Continuity::Discontinuous),
            (7.430435574066095, Continuity::Discontinuous),
            (7.430435574066095, Continuity::Continuous),
            (7.4314735160725895, Continuity::Continuous),
            (7.43201, Continuity::Discontinuous),
            (8.842953203579489, Continuity::Discontinuous),
            (8.844, Continuity::Continuous),
            (8.845047598681882, Continuity::Discontinuous),
        ];

        // Match Example.cpp accuracy
        let accuracy = 0.001;

        let path = Path::from_waypoints(&waypoints, accuracy);

        debug_path("correct_path_switching_points", &path, &waypoints);

        for (i, (point, expected)) in path
            .get_switching_points()
            .iter()
            .zip(expected_switching_points.iter())
            .enumerate()
        {
            println!("Test point {}:", i);
            assert_near!(point.0, expected.0);
            assert_eq!(point.1, expected.1);
        }
    }

    #[test]
    fn correct_segment_switching_points() {
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

        debug_path("correct_segment_switching_points", &path, &waypoints);

        for (segment, expected_points) in path.segments.iter().zip(expected_switching_points.iter())
        {
            match segment {
                PathSegment::Circular(s) => {
                    let switching_points = s.get_switching_points();

                    for (point, expected) in switching_points.iter().zip(expected_points.iter()) {
                        assert_near!(*point, *expected);
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
