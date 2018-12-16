mod circular_segment;
mod linear_segment;
mod segment;

pub use self::circular_segment::CircularPathSegment;
pub use self::linear_segment::LinearPathSegment;
pub use self::segment::PathSegment;
use crate::Coord;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;

/// Helpful methods to get information about a path
pub trait PathItem<N>: PartialEq
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Get length of path
    fn get_length(&self) -> f64;

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord<N>;

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord<N>;

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord<N>;
}

/// A switching point
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchingPoint {
    /// Position along the path at which this switching point occurs
    pub position: f64,
    /// Whether this switching point is discontinuous or not
    pub continuity: Continuity,
}

impl SwitchingPoint {
    /// Create a new switching point from position and continuity flag
    #[inline(always)]
    pub fn new(position: f64, continuity: Continuity) -> Self {
        Self {
            position,
            continuity,
        }
    }
}

/// Continuity flag
#[derive(Debug, Clone, PartialEq)]
pub enum Continuity {
    /// The path at a point is discontinuous
    Discontinuous,

    /// The path at a point is continuous
    Continuous,
}

/// A path with circular blends between segments
#[derive(Debug, PartialEq)]
pub struct Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Linear path segments and circular blends
    pub segments: Vec<PathSegment<N>>,

    /// Total path length
    length: f64,

    /// Switching points. Bool denotes whether point is discontinuous (`true`) or not (`false`)
    switching_points: Vec<SwitchingPoint>,
}

impl<N> Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Create a blended path from a set of waypoints
    ///
    /// The path must be differentiable, so small blends are added between linear segments
    pub fn from_waypoints(waypoints: &[Coord<N>], max_deviation: f64) -> Self {
        let mut start_offset = 0.0;
        let mut switching_points = Vec::with_capacity((waypoints.len() as f32 * 2.5) as usize);

        let segments = match waypoints.len() {
            0 | 1 => panic!("Path must contain at least two waypoints"),
            2 => vec![PathSegment::Linear(LinearPathSegment::from_waypoints(
                waypoints[0].clone(),
                waypoints[1].clone(),
            ))],
            _ => waypoints.windows(3).fold(
                Vec::with_capacity(waypoints.len() * 3),
                |mut segments, parts| {
                    if let [prev, curr, next] = parts {
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
                                    LinearPathSegment::from_waypoints(s.start, blend_start.clone())
                                        .with_start_offset(s.start_offset)
                                }
                                _ => panic!("Invalid path: expected last segment to be linear"),
                            })
                            .unwrap_or_else(|| {
                                LinearPathSegment::from_waypoints(prev.clone(), blend_start)
                                    .with_start_offset(start_offset)
                            });

                        start_offset += prev_segment.get_length();

                        // Switching point where linear segment touches blend (discontinuous)
                        // TODO: Get actual list of switching points when support for non-linear
                        // path segments (that aren't blends) is added.
                        switching_points
                            .push(SwitchingPoint::new(start_offset, Continuity::Discontinuous));

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
                                        Some(SwitchingPoint::new(p_offset, Continuity::Continuous))
                                    } else {
                                        None
                                    }
                                })
                                .collect(),
                        );

                        // Add blend segment length to path length total
                        start_offset = blend_end_offset;

                        let next_segment =
                            LinearPathSegment::from_waypoints(blend_end, next.clone())
                                .with_start_offset(start_offset);

                        // Switching point where linear segment touches blend
                        // TODO: Get actual list of switching points when support for non-linear
                        // path segments (that aren't blends) is added.
                        switching_points
                            .push(SwitchingPoint::new(start_offset, Continuity::Discontinuous));

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
                },
            ),
        };

        let length = start_offset
            + segments
                .last()
                .expect("Cannot get length of empty path")
                .get_length();

        Self {
            switching_points,
            segments,
            length,
        }
    }

    /// Get a path segment for a position along the entire path
    ///
    /// It will return the last segment in the path if a position greater than the total path length
    /// is given.
    pub fn get_segment_at_position(&self, position_along_path: f64) -> &PathSegment<N> {
        self.segments
            .iter()
            .find(|segment| segment.get_end_offset() > position_along_path)
            .unwrap_or_else(|| &self.segments.last().unwrap())
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
            .find(|sp| sp.position > position_along_path)
            .unwrap_or_else(|| SwitchingPoint::new(self.length, Continuity::Discontinuous))
    }
}

impl<N> PathItem<N> for Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Get the length of the complete path
    #[inline(always)]
    fn get_length(&self) -> f64 {
        self.length
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord<N> {
        self.get_segment_at_position(distance_along_line)
            .get_position(distance_along_line)
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord<N> {
        self.get_segment_at_position(distance_along_line)
            .get_tangent(distance_along_line)
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord<N> {
        self.get_segment_at_position(distance_along_line)
            .get_curvature(distance_along_line)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn path_with_two_waypoints() {
        let waypoints = vec![
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(2.0, 0.0, 0.0),
        ];

        let _path = Path::from_waypoints(&waypoints, 0.01);
    }

    #[test]
    #[should_panic]
    fn path_too_short() {
        let waypoints = vec![TestCoord3::new(1.0, 0.0, 0.0)];

        let _p1 = Path::from_waypoints(&waypoints, 0.01);
        let _p2 = Path::from_waypoints(&Vec::<TestCoord3>::new(), 0.01);
    }

    #[test]
    fn get_segment_at_position() {
        let waypoints = vec![
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(2.0, 0.0, 0.0),
            TestCoord3::new(5.0, 0.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.01);

        assert_eq!(
            path.get_segment_at_position(0.0),
            &PathSegment::Linear(LinearPathSegment::from_waypoints(
                TestCoord3::new(1.0, 0.0, 0.0),
                TestCoord3::new(2.0, 0.0, 0.0)
            ))
        );

        assert_eq!(
            path.get_segment_at_position(3.0),
            &PathSegment::Linear(
                LinearPathSegment::from_waypoints(
                    TestCoord3::new(2.0, 0.0, 0.0),
                    TestCoord3::new(5.0, 0.0, 0.0)
                )
                .with_start_offset(1.0)
            )
        );

        assert_eq!(
            path.get_segment_at_position(1.0),
            &PathSegment::Linear(LinearPathSegment {
                start: TestCoord3::new(2.0, 0.0, 0.0),
                end: TestCoord3::new(5.0, 0.0, 0.0),
                start_offset: 1.0,
                end_offset: 4.0,
                length: 3.0,
            })
        );
        assert_eq!(
            path.get_segment_at_position(1.01),
            &PathSegment::Linear(LinearPathSegment {
                start: TestCoord3::new(2.0, 0.0, 0.0),
                end: TestCoord3::new(5.0, 0.0, 0.0),
                start_offset: 1.0,
                end_offset: 4.0,
                length: 3.0,
            })
        );

        assert_eq!(
            path.get_segment_at_position(5.0),
            &PathSegment::Linear(LinearPathSegment {
                start: TestCoord3::new(2.0, 0.0, 0.0),
                end: TestCoord3::new(5.0, 0.0, 0.0),
                start_offset: 1.0,
                end_offset: 4.0,
                length: 3.0,
            })
        );
    }

    #[test]
    fn get_derivatives() {
        // Data from Example.cpp in C++ example code
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];

        // Match Example.cpp accuracy
        let accuracy = 0.001;

        // Tuple of (position, expected derivative, expected second derivative) taken from Example.cpp
        let expected = vec![
            (
                0.0,
                TestCoord3::new(0.0, 0.1961161351381840, 0.9805806756909202),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                0.5099017027977882,
                TestCoord3::new(0.0, 0.1961161351381840, 0.9805806756909202),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                1.0201861237223337,
                TestCoord3::new(0.0, 0.9710853103707811, 0.2387327375583182),
                TestCoord3::new(-0.0, 95.4477291274329076, -388.2495907845863030),
            ),
            (
                1.5569160753598927,
                TestCoord3::new(-0.0, 0.9844275755084820, -0.1757906384836575),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                2.5727348363119251,
                TestCoord3::new(-0.0, 0.9844275755084820, -0.1757906384836575),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                3.5614767161100072,
                TestCoord3::new(-0.0, 0.9844275755084820, -0.1757906384836575),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                3.8984742788054678,
                TestCoord3::new(0.7013343843696721, -0.6375767130633382, -0.3187883565316692),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                4.8831177467169011,
                TestCoord3::new(0.7013343843696721, -0.6375767130633382, -0.3187883565316692),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                5.4523088538682449,
                TestCoord3::new(-0.0499376169438922, -0.9987523388778448, -0.0),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                6.1453761368528133,
                TestCoord3::new(-0.0499376169438922, -0.9987523388778448, -0.0),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                7.1232278747423878,
                TestCoord3::new(-0.0499376169438922, -0.9987523388778448, -0.0),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                7.4612006384156970,
                TestCoord3::new(-0.7071067811865476, 0.7071067811865475, 0.0),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                8.3975630709442228,
                TestCoord3::new(-0.7071067811865476, 0.7071067811865475, 0.0),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                8.8718474912408851,
                TestCoord3::new(-0.0, -0.7071067811865476, 0.7071067811865476),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            (
                9.8017700948612454,
                TestCoord3::new(-0.0, -0.7071067811865476, 0.7071067811865476),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
        ];

        let path = Path::from_waypoints(&waypoints, accuracy);

        expected
            .into_iter()
            .for_each(|(pos, expected_deriv, expected_second_deriv)| {
                let deriv = path.get_tangent(pos);
                let second_deriv = path.get_curvature(pos);

                assert_near!(deriv, expected_deriv);
                assert_near!(second_deriv, expected_second_deriv);
            });
    }

    #[test]
    fn get_next_switching_point() {
        // Data from Example.cpp in C++ example code
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];

        // Match Example.cpp accuracy
        let accuracy = 0.001;

        let path = Path::from_waypoints(&waypoints, accuracy);

        assert_eq!(
            path.get_next_switching_point(0.0),
            SwitchingPoint::new(1.0173539279271488, Continuity::Discontinuous)
        );
        assert_eq!(
            path.get_next_switching_point(5.425844),
            SwitchingPoint::new(5.43325752688998, Continuity::Continuous)
        );
        assert_eq!(
            path.get_next_switching_point(path.get_length() - 0.01),
            SwitchingPoint::new(path.get_length(), Continuity::Discontinuous),
            "Expected last switching point to be end of path"
        );
    }

    #[test]
    fn length_limit_blend_size() {
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(1.0, 1.0, 0.0),
            TestCoord3::new(1.0, 2.0, 0.0),
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
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];

        // Switching generated from waypoints from Example.cpp
        let expected_switching_points = vec![
            SwitchingPoint::new(1.0173539279271488, Continuity::Discontinuous),
            SwitchingPoint::new(1.0173539279271488, Continuity::Continuous),
            SwitchingPoint::new(1.0207890617732325, Continuity::Continuous),
            SwitchingPoint::new(1.0212310438858092, Continuity::Discontinuous),
            SwitchingPoint::new(3.8614234182834446, Continuity::Discontinuous),
            SwitchingPoint::new(3.8626971078471364, Continuity::Continuous),
            SwitchingPoint::new(3.8633009591232206, Continuity::Discontinuous),
            SwitchingPoint::new(5.425842981796586, Continuity::Discontinuous),
            SwitchingPoint::new(5.43325752688998, Continuity::Continuous),
            SwitchingPoint::new(5.43372148747555, Continuity::Discontinuous),
            SwitchingPoint::new(7.430435574066095, Continuity::Discontinuous),
            SwitchingPoint::new(7.430435574066095, Continuity::Continuous),
            SwitchingPoint::new(7.4314735160725895, Continuity::Continuous),
            SwitchingPoint::new(7.432009534887585, Continuity::Discontinuous),
            SwitchingPoint::new(8.842953203579489, Continuity::Discontinuous),
            SwitchingPoint::new(8.844000401130685, Continuity::Continuous),
            SwitchingPoint::new(8.845047598681882, Continuity::Discontinuous),
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
            assert_near!(point.position, expected.position);
            assert_eq!(point.continuity, expected.continuity);
        }
    }

    #[test]
    fn debug_switching_points() {
        // Data from Example.cpp in C++ example code
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 0.0),
            TestCoord3::new(0.0, 3.0, 0.0),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 0.0),
        ];

        let accuracy = 0.05;

        let path = Path::from_waypoints(&waypoints, accuracy);

        debug_path_switching_points("debug_switching_points", &path, &waypoints);
    }

    #[test]
    fn correct_segment_switching_points() {
        // Data from Example.cpp in C++ example code
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
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
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(1.0, 2.0, 0.0),
            TestCoord3::new(1.5, 1.5, 0.0),
            TestCoord3::new(3.0, 5.0, 0.0),
            TestCoord3::new(4.0, 6.0, 0.0),
            TestCoord3::new(5.0, 5.0, 0.0),
            TestCoord3::new(4.0, 4.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);

        debug_path("path_with_blends", &path, &waypoints);

        assert!(true);
    }

    #[test]
    fn get_pos_in_first_segment() {
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(1.0, 1.0, 0.0),
            TestCoord3::new(2.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(0.5);

        debug_path_point("get_pos_in_first_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos, TestCoord3::new(0.0, 0.5, 0.0));
    }

    #[test]
    fn get_pos_in_last_segment() {
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(1.0, 1.0, 0.0),
            TestCoord3::new(2.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(path.get_length() - 0.70710678118);

        debug_path_point("get_pos_in_last_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos, TestCoord3::new(1.5, 1.5, 0.0));
    }

    #[test]
    fn get_pos_in_last_segment_other() {
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(1.0, 1.0, 0.0),
            TestCoord3::new(2.5, 0.5, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(path.get_length() - 0.2);

        debug_path_point("get_pos_in_last_segment_other", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.4688780239495878);
        assert_near!(
            pos,
            TestCoord3::new(2.310263340389897, 0.5632455532033677, 0.0)
        );
    }

    #[test]
    fn get_final_position() {
        let waypoints = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(1.0, 1.0, 0.0),
            TestCoord3::new(2.0, 2.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);
        let pos = path.get_position(path.get_length());

        debug_path_point("get_pos_in_last_segment", &path, &waypoints, &pos);

        assert_near!(path.get_length(), 3.2586540784544042);
        assert_near!(pos, TestCoord3::new(2.0, 2.0, 0.0));
    }
}
