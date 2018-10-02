mod circular_segment;
mod linear_segment;
mod segment;

pub use self::circular_segment::CircularPathSegment;
pub use self::linear_segment::LinearPathSegment;
pub use self::segment::PathSegment;
use crate::Coord;
use nalgebra::allocator::Allocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;
use nalgebra::Real;
use nalgebra::VectorN;

/// Helpful methods to get information about a path
pub trait PathItem<N, D>: PartialEq
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Get length of path
    fn get_length(&self) -> N;

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: N) -> VectorN<N, D>;

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: N) -> VectorN<N, D>;

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: N) -> VectorN<N, D>;
}

/// A switching point
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchingPoint<N: Real> {
    /// Position along the path at which this switching point occurs
    pub position: N,
    /// Whether this switching point is discontinuous or not
    pub continuity: Continuity,
}

impl<N> SwitchingPoint<N>
where
    N: Real,
{
    /// Create a new switching point from position and continuity flag
    #[inline(always)]
    pub fn new(position: N, continuity: Continuity) -> Self {
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
pub struct Path<N: Real, D: DimName>
where
    DefaultAllocator: Allocator<N, D>,
{
    /// Linear path segments and circular blends
    pub segments: Vec<PathSegment<N, D>>,

    /// Total path length
    length: N,

    /// Switching points. Bool denotes whether point is discontinuous (`true`) or not (`false`)
    switching_points: Vec<SwitchingPoint<N>>,
}

impl<N, D> Path<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Create a blended path from a set of waypoints
    ///
    /// The path must be differentiable, so small blends are added between linear segments
    pub fn from_waypoints(waypoints: &Vec<VectorN<N, D>>, max_deviation: N) -> Self {
        let mut start_offset = nalgebra::convert(0.0);
        let mut switching_points = Vec::with_capacity((waypoints.len() as f32 * 2.5) as usize);

        let segments = waypoints.windows(3).fold(
            Vec::with_capacity(waypoints.len() * 3),
            |mut segments, parts| {
                if let &[prev, curr, next] = parts {
                    let blend_segment =
                        CircularPathSegment::from_waypoints(&prev, &curr, &next, max_deviation);

                    let blend_start = blend_segment.get_position(nalgebra::convert(0.0));
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
                        })
                        .unwrap_or(
                            LinearPathSegment::from_waypoints(prev, blend_start)
                                .with_start_offset(start_offset),
                        );

                    start_offset += prev_segment.get_length();

                    // Switching point where linear segment touches blend (discontinuous)
                    // TODO: Get actual list of switching points when support for non-linear
                    // path segments (that aren't blends) is added.
                    switching_points
                        .push(SwitchingPoint::new(start_offset, Continuity::Discontinuous));

                    let blend_segment = blend_segment.with_start_offset(start_offset);
                    let blend_switching_points = blend_segment.get_switching_points();
                    let blend_end_offset = blend_segment.start_offset + blend_segment.get_length();

                    // Get switching points over the duration of the blend segment
                    switching_points.append(
                        &mut blend_switching_points
                            .iter()
                            .filter_map(|p| {
                                let p_offset = *p + blend_segment.start_offset;

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

                    let next_segment = LinearPathSegment::from_waypoints(blend_end, next)
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
        );

        let length = start_offset + segments.last().unwrap().get_length();

        Self {
            switching_points,
            segments,
            length,
        }
    }

    /// Get a path segment for a position along the entire path
    pub fn get_segment_at_position(&self, position_along_path: N) -> &PathSegment<N, D> {
        let clamped = position_along_path.min(self.length);

        self.segments
            .iter()
            .rev()
            .find(|segment| segment.get_start_offset() <= clamped)
            .unwrap()
    }

    /// Get all switching points along this path
    pub fn get_switching_points(&self) -> &Vec<SwitchingPoint<N>> {
        &self.switching_points
    }

    /// Get position of next switching point after a position along the path
    ///
    /// Returns the end of the path as position if no switching point could be found
    pub fn get_next_switching_point(&self, position_along_path: N) -> SwitchingPoint<N> {
        self.switching_points
            .iter()
            .cloned()
            .skip_while(|sp| sp.position < position_along_path)
            .next()
            .unwrap_or(SwitchingPoint::new(self.length, Continuity::Discontinuous))
    }
}

impl<N, D> PathItem<N, D> for Path<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Get the length of the complete path
    #[inline(always)]
    fn get_length(&self) -> N {
        self.length
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: N) -> VectorN<N, D> {
        self.get_segment_at_position(distance_along_line)
            .get_position(distance_along_line)
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: N) -> VectorN<N, D> {
        self.get_segment_at_position(distance_along_line)
            .get_tangent(distance_along_line)
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: N) -> VectorN<N, D> {
        self.get_segment_at_position(distance_along_line)
            .get_curvature(distance_along_line)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn get_segment_at_position() {
        let waypoints = vec![
            Coord::new(1.0, 0.0, 0.0),
            Coord::new(2.0, 0.0, 0.0),
            Coord::new(5.0, 0.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.01);

        assert_eq!(
            path.get_segment_at_position(0.0),
            &PathSegment::Linear(LinearPathSegment::from_waypoints(
                Coord::new(1.0, 0.0, 0.0),
                Coord::new(2.0, 0.0, 0.0)
            ))
        );

        assert_eq!(
            path.get_segment_at_position(3.0),
            &PathSegment::Linear(
                LinearPathSegment::from_waypoints(
                    Coord::new(2.0, 0.0, 0.0),
                    Coord::new(5.0, 0.0, 0.0)
                )
                .with_start_offset(1.0)
            )
        );

        assert_eq!(
            path.get_segment_at_position(1.0),
            &PathSegment::Linear(LinearPathSegment {
                start: Coord::new(2.0, 0.0, 0.0),
                end: Coord::new(5.0, 0.0, 0.0),
                start_offset: 1.0,
                length: 3.0,
            })
        );
        assert_eq!(
            path.get_segment_at_position(1.01),
            &PathSegment::Linear(LinearPathSegment {
                start: Coord::new(2.0, 0.0, 0.0),
                end: Coord::new(5.0, 0.0, 0.0),
                start_offset: 1.0,
                length: 3.0,
            })
        );

        assert_eq!(
            path.get_segment_at_position(5.0),
            &PathSegment::Linear(LinearPathSegment {
                start: Coord::new(2.0, 0.0, 0.0),
                end: Coord::new(5.0, 0.0, 0.0),
                start_offset: 1.0,
                length: 3.0,
            })
        );
    }

    #[test]
    fn get_derivatives() {
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

        // Tuple of (position, expected derivative, expected second derivative) taken from Example.cpp
        let expected = vec![
            (
                0.0,
                Coord::new(0.0, 0.1961161351381840, 0.9805806756909202),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                0.5099017027977882,
                Coord::new(0.0, 0.1961161351381840, 0.9805806756909202),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                1.0201861237223337,
                Coord::new(0.0, 0.9710853103707811, 0.2387327375583182),
                Coord::new(-0.0, 95.4477291274329076, -388.2495907845863030),
            ),
            (
                1.5569160753598927,
                Coord::new(-0.0, 0.9844275755084820, -0.1757906384836575),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                2.5727348363119251,
                Coord::new(-0.0, 0.9844275755084820, -0.1757906384836575),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                3.5614767161100072,
                Coord::new(-0.0, 0.9844275755084820, -0.1757906384836575),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                3.8984742788054678,
                Coord::new(0.7013343843696721, -0.6375767130633382, -0.3187883565316692),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                4.8831177467169011,
                Coord::new(0.7013343843696721, -0.6375767130633382, -0.3187883565316692),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                5.4523088538682449,
                Coord::new(-0.0499376169438922, -0.9987523388778448, -0.0),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                6.1453761368528133,
                Coord::new(-0.0499376169438922, -0.9987523388778448, -0.0),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                7.1232278747423878,
                Coord::new(-0.0499376169438922, -0.9987523388778448, -0.0),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                7.4612006384156970,
                Coord::new(-0.7071067811865476, 0.7071067811865475, 0.0),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                8.3975630709442228,
                Coord::new(-0.7071067811865476, 0.7071067811865475, 0.0),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                8.8718474912408851,
                Coord::new(-0.0, -0.7071067811865476, 0.7071067811865476),
                Coord::new(0.0, 0.0, 0.0),
            ),
            (
                9.8017700948612454,
                Coord::new(-0.0, -0.7071067811865476, 0.7071067811865476),
                Coord::new(0.0, 0.0, 0.0),
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
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 0.2, 0.0),
            Coord::new(0.0, 3.0, 0.0),
            Coord::new(1.1, 2.0, 0.0),
            Coord::new(1.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(0.0, 0.0, 0.0),
        ];

        let accuracy = 0.05;

        let path = Path::from_waypoints(&waypoints, accuracy);

        debug_path_switching_points("debug_switching_points", &path, &waypoints);
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
