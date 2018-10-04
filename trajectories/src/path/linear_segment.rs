use super::PathItem;
use crate::Coord;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;

/// Circular path segment
///
/// Used to blend two straight path segments along a circular path. `x` and `y` form a plane on
/// on which the blend circle lies, with its center at `center`. Radius is radius.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearPathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Start coordinate
    pub start: Coord<N>,

    /// End coordinate
    pub end: Coord<N>,

    /// Length of this segment
    pub length: f64,

    /// Path start offset
    pub start_offset: f64,
}

impl<N> LinearPathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    pub fn from_waypoints(start: Coord<N>, end: Coord<N>) -> Self {
        let length = (&end - &start).norm();

        Self {
            start,
            end,
            length,
            start_offset: 0.0,
        }
    }

    /// Clone with a start offset
    pub fn with_start_offset(&self, start_offset: f64) -> Self {
        Self {
            start_offset,
            ..self.clone()
        }
    }

    /// Get switching points for linear segment
    ///
    /// There are no switching points for a linear segment, so this method will always return an
    /// empty list.
    // TODO: Trait
    pub fn get_switching_points(&self) -> Vec<f64> {
        Vec::new()
    }
}

impl<N> PathItem<N> for LinearPathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Get position ("robot configuration" in paper parlance) along path from normalised distance
    /// along it (`s`)
    fn get_position(&self, distance_along_line: f64) -> Coord<N> {
        &self.start
            + ((&self.end - &self.start) * (distance_along_line - self.start_offset) / self.length)
    }

    /// Get derivative (tangent) of point along path
    fn get_tangent(&self, _distance_along_line: f64) -> Coord<N> {
        (&self.end - &self.start) / self.length
    }

    /// Get second derivative (rate of change of tangent, aka curvature) of point along path
    ///
    /// The curvature of a linear path is 0
    fn get_curvature(&self, _distance_along_line: f64) -> Coord<N> {
        Coord::repeat(0.0)
    }

    /// Get the length of this line
    fn get_length(&self) -> f64 {
        self.length
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn no_switching_points() {
        let line = LinearPathSegment::from_waypoints(
            TestCoord3::repeat(0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
        );

        assert_eq!(line.get_switching_points(), Vec::new());
    }

    #[test]
    fn line_length_1_position() {
        let line = LinearPathSegment::from_waypoints(
            TestCoord3::repeat(0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
        );
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_end = line.get_position(1.0);

        assert_near!(pos_start, TestCoord3::new(0.0, 0.0, 0.0));
        assert_near!(pos_quarter, TestCoord3::new(0.25, 0.0, 0.0));
        assert_near!(pos_three_quarter, TestCoord3::new(0.75, 0.0, 0.0));
        assert_near!(pos_end, TestCoord3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn diagonal_line_position() {
        let line = LinearPathSegment::from_waypoints(
            TestCoord3::repeat(0.0),
            TestCoord3::new(1.0, 1.0, 0.0),
        );
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_1 = line.get_position(1.0);
        let pos_end = line.get_position(1.41421356237);

        let len = 2.0_f64.sqrt();

        // Sqrt(2)
        assert_eq!(line.get_length(), len);

        assert_near!(pos_start, TestCoord3::new(0.0, 0.0, 0.0));
        assert_near!(
            pos_quarter,
            TestCoord3::new(0.1767766952, 0.1767766952, 0.0)
        );
        assert_near!(
            pos_three_quarter,
            TestCoord3::new(0.530330085, 0.530330085, 0.0)
        );
        assert_near!(pos_1, TestCoord3::new(0.707106781, 0.707106781, 0.0));
        assert_near!(pos_end, TestCoord3::new(1.0, 1.0, 0.0));
    }

    #[test]
    fn diagonal_not_at_zero() {
        let line = LinearPathSegment::from_waypoints(
            TestCoord3::new(2.0, 2.0, 0.0),
            TestCoord3::new(3.0, 3.0, 0.0),
        );
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_1 = line.get_position(1.0);
        let pos_end = line.get_position(1.41421356237);

        let len = 2.0_f64.sqrt();

        // Sqrt(2)
        assert_eq!(line.get_length(), len);

        assert_near!(pos_start, TestCoord3::new(2.0, 2.0, 0.0));
        assert_near!(
            pos_quarter,
            TestCoord3::new(2.1767766952, 2.1767766952, 0.0)
        );
        assert_near!(
            pos_three_quarter,
            TestCoord3::new(2.530330085, 2.530330085, 0.0)
        );
        assert_near!(pos_1, TestCoord3::new(2.707106781, 2.707106781, 0.0));
        assert_near!(pos_end, TestCoord3::new(3.0, 3.0, 0.0));
    }
}
