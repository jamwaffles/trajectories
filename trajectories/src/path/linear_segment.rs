use super::PathItem;
use crate::Coord;
use nalgebra::allocator::Allocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;
use nalgebra::Real;
use nalgebra::VectorN;

/// Linear path segment
#[derive(Clone, Debug, PartialEq)]
pub struct LinearPathSegment<N: Real, D: DimName>
where
    DefaultAllocator: Allocator<N, D>,
{
    /// Start coordinate
    pub start: VectorN<N, D>,

    /// End coordinate
    pub end: VectorN<N, D>,

    /// Length of this segment
    pub length: N,

    /// Path start offset
    pub start_offset: N,
}

impl<N, D> LinearPathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    pub fn from_waypoints(start: VectorN<N, D>, end: VectorN<N, D>) -> Self {
        let length = (end - start).norm();

        Self {
            start,
            end,
            length,
            start_offset: nalgebra::convert(0.0),
        }
    }

    /// Clone with a start offset
    pub fn with_start_offset(&self, start_offset: N) -> Self {
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
    pub fn get_switching_points(&self) -> Vec<N> {
        Vec::new()
    }
}

impl<N, D> PathItem<N, D> for LinearPathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Get position ("robot configuration" in paper parlance) along path from normalised distance
    /// along it (`s`)
    fn get_position(&self, distance_along_line: N) -> VectorN<N, D> {
        self.start
            + ((self.end - self.start) * (distance_along_line - self.start_offset) / self.length)
    }

    /// Get derivative (tangent) of point along path
    fn get_tangent(&self, _distance_along_line: N) -> VectorN<N, D> {
        (self.end - self.start) / self.length
    }

    /// Get second derivative (rate of change of tangent, aka curvature) of point along path
    ///
    /// The curvature of a linear path is 0
    fn get_curvature(&self, _distance_along_line: N) -> VectorN<N, D> {
        VectorN::<N, D>::zeros()
    }

    /// Get the length of this line
    fn get_length(&self) -> N {
        self.length
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_switching_points() {
        let line = LinearPathSegment::from_waypoints(Coord::repeat(0.0), Coord::new(1.0, 0.0, 0.0));

        assert_eq!(line.get_switching_points(), Vec::new());
    }

    #[test]
    fn line_length_1_position() {
        let line = LinearPathSegment::from_waypoints(Coord::repeat(0.0), Coord::new(1.0, 0.0, 0.0));
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_end = line.get_position(1.0);

        assert_near!(pos_start, Coord::new(0.0, 0.0, 0.0));
        assert_near!(pos_quarter, Coord::new(0.25, 0.0, 0.0));
        assert_near!(pos_three_quarter, Coord::new(0.75, 0.0, 0.0));
        assert_near!(pos_end, Coord::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn diagonal_line_position() {
        let line = LinearPathSegment::from_waypoints(Coord::repeat(0.0), Coord::new(1.0, 1.0, 0.0));
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_1 = line.get_position(1.0);
        let pos_end = line.get_position(1.41421356237);

        let len = 2.0_f64.sqrt();

        // Sqrt(2)
        assert_eq!(line.get_length(), len);

        assert_near!(pos_start, Coord::new(0.0, 0.0, 0.0));
        assert_near!(pos_quarter, Coord::new(0.1767766952, 0.1767766952, 0.0));
        assert_near!(pos_three_quarter, Coord::new(0.530330085, 0.530330085, 0.0));
        assert_near!(pos_1, Coord::new(0.707106781, 0.707106781, 0.0));
        assert_near!(pos_end, Coord::new(1.0, 1.0, 0.0));
    }

    #[test]
    fn diagonal_not_at_zero() {
        let line =
            LinearPathSegment::from_waypoints(Coord::new(2.0, 2.0, 0.0), Coord::new(3.0, 3.0, 0.0));
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_1 = line.get_position(1.0);
        let pos_end = line.get_position(1.41421356237);

        let len = 2.0_f64.sqrt();

        // Sqrt(2)
        assert_eq!(line.get_length(), len);

        assert_near!(pos_start, Coord::new(2.0, 2.0, 0.0));
        assert_near!(pos_quarter, Coord::new(2.1767766952, 2.1767766952, 0.0));
        assert_near!(pos_three_quarter, Coord::new(2.530330085, 2.530330085, 0.0));
        assert_near!(pos_1, Coord::new(2.707106781, 2.707106781, 0.0));
        assert_near!(pos_end, Coord::new(3.0, 3.0, 0.0));
    }
}
