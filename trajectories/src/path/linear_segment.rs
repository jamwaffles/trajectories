use super::PathItem;
use Coord;

/// Circular path segment
///
/// Used to blend two straight path segments along a circular path. `x` and `y` form a plane on
/// on which the blend circle lies, with its center at `center`. Radius is radius.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LinearPathSegment {
    /// Start coordinate
    pub start: Coord,

    /// End coordinate
    pub end: Coord,

    /// Length of this segment
    pub length: f64,

    /// Path start offset
    pub start_offset: f64,
}

impl LinearPathSegment {
    pub fn from_waypoints(start: Coord, end: Coord) -> Self {
        let length = (end - start).norm();

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
}

impl PathItem for LinearPathSegment {
    /// Get position ("robot configuration" in paper parlance) along path from normalised distance
    /// along it (`s`)
    fn get_position(&self, distance_along_line: f64) -> Coord {
        self.start + ((self.end - self.start) * distance_along_line / self.length)
    }

    /// Get derivative (tangent) of point along path
    fn get_tangent(&self, _distance_along_line: f64) -> Coord {
        (self.end - self.start) / self.length
    }

    /// Get second derivative (rate of change of tangent, aka curvature) of point along path
    ///
    /// The curvature of a linear path is 0
    fn get_curvature(&self, _distance_along_line: f64) -> Coord {
        Coord::repeat(0.0)
    }

    /// Get the length of this line
    fn get_length(&self) -> f64 {
        self.length
    }

    /// Get switching points for linear segment
    ///
    /// There are no switching points for a linear segment, so this method returns `None`
    fn get_switching_points(&self) -> Option<Vec<f64>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_switching_points() {
        let line = LinearPathSegment::from_waypoints(Coord::repeat(0.0), Coord::new(1.0, 0.0, 0.0));

        assert_eq!(line.get_switching_points(), None);
    }

    #[test]
    fn line_length_1_position() {
        let line = LinearPathSegment::from_waypoints(Coord::repeat(0.0), Coord::new(1.0, 0.0, 0.0));
        let pos_start = line.get_position(0.0);
        let pos_quarter = line.get_position(0.25);
        let pos_three_quarter = line.get_position(0.75);
        let pos_end = line.get_position(1.0);

        assert_near!(pos_start.x, 0.0);
        assert_near!(pos_start.y, 0.0);
        assert_near!(pos_quarter.x, 0.25);
        assert_near!(pos_quarter.y, 0.0);
        assert_near!(pos_three_quarter.x, 0.75);
        assert_near!(pos_three_quarter.y, 0.0);
        assert_near!(pos_end.x, 1.0);
        assert_near!(pos_end.y, 0.0);
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

        assert_near!(pos_start.x, 0.0);
        assert_near!(pos_start.y, 0.0);
        assert_near!(pos_quarter.x, 0.17677669529663687);
        assert_near!(pos_quarter.y, 0.17677669529663687);
        assert_near!(pos_three_quarter.x, 0.5303300858899106);
        assert_near!(pos_three_quarter.y, 0.5303300858899106);
        assert_near!(pos_1.x, 0.7071067811865475);
        assert_near!(pos_1.y, 0.7071067811865475);
        assert_near!(pos_end.x, 1.0);
        assert_near!(pos_end.y, 1.0);
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

        assert_near!(pos_start.x, 2.0);
        assert_near!(pos_start.y, 2.0);
        assert_near!(pos_quarter.x, 2.176776695296637);
        assert_near!(pos_quarter.y, 2.176776695296637);
        assert_near!(pos_three_quarter.x, 2.5303300858899105);
        assert_near!(pos_three_quarter.y, 2.5303300858899105);
        assert_near!(pos_1.x, 2.7071067811865475);
        assert_near!(pos_1.y, 2.7071067811865475);
        assert_near!(pos_end.x, 3.0);
        assert_near!(pos_end.y, 3.0);
    }
}
