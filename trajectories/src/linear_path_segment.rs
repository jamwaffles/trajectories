use super::Coord;
use PathItem;

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
        self.start + ((self.end - self.start) * distance_along_line)
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
}

#[cfg(test)]
mod tests {
    // TODO
}
