use super::Coord;

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
}

impl LinearPathSegment {
    pub fn from_waypoints(start: Coord, end: Coord) -> Self {
        let length = (end - start).norm();

        Self { start, end, length }
    }

    /// Get position ("robot configuration" in paper parlance) along path from normalised distance
    /// along it (`s`)
    pub fn get_position(&self, distance_along_line: f64) -> Option<Coord> {
        // s /= length;
        // s = std::max(0.0, std::min(1.0, s));
        // return (1.0 - s) * start + s * end;

        // let pos = distance_along_line / self.length;

        // if pos < 0.0 || pos > 1.0 {
        //     None
        // } else {
        //     Some((1.0 - pos) * self.start + pos * self.end)
        // }

        Some((self.start + self.end) * distance_along_line)
    }

    /// Get derivative (tangent) of point along path
    pub fn get_tangent(&self, distance_along_line: f64) -> Coord {
        (self.end - self.start).normalize()
    }

    /// Get second derivative (rate of change of tangent, aka curvature) of point along path
    ///
    /// The curvature of a linear path is 0
    pub fn get_curvature(&self, distance_along_line: f64) -> Coord {
        Coord::repeat(0.0)
    }
}

#[cfg(test)]
mod tests {
    // TODO
}
