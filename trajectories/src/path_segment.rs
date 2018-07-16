use super::Coord;

pub enum PathSegment {
    Circular,
    Linear,
}

/// Circular path segment
///
/// Used to blend two straight path segments along a circular path. `x` and `y` form a plane on
/// on which the blend circle lies, with its center at `center`. Radius is radius.
pub struct CircularPathSegment {
    /// Center point of circle
    pub center: Coord,

    /// Radius of circle
    pub radius: f64,

    /// First vector along which the blend circle lies
    pub x: Coord,

    /// Second vector along which the blend circle lies
    pub y: Coord,
}

impl CircularPathSegment {
    pub fn new(center: Coord, radius: f64, x: Coord, y: Coord) -> Self {
        Self {
            center,
            radius,
            x,
            y,
        }
    }
}

pub struct LinearPathSegment {}
