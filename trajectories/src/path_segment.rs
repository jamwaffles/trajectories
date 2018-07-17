use super::Coord;

#[derive(Copy, Clone, Debug)]
pub enum PathSegment {
    Circular,
    Linear,
}

/// Circular path segment
///
/// Used to blend two straight path segments along a circular path. `x` and `y` form a plane on
/// on which the blend circle lies, with its center at `center`. Radius is radius.
#[derive(Copy, Clone, Debug)]
pub struct CircularPathSegment {
    /// Center point of circle
    pub center: Coord,

    /// Radius of circle
    pub radius: f64,

    /// First vector along which the blend circle lies
    pub x: Coord,

    /// Second vector along which the blend circle lies
    pub y: Coord,

    /// Length of this arc
    pub length: f64,
}

pub struct LinearPathSegment {}
