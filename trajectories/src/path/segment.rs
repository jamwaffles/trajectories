use crate::path::{CircularPathSegment, LinearPathSegment, PathItem};
use crate::Coord;
use nalgebra::allocator::Allocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;
use nalgebra::Real;
use nalgebra::VectorN;

#[derive(Debug, Clone, PartialEq)]
pub enum PathSegment<N: Real, D: DimName>
where
    DefaultAllocator: Allocator<N, D>,
{
    Linear(LinearPathSegment<N, D>),
    Circular(CircularPathSegment<N, D>),
}

impl<N, D> PathItem<N, D> for PathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Get length of path
    fn get_length(&self) -> N {
        match self {
            PathSegment::Linear(s) => s.get_length(),
            PathSegment::Circular(s) => s.get_length(),
        }
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: N) -> VectorN<N, D> {
        match self {
            PathSegment::Linear(s) => s.get_position(distance_along_line),
            PathSegment::Circular(s) => s.get_position(distance_along_line),
        }
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: N) -> VectorN<N, D> {
        match self {
            PathSegment::Linear(s) => s.get_tangent(distance_along_line),
            PathSegment::Circular(s) => s.get_tangent(distance_along_line),
        }
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: N) -> VectorN<N, D> {
        match self {
            PathSegment::Linear(s) => s.get_curvature(distance_along_line),
            PathSegment::Circular(s) => s.get_curvature(distance_along_line),
        }
    }
}

impl<N, D> PathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Clone segment and give it a start offset
    // TODO: Trait
    pub fn with_start_offset(&self, offset: N) -> Self {
        match self {
            PathSegment::Linear(s) => PathSegment::Linear(s.with_start_offset(offset)),
            PathSegment::Circular(s) => PathSegment::Circular(s.with_start_offset(offset)),
        }
    }

    /// Get start offset of this segment
    // TODO: Trait
    pub fn get_start_offset(&self) -> N {
        match self {
            PathSegment::Linear(s) => s.start_offset,
            PathSegment::Circular(s) => s.start_offset,
        }
    }

    /// Get the switching points for this path segment
    // TODO: Trait?
    pub fn get_switching_points(&self) -> Vec<N> {
        match self {
            PathSegment::Linear(s) => s.get_switching_points(),
            PathSegment::Circular(s) => s.get_switching_points(),
        }
    }
}
