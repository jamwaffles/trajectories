use crate::path::{CircularPathSegment, LinearPathSegment, PathItem};
use alga::general::Real;
use alga::linear::FiniteDimInnerSpace;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;

#[derive(Debug, Clone, PartialEq)]
pub enum PathSegment<N, V>
where
    N: FiniteDimInnerSpace + Copy,
    V: Real,
{
    Linear(LinearPathSegment<N, V>),
    Circular(CircularPathSegment<N, V>),
}

impl<N, V> PathItem<N, V> for PathSegment<N, V>
where
    N: FiniteDimInnerSpace + Copy,
    V: Real,
{
    /// Get length of path
    fn get_length(&self) -> V {
        match self {
            PathSegment::Linear(s) => s.get_length(),
            PathSegment::Circular(s) => s.get_length(),
        }
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: V) -> N {
        match self {
            PathSegment::Linear(s) => s.get_position(distance_along_line),
            PathSegment::Circular(s) => s.get_position(distance_along_line),
        }
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: V) -> N {
        match self {
            PathSegment::Linear(s) => s.get_tangent(distance_along_line),
            PathSegment::Circular(s) => s.get_tangent(distance_along_line),
        }
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: V) -> N {
        match self {
            PathSegment::Linear(s) => s.get_curvature(distance_along_line),
            PathSegment::Circular(s) => s.get_curvature(distance_along_line),
        }
    }
}

impl<N, V> PathSegment<N, V>
where
    N: FiniteDimInnerSpace + Copy,
    V: Real,
{
    /// Clone segment and give it a start offset
    // TODO: Trait
    pub fn with_start_offset(self, offset: V) -> Self {
        match self {
            PathSegment::Linear(s) => PathSegment::Linear(s.with_start_offset(offset)),
            PathSegment::Circular(s) => PathSegment::Circular(s.with_start_offset(offset)),
        }
    }

    /// Get start offset of this segment
    // TODO: Trait
    pub fn get_start_offset(&self) -> V {
        match self {
            PathSegment::Linear(s) => s.start_offset,
            PathSegment::Circular(s) => s.start_offset,
        }
    }

    /// Get end offset of this segment
    pub fn get_end_offset(&self) -> V {
        match self {
            PathSegment::Linear(s) => s.get_end_offset(),
            PathSegment::Circular(s) => s.get_end_offset(),
        }
    }

    /// Get the switching points for this path segment
    // TODO: Trait?
    pub fn get_switching_points(&self) -> Vec<V> {
        match self {
            PathSegment::Linear(s) => s.get_switching_points(),
            PathSegment::Circular(s) => s.get_switching_points(),
        }
    }
}
