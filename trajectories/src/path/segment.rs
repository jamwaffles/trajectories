use crate::path::{CircularPathSegment, LinearPathSegment, PathItem};
use crate::Coord;
use core::cmp::Ordering;
use nalgebra::allocator::Allocator;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;

#[derive(Debug, Clone, PartialEq)]
pub enum PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    Linear(LinearPathSegment<N>),
    Circular(CircularPathSegment<N>),
}

impl<N> PartialOrd for PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    fn partial_cmp(&self, other: &PathSegment<N>) -> Option<Ordering> {
        Some(
            self.start_offset()
                .partial_cmp(&other.start_offset())
                .expect(&format!(
                    "Could not compare path offsets between {:?} and {:?}",
                    self, other
                )),
        )
    }
}

impl<N> PathItem<N> for PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    /// Get length of path
    fn len(&self) -> f64 {
        match self {
            PathSegment::Linear(s) => s.len(),
            PathSegment::Circular(s) => s.len(),
        }
    }

    /// Get position at a point along path
    fn position(&self, distance_along_line: f64) -> Coord<N> {
        match self {
            PathSegment::Linear(s) => s.position(distance_along_line),
            PathSegment::Circular(s) => s.position(distance_along_line),
        }
    }

    /// Get first derivative (tangent) at a point
    fn tangent(&self, distance_along_line: f64) -> Coord<N> {
        match self {
            PathSegment::Linear(s) => s.tangent(distance_along_line),
            PathSegment::Circular(s) => s.tangent(distance_along_line),
        }
    }

    /// Get second derivative (curvature) at a point
    fn curvature(&self, distance_along_line: f64) -> Coord<N> {
        match self {
            PathSegment::Linear(s) => s.curvature(distance_along_line),
            PathSegment::Circular(s) => s.curvature(distance_along_line),
        }
    }
}

impl<N> PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    /// Clone segment and give it a start offset
    // TODO: Trait
    pub fn with_start_offset(self, offset: f64) -> Self {
        match self {
            PathSegment::Linear(s) => PathSegment::Linear(s.with_start_offset(offset)),
            PathSegment::Circular(s) => PathSegment::Circular(s.with_start_offset(offset)),
        }
    }

    /// Get start offset of this segment
    // TODO: Trait
    pub fn start_offset(&self) -> f64 {
        match self {
            PathSegment::Linear(s) => s.start_offset,
            PathSegment::Circular(s) => s.start_offset,
        }
    }

    /// Get end offset of this segment
    pub fn end_offset(&self) -> f64 {
        match self {
            PathSegment::Linear(s) => s.end_offset(),
            PathSegment::Circular(s) => s.end_offset(),
        }
    }

    /// Get the switching points for this path segment
    // TODO: Trait?
    pub fn switching_points(&self) -> Vec<f64> {
        match self {
            PathSegment::Linear(s) => s.switching_points(),
            PathSegment::Circular(s) => s.switching_points(),
        }
    }
}
