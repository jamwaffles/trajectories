use crate::Coord;
use nalgebra::{
    allocator::{Allocator, SameShapeVectorAllocator},
    DefaultAllocator, DimName,
};

/// Helpful methods to get information about a path
pub trait PathItem<N>: PartialEq
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    /// Get length of path
    fn len(&self) -> f64;

    /// Get position at a point along path
    fn position(&self, distance_along_line: f64) -> Coord<N>;

    /// Get first derivative (tangent) at a point
    fn tangent(&self, distance_along_line: f64) -> Coord<N>;

    /// Get second derivative (curvature) at a point
    fn curvature(&self, distance_along_line: f64) -> Coord<N>;
}
