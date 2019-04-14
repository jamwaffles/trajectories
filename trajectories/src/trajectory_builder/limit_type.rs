//! The kind of limit that exists at a certain point

use crate::Coord;
use nalgebra::{
    allocator::{Allocator, SameShapeVectorAllocator},
    DefaultAllocator, DimName,
};

/// Limited by velocity or acceleration
#[derive(Debug)]
pub enum LimitType<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    Velocity(Coord<N>),
    Acceleration(Coord<N>),
}
