use crate::Coord;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

#[derive(Debug, Copy, Clone)]
pub struct Waypoint<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub(crate) position: Coord<N>,
    pub(crate) velocity: Coord<N>,
}

impl<N> Waypoint<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn new(position: Coord<N>, velocity: Coord<N>) -> Self {
        Self { position, velocity }
    }
}
