use crate::Coord;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

#[derive(Debug, Copy, Clone)]
pub struct LinearPathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub(crate) start: Coord<N>,
    pub(crate) end: Coord<N>,

    length: f64,
}

impl<N> LinearPathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn new(start: Coord<N>, end: Coord<N>) -> Self {
        Self {
            start,
            end,
            length: (end - start).norm(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    Linear(LinearPathSegment<N>),
}

impl<N> PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn linear(start: Coord<N>, end: Coord<N>) -> Self {
        PathSegment::Linear(LinearPathSegment::new(start, end))
    }

    pub fn len(&self) -> f64 {
        match self {
            PathSegment::Linear(segment) => segment.length,
        }
    }
}
