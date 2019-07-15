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

    pub(crate) start_vel: Option<Coord<N>>,
    pub(crate) end_vel: Option<Coord<N>>,

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
            start_vel: None,
            end_vel: None,
            length: (end - start).norm(),
        }
    }

    pub fn with_velocity(self, start_vel: Coord<N>, end_vel: Coord<N>) -> Self {
        Self {
            start_vel: Some(start_vel),
            end_vel: Some(end_vel),
            ..self
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
