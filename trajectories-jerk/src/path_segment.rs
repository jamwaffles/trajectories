use crate::waypoint::Waypoint;
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

    pub(crate) start_velocity: Coord<N>,
    pub(crate) end_velocity: Coord<N>,

    length: f64,
}

impl<N> LinearPathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn from_waypoints(start: Waypoint<N>, end: Waypoint<N>) -> Self {
        Self {
            start: start.position,
            end: end.position,
            start_velocity: start.velocity,
            end_velocity: end.velocity,
            length: (end.position - start.position).norm(),
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
    pub fn linear(start: Waypoint<N>, end: Waypoint<N>) -> Self {
        PathSegment::Linear(LinearPathSegment::from_waypoints(start, end))
    }

    pub fn len(&self) -> f64 {
        match self {
            PathSegment::Linear(segment) => segment.length,
        }
    }
}
