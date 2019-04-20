use crate::Coord;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

#[derive(Debug, Copy, Clone)]
pub struct PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    start: Coord<N>,
    end: Coord<N>,
}

impl<N> PathSegment<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn from_waypoints(start: Coord<N>, end: Coord<N>) -> Self {
        Self { start, end }
    }
}

#[derive(Debug, Clone)]
pub struct Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    segments: Vec<PathSegment<N>>,
}

impl<N> Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn from_waypoints(waypoints: &[Coord<N>]) -> Result<Self, PathErrorKind> {
        if waypoints.len() < 2 {
            return Err(PathErrorKind::TooShort);
        }

        let segments = waypoints
            .windows(2)
            .map(|parts| match parts {
                [start, end] => PathSegment::from_waypoints(*start, *end),
                _ => panic!("Bad parts"),
            })
            .collect();

        Ok(Self { segments })
    }
}

pub enum PathErrorKind {
    TooShort,
}
