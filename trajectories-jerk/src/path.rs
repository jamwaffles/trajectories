use crate::path_segment::PathSegment;
use crate::Coord;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

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
                [start, end] => PathSegment::linear(*start, *end),
                _ => panic!("Path requires at least two waypoints"),
            })
            .collect();

        Ok(Self { segments })
    }

    // TODO: Another method to construct a path from a GCode program. Or impl From<thing>?
}

pub enum PathErrorKind {
    TooShort,
}
