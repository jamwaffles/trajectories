use super::TrajectoryOptions;
use crate::path_segment::PathSegment;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

pub struct TrajectorySegment<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    // Reference to original path segment
    path_segment: &'a PathSegment<N>,

    /// Start time from beginning of trajectory
    start_offset: f64,

    /// The duration of this segment
    time: f64,
}

impl<'a, N> TrajectorySegment<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn new(
        path_segment: &'a PathSegment<N>,
        options: TrajectoryOptions<N>,
        start_offset: f64,
    ) -> Self {
        let time = match path_segment {
            PathSegment::Linear(segment) => options
                .velocity_limit
                .component_mul(&(segment.end - segment.start).normalize())
                .norm(),
        };

        Self {
            path_segment,
            time,
            start_offset,
        }
    }

    pub fn start_offset(&self) -> f64 {
        self.start_offset
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn len(&self) -> f64 {
        self.path_segment.len()
    }
}
