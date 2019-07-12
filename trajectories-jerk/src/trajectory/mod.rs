mod linear_trajectory;
mod trajectory_segment;

use self::trajectory_segment::TrajectorySegment;
use crate::Coord;
pub use linear_trajectory::LinearTrajectory;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

#[derive(Debug, Copy, Clone)]
pub struct TrajectoryOptions<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub velocity_limit: Coord<N>,
    pub acceleration_limit: Coord<N>,
}
