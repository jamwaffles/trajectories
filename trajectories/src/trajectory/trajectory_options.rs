use crate::Coord;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;

/// Trajectory creation options
#[derive(Debug, Clone)]
pub struct TrajectoryOptions<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Velocity limit for each axis
    pub velocity_limit: Coord<N>,

    /// Acceleration limit for each axis
    pub acceleration_limit: Coord<N>,

    /// Epsilon for comparing floats to a "close enough" threshold
    pub epsilon: f64,

    /// Timestep granularity that the trajectory should be generated to
    pub timestep: f64,
}

impl<N> Default for TrajectoryOptions<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    fn default() -> Self {
        Self {
            velocity_limit: Coord::repeat(1.0),
            acceleration_limit: Coord::repeat(1.0),
            epsilon: crate::TRAJECTORY_EPSILON,
            timestep: 0.1,
        }
    }
}
