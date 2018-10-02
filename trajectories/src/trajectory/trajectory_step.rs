//! A discrete step along a complete trajectory

use nalgebra::Real;

/// Trajectory step
#[derive(Debug, Clone)]
pub(crate) struct TrajectoryStep<N: Real> {
    /// Position
    pub(crate) position: N,
    /// Velocity
    pub(crate) velocity: N,
    /// Time
    pub(crate) time: N,
}

impl<N> TrajectoryStep<N>
where
    N: Real,
{
    #[inline(always)]
    pub fn new(position: N, velocity: N) -> Self {
        Self {
            position,
            velocity,
            time: nalgebra::convert(0.0),
        }
    }

    pub fn with_time(self, time: N) -> Self {
        Self { time, ..self }
    }
}

impl<N> Default for TrajectoryStep<N>
where
    N: Real,
{
    fn default() -> Self {
        Self {
            position: nalgebra::convert(0.0),
            velocity: nalgebra::convert(0.0),
            time: nalgebra::convert(0.0),
        }
    }
}
