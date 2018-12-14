//! A discrete step along a complete trajectory

use alga::general::Real;

/// Trajectory step
#[derive(Debug, PartialEq, Copy, Clone)]
pub(crate) struct TrajectoryStep<V> {
    /// Position
    pub(crate) position: V,
    /// Velocity
    pub(crate) velocity: V,
    /// Time
    pub(crate) time: V,
}

impl<V> TrajectoryStep<V>
where
    V: Real,
{
    #[inline(always)]
    pub fn new(position: V, velocity: V) -> Self {
        Self {
            position,
            velocity,
            time: nalgebra::convert(0.0),
        }
    }

    pub fn with_time(self, time: V) -> Self {
        Self { time, ..self }
    }
}

impl<V> Default for TrajectoryStep<V>
where
    V: Real,
{
    fn default() -> Self {
        Self {
            position: nalgebra::convert(0.0),
            velocity: nalgebra::convert(0.0),
            time: nalgebra::convert(0.0),
        }
    }
}
