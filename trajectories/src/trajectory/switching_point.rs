//! Trajectory switching point

use crate::trajectory::TrajectoryStep;
use nalgebra::Real;

#[derive(Debug, Clone)]
pub struct SwitchingPoint<N: Real> {
    // TODO: Split this out into position and velocity
    pub(crate) pos: TrajectoryStep<N>,
    pub(crate) before_acceleration: N,
    pub(crate) after_acceleration: N,
}

impl<N> Default for SwitchingPoint<N>
where
    N: Real,
{
    fn default() -> Self {
        SwitchingPoint {
            pos: TrajectoryStep::new(nalgebra::convert(0.0), nalgebra::convert(0.0)),
            before_acceleration: nalgebra::convert(0.0),
            after_acceleration: nalgebra::convert(0.0),
        }
    }
}
