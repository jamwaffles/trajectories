//! Trajectory switching point

use crate::trajectory::TrajectoryStep;
use alga::general::Real;

#[derive(Debug, PartialEq, Clone)]
pub struct SwitchingPoint<V> {
    // TODO: Split this out into position and velocity
    pub(crate) pos: TrajectoryStep<V>,
    pub(crate) before_acceleration: V,
    pub(crate) after_acceleration: V,
}

impl<V> Default for SwitchingPoint<V>
where
    V: Real,
{
    fn default() -> Self {
        SwitchingPoint {
            pos: TrajectoryStep::new(nalgebra::convert(0.0), nalgebra::convert(0.0)),
            before_acceleration: nalgebra::convert(0.0),
            after_acceleration: nalgebra::convert(0.0),
        }
    }
}
