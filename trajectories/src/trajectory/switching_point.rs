//! Trajectory switching point

use crate::trajectory::TrajectoryStep;

#[derive(Debug, Clone)]
pub struct SwitchingPoint {
    // TODO: Split this out into position and velocity
    pub(crate) pos: TrajectoryStep,
    pub(crate) before_acceleration: f64,
    pub(crate) after_acceleration: f64,
}

impl Default for SwitchingPoint {
    fn default() -> Self {
        SwitchingPoint {
            pos: TrajectoryStep::new(0.0, 0.0),
            before_acceleration: 0.0,
            after_acceleration: 0.0,
        }
    }
}
