//! Trajectory switching point

use crate::trajectory::TrajectoryStep;

#[derive(Debug, PartialEq, Clone)]
pub struct TrajectorySwitchingPoint {
    // TODO: Split this out into position and velocity
    pub(crate) pos: TrajectoryStep,
    pub(crate) before_acceleration: f64,
    pub(crate) after_acceleration: f64,
}

impl Default for TrajectorySwitchingPoint {
    fn default() -> Self {
        TrajectorySwitchingPoint {
            pos: TrajectoryStep::new(0.0, 0.0),
            before_acceleration: 0.0,
            after_acceleration: 0.0,
        }
    }
}
