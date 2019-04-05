//! A discrete step along a complete trajectory

/// Trajectory step
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct TrajectoryStep {
    /// Position
    pub position: f64,
    /// Velocity
    pub velocity: f64,
    /// Time
    pub time: f64,
}

impl TrajectoryStep {
    #[inline(always)]
    pub fn new(position: f64, velocity: f64) -> Self {
        Self {
            position,
            velocity,
            time: 0.0,
        }
    }

    pub fn with_time(self, time: f64) -> Self {
        Self { time, ..self }
    }

    pub fn time(mut self, time: f64) {
        self.time = time;
    }
}

impl Default for TrajectoryStep {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            time: 0.0,
        }
    }
}
