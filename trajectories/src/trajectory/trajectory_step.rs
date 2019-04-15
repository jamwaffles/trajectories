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
    /// Create a new trajectory step from position and velocity
    #[inline(always)]
    pub fn new(position: f64, velocity: f64) -> Self {
        Self {
            position,
            velocity,
            time: 0.0,
        }
    }

    /// Add a time value to a current trajectory step
    pub fn with_time(self, time: f64) -> Self {
        Self { time, ..self }
    }

    /// Set the time for this trajectory step
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
