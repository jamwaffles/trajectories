use super::Continuity;

/// A switching point
#[derive(Debug, Clone, PartialEq)]
pub struct PathSwitchingPoint {
    /// Position along the path at which this switching point occurs
    pub position: f64,
    /// Whether this switching point is discontinuous or not
    pub continuity: Continuity,
}

impl PathSwitchingPoint {
    /// Create a new switching point from position and continuity flag
    #[inline(always)]
    pub fn new(position: f64, continuity: Continuity) -> Self {
        Self {
            position,
            continuity,
        }
    }
}
