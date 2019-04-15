//! Whether to get the minimum or maximum from a function

/// Minimum or maximum
#[derive(Debug)]
pub enum MinMax {
    /// Minimum
    Min,

    /// Maximum
    Max,
}

impl MinMax {
    /// Get min/max as a multiplier (`1.0` or `-1.0` respectively)
    pub fn as_multiplier(&self) -> f64 {
        match self {
            MinMax::Min => -1.0,
            MinMax::Max => 1.0,
        }
    }
}
