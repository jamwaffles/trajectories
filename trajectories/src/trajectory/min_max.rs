//! Whether to get the minimum or maximum from a function

/// Minimum or maximum
#[derive(Debug)]
pub enum MinMax {
    Min,
    Max,
}

impl MinMax {
    pub fn as_multiplier(&self) -> f64 {
        match self {
            MinMax::Min => -1.0,
            MinMax::Max => 1.0,
        }
    }
}
