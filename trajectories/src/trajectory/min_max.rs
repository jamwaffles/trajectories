//! Whether to get the minimum or maximum from a function

use nalgebra::Real;

/// Minimum or maximum
#[derive(Debug)]
pub enum MinMax {
    Min,
    Max,
}

impl MinMax {
    pub fn as_multiplier<N>(&self) -> N
    where
        N: Real,
    {
        match self {
            MinMax::Min => nalgebra::convert(-1.0),
            MinMax::Max => nalgebra::convert(1.0),
        }
    }
}
