//! Test/debug/profiling helpers
//!
//! These utilities should NOT be used in production code

mod helpers;
mod profile;

pub use self::helpers::*;
pub use self::profile::*;
use nalgebra::Vector3;
use nalgebra::Vector4;

/// 3 dimensional dobule precision vector for use in test code
pub type TestCoord3 = Vector3<f64>;

/// 4 dimensional dobule precision vector for use in test code
pub type TestCoord4 = Vector4<f64>;
