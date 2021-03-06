mod path;
mod path_segment;
#[doc(hidden)]
pub mod test_helpers;
mod trajectory;
mod waypoint;

pub use crate::path::Path;
pub use crate::trajectory::{LinearTrajectory, TrajectoryOptions};
pub use crate::waypoint::Waypoint;
use nalgebra::VectorN;

/// Type alias for all vector operations
pub type Coord<N> = VectorN<f64, N>;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
