use path::{Path, PathItem};
use std;
use Coord;

/// A (position, velocity) pair
#[derive(Debug)]
struct PositionAndVelocity(
    /// Position
    f64,
    /// Velocity
    f64,
);

/// Whether to get the minimum or maximum
#[derive(Debug)]
enum MinMax {
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

/// Motion trajectory
#[derive(Debug)]
pub struct Trajectory {
    path: Path,
    velocity_limit: Coord,
    acceleration_limit: Coord,
}

impl Trajectory {
    /// Create a new trajectory from a given path and max velocity and acceleration
    pub fn new(path: Path, velocity_limit: Coord, acceleration_limit: Coord) -> Self {
        let path = Self {
            path,
            velocity_limit,
            acceleration_limit,
        };

        let _ = path.get_min_max_path_acceleration(PositionAndVelocity(0.0, 0.0), MinMax::Max);

        path
    }

    /// Find minimum or maximum acceleration at a point along path
    fn get_min_max_path_acceleration(&self, pos_vel: PositionAndVelocity, min_max: MinMax) -> f64 {
        let PositionAndVelocity(position, velocity) = pos_vel;

        let derivative = self.path.get_tangent(velocity);
        let second_derivative = self.path.get_curvature(velocity);
        let factor = min_max.as_multiplier();

        let res = self
            .acceleration_limit
            .iter()
            .zip(derivative.iter().zip(second_derivative.iter()))
            .fold(
                std::f64::MAX,
                |acc,
                 (
                    acceleration_limit_component,
                    (derivative_component, second_derivative_component),
                )| {
                    if *derivative_component != 0.0 {
                        acc.min(
                            acceleration_limit_component / derivative_component.abs()
                                - factor * second_derivative_component * velocity.powi(2)
                                    / derivative_component,
                        )
                    } else {
                        acc
                    }
                },
            );

        res * factor
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_trajectory() {
        let waypoints: Vec<Coord> = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(1.0, 2.0, 0.0),
            Coord::new(1.5, 1.5, 0.0),
            Coord::new(3.0, 5.0, 0.0),
            Coord::new(4.0, 6.0, 0.0),
            Coord::new(5.0, 5.0, 0.0),
            Coord::new(4.0, 4.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);

        let traj = Trajectory::new(path, Coord::repeat(1.0), Coord::repeat(1.0));
    }
}
