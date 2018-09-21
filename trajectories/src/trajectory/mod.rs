use path::{Continuity, Path, PathItem, SwitchingPoint};
use std;
use Coord;
use TRAJ_EPSILON;

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

#[derive(Debug)]
struct AccelerationSwitchingPoint {
    has_reached_end: bool,
    velocity: f64,
    before_acceleration: f64,
    after_acceleration: f64,
}

impl Default for AccelerationSwitchingPoint {
    fn default() -> Self {
        AccelerationSwitchingPoint {
            has_reached_end: false,
            velocity: 0.0,
            before_acceleration: 0.0,
            after_acceleration: 0.0,
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

        let _ = path.get_min_max_path_acceleration(&PositionAndVelocity(0.0, 0.0), MinMax::Max);

        path
    }

    /// Find minimum or maximum acceleration at a point along path
    fn get_min_max_path_acceleration(&self, pos_vel: &PositionAndVelocity, min_max: MinMax) -> f64 {
        let &PositionAndVelocity(position, velocity) = pos_vel;

        let derivative = self.path.get_tangent(position);
        let second_derivative = self.path.get_curvature(position);
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

    /// Find the maximum allowable scalar velocity given a position along the path
    ///
    /// This method calculates the velocity limit as the smallest component of the tangent
    /// (derivative of position, i.e velocity) at the given point.
    fn get_max_velocity_from_velocity(&self, position_along_path: f64) -> f64 {
        let tangent = self.path.get_tangent(position_along_path);

        tangent.iter().zip(self.velocity_limit.iter()).fold(
            std::f64::MAX,
            |acc, (tangent_component, component_max)| {
                acc.min(component_max / tangent_component.abs())
            },
        )
    }

    /// Find maximum allowable velocity as limited by the acceleration at a point on the path
    fn get_max_velocity_from_acceleration(&self, position_along_path: f64) -> f64 {
        let velocity = self.path.get_tangent(position_along_path);
        let acceleration = self.path.get_curvature(position_along_path);

        let n = velocity.len();

        let mut max_path_velocity = std::f64::INFINITY;

        for i in 0..n {
            if velocity[i] != 0.0 {
                for j in (i + 1)..n {
                    // TODO: Come up with a less mathsy name
                    let a_ij = acceleration[i] / velocity[i] - acceleration[j] / velocity[j];

                    if a_ij != 0.0 {
                        max_path_velocity = max_path_velocity.min(
                            (self.acceleration_limit[i] / velocity[i].abs()
                                + self.acceleration_limit[j] / velocity[j])
                                .sqrt()
                                / a_ij.abs(),
                        );
                    }
                }
            } else {
                max_path_velocity = max_path_velocity
                    .min((self.acceleration_limit[i] / acceleration[i].abs()).sqrt());
            }
        }

        max_path_velocity
    }

    /// Get the derivative of the max velocity at a point along the path
    ///
    /// The max velocity in this case is bounded by the acceleration limits at the point
    fn get_max_velocity_from_acceleration_derivative(&self, position_along_path: f64) -> f64 {
        (self.get_max_velocity_from_acceleration(position_along_path + TRAJ_EPSILON)
            - self.get_max_velocity_from_acceleration(position_along_path - TRAJ_EPSILON))
            / (2.0 * TRAJ_EPSILON)
    }

    /// Get the next acceleration switching point after the current position
    fn get_next_acceleration_switching_point(
        &self,
        position_along_path: f64,
    ) -> AccelerationSwitchingPoint {
        let mut ret = AccelerationSwitchingPoint::default();

        loop {
            let switching_point = self.path.get_next_switching_point(position_along_path);

            if switching_point.position > self.path.get_length() - TRAJ_EPSILON {
                ret.has_reached_end = true;

                break ret;
            }

            match switching_point.continuity {
                Continuity::Discontinuous => {
                    let before_velocity = self.get_max_velocity_from_acceleration(
                        switching_point.position - TRAJ_EPSILON,
                    );
                    let after_velocity = self.get_max_velocity_from_acceleration(
                        switching_point.position + TRAJ_EPSILON,
                    );

                    ret.velocity = before_velocity.min(after_velocity);

                    let before_point =
                        PositionAndVelocity(switching_point.position - TRAJ_EPSILON, ret.velocity);
                    let after_point =
                        PositionAndVelocity(switching_point.position + TRAJ_EPSILON, ret.velocity);

                    ret.before_acceleration =
                        self.get_min_max_path_acceleration(&before_point, MinMax::Min);
                    ret.after_acceleration =
                        self.get_min_max_path_acceleration(&after_point, MinMax::Max);

                    if (before_velocity > after_velocity
                        || self.get_min_max_phase_slope(&before_point, MinMax::Min) > self
                            .get_max_velocity_from_acceleration_derivative(
                                switching_point.position - 2.0 * TRAJ_EPSILON,
                            ))
                        && (before_velocity < after_velocity
                            || self.get_min_max_phase_slope(&after_point, MinMax::Max) > self
                                .get_max_velocity_from_acceleration_derivative(
                                    switching_point.position + 2.0 * TRAJ_EPSILON,
                                )) {
                        break ret;
                    }
                }
                Continuity::Continuous => {
                    let velocity =
                        self.get_max_velocity_from_acceleration(switching_point.position);

                    let before_acceleration = 0.0;
                    let after_acceleration = 0.0;

                    if self.get_max_velocity_from_acceleration_derivative(
                        switching_point.position - TRAJ_EPSILON,
                    ) < 0.0
                        && self.get_max_velocity_from_acceleration_derivative(
                            switching_point.position + TRAJ_EPSILON,
                        ) > 0.0
                    {
                        break AccelerationSwitchingPoint {
                            has_reached_end: false,
                            velocity,
                            before_acceleration,
                            after_acceleration,
                        };
                    }
                }
            }
        }
    }

    /// Get the next velocity switching point after the current position
    fn get_next_velocity_switching_point(&self, position_along_path: f64) {
        unimplemented!()
    }

    /// Get the minimum or maximum phase slope for a position along the path
    ///
    /// TODO: Figure out what phase slope means in this context
    fn get_min_max_phase_slope(&self, pos_vel: &PositionAndVelocity, min_max: MinMax) -> f64 {
        self.get_min_max_path_acceleration(&pos_vel, min_max) / pos_vel.0
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
