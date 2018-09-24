use path::{Continuity, Path, PathItem, SwitchingPoint};
use std;
use Coord;
use TRAJ_EPSILON;

/// A (position, velocity) pair
#[derive(Debug, Clone)]
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

#[derive(Debug, Clone)]
struct TrajectorySwitchingPoint {
    position: PositionAndVelocity,
    before_acceleration: f64,
    after_acceleration: f64,
}

impl Default for TrajectorySwitchingPoint {
    fn default() -> Self {
        TrajectorySwitchingPoint {
            position: PositionAndVelocity(0.0, 0.0),
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

    /// Get next switching point along the path, bounded by velocity or acceleration
    fn get_next_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut acceleration_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next acceleration switching point
        while let Some(point) = self.get_next_acceleration_switching_point(
            acceleration_switching_point
                .clone()
                .map(|p| p.position.0)
                .unwrap_or(position_along_path),
        ) {
            if point.position.1 > self.get_max_velocity_from_velocity(point.position.0) {
                break;
            }

            acceleration_switching_point = Some(point);
        }

        let mut velocity_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next velocity switching point
        while let Some(point) = self.get_next_velocity_switching_point(
            velocity_switching_point
                .clone()
                .map(|p| p.position.0)
                .unwrap_or(position_along_path),
        ) {
            if point.position.0 > acceleration_switching_point
                .clone()
                .expect("Accel switching point")
                .position
                .0
                || (point.position.1
                    <= self.get_max_velocity_from_acceleration(point.position.0 - TRAJ_EPSILON)
                    && point.position.1
                        <= self.get_max_velocity_from_acceleration(point.position.0 + TRAJ_EPSILON))
            {
                break;
            }

            velocity_switching_point = Some(point);
        }

        match (acceleration_switching_point, velocity_switching_point) {
            (Some(acc_point), Some(vel_point)) => {
                if acc_point.position.0 <= vel_point.position.0 {
                    Some(acc_point)
                } else {
                    Some(vel_point)
                }
            }
            (Some(acc_point), None) => Some(acc_point),
            (None, Some(vel_point)) => Some(vel_point),
            (None, None) => None,
        }
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

    /// Get the derivative of the max acceleration-bounded velocity at a point along the path
    fn get_max_velocity_from_velocity_derivative(&self, position_along_path: f64) -> f64 {
        let tangent = self.path.get_tangent(position_along_path);
        let mut max_velocity = std::f64::MAX;
        let mut constraint_axis = 0;

        // TODO: Use iterators
        for i in 0..self.velocity_limit.len() {
            let component_velocity = self.velocity_limit[i] / tangent[i].abs();

            if component_velocity < max_velocity {
                max_velocity = component_velocity;
                constraint_axis = i;
            }
        }

        -(self.velocity_limit[constraint_axis]
            * self.path.get_curvature(position_along_path)[constraint_axis]
            / (tangent[constraint_axis].powi(2)))
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

    /// Get the minimum or maximum phase slope for a position along the path
    ///
    /// TODO: Figure out what phase slope means in this context and give it a better name
    fn get_min_max_phase_slope(&self, pos_vel: &PositionAndVelocity, min_max: MinMax) -> f64 {
        self.get_min_max_path_acceleration(&pos_vel, min_max) / pos_vel.0
    }

    /// Get the next acceleration-bounded switching point after the current position
    fn get_next_acceleration_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut velocity = 0.0;
        let mut current_point = SwitchingPoint::new(position_along_path, Continuity::Continuous);

        // TODO: Use iterators here, infinite loops suck
        loop {
            current_point = self.path.get_next_switching_point(current_point.position);

            if current_point.position > self.path.get_length() - TRAJ_EPSILON {
                break None;
            }

            match current_point.continuity {
                Continuity::Discontinuous => {
                    let before_velocity = self
                        .get_max_velocity_from_acceleration(current_point.position - TRAJ_EPSILON);
                    let after_velocity = self
                        .get_max_velocity_from_acceleration(current_point.position + TRAJ_EPSILON);

                    velocity = before_velocity.min(after_velocity);

                    let before_point =
                        PositionAndVelocity(current_point.position - TRAJ_EPSILON, velocity);
                    let after_point =
                        PositionAndVelocity(current_point.position + TRAJ_EPSILON, velocity);

                    let before_acceleration =
                        self.get_min_max_path_acceleration(&before_point, MinMax::Min);
                    let after_acceleration =
                        self.get_min_max_path_acceleration(&after_point, MinMax::Max);

                    if (before_velocity > after_velocity
                        || self.get_min_max_phase_slope(&before_point, MinMax::Min) > self
                            .get_max_velocity_from_acceleration_derivative(
                                current_point.position - 2.0 * TRAJ_EPSILON,
                            ))
                        && (before_velocity < after_velocity
                            || self.get_min_max_phase_slope(&after_point, MinMax::Max) > self
                                .get_max_velocity_from_acceleration_derivative(
                                    current_point.position + 2.0 * TRAJ_EPSILON,
                                )) {
                        break Some(TrajectorySwitchingPoint {
                            position: PositionAndVelocity(current_point.position, velocity),
                            before_acceleration,
                            after_acceleration,
                        });
                    }
                }
                Continuity::Continuous => {
                    let velocity = self.get_max_velocity_from_acceleration(current_point.position);

                    if self.get_max_velocity_from_acceleration_derivative(
                        current_point.position - TRAJ_EPSILON,
                    ) < 0.0
                        && self.get_max_velocity_from_acceleration_derivative(
                            current_point.position + TRAJ_EPSILON,
                        ) > 0.0
                    {
                        break Some(TrajectorySwitchingPoint {
                            position: PositionAndVelocity(current_point.position, velocity),
                            before_acceleration: 0.0,
                            after_acceleration: 0.0,
                        });
                    }
                }
            }
        }
    }

    // TODO: Benchmark and optimise this method. There are two loops which may be reducable to one
    /// Search along the path for the next velocity switching point after the current position
    ///
    /// This method performs a broad search first, stepping along the path from the current position
    /// in coarse increments. It then binary searches through the interval between two coarse steps
    /// to find the specific switching point to within a more accurate epsilon.
    fn get_next_velocity_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        // Broad phase search step
        let step_size = 0.001;
        let mut position = position_along_path;
        let mut start = false;

        position -= step_size;

        // Broad phase
        // TODO: Iterators
        loop {
            position += step_size;

            if self.get_min_max_phase_slope(
                &PositionAndVelocity(position, self.get_max_velocity_from_velocity(position)),
                MinMax::Min,
            ) >= self.get_max_velocity_from_velocity_derivative(position)
                || position >= self.path.get_length()
                || self.get_min_max_phase_slope(
                    &PositionAndVelocity(position, self.get_max_velocity_from_velocity(position)),
                    MinMax::Min,
                ) <= self.get_max_velocity_from_velocity_derivative(position)
            {
                break;
            }
        }

        // There are no switching points after the current position
        if position >= self.path.get_length() {
            return None;
        }

        // Create an interval to search within to find the actual switching point
        let mut prev_position = position - step_size;
        let mut after_position = position;

        // Binary search through interval to find switching point within an epsilon
        // TODO: Iterators
        while after_position - prev_position > TRAJ_EPSILON {
            position = (prev_position + after_position) / 2.0;

            if self.get_min_max_phase_slope(
                &PositionAndVelocity(position, self.get_max_velocity_from_velocity(position)),
                MinMax::Min,
            ) > self.get_max_velocity_from_velocity_derivative(position)
            {
                prev_position = position
            } else {
                after_position = position
            }
        }

        let before_acceleration = self.get_min_max_path_acceleration(
            &PositionAndVelocity(
                prev_position,
                self.get_max_velocity_from_velocity(prev_position),
            ),
            MinMax::Min,
        );
        let after_acceleration = self.get_min_max_path_acceleration(
            &PositionAndVelocity(
                after_position,
                self.get_max_velocity_from_velocity(after_position),
            ),
            MinMax::Max,
        );

        Some(TrajectorySwitchingPoint {
            before_acceleration,
            after_acceleration,
            position: PositionAndVelocity(
                after_position,
                self.get_max_velocity_from_velocity(after_position),
            ),
        })
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
