use super::{
    limits::{
        max_acceleration_at, max_acceleration_derivative_at, max_velocity_at,
        max_velocity_derivative_at,
    },
    LimitType, MinMax, TrajectoryStep, TrajectorySwitchingPoint,
};
use crate::path::{Continuity, PathItem, PathSwitchingPoint};
use crate::{Path, TrajectoryOptions};
use nalgebra::{
    allocator::{Allocator, SameShapeVectorAllocator},
    DefaultAllocator, DimName,
};

pub struct TrajectorySwitchingPoints<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    path: Path<N>,
    options: TrajectoryOptions<N>,
}

impl<N> TrajectorySwitchingPoints<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    pub fn from_path(path: Path<N>, options: TrajectoryOptions<N>) -> Result<Self, String> {
        Ok(Self { path, options })
    }

    pub fn next_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        Self::internal_rename_me_next_switching_point(
            &self.path,
            position_along_path,
            &self.options,
        )
    }

    #[cfg(test)]
    fn next_velocity_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        Self::internal_rename_me_next_velocity_switching_point(
            &self.path,
            position_along_path,
            &self.options,
        )
    }

    #[cfg(test)]
    fn next_acceleration_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        Self::internal_rename_me_next_acceleration_switching_point(
            &self.path,
            position_along_path,
            &self.options,
        )
    }

    /// Get next switching point along the path, bounded by velocity or acceleration
    fn internal_rename_me_next_switching_point(
        path: &Path<N>,
        position_along_path: f64,
        options: &TrajectoryOptions<N>,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut acceleration_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next acceleration switching point
        while let Some(point) = Self::internal_rename_me_next_acceleration_switching_point(
            path,
            acceleration_switching_point
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
            options,
        ) {
            acceleration_switching_point = Some(point);

            if point.pos.velocity
                <= max_velocity_at(
                    path,
                    point.pos.position,
                    LimitType::Velocity(options.velocity_limit.clone()),
                )
            {
                break;
            }
        }

        trace!(
            "RS next_accel_sw_point (pos_along_path;sw_pos;sw_vel),{},{},{}",
            position_along_path,
            acceleration_switching_point
                .map(|p| p.pos.position)
                .unwrap_or(0.0),
            acceleration_switching_point
                .map(|p| p.pos.velocity)
                .unwrap_or(0.0),
        );

        let mut velocity_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next velocity switching point
        while let Some(point) = Self::internal_rename_me_next_velocity_switching_point(
            path,
            velocity_switching_point
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
            options,
        ) {
            velocity_switching_point = Some(point);

            if point.pos.position
                > acceleration_switching_point
                    .map(|p| p.pos.position)
                    .expect("Acceleration switching point")
                || (point.pos.velocity
                    <= max_velocity_at(
                        &path,
                        point.pos.position - options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                    )
                    && point.pos.velocity
                        <= max_velocity_at(
                            path,
                            point.pos.position + options.epsilon,
                            LimitType::Acceleration(options.acceleration_limit.clone()),
                        ))
            {
                break;
            }
        }

        trace!(
            "RS next_vel_sw_point (pos_along_path;sw_pos;sw_vel),{},{},{}",
            position_along_path,
            velocity_switching_point
                .map(|p| p.pos.position)
                .unwrap_or(0.0),
            velocity_switching_point
                .map(|p| p.pos.velocity)
                .unwrap_or(0.0),
        );

        // Return the next earliest switching point (if any)
        let result = match (acceleration_switching_point, velocity_switching_point) {
            (Some(accel_point), Some(vel_point)) => {
                if accel_point.pos.position <= vel_point.pos.position {
                    Some(accel_point)
                } else {
                    None
                }
            }
            (Some(accel_point), None) => Some(accel_point),
            (None, Some(vel_point)) => Some(vel_point),
            _ => None,
        };

        trace!(
            "RS next_sw_point (pos_along_path;sw_pos;sw_vel;before_accel;after_accel),{},{},{},{},{}",
            position_along_path,
            result.map(|p| p.pos.position).unwrap_or(0.0),
            result.map(|p| p.pos.velocity).unwrap_or(0.0),
            result.map(|p| p.before_acceleration).unwrap_or(0.0),
            result.map(|p| p.after_acceleration).unwrap_or(0.0),
        );

        result
    }

    /// Get the next acceleration-bounded switching point after the current position
    fn internal_rename_me_next_acceleration_switching_point(
        path: &Path<N>,
        position_along_path: f64,
        options: &TrajectoryOptions<N>,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut velocity;
        let mut current_point =
            &PathSwitchingPoint::new(position_along_path, Continuity::Continuous);

        while current_point.position <= path.len() {
            current_point = path.next_switching_point(current_point.position)?;

            match current_point.continuity {
                Continuity::Discontinuous => {
                    let before_velocity = max_velocity_at(
                        path,
                        current_point.position - options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                    );
                    let after_velocity = max_velocity_at(
                        path,
                        current_point.position + options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                    );

                    velocity = before_velocity.min(after_velocity);

                    let before_point =
                        TrajectoryStep::new(current_point.position - options.epsilon, velocity);
                    let after_point =
                        TrajectoryStep::new(current_point.position + options.epsilon, velocity);

                    let before_acceleration =
                        max_acceleration_at(&path, &before_point, MinMax::Min, &options);
                    let after_acceleration =
                        max_acceleration_at(&path, &after_point, MinMax::Max, &options);

                    let before_phase_slope =
                        max_acceleration_derivative_at(&path, &before_point, MinMax::Min, &options);
                    let after_phase_slope =
                        max_acceleration_derivative_at(&path, &after_point, MinMax::Max, &options);

                    let before_max_velocity_deriv = max_velocity_derivative_at(
                        path,
                        current_point.position - 2.0 * options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                        &options,
                    );
                    let after_max_velocity_deriv = max_velocity_derivative_at(
                        path,
                        current_point.position + 2.0 * options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                        &options,
                    );

                    if (before_velocity > after_velocity
                        || before_phase_slope > before_max_velocity_deriv)
                        && (before_velocity < after_velocity
                            || after_phase_slope < after_max_velocity_deriv)
                    {
                        trace!(
                            "RS acc_sw_discont (in_pos;next_pos;next_vel),{},{},{}",
                            position_along_path,
                            current_point.position,
                            velocity
                        );
                        return Some(TrajectorySwitchingPoint {
                            pos: TrajectoryStep::new(current_point.position, velocity),
                            before_acceleration,
                            after_acceleration,
                        });
                    }
                }
                Continuity::Continuous => {
                    let velocity = max_velocity_at(
                        path,
                        current_point.position,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                    );

                    let low_deriv = max_velocity_derivative_at(
                        path,
                        current_point.position - options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                        &options,
                    );
                    let high_deriv = max_velocity_derivative_at(
                        path,
                        current_point.position + options.epsilon,
                        LimitType::Acceleration(options.acceleration_limit.clone()),
                        &options,
                    );

                    if low_deriv < 0.0 && high_deriv > 0.0 {
                        trace!(
                            "RS acc_sw_cont (in_pos;next_pos;next_vel),{},{},{}",
                            position_along_path,
                            current_point.position,
                            velocity
                        );
                        return Some(TrajectorySwitchingPoint {
                            pos: TrajectoryStep::new(current_point.position, velocity),
                            before_acceleration: 0.0,
                            after_acceleration: 0.0,
                        });
                    }
                }
            }
        }

        None
    }

    // TODO: Benchmark and optimise this method. There are two loops which may be reducable to one
    /// Search along the path for the next velocity switching point after the current position
    ///
    /// This method performs a broad search first, stepping along the path from the current position
    /// in coarse increments. It then binary searches through the interval between two coarse steps
    /// to find the specific switching point to within a more accurate epsilon.
    fn internal_rename_me_next_velocity_switching_point(
        path: &Path<N>,
        position_along_path: f64,
        options: &TrajectoryOptions<N>,
    ) -> Option<TrajectorySwitchingPoint> {
        // Broad phase search step
        let step_size = 0.001;
        let accuracy = options.epsilon;
        let mut position = position_along_path - step_size;
        let mut prev_slope = max_acceleration_derivative_at(
            &path,
            &TrajectoryStep::new(
                position,
                max_velocity_at(
                    path,
                    position,
                    LimitType::Velocity(options.velocity_limit.clone()),
                ),
            ),
            MinMax::Min,
            &options,
        );
        let mut prev_deriv = max_velocity_derivative_at(
            path,
            position,
            LimitType::Velocity(options.velocity_limit.clone()),
            &options,
        );;

        // Move along path until a sign change is detected. This defines an interval within which a
        // velocity switching point occurs. Think of the peak or trough of a sawtooth wave.
        // Bisection is used after this broad phase to more accurately determine the position of the
        // local minimum
        while position < path.len() {
            position += step_size;

            let slope = max_acceleration_derivative_at(
                &path,
                &TrajectoryStep::new(
                    position,
                    max_velocity_at(
                        path,
                        position,
                        LimitType::Velocity(options.velocity_limit.clone()),
                    ),
                ),
                MinMax::Min,
                &options,
            );

            let deriv = max_velocity_derivative_at(
                path,
                position,
                LimitType::Velocity(options.velocity_limit.clone()),
                &options,
            );

            if prev_slope >= prev_deriv && slope <= deriv {
                break;
            }

            prev_slope = slope;
            prev_deriv = deriv;
        }

        trace!(
            "RS end_condition (pathPos;path.getLength()),{},{}",
            position,
            path.len()
        );

        if position >= path.len() {
            return None;
        }

        // Create an interval to search within to find the actual switching point
        let mut prev_position = position - step_size;
        let mut after_position = position;

        // Binary search through interval to find switching point within an epsilon
        while after_position - prev_position > accuracy {
            position = (prev_position + after_position) / 2.0;

            if max_acceleration_derivative_at(
                &path,
                &TrajectoryStep::new(
                    position,
                    max_velocity_at(
                        path,
                        position,
                        LimitType::Velocity(options.velocity_limit.clone()),
                    ),
                ),
                MinMax::Min,
                &options,
            ) > max_velocity_derivative_at(
                path,
                position,
                LimitType::Velocity(options.velocity_limit.clone()),
                &options,
            ) {
                prev_position = position
            } else {
                after_position = position
            }
        }

        let after_position = TrajectoryStep::new(
            after_position,
            max_velocity_at(
                path,
                after_position,
                LimitType::Velocity(options.velocity_limit.clone()),
            ),
        );

        let before_acceleration = max_acceleration_at(
            &path,
            &TrajectoryStep::new(
                prev_position,
                max_velocity_at(
                    path,
                    prev_position,
                    LimitType::Velocity(options.velocity_limit.clone()),
                ),
            ),
            MinMax::Min,
            &options,
        );
        let after_acceleration = max_acceleration_at(&path, &after_position, MinMax::Max, &options);

        Some(TrajectorySwitchingPoint {
            before_acceleration,
            after_acceleration,
            pos: after_position,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::PathOptions;
    use crate::test_helpers::TestCoord3;

    #[test]
    fn velocity_switching_points() {
        let expected_points = vec![
            (0.5, 1.0207890624999982),
            (1.02, 1.0207890625),
            (2.0, 3.8633017578122955),
            (3.8616, 3.8633011718750003),
            (4.0, 5.433721679687979),
            (5.431, 5.433721679687501),
            (7.1, 7.43147363281261),
            (7.431, 7.4314736328125),
            (8.0, 8.845047851562033),
            (8.843, 8.845047851562498),
            (8.845, 8.845047851562502),
        ];

        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];

        let path = Path::from_waypoints(
            &waypoints,
            PathOptions {
                max_deviation: 0.001,
            },
        );

        // Same epsilon as Example.cpp for equal comparison
        let traj = TrajectorySwitchingPoints::from_path(
            path,
            TrajectoryOptions {
                velocity_limit: TestCoord3::repeat(1.0),
                acceleration_limit: TestCoord3::repeat(1.0),
                epsilon: 0.000001,
                timestep: 0.001,
            },
        )
        .unwrap();

        for (pos, next_point) in expected_points {
            assert_eq!(
                traj.next_velocity_switching_point(pos)
                    .map(|p| p.pos.position),
                Some(next_point),
                "Expected position {} to have next switching point {}",
                pos,
                next_point
            );
        }
    }

    #[test]
    fn acceleration_switching_points() {
        let expected_points = vec![
            (0.5, 1.0173539279271488),
            (1.02, 1.0207890617732325),
            (2.0, 3.8614234182834446),
            (3.8616, 3.8626971078471364),
            (4.0, 5.4258429817965856),
            (5.431, 5.4332575268899799),
            (7.1, 7.4304355740660952),
            (7.431, 7.4314735160725895),
            (8.0, 8.8429532035794889),
            (8.843, 8.8440004011306854),
            (8.845, 8.845047598681882),
        ];

        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];

        let path = Path::from_waypoints(
            &waypoints,
            PathOptions {
                max_deviation: 0.001,
            },
        );

        // Same epsilon as Example.cpp for equal comparison
        let traj = TrajectorySwitchingPoints::from_path(
            path,
            TrajectoryOptions {
                velocity_limit: TestCoord3::repeat(1.0),
                acceleration_limit: TestCoord3::repeat(1.0),
                epsilon: 0.000001,
                timestep: 0.001,
            },
        )
        .unwrap();

        for (pos, next_point) in expected_points {
            assert_eq!(
                traj.next_acceleration_switching_point(pos)
                    .map(|p| p.pos.position),
                Some(next_point)
            );
        }
    }
}
