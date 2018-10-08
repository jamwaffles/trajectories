mod limit;
mod min_max;
mod path_position;
mod switching_point;
mod trajectory_step;

use self::limit::Limit;
use self::min_max::MinMax;
use self::path_position::PathPosition;
use self::switching_point::SwitchingPoint as TrajectorySwitchingPoint;
use self::trajectory_step::TrajectoryStep;
use crate::path::{Continuity, Path, PathItem, SwitchingPoint};
use crate::Coord;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;
use std;

/// Motion trajectory
#[derive(Debug)]
pub struct Trajectory<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    path: Path<N>,
    velocity_limit: Coord<N>,
    acceleration_limit: Coord<N>,
    timestep: f64,
    trajectory: Vec<TrajectoryStep>,
    epsilon: f64,
}

impl<N> Trajectory<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
{
    /// Create a new trajectory from a given path and max velocity and acceleration
    pub fn new(
        path: Path<N>,
        velocity_limit: Coord<N>,
        acceleration_limit: Coord<N>,
        epsilon: f64,
        timestep: f64,
    ) -> Self {
        let mut traj = Self {
            path,
            velocity_limit,
            acceleration_limit,
            timestep,
            trajectory: Vec::new(),
            epsilon,
        };

        traj.setup();

        traj
    }

    /// Get duration of complete trajectory
    pub fn get_duration(&self) -> f64 {
        self.trajectory
            .last()
            .expect("Could not get duration of empty trajectory")
            .time
    }

    /// Get a position in n-dimensional space given a time along the trajectory
    pub fn get_position(&self, time: f64) -> Coord<N> {
        let (previous, current) = self.get_trajectory_segment(time);

        let mut segment_len = current.time - previous.time;
        let acceleration = 2.0
            * (current.position - previous.position - segment_len * previous.velocity)
            / segment_len.powi(2);

        segment_len = time - previous.time;

        let position = previous.position
            + segment_len * previous.velocity * 0.5 * segment_len.powi(2) * acceleration;

        self.path.get_position(position)
    }

    /// Get velocity for each joint at a time along the path
    pub fn get_velocity(&self, time: f64) -> Coord<N> {
        let (previous, current) = self.get_trajectory_segment(time);

        let segment_len = current.time - previous.time;
        let acceleration = 2.0
            * (current.position - previous.position - segment_len * previous.velocity)
            / segment_len.powi(2);

        let position = previous.position
            + segment_len * previous.velocity * 0.5 * segment_len.powi(2) * acceleration;
        let velocity = previous.velocity + segment_len * acceleration;

        self.path.get_tangent(position) * velocity
    }

    /// Get the (previous_segment, segment) of the trajectory that the given time lies on
    fn get_trajectory_segment<'a>(&'a self, time: f64) -> (&'a TrajectoryStep, &'a TrajectoryStep) {
        let traj_len = self.trajectory.len();

        let pos = self
            .trajectory
            .iter()
            .rev()
            .position(|segment| segment.time <= time)
            // Iter is reversed, so munge index-from-end to index-from-start
            .map(|pos| traj_len - pos - 1)
            .expect("Get segment position")
            .max(1);

        let prev = self.trajectory.get(pos - 1).unwrap_or(
            self.trajectory
                .first()
                .expect("Cannot get segment of empty trajectory"),
        );
        let current = self
            .trajectory
            .get(pos)
            .expect("Segment position is invalid");

        (prev, current)
    }

    /// Compute complete trajectory
    fn setup(&mut self) {
        let mut trajectory = vec![TrajectoryStep::new(0.0, 0.0)];
        let mut switching_point = TrajectorySwitchingPoint {
            before_acceleration: 0.0,
            after_acceleration: self
                .get_acceleration_at(&TrajectoryStep::new(0.0, 0.0), MinMax::Max),
            pos: TrajectoryStep::new(0.0, 0.0),
        };

        loop {
            trace!("Setup loop");

            let (fwd, pos) =
                self.integrate_forward(&trajectory, switching_point.after_acceleration);

            trajectory.extend(fwd);

            trace!("Setup loop 2");

            if pos == PathPosition::End {
                break;
            }

            if let Some(new_switching_point) = self.get_next_switching_point(
                trajectory
                    .last()
                    .expect("Setup has empty trajectory")
                    .position,
            ) {
                switching_point = new_switching_point;
            } else {
                // Break if we've reached the end of the path
                break;
            }

            trace!("Setup loop 3");

            if let Some(updated_traj) = self.integrate_backward(&trajectory, &switching_point) {
                trajectory = updated_traj;
            }

            trace!("Setup loop 4");
        }

        // Backwards integrate last section
        if let Some(updated_traj) = self.integrate_backward(
            &trajectory,
            &TrajectorySwitchingPoint {
                pos: TrajectoryStep::new(self.path.get_length(), 0.0),
                before_acceleration: self.get_acceleration_at(
                    &TrajectoryStep::new(self.path.get_length(), 0.0),
                    MinMax::Min,
                ),
                after_acceleration: 0.0,
            },
        ) {
            trajectory = updated_traj;
        } else {
            panic!("Last section integrate backward failed");
        }

        // Set times on segments
        let timed = std::iter::once(TrajectoryStep::new(0.0, 0.0).with_time(0.0))
            .chain(trajectory.windows(2).scan(0.0, |t, parts| {
                if let &[ref previous, ref current] = parts {
                    *t += (current.position - previous.position)
                        / ((current.velocity + previous.velocity) / 2.0);

                    Some(current.clone().with_time(*t))
                } else {
                    panic!("Time windows");
                }
            }))
            .collect::<Vec<TrajectoryStep>>();

        self.trajectory = timed;
    }

    fn integrate_forward(
        &self,
        trajectory: &Vec<TrajectoryStep>,
        start_acceleration: f64,
    ) -> (Vec<TrajectoryStep>, PathPosition) {
        let mut new_points = Vec::new();
        let TrajectoryStep {
            mut position,
            mut velocity,
            ..
        } = trajectory.last().expect("Empty traj");
        let mut acceleration = start_acceleration;

        loop {
            trace!(
                "Integrate forward loop, by {}, position {} out of {}",
                self.timestep,
                position,
                self.path.get_length()
            );

            let next_discontinuity =
                self.path
                    .get_switching_points()
                    .iter()
                    .find(|switching_point| {
                        switching_point.position > position
                            && switching_point.continuity == Continuity::Discontinuous
                    });

            let old_position = position;
            let old_velocity = velocity;

            velocity += self.timestep * acceleration;
            position += self.timestep * 0.5 * (old_velocity + velocity);

            if let Some(next_disc) = next_discontinuity {
                if position > next_disc.position {
                    velocity = old_velocity
                        + (next_disc.position - old_position) * (velocity - old_velocity)
                            / (position - old_position);
                    position = next_disc.position;
                }
            }

            if position > self.path.get_length() {
                new_points.push(TrajectoryStep::new(position, velocity));

                break (new_points, PathPosition::End);
            } else if velocity < 0.0 {
                panic!("Integrate forward velocity cannot be 0");
            }

            let max_velocity_at_position = self.max_velocity_at(position, Limit::Velocity);

            if velocity > max_velocity_at_position
                && self.get_phase_slope(
                    &TrajectoryStep::new(
                        old_position,
                        self.max_velocity_at(old_position, Limit::Velocity),
                    ),
                    MinMax::Min,
                ) <= self.max_velocity_derivative_at(old_position, Limit::Velocity)
            {
                velocity = max_velocity_at_position;
            }

            let new_point = TrajectoryStep::new(position, velocity);

            new_points.push(new_point.clone());

            acceleration = self.get_acceleration_at(&new_point, MinMax::Max);

            if velocity > self.max_velocity_at(position, Limit::Acceleration)
                || velocity > max_velocity_at_position
            {
                let overshoot = new_points.pop().expect("No overshoot available");
                let last_point = new_points
                    .last()
                    .unwrap_or(
                        trajectory
                            .last()
                            .expect("Could not get last point of empty trajectory"),
                    )
                    .clone();

                let mut before = last_point.position;
                let mut before_velocity = last_point.velocity;
                let mut after = overshoot.position;
                let mut after_velocity = overshoot.velocity;
                let mut midpoint;
                let mut midpoint_velocity;

                while after - before > self.epsilon {
                    trace!(
                        "Integrate forward bisection, before {} after {}",
                        before,
                        after
                    );
                    midpoint = 0.5 * (before + after);
                    midpoint_velocity = 0.5 * (before_velocity + after_velocity);

                    let max_midpoint_velocity = self.max_velocity_at(midpoint, Limit::Velocity);

                    if midpoint_velocity > max_midpoint_velocity
                        && self.get_phase_slope(
                            &TrajectoryStep::new(
                                before,
                                self.max_velocity_at(before, Limit::Velocity),
                            ),
                            MinMax::Min,
                        ) <= self.max_velocity_derivative_at(before, Limit::Velocity)
                    {
                        midpoint_velocity = max_midpoint_velocity;
                    }

                    if midpoint_velocity > self.max_velocity_at(midpoint, Limit::Acceleration)
                        || midpoint_velocity > max_midpoint_velocity
                    {
                        after = midpoint;
                        after_velocity = midpoint_velocity;
                    } else {
                        before = midpoint;
                        before_velocity = midpoint_velocity;
                    }
                }

                let new_point = TrajectoryStep::new(before, before_velocity);
                new_points.push(new_point.clone());

                if self.max_velocity_at(after, Limit::Acceleration)
                    < self.max_velocity_at(after, Limit::Velocity)
                {
                    if next_discontinuity.is_some()
                        && after > next_discontinuity.expect("No next discontinuity").position
                    {
                        break (new_points, PathPosition::NotEnd);
                    }

                    if self.get_phase_slope(&new_point, MinMax::Max)
                        > self.max_velocity_derivative_at(new_point.position, Limit::Acceleration)
                    {
                        break (new_points, PathPosition::NotEnd);
                    }
                } else if self.get_phase_slope(&new_point, MinMax::Min)
                    > self.max_velocity_derivative_at(new_point.position, Limit::Velocity)
                {
                    break (new_points, PathPosition::NotEnd);
                }
            }
        }
    }

    fn integrate_backward(
        &self,
        start_trajectory: &Vec<TrajectoryStep>,
        start_switching_point: &TrajectorySwitchingPoint,
    ) -> Option<Vec<TrajectoryStep>> {
        let TrajectorySwitchingPoint {
            pos:
                TrajectoryStep {
                    mut position,
                    mut velocity,
                    ..
                },
            mut before_acceleration,
            ..
        } = start_switching_point.clone();
        let mut slope = 0.0;
        let mut it = start_trajectory.windows(2).rev();
        let mut new_trajectory: Vec<TrajectoryStep> = Vec::new();
        let mut parts = it.next();

        while let Some(&[ref start1, ref _start2]) = parts {
            trace!("Integrate backward loop, position {}", position);

            if position < 0.0 {
                break;
            }

            if start1.position <= position {
                let new_point = TrajectoryStep::new(position, velocity);

                velocity -= self.timestep * before_acceleration;
                position -= self.timestep * 0.5 * (velocity + new_point.velocity);
                before_acceleration =
                    self.get_acceleration_at(&TrajectoryStep::new(position, velocity), MinMax::Min);
                slope = (new_point.velocity - velocity) / (new_point.position - position);

                new_trajectory.push(new_point);

                if velocity < 0.0 {
                    panic!("Velocity cannot be less than zero");
                }
            } else {
                parts = it.next();
            }

            // If let here due to the `parts = it.next()` above
            // TODO: Refactor so I don't have to do this
            if let Some(&[ref start1, ref start2]) = parts {
                let start_slope =
                    (start2.velocity - start1.velocity) / (start2.position - start1.position);

                // Normalised position along segment where intersection occurs
                let intersection_position = (start1.velocity - velocity + slope * position
                    - start_slope * start1.position)
                    / (slope - start_slope);

                // Check for intersection between path and current segment
                if start1.position.max(position) - self.epsilon <= intersection_position
                    && intersection_position <= self.epsilon + start2.position.min(
                        new_trajectory
                            .last()
                            .expect("Integrate backwards cannot get last point of empty trajectory")
                            .position,
                    ) {
                    let intersection_velocity =
                        start1.velocity + start_slope * (intersection_position - start1.position);

                    // Add intersection point
                    new_trajectory.push(TrajectoryStep::new(
                        intersection_position,
                        intersection_velocity,
                    ));

                    let ret = start_trajectory
                        .into_iter()
                        .cloned()
                        // Remove items in current trajectory after intersection point
                        .filter(|step| step.position < start2.position)
                        // Append new items
                        .chain(new_trajectory.into_iter().rev())
                        .collect::<Vec<TrajectoryStep>>();

                    return Some(ret);
                }
            }
        }

        panic!("Path is invalid: Integrate backwards did not hit start trajectory");
    }

    /// Get next switching point along the path, bounded by velocity or acceleration
    fn get_next_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut acceleration_switching_point: Option<TrajectorySwitchingPoint> =
            Some(TrajectorySwitchingPoint {
                pos: TrajectoryStep::new(position_along_path, 0.0),
                ..TrajectorySwitchingPoint::default()
            });

        // Find the next acceleration switching point
        while let Some(point) = self.get_next_acceleration_switching_point(
            acceleration_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
        ) {
            trace!(
                "Get accel point pos {} vel {} from len {}, max_vel {}",
                point.pos.position,
                point.pos.velocity,
                self.path.get_length(),
                self.max_velocity_at(point.pos.position, Limit::Velocity)
            );
            acceleration_switching_point = Some(point.clone());

            if point.pos.velocity <= self.max_velocity_at(point.pos.position, Limit::Velocity) {
                break;
            }
        }

        let mut velocity_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next velocity switching point
        while let Some(point) = self.get_next_velocity_switching_point(
            velocity_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
        ) {
            trace!("Get vel point pos {}", point.pos.position);
            velocity_switching_point = Some(point.clone());

            if point.pos.position > acceleration_switching_point
                .clone()
                .expect("Accel switching point")
                .pos
                .position
                || (point.pos.velocity
                    <= self.max_velocity_at(point.pos.position - self.epsilon, Limit::Acceleration)
                    && point.pos.velocity <= self
                        .max_velocity_at(point.pos.position + self.epsilon, Limit::Acceleration))
            {
                break;
            }
        }

        // Return the next earliest switching point (if any)
        match (acceleration_switching_point, velocity_switching_point) {
            (Some(ref accel_point), Some(ref vel_point))
                if accel_point.pos.position <= vel_point.pos.position =>
            {
                Some(accel_point.clone())
            }
            (Some(accel_point), None) => Some(accel_point),
            (None, Some(vel_point)) => Some(vel_point),
            _ => None,
        }
    }

    /// Find minimum or maximum acceleration at a point along path
    fn get_acceleration_at(&self, pos_vel: &TrajectoryStep, min_max: MinMax) -> f64 {
        let &TrajectoryStep {
            position, velocity, ..
        } = pos_vel;

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

    /// Find the maximum allowable velocity at a point, limited by either max acceleration or max
    /// velocity.
    fn max_velocity_at(&self, position_along_path: f64, limit: Limit) -> f64 {
        match limit {
            Limit::Velocity => self.get_max_velocity_from_velocity(position_along_path),
            Limit::Acceleration => self.get_max_velocity_from_acceleration(position_along_path),
        }
    }

    fn get_max_velocity_from_velocity(&self, position_along_path: f64) -> f64 {
        self.velocity_limit
            .component_div(&self.path.get_tangent(position_along_path))
            .amin()
    }

    /// Find maximum allowable velocity as limited by the acceleration at a point on the path
    fn get_max_velocity_from_acceleration(&self, position_along_path: f64) -> f64 {
        let segment = self.path.get_segment_at_position(position_along_path);
        let velocity = segment.get_tangent(position_along_path);
        let acceleration = segment.get_curvature(position_along_path);

        let n = nalgebra::dimension::<Coord<N>>();

        let mut max_path_velocity = std::f64::INFINITY;

        for i in 0..n {
            if velocity[i] != 0.0 {
                for j in (i + 1)..n {
                    if velocity[j] != 0.0 {
                        // TODO: Come up with a less mathsy name
                        let a_ij = acceleration[i] / velocity[i] - acceleration[j] / velocity[j];

                        if a_ij != 0.0 {
                            max_path_velocity = max_path_velocity.min(
                                ((self.acceleration_limit[i] / velocity[i].abs()
                                    + self.acceleration_limit[j] / velocity[j].abs())
                                    / a_ij.abs())
                                .sqrt(),
                            );
                        }
                    }
                }
            } else if acceleration[i] != 0.0 {
                max_path_velocity = max_path_velocity
                    .min((self.acceleration_limit[i] / acceleration[i].abs()).sqrt());
            }
        }

        max_path_velocity
    }

    fn max_velocity_derivative_at(&self, position_along_path: f64, limit: Limit) -> f64 {
        match limit {
            Limit::Velocity => self.get_max_velocity_from_velocity_derivative(position_along_path),
            Limit::Acceleration => {
                self.get_max_velocity_from_acceleration_derivative(position_along_path)
            }
        }
    }

    /// Get the derivative of the max acceleration-bounded velocity at a point along the path
    fn get_max_velocity_from_velocity_derivative(&self, position_along_path: f64) -> f64 {
        let tangent = self.path.get_tangent(position_along_path);
        let mut max_velocity = std::f64::MAX;
        let mut constraint_axis = 0;

        // TODO: Use iterators
        for i in 0..nalgebra::dimension::<Coord<N>>() {
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

    /// Get the derivative of the max velocity at a point along the path
    ///
    /// The max velocity in this case is bounded by the acceleration limits at the point
    fn get_max_velocity_from_acceleration_derivative(&self, position_along_path: f64) -> f64 {
        (self.max_velocity_at(position_along_path + self.epsilon, Limit::Acceleration)
            - self.max_velocity_at(position_along_path - self.epsilon, Limit::Acceleration))
            / (2.0 * self.epsilon)
    }

    /// Get the minimum or maximum phase slope for a position along the path
    ///
    /// TODO: Figure out what phase slope means in this context and give it a better name
    fn get_phase_slope(&self, pos_vel: &TrajectoryStep, min_max: MinMax) -> f64 {
        self.get_acceleration_at(&pos_vel, min_max) / pos_vel.position
    }

    /// Get the next acceleration-bounded switching point after the current position
    fn get_next_acceleration_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut velocity;
        let mut current_point = SwitchingPoint::new(position_along_path, Continuity::Continuous);

        while current_point.position <= self.path.get_length() - self.epsilon {
            current_point = self.path.get_next_switching_point(current_point.position);

            match current_point.continuity {
                Continuity::Discontinuous => {
                    let before_velocity = self.max_velocity_at(
                        current_point.position - self.epsilon,
                        Limit::Acceleration,
                    );
                    let after_velocity = self.max_velocity_at(
                        current_point.position + self.epsilon,
                        Limit::Acceleration,
                    );

                    velocity = before_velocity.min(after_velocity);

                    let before_point =
                        TrajectoryStep::new(current_point.position - self.epsilon, velocity);
                    let after_point =
                        TrajectoryStep::new(current_point.position + self.epsilon, velocity);

                    let before_acceleration = self.get_acceleration_at(&before_point, MinMax::Min);
                    let after_acceleration = self.get_acceleration_at(&after_point, MinMax::Max);

                    let before_phase_slope = self.get_phase_slope(&before_point, MinMax::Min);
                    let after_phase_slope = self.get_phase_slope(&after_point, MinMax::Max);

                    let before_max_velocity_deriv = self.max_velocity_derivative_at(
                        current_point.position - 2.0 * self.epsilon,
                        Limit::Acceleration,
                    );
                    let after_max_velocity_deriv = self.max_velocity_derivative_at(
                        current_point.position + 2.0 * self.epsilon,
                        Limit::Acceleration,
                    );

                    if (before_velocity > after_velocity
                        || before_phase_slope > before_max_velocity_deriv)
                        && (before_velocity < after_velocity
                            || after_phase_slope < after_max_velocity_deriv)
                    {
                        return Some(TrajectorySwitchingPoint {
                            pos: TrajectoryStep::new(current_point.position, velocity),
                            before_acceleration,
                            after_acceleration,
                        });
                    }
                }
                Continuity::Continuous => {
                    let velocity =
                        self.max_velocity_at(current_point.position, Limit::Acceleration);

                    let low_deriv = self.max_velocity_derivative_at(
                        current_point.position - self.epsilon,
                        Limit::Acceleration,
                    );
                    let high_deriv = self.max_velocity_derivative_at(
                        current_point.position + self.epsilon,
                        Limit::Acceleration,
                    );

                    if low_deriv < 0.0 && high_deriv > 0.0 {
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
    fn get_next_velocity_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        // Broad phase search step
        let step_size = 0.001;
        let mut position = position_along_path;
        let mut start = true;

        // Broad phase - find two adjacent points that exceed velocity limits
        // TODO: Refactor `start = true` weirdness whilst maintaining speed. For some reason, moving
        // the first condition into the second massively slows down the algo.
        while {
            let slope = self.get_phase_slope(
                &TrajectoryStep::new(position, self.max_velocity_at(position, Limit::Velocity)),
                MinMax::Min,
            );
            let deriv = self.max_velocity_derivative_at(position, Limit::Velocity);

            if slope >= deriv {
                start = false;
            }

            (start == true || slope >= deriv) && position < self.path.get_length()
        } {
            position += step_size;
        }

        if position >= self.path.get_length() {
            return None;
        }

        // Create an interval to search within to find the actual switching point
        let mut prev_position = position - step_size;
        let mut after_position = position;

        // Binary search through interval to find switching point within an epsilon
        while after_position - prev_position > self.epsilon {
            position = (prev_position + after_position) / 2.0;

            if self.get_phase_slope(
                &TrajectoryStep::new(position, self.max_velocity_at(position, Limit::Velocity)),
                MinMax::Min,
            ) > self.max_velocity_derivative_at(position, Limit::Velocity)
            {
                prev_position = position
            } else {
                after_position = position
            }
        }

        let after_position = TrajectoryStep::new(
            after_position,
            self.max_velocity_at(after_position, Limit::Velocity),
        );

        let before_acceleration = self.get_acceleration_at(
            &TrajectoryStep::new(
                prev_position,
                self.max_velocity_at(prev_position, Limit::Velocity),
            ),
            MinMax::Min,
        );
        let after_acceleration = self.get_acceleration_at(&after_position, MinMax::Max);

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
    use crate::test_helpers::*;

    #[test]
    fn velocity_switching_points() {
        let expected_points = vec![
            (0.5, 1.0207890624999982),
            (1.02, 1.0207890625),
            (2.0, 5.433721679687759),
            (3.8616, 3.862918359375),
            (4.0, 5.433721679687979),
            (5.431, 5.433721679687501),
            (7.1, 7.432009765625111),
            (7.431, 7.432009765625001),
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

        // Same epsilon as Example.cpp for equal comparison
        let traj = Trajectory::new(
            Path::from_waypoints(&waypoints, 0.001),
            TestCoord3::repeat(1.0),
            TestCoord3::repeat(1.0),
            0.000001,
            0.001,
        );

        for (pos, next_point) in expected_points {
            assert_eq!(
                traj.get_next_velocity_switching_point(pos)
                    .map(|p| p.pos.position),
                Some(next_point)
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

        // Same epsilon as Example.cpp for equal comparison
        let traj = Trajectory::new(
            Path::from_waypoints(&waypoints, 0.001),
            TestCoord3::repeat(1.0),
            TestCoord3::repeat(1.0),
            0.000001,
            0.001,
        );

        for (pos, next_point) in expected_points {
            assert_eq!(
                traj.get_next_acceleration_switching_point(pos)
                    .map(|p| p.pos.position),
                Some(next_point)
            );
        }
    }

    #[test]
    fn create_example_cpp_trajectory() {
        let waypoints: Vec<TestCoord3> = vec![
            TestCoord3::new(0.0, 0.0, 0.0),
            TestCoord3::new(0.0, 0.2, 1.0),
            TestCoord3::new(0.0, 3.0, 0.5),
            TestCoord3::new(1.1, 2.0, 0.0),
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(0.0, 1.0, 0.0),
            TestCoord3::new(0.0, 0.0, 1.0),
        ];
        let mut rows = Vec::new();

        let path = Path::from_waypoints(&waypoints, 0.001);

        // Same epsilon as Example.cpp for equal comparison
        let traj = Trajectory::new(
            path,
            TestCoord3::repeat(1.0),
            TestCoord3::repeat(1.0),
            0.000001,
            0.001,
        );

        let mut t = 0.0;
        let duration = traj.get_duration();

        while t < duration {
            let p = traj.get_position(t);
            let v = traj.get_velocity(t);

            rows.push(TrajectoryStepRow::from_coords(t, &p, &v));

            t += 0.1;
        }

        let p_final = traj.get_position(duration);
        let v_final = traj.get_velocity(duration);

        rows.push(TrajectoryStepRow::from_coords(duration, &p_final, &v_final));

        write_debug_csv("../target/plot_native.csv".into(), &rows);

        assert_eq!(traj.trajectory.len(), 14814);
        assert_near!(duration, 14.802832847319937);
    }
}
