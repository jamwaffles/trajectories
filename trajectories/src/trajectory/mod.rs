mod limit;
mod min_max;
mod path_position;
mod trajectory_options;
mod trajectory_step;
mod trajectory_switching_point;

use self::limit::Limit;
use self::min_max::MinMax;
use self::path_position::PathPosition;
pub use self::trajectory_options::TrajectoryOptions;
use self::trajectory_step::TrajectoryStep;
use self::trajectory_switching_point::TrajectorySwitchingPoint;
use crate::path::{Continuity, Path, PathItem, PathSwitchingPoint};
use crate::Coord;
use nalgebra::allocator::Allocator;
use nalgebra::allocator::SameShapeVectorAllocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;
use std;
use std::time::Instant;

/// Motion trajectory
#[derive(Debug)]
pub struct Trajectory<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    path: &'a Path<N>,
    velocity_limit: Coord<N>,
    acceleration_limit: Coord<N>,
    timestep: f64,
    trajectory: Vec<TrajectoryStep>,
    epsilon: f64,
}

impl<'a, N> Trajectory<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    // TODO: Stop panicking all over the place and actually use the error arm of this `Result`
    /// Create a new trajectory from a given path and max velocity and acceleration
    pub fn new(path: &'a Path<N>, options: TrajectoryOptions<N>) -> Result<Self, String> {
        let TrajectoryOptions {
            velocity_limit,
            acceleration_limit,
            timestep,
            epsilon,
        } = options;

        let mut traj = Self {
            path,
            velocity_limit,
            acceleration_limit,
            timestep,
            trajectory: Vec::new(),
            epsilon,
        };

        let start = Instant::now();

        traj.setup()?;

        info!(
            "Created trajectory with {} segments in {} ms",
            traj.trajectory.len(),
            start.elapsed().as_millis()
        );

        Ok(traj)
    }

    /// Get duration of complete trajectory
    pub fn duration(&self) -> f64 {
        self.trajectory.last().map(|step| step.time).unwrap_or(0.0)
    }

    /// Get a position in n-dimensional space given a time along the trajectory
    pub fn position(&self, time: f64) -> Coord<N> {
        let (previous, current) = self.trajectory_segment(time);

        trace!(
            "RS get_pos (time;prev_pos;prev_vel;curr_pos;curr_vel),{},{},{},{},{}",
            time,
            previous.position,
            previous.velocity,
            current.position,
            current.velocity
        );

        let duration = current.time - previous.time;
        let acceleration = 2.0
            * (current.position - previous.position - duration * previous.velocity)
            / duration.powi(2);

        let duration = time - previous.time;

        let position = previous.position
            + duration * previous.velocity
            + 0.5 * duration.powi(2) * acceleration;

        self.path.position(position)
    }

    /// Get velocity for each joint at a time along the path
    pub fn velocity(&self, time: f64) -> Coord<N> {
        let (previous, current) = self.trajectory_segment(time);

        let duration = current.time - previous.time;
        let acceleration = 2.0
            * (current.position - previous.position - duration * previous.velocity)
            / duration.powi(2);

        let duration = time - previous.time;

        let position = previous.position
            + duration * previous.velocity
            + 0.5 * duration.powi(2) * acceleration;

        let velocity = previous.velocity + duration * acceleration;

        self.path.tangent(position) * velocity
    }

    /// Get a reference to the generated trajectory
    pub fn trajectory(&self) -> &Vec<TrajectoryStep> {
        &self.trajectory
    }

    // TODO: Return an Option
    /// Get the (previous_segment, segment) of the trajectory that the given time lies on
    ///
    /// This gets an interval of the trajectory along which `time` lies. Other methods interpolate
    /// along this interval and do things like find the exact position at the given time.
    fn trajectory_segment(&self, time: f64) -> (&TrajectoryStep, &TrajectoryStep) {
        // If there is only one segment, create a "window" over just itself
        // TODO: Gracefully handle case where trajectory is empty
        if self.trajectory.len() < 2 {
            return (&self.trajectory[0], &self.trajectory[0]);
        }

        self.trajectory
            .windows(2)
            .find_map(|window| {
                if let [ref prev, ref curr] = window {
                    if time >= prev.time && time <= curr.time {
                        Some((prev, curr))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .unwrap_or_else(|| {
                panic!(
                    "Expected to find traj segment for time {} (total time {:?})",
                    time,
                    self.trajectory.last().map(|s| s.time)
                )
            })
    }

    /// Compute complete trajectory
    fn setup(&mut self) -> Result<(), String> {
        let mut trajectory = vec![TrajectoryStep::new(0.0, 0.0)];
        let mut switching_point = TrajectorySwitchingPoint {
            before_acceleration: 0.0,
            after_acceleration: self.acceleration_at(&TrajectoryStep::new(0.0, 0.0), MinMax::Max),
            pos: TrajectoryStep::new(0.0, 0.0),
        };

        let mut dbg_iter = 0;

        loop {
            debug!("--- ");
            debug!("Setup loop, iter {}", dbg_iter);

            let (fwd, is_end, stop_position) =
                self.integrate_forward(&trajectory, switching_point.after_acceleration)?;

            debug!(
                "RS integ_fwd_end_step (pos;vel),{},{}",
                fwd.last().unwrap().position,
                fwd.last().unwrap().velocity,
            );

            trajectory.extend(fwd);

            if is_end == PathPosition::End {
                break;
            }

            if let Some(new_switching_point) = self.next_switching_point(stop_position) {
                debug!(
                    "Switching point {:?} from stop_position {}",
                    new_switching_point, stop_position
                );

                trace!(
                    "RS forward_sw_point (pos;vel;beforeAccel;afterAccel),{},{},{},{}",
                    switching_point.pos.position,
                    switching_point.pos.velocity,
                    switching_point.before_acceleration,
                    switching_point.after_acceleration
                );

                switching_point = new_switching_point;
            } else {
                // Break if we've reached the end of the path
                break;
            }

            debug!("Setup loop 3, iter {}", dbg_iter);

            let (splice_index, updated_traj) =
                self.integrate_backward(&trajectory, &switching_point)?;

            trace!(
                "Splice {} on len {} with {} new items",
                splice_index,
                trajectory.len(),
                updated_traj.len()
            );

            let _ = trajectory.split_off(splice_index);
            trajectory.extend(updated_traj);

            debug!("Setup loop 4, iter {}", dbg_iter);

            dbg_iter += 1;
        }

        // Backwards integrate last section
        let (splice_index, updated_traj) = self
            .integrate_backward(
                &trajectory,
                &TrajectorySwitchingPoint {
                    pos: TrajectoryStep::new(self.path.len(), 0.0),
                    before_acceleration: self
                        .acceleration_at(&TrajectoryStep::new(self.path.len(), 0.0), MinMax::Min),
                    after_acceleration: 0.0,
                },
            )
            .map_err(|e| format!("Last section integrate backwards failed: {}", e))?;

        let _ = trajectory.split_off(splice_index);
        trajectory.extend(updated_traj);

        // Set times on segments
        // TODO: Stop doing this weird iter::once stuff. Instead, calculate all the positions
        // properly in the windows()/scan() part
        let timed = std::iter::once(*trajectory.first().unwrap())
            .chain(trajectory.windows(2).scan(0.0, |t, parts| {
                if let [previous, current] = parts {
                    *t += (current.position - previous.position)
                        / ((current.velocity + previous.velocity) / 2.0);

                    Some(current.with_time(*t))
                } else {
                    panic!("Time windows");
                }
            }))
            .collect::<Vec<TrajectoryStep>>();

        self.trajectory = timed;

        Ok(())
    }

    /// Integrate forward returning:
    ///
    /// * A new trajectory segment to append
    /// * A flag indicating whether the end of the path was reached
    /// * The new position from which to continue processing
    fn integrate_forward(
        &self,
        trajectory: &[TrajectoryStep],
        start_acceleration: f64,
    ) -> Result<(Vec<TrajectoryStep>, PathPosition, f64), String> {
        let mut new_points = Vec::new();
        let last = trajectory.last().ok_or(String::from(
            "Attempted to get last element of empty trajectory",
        ))?;
        let TrajectoryStep {
            mut position,
            mut velocity,
            ..
        } = last;
        let mut acceleration = start_acceleration;

        loop {
            trace!(
                "Integrate forward loop, by {}, position {} out of {}",
                self.timestep,
                position,
                self.path.len()
            );

            // TODO: Keep next_discontinuity iterator cached for perf
            let next_discontinuity = self.path.switching_points().iter().find(|switching_point| {
                switching_point.position > position
                    && switching_point.continuity == Continuity::Discontinuous
            });

            let old_position = position;
            let old_velocity = velocity;

            velocity += self.timestep * acceleration;
            position += self.timestep * 0.5 * (old_velocity + velocity);

            // If we've overstepped the next found discontinuity, move backwards to the position of
            // the discontinuity and calculate its velocity at that point
            if let Some(next_discontinuity) = next_discontinuity {
                if position > next_discontinuity.position {
                    velocity = old_velocity
                        + (next_discontinuity.position - old_position) * (velocity - old_velocity)
                            / (position - old_position);
                    position = next_discontinuity.position;
                }
            }

            if position > self.path.len() {
                new_points.push(TrajectoryStep::new(position, velocity));

                break Ok((new_points, PathPosition::End, position));
            } else if velocity < 0.0 {
                panic!(
                    "Integrate forward velocity cannot be 0, position {}, velocity {}",
                    position, velocity
                );
            }

            let max_velocity_at_position = self.max_velocity_at(position, Limit::Velocity);

            if velocity > max_velocity_at_position
                && self.phase_slope(
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

            new_points.push(new_point);

            acceleration = self.acceleration_at(&new_point, MinMax::Max);

            if velocity > self.max_velocity_at(position, Limit::Acceleration)
                || velocity > max_velocity_at_position
            {
                let overshoot = new_points
                    .pop()
                    .ok_or(String::from("Attempted to pop last element off empty vec"))?;
                let last_point = new_points.last().unwrap_or(last);

                let mut before = last_point.position;
                let mut before_velocity = last_point.velocity;
                let mut after = overshoot.position;
                let mut after_velocity = overshoot.velocity;

                // Bisect (binary search) within step to find where overshoot occurred to within an
                // epsilon
                while after - before > self.epsilon {
                    trace!(
                        "Integrate forward bisection, before {} after {}",
                        before,
                        after
                    );
                    let midpoint = 0.5 * (before + after);
                    let mut midpoint_velocity = 0.5 * (before_velocity + after_velocity);

                    let max_midpoint_velocity = self.max_velocity_at(midpoint, Limit::Velocity);

                    if midpoint_velocity > max_midpoint_velocity
                        && self.phase_slope(
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
                let position = new_point.position;
                new_points.push(new_point);

                if self.max_velocity_at(after, Limit::Acceleration)
                    < self.max_velocity_at(after, Limit::Velocity)
                {
                    if let Some(next) = next_discontinuity {
                        if after > next.position {
                            break Ok((new_points, PathPosition::NotEnd, position));
                        }
                    }

                    if self.phase_slope(&new_point, MinMax::Max)
                        > self.max_velocity_derivative_at(new_point.position, Limit::Acceleration)
                    {
                        break Ok((new_points, PathPosition::NotEnd, position));
                    }
                } else if self.phase_slope(&new_point, MinMax::Min)
                    > self.max_velocity_derivative_at(new_point.position, Limit::Velocity)
                {
                    break Ok((new_points, PathPosition::NotEnd, position));
                }
            }
        }
    }

    /// Integrate backward from the end of the path until an intersection point is found. The parts
    /// of the current trajectory after this point are removed and replaced with the reverse
    /// integration.
    fn integrate_backward(
        &self,
        start_trajectory: &[TrajectoryStep],
        start_switching_point: &TrajectorySwitchingPoint,
    ) -> Result<(usize, Vec<TrajectoryStep>), String> {
        let TrajectorySwitchingPoint {
            pos:
                TrajectoryStep {
                    mut position,
                    mut velocity,
                    ..
                },
            mut before_acceleration,
            ..
        } = start_switching_point;
        let mut slope = 0.0;
        let mut it = start_trajectory.windows(2).rev();
        let mut new_trajectory: Vec<TrajectoryStep> = Vec::new();
        let mut parts = it.next();
        let mut splice_index = start_trajectory.len() - 1;

        // Loop backwards from end of path, replacing path segments with new position and velocity
        // until an intersection is encountered, or we hit the beginning of the path
        while let Some(&[ref start1, ref _start2]) = parts {
            trace!("Integrate backward loop, position {}", position);

            if position < 0.0 {
                break;
            }

            trace!(
                "Backward start1 pos vs postition {} : {}",
                start1.position,
                position
            );

            if start1.position <= position {
                let new_point = TrajectoryStep::new(position, velocity);

                velocity -= self.timestep * before_acceleration;
                position -= self.timestep * 0.5 * (velocity + new_point.velocity);
                before_acceleration =
                    self.acceleration_at(&TrajectoryStep::new(position, velocity), MinMax::Min);
                slope = (new_point.velocity - velocity) / (new_point.position - position);

                new_trajectory.push(new_point);

                trace!(
                    "RS back_step (pathPos;pathVel;acceleration;slope),{},{},{},{}",
                    position,
                    velocity,
                    before_acceleration,
                    slope
                );

                if velocity < 0.0 {
                    return Err(format!(
                        "Velocity cannot be less than zero at position {}, got {} (acceleration {}, slope {})",
                        position, velocity, before_acceleration, slope
                    ));
                }
            } else {
                parts = it.next();
                splice_index -= 1;
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

                trace!(
                    "RS intersection_values (i_pos;start_slope),{},{}",
                    intersection_position,
                    start_slope
                );

                // Check for intersection between path and current segment
                if start1.position.max(position) - self.epsilon <= intersection_position
                    && intersection_position
                        <= self.epsilon
                            + start2.position.min(
                                new_trajectory
                                    .last()
                                    .ok_or(String::from(
                                        "Could not get last point of empty trajectory",
                                    ))?
                                    .position,
                            )
                {
                    let intersection_velocity =
                        start1.velocity + start_slope * (intersection_position - start1.position);

                    // Add intersection point
                    new_trajectory.push(TrajectoryStep::new(
                        intersection_position,
                        intersection_velocity,
                    ));

                    let ret = new_trajectory.into_iter().rev().collect();

                    trace!(
                        "RS back_splice_idx (start_sw_pos;splice_idx),{},{}",
                        start_switching_point.pos.position,
                        splice_index
                    );

                    return Ok((splice_index, ret));
                }
            }
        }

        Err(format!("Path is invalid: Integrate backwards did not hit start trajectory, start position {} velocity {}", position, velocity))

        // let TrajectorySwitchingPoint {
        //     pos:
        //         TrajectoryStep {
        //             position: mut pos,
        //             velocity: mut vel,
        //             ..
        //         },
        //     before_acceleration: mut acc,
        //     ..
        // } = *start_switching_point;

        // let mut slope = 0.0;
        // let mut new_traj: Vec<TrajectoryStep> = Vec::new();
        // let mut next_iter = false;
        // let mut splice_index = start_trajectory.len();

        // for parts in start_trajectory.windows(2).rev() {
        //     match parts {
        //         [ref start1, ref start2] => {
        //             splice_index -= 1;

        //             while start1.position <= pos {
        //                 if !next_iter {
        //                     let new_point = TrajectoryStep::new(pos, vel);

        //                     vel -= self.timestep * acc;
        //                     pos -= self.timestep * 0.5 * (vel + new_point.velocity);
        //                     acc = self.acceleration_at(&TrajectoryStep::new(pos, vel), MinMax::Min);
        //                     slope = (new_point.velocity - vel) / (new_point.position - pos);

        //                     new_traj.push(new_point);
        //                 }

        //                 // Update pos/vel/acc in next iteration of loop
        //                 next_iter = false;

        //                 trace!(
        //                     "RS back_step (pathPos;pathVel;acceleration;slope),{},{},{},{}",
        //                     pos,
        //                     vel,
        //                     acc,
        //                     slope
        //                 );

        //                 if vel < 0.0 {
        //                     return Err(format!(
        //                         "Velocity cannot be less than zero at position {}, got {} (acceleration {}, slope {})",
        //                         pos, vel, acc, slope
        //                     ));
        //                 }

        //                 let start_slope = (start2.velocity - start1.velocity)
        //                     / (start2.position - start1.position);

        //                 // Normalised position along segment where intersection occurs
        //                 let intersection_position = (start1.velocity - vel + slope * pos
        //                     - start_slope * start1.position)
        //                     / (slope - start_slope);

        //                 // Check for intersection between path and current segment
        //                 if start1.position.max(pos) - self.epsilon <= intersection_position
        //                     && intersection_position
        //                         <= self.epsilon
        //                             + start2.position.min(
        //                                 new_traj
        //                                     .last()
        //                                     .ok_or(String::from(
        //                                         "Could not get last point of empty trajectory",
        //                                     ))?
        //                                     .position,
        //                             )
        //                 {
        //                     let intersection_velocity = start1.velocity
        //                         + start_slope * (intersection_position - start1.position);

        //                     // Add intersection point
        //                     new_traj.push(TrajectoryStep::new(
        //                         intersection_position,
        //                         intersection_velocity,
        //                     ));

        //                     let ret = new_traj.into_iter().rev().collect();

        //                     trace!(
        //                         "RS back_splice_idx (start_sw_pos;splice_idx),{},{}",
        //                         start_switching_point.pos.position,
        //                         splice_index
        //                     );

        //                     return Ok((splice_index, ret));
        //                 }
        //             }

        //             // Skip computation at beginning of next loop
        //             next_iter = true;
        //         }
        //         _ => return Err(format!("Not enough parts: {}, expected 2", parts.len())),
        //     }
        // }

        // Err(format!("Path is invalid: Integrate backwards did not hit start trajectory, start position {} velocity {}, end position {} velocity {}",
        //     start_switching_point.pos.position, start_switching_point.pos.velocity, pos, vel
        // ))
    }

    /// Get next switching point along the path, bounded by velocity or acceleration
    fn next_switching_point(&self, position_along_path: f64) -> Option<TrajectorySwitchingPoint> {
        let mut acceleration_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next acceleration switching point
        while let Some(point) = self.next_acceleration_switching_point(
            acceleration_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
        ) {
            acceleration_switching_point = Some(point.clone());

            if point.pos.velocity <= self.max_velocity_at(point.pos.position, Limit::Velocity) {
                break;
            }
        }

        trace!(
            "RS next_accel_sw_point (pos_along_path;sw_pos;sw_vel),{},{},{}",
            position_along_path,
            acceleration_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(0.0),
            acceleration_switching_point
                .clone()
                .map(|p| p.pos.velocity)
                .unwrap_or(0.0),
        );

        let mut velocity_switching_point: Option<TrajectorySwitchingPoint> = None;

        // Find the next velocity switching point
        while let Some(point) = self.next_velocity_switching_point(
            velocity_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
        ) {
            velocity_switching_point = Some(point.clone());

            if point.pos.position
                > acceleration_switching_point
                    // TODO: Fix clone
                    .clone()
                    .map(|p| p.pos.position)
                    .expect("Acceleration switching point")
                || (point.pos.velocity
                    <= self.max_velocity_at(point.pos.position - self.epsilon, Limit::Acceleration)
                    && point.pos.velocity
                        <= self.max_velocity_at(
                            point.pos.position + self.epsilon,
                            Limit::Acceleration,
                        ))
            {
                break;
            }
        }

        trace!(
            "RS next_vel_sw_point (pos_along_path;sw_pos;sw_vel),{},{},{}",
            position_along_path,
            velocity_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(0.0),
            velocity_switching_point
                .clone()
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
            result.clone().map(|p| p.pos.position).unwrap_or(0.0),
            result.clone().map(|p| p.pos.velocity).unwrap_or(0.0),
            result.clone().map(|p| p.before_acceleration).unwrap_or(0.0),
            result.clone().map(|p| p.after_acceleration).unwrap_or(0.0),
        );

        result
    }

    /// Find minimum or maximum acceleration at a point along path
    fn acceleration_at(&self, pos_vel: &TrajectoryStep, min_max: MinMax) -> f64 {
        let &TrajectoryStep {
            position, velocity, ..
        } = pos_vel;

        let derivative = self.path.tangent(position);
        let second_derivative = self.path.curvature(position);
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

        trace!(
            "RS acc_at (pathPos;pathVel;factor*maxPathAcceleration),{},{},{}",
            position,
            velocity,
            res * factor
        );

        res * factor
    }

    /// Find the maximum allowable velocity at a point, limited by either max acceleration or max
    /// velocity.
    fn max_velocity_at(&self, position_along_path: f64, limit: Limit) -> f64 {
        match limit {
            Limit::Velocity => self.max_velocity_from_velocity(position_along_path),
            Limit::Acceleration => self.max_velocity_from_acceleration(position_along_path),
        }
    }

    fn max_velocity_from_velocity(&self, position_along_path: f64) -> f64 {
        let tangent = self.path.tangent(position_along_path);

        let result = self.velocity_limit.component_div(&tangent).amin();

        trace!(
            "RS max_vel_from_vel (pos;vel),{},{}",
            position_along_path,
            result
        );

        result
    }

    /// Find maximum allowable velocity as limited by the acceleration at a point on the path
    fn max_velocity_from_acceleration(&self, position_along_path: f64) -> f64 {
        let segment = self.path.segment_at_position(position_along_path);
        let vel = segment.tangent(position_along_path);
        let vel_abs = vel.abs();
        let acceleration = segment.curvature(position_along_path);

        let n = nalgebra::dimension::<Coord<N>>();

        let mut max_path_velocity = std::f64::INFINITY;

        for i in 0..n {
            if vel[i] != 0.0 {
                for j in (i + 1)..n {
                    if vel[j] != 0.0 {
                        // TODO: Come up with a less mathsy name
                        let a_ij = acceleration[i] / vel[i] - acceleration[j] / vel[j];

                        if a_ij != 0.0 {
                            max_path_velocity = max_path_velocity.min(
                                ((self.acceleration_limit[i] / vel_abs[i]
                                    + self.acceleration_limit[j] / vel_abs[j])
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

        trace!(
            "RS max_vel_from_acc (pos;vel),{},{}",
            position_along_path,
            max_path_velocity
        );

        max_path_velocity
    }

    fn max_velocity_derivative_at(&self, position_along_path: f64, limit: Limit) -> f64 {
        match limit {
            Limit::Velocity => self.max_velocity_from_velocity_derivative(position_along_path),
            Limit::Acceleration => {
                self.max_velocity_from_acceleration_derivative(position_along_path)
            }
        }
    }

    /// Get the derivative of the max acceleration-bounded velocity at a point along the path
    fn max_velocity_from_velocity_derivative(&self, position_along_path: f64) -> f64 {
        let tangent = self.path.tangent(position_along_path);
        let tangent_abs = tangent.abs();
        let velocity = self.velocity_limit.component_div(&tangent_abs);

        // Find the component index with the smallest value
        let (constraint_axis, _) = velocity
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap_or((0, &std::f64::MAX));

        let result = -(self.velocity_limit[constraint_axis]
            * self.path.curvature(position_along_path)[constraint_axis])
            / (tangent[constraint_axis] * tangent_abs[constraint_axis]);

        trace!(
            "RS max_vel_vel_deriv (pos;vel),{},{}",
            position_along_path,
            result
        );

        result
    }

    /// Get the derivative of the max velocity at a point along the path
    ///
    /// The max velocity in this case is bounded by the acceleration limits at the point
    fn max_velocity_from_acceleration_derivative(&self, position_along_path: f64) -> f64 {
        (self.max_velocity_at(position_along_path + self.epsilon, Limit::Acceleration)
            - self.max_velocity_at(position_along_path - self.epsilon, Limit::Acceleration))
            / (2.0 * self.epsilon)
    }

    /// Get the minimum or maximum phase slope for a position along the path
    ///
    /// TODO: Figure out what phase slope means in this context and give it a better name
    fn phase_slope(&self, pos_vel: &TrajectoryStep, min_max: MinMax) -> f64 {
        self.acceleration_at(&pos_vel, min_max) / pos_vel.velocity
    }

    /// Get the next acceleration-bounded switching point after the current position
    fn next_acceleration_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut velocity;
        let mut current_point =
            &PathSwitchingPoint::new(position_along_path, Continuity::Continuous);

        while current_point.position <= self.path.len() {
            current_point = self.path.next_switching_point(current_point.position)?;

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

                    let before_acceleration = self.acceleration_at(&before_point, MinMax::Min);
                    let after_acceleration = self.acceleration_at(&after_point, MinMax::Max);

                    let before_phase_slope = self.phase_slope(&before_point, MinMax::Min);
                    let after_phase_slope = self.phase_slope(&after_point, MinMax::Max);

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
    fn next_velocity_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        // Broad phase search step
        let step_size = 0.001;
        // TODO: Use self.epsilon again
        let accuracy = 0.000001;
        let mut position = position_along_path - step_size;
        let mut prev_slope = self.phase_slope(
            &TrajectoryStep::new(position, self.max_velocity_at(position, Limit::Velocity)),
            MinMax::Min,
        );
        let mut prev_deriv = self.max_velocity_derivative_at(position, Limit::Velocity);;

        // Move along path until a sign change is detected. This defines an interval within which a
        // velocity switching point occurs. Think of the peak or trough of a sawtooth wave.
        // Bisection is used after this broad phase to more accurately determine the position of the
        // local minimum
        while position < self.path.len() {
            position += step_size;

            let slope = self.phase_slope(
                &TrajectoryStep::new(position, self.max_velocity_at(position, Limit::Velocity)),
                MinMax::Min,
            );

            let deriv = self.max_velocity_derivative_at(position, Limit::Velocity);

            if prev_slope >= prev_deriv && slope <= deriv {
                break;
            }

            prev_slope = slope;
            prev_deriv = deriv;
        }

        trace!(
            "RS end_condition (pathPos;path.getLength()),{},{}",
            position,
            self.path.len()
        );

        if position >= self.path.len() {
            return None;
        }

        // Create an interval to search within to find the actual switching point
        let mut prev_position = position - step_size;
        let mut after_position = position;

        // Binary search through interval to find switching point within an epsilon
        while after_position - prev_position > accuracy {
            position = (prev_position + after_position) / 2.0;

            if self.phase_slope(
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

        let before_acceleration = self.acceleration_at(
            &TrajectoryStep::new(
                prev_position,
                self.max_velocity_at(prev_position, Limit::Velocity),
            ),
            MinMax::Min,
        );
        let after_acceleration = self.acceleration_at(&after_position, MinMax::Max);

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
    use crate::PathOptions;

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
        let traj = Trajectory::new(
            &path,
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
        let traj = Trajectory::new(
            &path,
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

        let path = Path::from_waypoints(
            &waypoints,
            PathOptions {
                max_deviation: 0.001,
            },
        );

        // Same epsilon as Example.cpp for equal comparison
        let traj = Trajectory::new(
            &path,
            TrajectoryOptions {
                velocity_limit: TestCoord3::repeat(1.0),
                acceleration_limit: TestCoord3::repeat(1.0),
                epsilon: 0.000001,
                timestep: 0.001,
            },
        )
        .unwrap();

        let mut t = 0.0;
        let duration = traj.duration();

        while t < duration {
            let p = traj.position(t);
            let v = traj.velocity(t);

            rows.push(TrajectoryStepRow::from_coords(t, &p, &v));

            t += 0.1;
        }

        let p_final = traj.position(duration);
        let v_final = traj.velocity(duration);

        rows.push(TrajectoryStepRow::from_coords(duration, &p_final, &v_final));

        write_debug_csv("../target/plot_native.csv".into(), &rows);

        assert_eq!(traj.trajectory.len(), 14814);
        assert_near!(duration, 14.802832847319937);
    }
}
