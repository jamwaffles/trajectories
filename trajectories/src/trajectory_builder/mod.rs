pub(crate) mod limit_type;
pub(crate) mod limits;
pub(crate) mod min_max;
mod path_position;
mod trajectory_switching_point;
mod trajectory_switching_points;

use self::limit_type::LimitType;
use self::min_max::MinMax;
use self::path_position::PathPosition;
use self::trajectory_switching_point::TrajectorySwitchingPoint;
use self::trajectory_switching_points::TrajectorySwitchingPoints;
use crate::path::{Continuity, PathItem};
use crate::trajectory::TrajectoryStep;
use crate::{Path, TrajectoryOptions};
use limits::{
    max_acceleration_at, max_acceleration_derivative_at, max_velocity_at,
    max_velocity_derivative_at,
};
use nalgebra::{
    allocator::{Allocator, SameShapeVectorAllocator},
    storage::Owned,
    DefaultAllocator, DimName,
};

pub struct TrajectoryBuilder<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
    Owned<f64, N>: Copy,
{
    options: TrajectoryOptions<N>,
    path: &'a Path<N>,
    switching_points: TrajectorySwitchingPoints<'a, N>,
}

impl<'a, N> TrajectoryBuilder<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
    Owned<f64, N>: Copy,
{
    pub fn from_path(path: &'a Path<N>, options: TrajectoryOptions<N>) -> Self {
        Self {
            path,
            options,
            switching_points: TrajectorySwitchingPoints::from_path(&path, options).unwrap(),
        }
    }

    pub fn into_steps(self) -> Result<Vec<TrajectoryStep>, String> {
        let mut trajectory = vec![TrajectoryStep::new(0.0, 0.0)];
        let mut switching_point = TrajectorySwitchingPoint {
            before_acceleration: 0.0,
            after_acceleration: max_acceleration_at(
                &self.path,
                &TrajectoryStep::new(0.0, 0.0),
                MinMax::Max,
                &self.options,
            ),
            pos: TrajectoryStep::new(0.0, 0.0),
        };

        let mut dbg_iter = 0;

        loop {
            debug!("--- ");
            debug!("Setup loop, iter {}", dbg_iter);

            let (fwd, is_end, stop_position) =
                self.integrate_forward(&trajectory, switching_point.after_acceleration)?;

            instrument!(
                "integ_fwd_end_step",
                (fwd.last().unwrap().position, fwd.last().unwrap().velocity)
            );

            trajectory.extend(fwd);

            if is_end == PathPosition::End {
                break;
            }

            if let Some(new_switching_point) =
                self.switching_points.next_switching_point(stop_position)
            {
                debug!(
                    "Switching point {:?} from stop_position {}",
                    new_switching_point, stop_position
                );

                // trace!(
                //     "RS forward_sw_point (pos;vel;beforeAccel;afterAccel),{},{},{},{}",
                //     switching_point.pos.position,
                //     switching_point.pos.velocity,
                //     switching_point.before_acceleration,
                //     switching_point.after_acceleration
                // );
                instrument!(
                    "forward_sw_point",
                    (
                        switching_point.pos.position,
                        switching_point.pos.velocity,
                        switching_point.before_acceleration,
                        switching_point.after_acceleration
                    )
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
                    before_acceleration: max_acceleration_at(
                        &self.path,
                        &TrajectoryStep::new(self.path.len(), 0.0),
                        MinMax::Min,
                        &self.options,
                    ),
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

        Ok(timed)
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
        let last = trajectory
            .last()
            .ok_or_else(|| String::from("Attempted to get last element of empty trajectory"))?;
        let TrajectoryStep {
            mut position,
            mut velocity,
            ..
        } = last;
        let mut acceleration = start_acceleration;

        loop {
            trace!(
                "Integrate forward loop, by {}, position {} out of {}",
                self.options.timestep,
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

            velocity += self.options.timestep * acceleration;
            position += self.options.timestep * 0.5 * (old_velocity + velocity);

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
                break Err(format!(
                    "Integrate forward velocity cannot be 0, position {}, velocity {}",
                    position, velocity
                ));
            }

            let max_velocity_at_position = max_velocity_at(
                self.path,
                position,
                LimitType::Velocity(self.options.velocity_limit),
            );

            if velocity > max_velocity_at_position
                && max_acceleration_derivative_at(
                    &self.path,
                    &TrajectoryStep::new(
                        old_position,
                        max_velocity_at(
                            self.path,
                            old_position,
                            LimitType::Velocity(self.options.velocity_limit),
                        ),
                    ),
                    MinMax::Min,
                    &self.options,
                ) <= max_velocity_derivative_at(
                    self.path,
                    old_position,
                    LimitType::Velocity(self.options.velocity_limit),
                    &self.options,
                )
            {
                velocity = max_velocity_at_position;
            }

            let new_point = TrajectoryStep::new(position, velocity);

            new_points.push(new_point);

            acceleration = max_acceleration_at(&self.path, &new_point, MinMax::Max, &self.options);

            if velocity
                > max_velocity_at(
                    self.path,
                    position,
                    LimitType::Acceleration(self.options.acceleration_limit),
                )
                || velocity > max_velocity_at_position
            {
                let overshoot = new_points
                    .pop()
                    .ok_or_else(|| String::from("Attempted to pop last element off empty vec"))?;
                let last_point = new_points.last().unwrap_or(last);

                let mut before = last_point.position;
                let mut before_velocity = last_point.velocity;
                let mut after = overshoot.position;
                let mut after_velocity = overshoot.velocity;

                // Bisect (binary search) within step to find where overshoot occurred to within an
                // epsilon
                while after - before > self.options.epsilon {
                    trace!(
                        "Integrate forward bisection, before {} after {}",
                        before,
                        after
                    );
                    let midpoint = 0.5 * (before + after);
                    let mut midpoint_velocity = 0.5 * (before_velocity + after_velocity);

                    let max_midpoint_velocity = max_velocity_at(
                        self.path,
                        midpoint,
                        LimitType::Velocity(self.options.velocity_limit),
                    );

                    if midpoint_velocity > max_midpoint_velocity
                        && max_acceleration_derivative_at(
                            &self.path,
                            &TrajectoryStep::new(
                                before,
                                max_velocity_at(
                                    self.path,
                                    before,
                                    LimitType::Velocity(self.options.velocity_limit),
                                ),
                            ),
                            MinMax::Min,
                            &self.options,
                        ) <= max_velocity_derivative_at(
                            self.path,
                            before,
                            LimitType::Velocity(self.options.velocity_limit),
                            &self.options,
                        )
                    {
                        midpoint_velocity = max_midpoint_velocity;
                    }

                    if midpoint_velocity
                        > max_velocity_at(
                            self.path,
                            midpoint,
                            LimitType::Acceleration(self.options.acceleration_limit),
                        )
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

                if max_velocity_at(
                    self.path,
                    after,
                    LimitType::Acceleration(self.options.acceleration_limit),
                ) < max_velocity_at(
                    self.path,
                    after,
                    LimitType::Velocity(self.options.velocity_limit),
                ) {
                    if let Some(next) = next_discontinuity {
                        if after > next.position {
                            break Ok((new_points, PathPosition::NotEnd, position));
                        }
                    }

                    if max_acceleration_derivative_at(
                        &self.path,
                        &new_point,
                        MinMax::Max,
                        &self.options,
                    ) > max_velocity_derivative_at(
                        self.path,
                        new_point.position,
                        LimitType::Acceleration(self.options.acceleration_limit),
                        &self.options,
                    ) {
                        break Ok((new_points, PathPosition::NotEnd, position));
                    }
                } else if max_acceleration_derivative_at(
                    &self.path,
                    &new_point,
                    MinMax::Min,
                    &self.options,
                ) > max_velocity_derivative_at(
                    self.path,
                    new_point.position,
                    LimitType::Velocity(self.options.velocity_limit),
                    &self.options,
                ) {
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

                velocity -= self.options.timestep * before_acceleration;
                position -= self.options.timestep * 0.5 * (velocity + new_point.velocity);
                before_acceleration = max_acceleration_at(
                    &self.path,
                    &TrajectoryStep::new(position, velocity),
                    MinMax::Min,
                    &self.options,
                );
                slope = (new_point.velocity - velocity) / (new_point.position - position);

                new_trajectory.push(new_point);

                // trace!(
                //     "RS back_step (pathPos;pathVel;acceleration;slope),{},{},{},{}",
                //     position,
                //     velocity,
                //     before_acceleration,
                //     slope
                // );
                instrument!(
                    "back_step",
                    (position, velocity, before_acceleration, slope)
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

                instrument!("intersection_values", (intersection_position, start_slope));

                // Check for intersection between path and current segment
                if start1.position.max(position) - self.options.epsilon <= intersection_position
                    && intersection_position
                        <= self.options.epsilon
                            + start2.position.min(
                                new_trajectory
                                    .last()
                                    .ok_or_else(|| {
                                        String::from("Could not get last point of empty trajectory")
                                    })?
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

                    instrument!(
                        "back_splice_idx",
                        (start_switching_point.pos.position, splice_index)
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

        //                     vel -= self.options.timestep * acc;
        //                     pos -= self.options.timestep * 0.5 * (vel + new_point.velocity);
        //                     acc = max_acceleration_at(&self.path, &TrajectoryStep::new(pos, vel), MinMax::Min);
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
        //                 if start1.position.max(pos) - self.options.epsilon <= intersection_position
        //                     && intersection_position
        //                         <= self.options.epsilon
        //                             + start2.position.min(
        //                                 new_traj
        //                                     .last()
        //                                     .ok_or_else(|| String::from(
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
}
