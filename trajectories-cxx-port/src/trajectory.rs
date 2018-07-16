use super::path::Path;
use super::trajectorystep::TrajectoryStep;
use super::Coord;
use std::f64;
use std::fs::File;
use std::io::Write;
use std::path::Path as FilePath;

// TODO: Change for something less rigid
const EPS: f64 = 0.000001;

pub struct Trajectory {
    path: Path,
    max_velocity: Coord,
    max_acceleration: Coord,
    /// Dimensionality of vectors
    n: usize,
    valid: bool,
    trajectory: Vec<TrajectoryStep>,
    // non-empty only if the trajectory generation failed. (Wtf does this mean?)
    end_trajectory: Option<Vec<TrajectoryStep>>,
    timestep: f64,
    cached_time: f64,
    // cached_trajectory_segment: Iterator<Item = TrajectoryStep>,
}

impl Trajectory {
    pub fn from_path(
        path: &Path,
        max_velocity: Coord,
        max_acceleration: Coord,
        timestep: f64,
    ) -> Self {
        let mut new = Self {
            cached_time: f64::MAX,
            max_acceleration,
            max_velocity,
            n: max_velocity.len(),
            // TODO: Stop cloning
            path: path.clone(),
            timestep,
            valid: true,
            end_trajectory: None,
            trajectory: vec![TrajectoryStep::new(0.0, 0.0)],
        };

        new.initialise();

        new
    }

    fn initialise(&mut self) {
        // trajectory.push_back(TrajectoryStep(0.0, 0.0));
        // double afterAcceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
        // while(valid && !integrateForward(trajectory, afterAcceleration) && valid) {
        //     double beforeAcceleration;
        //     TrajectoryStep switchingPoint;
        //     if(getNextSwitchingPoint(trajectory.back().pathPos, switchingPoint, beforeAcceleration, afterAcceleration)) {
        //         break;
        //     }
        //     integrateBackward(trajectory, switchingPoint.pathPos, switchingPoint.pathVel, beforeAcceleration);
        // }

        let mut after_acceleration = self.get_min_max_path_acceleration(0.0, 0.0, true);

        while let Some(new_accel) = self.integrate_forward(after_acceleration) {
            if !self.valid {
                break;
            }
            after_acceleration = new_accel;
            let mut before_acceleration = 0.0;

            let (path_end_reached, switching_point, before_acceleration, new_after_acceleration) =
                self.get_next_switching_point(
                    self.trajectory
                        .last()
                        .expect("Trajectory is empty")
                        .path_pos,
                );

            after_acceleration = new_after_acceleration;

            if path_end_reached {
                break;
            }

            self.integrate_backward(
                switching_point.path_pos,
                switching_point.path_vel,
                after_acceleration,
            );
        }

        // if(valid) {
        //     double beforeAcceleration = getMinMaxPathAcceleration(path.getLength(), 0.0, false);
        //     integrateBackward(trajectory, path.getLength(), 0.0, beforeAcceleration);
        // }

        if self.valid {
            let before_acceleration =
                self.get_min_max_path_acceleration(self.path.get_length(), 0.0, false);
            let len = self.path.get_length();

            self.integrate_backward(len, 0.0, before_acceleration);
        }

        // if(valid) {
        //     // calculate timing
        //     list<TrajectoryStep>::iterator previous = trajectory.begin();
        //     list<TrajectoryStep>::iterator it = previous;
        //     it->time = 0.0;
        //     it++;
        //     while(it != trajectory.end()) {
        //         it->time = previous->time + (it->pathPos - previous->pathPos) / ((it->pathVel + previous->pathVel) / 2.0);
        //         previous = it;
        //         it++;
        //     }
        // }

        // Calculate timing
        if self.valid {
            let new_traj = self.trajectory
                .windows(2)
                .map(|items| {
                    println!("WINDOW {:?}", items);
                    match items {
                        &[previous, it] => {
                            let mut step = it.clone();

                            step.set_time(
                                previous.time
                                    + (it.path_pos - previous.path_pos)
                                        / ((it.path_vel + previous.path_vel) / 2.0),
                            );

                            step
                        }
                        _ => unreachable!(),
                    }
                })
                .collect();

            self.trajectory = new_traj;
        }
    }

    pub fn output_phase_plane_trajectory(&self) {
        let velocity_path = FilePath::new("max_velocity.txt");
        let trajectory_path = FilePath::new("trajectory.txt");
        let mut velocity_file =
            File::create(&velocity_path).expect("Could not create velocity log");
        let mut trajectory_file =
            File::create(&trajectory_path).expect("Could not create trajectory log");

        let mut i = 0.0;

        while i < self.path.get_length() {
            writeln!(velocity_file, "Arse {}\n", i).expect("Couldn't write to file");

            i += 0.1;
        }

        for t in self.trajectory.iter() {
            writeln!(trajectory_file, "Wire {} {}\n", t.path_pos, t.path_vel)
                .expect("Couldn't write to file");
        }

        if let Some(ref end_traj) = self.end_trajectory {
            for t in end_traj.iter() {
                writeln!(trajectory_file, "Stuff {} {}\n", t.path_pos, t.path_vel)
                    .expect("Couldn't write to file");
            }
        }
    }
    pub fn is_valid(&self) -> bool {
        self.valid
    }
    pub fn get_duration(&self) -> f64 {
        self.trajectory.last().unwrap().time
    }
    pub fn get_position(&self, time: f64) -> Coord {
        let (previous, it) = self.get_trajectory_segment(time);

        let mut time_step = it.time - previous.time;

        let acceleration = 2.0 * (it.path_pos - previous.path_pos - time_step * previous.path_vel)
            / (time_step * time_step);

        time_step = time - previous.time;

        let path_pos = previous.path_pos
            + time_step * previous.path_vel
            + 0.5 * time_step * time_step * acceleration;

        self.path.get_config(path_pos)
    }
    pub fn get_velocity(&self, time: f64) -> Coord {
        let (previous, it) = self.get_trajectory_segment(time);

        let mut time_step = it.time - previous.time;

        let acceleration = 2.0 * (it.path_pos - previous.path_pos - time_step * previous.path_vel)
            / (time_step * time_step);

        time_step = time - previous.time;

        let path_pos = previous.path_pos
            + time_step * previous.path_vel
            + 0.5 * time_step * time_step * acceleration;
        let path_vel = previous.path_vel + time_step * acceleration;

        self.path.get_tangent(path_pos) * path_vel
    }

    /// Returns new acceleration value, or none if end of path reached
    fn integrate_forward(&mut self, acceleration: f64) -> Option<f64> {
        // double pathPos = trajectory.back().pathPos;
        // double pathVel = trajectory.back().pathVel;

        // list<pair<double, bool> > switchingPoints = path.getSwitchingPoints();
        // list<pair<double, bool> >::iterator nextDiscontinuity = switchingPoints.begin();

        let mut path_pos = self.trajectory.last().unwrap().path_pos;
        let mut path_vel = self.trajectory.last().unwrap().path_vel;

        let switching_points = self.path.get_switching_points();

        // let mut next_discontinuity = switching_points.iter();

        // while(true)
        // {
        loop {
            //     while(nextDiscontinuity != switchingPoints.end() && (nextDiscontinuity->first <= pathPos || !nextDiscontinuity->second)) {
            //         nextDiscontinuity++;
            //     }

            let next_discontinuity = switching_points.iter().find(|sp| sp.0 > path_pos && sp.1);

            //     double oldPathPos = pathPos;
            //     double oldPathVel = pathVel;

            let old_path_pos = path_pos;
            let old_path_vel = path_vel;

            //     pathVel += timeStep * acceleration;
            //     pathPos += timeStep * 0.5 * (oldPathVel + pathVel);

            path_vel += self.timestep * acceleration;
            path_pos += self.timestep * 0.5 * (old_path_vel + path_vel);

            //     if(nextDiscontinuity != switchingPoints.end() && pathPos > nextDiscontinuity->first) {
            //         pathVel = oldPathVel + (nextDiscontinuity->first - oldPathPos) * (pathVel - oldPathVel) / (pathPos - oldPathPos);
            //         pathPos = nextDiscontinuity->first;
            //     }

            if let Some(next_dis) = next_discontinuity {
                if path_pos > next_dis.0 {
                    path_vel = old_path_vel
                        + (next_dis.0 - old_path_pos) * (path_vel - old_path_vel)
                            / (path_pos - old_path_pos);
                    path_pos = next_dis.0;
                }
            }

            //     if(pathPos > path.getLength()) {
            //         trajectory.push_back(TrajectoryStep(pathPos, pathVel));
            //         return true;
            //     }
            //     else if(pathVel < 0.0) {
            //         valid = false;
            //         cout << "error" << endl;
            //         return true;
            //     }

            if path_pos > self.path.get_length() {
                self.trajectory
                    .push(TrajectoryStep::new(path_pos, path_vel));
                return None;
            } else if path_vel < 0.0 {
                self.valid = false;
                eprintln!("Error");
                return None;
            }

            //     if(pathVel > getVelocityMaxPathVelocity(pathPos)
            //         && getMinMaxPhaseSlope(oldPathPos, getVelocityMaxPathVelocity(oldPathPos), false) <= getVelocityMaxPathVelocityDeriv(oldPathPos))
            //     {
            //         pathVel = getVelocityMaxPathVelocity(pathPos);
            //     }

            if path_vel > self.get_velocity_max_path_velocity(path_pos)
                && self.get_min_max_phase_slope(
                    old_path_pos,
                    self.get_velocity_max_path_velocity(old_path_pos),
                    false,
                ) <= self.get_velocity_max_path_velocity_deriv(old_path_pos)
            {
                path_vel = self.get_velocity_max_path_velocity(path_pos);
            }

            //     trajectory.push_back(TrajectoryStep(pathPos, pathVel));
            //     acceleration = getMinMaxPathAcceleration(pathPos, pathVel, true);

            self.trajectory
                .push(TrajectoryStep::new(path_pos, path_vel));
            let new_acceleration = self.get_min_max_path_acceleration(path_pos, path_vel, true);

            //     if(pathVel > getAccelerationMaxPathVelocity(pathPos) || pathVel > getVelocityMaxPathVelocity(pathPos)) {
            if path_vel > self.get_acceleration_max_path_velocity(path_pos)
                || path_vel > self.get_velocity_max_path_velocity(path_pos)
            {
                //         // find more accurate intersection with max-velocity curve using bisection
                //         TrajectoryStep overshoot = trajectory.back();
                //         trajectory.pop_back();
                //         double before = trajectory.back().pathPos;
                //         double beforePathVel = trajectory.back().pathVel;
                //         double after = overshoot.pathPos;
                //         double afterPathVel = overshoot.pathVel;

                // TODO: Fix clone
                let overshoot = self.trajectory.last().unwrap().clone();
                let penultimate = self.trajectory
                    .get(self.trajectory.len() - 2)
                    .unwrap()
                    .clone();

                let mut before = penultimate.path_pos;
                let mut before_path_vel = penultimate.path_vel;
                let mut after = overshoot.path_pos;
                let mut after_path_vel = overshoot.path_vel;

                //         while(after - before > eps) {

                while after - before > EPS {
                    //             const double midpoint = 0.5 * (before + after);
                    //             double midpointPathVel = 0.5 * (beforePathVel + afterPathVel);

                    let midpoint = 0.5 * (before + after);
                    let mut midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

                    //             if(midpointPathVel > getVelocityMaxPathVelocity(midpoint)
                    //                 && getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <= getVelocityMaxPathVelocityDeriv(before))
                    //             {
                    //                 midpointPathVel = getVelocityMaxPathVelocity(midpoint);
                    //             }

                    if midpoint_path_vel > self.get_velocity_max_path_velocity(midpoint)
                        && self.get_min_max_phase_slope(
                            before,
                            self.get_velocity_max_path_velocity(before),
                            false,
                        )
                            <= self.get_velocity_max_path_velocity_deriv(before)
                    {
                        midpoint_path_vel = self.get_velocity_max_path_velocity(midpoint);
                    }

                    //             if(midpointPathVel > getAccelerationMaxPathVelocity(midpoint) || midpointPathVel > getVelocityMaxPathVelocity(midpoint)) {
                    //                 after = midpoint;
                    //                 afterPathVel = midpointPathVel;
                    //             }
                    //             else {
                    //                 before = midpoint;
                    //                 beforePathVel = midpointPathVel;
                    //             }
                    //         }
                    if midpoint_path_vel > self.get_acceleration_max_path_velocity(midpoint)
                        || midpoint_path_vel > self.get_velocity_max_path_velocity(midpoint)
                    {
                        after = midpoint;
                        after_path_vel = midpoint_path_vel;
                    } else {
                        before = midpoint;
                        before_path_vel = midpoint_path_vel;
                    }
                }

                //         trajectory.push_back(TrajectoryStep(before, beforePathVel));
                self.trajectory.pop();
                self.trajectory
                    .push(TrajectoryStep::new(before, before_path_vel));

                //         if(getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after)) {
                //             if(after > nextDiscontinuity->first) {
                //                 return false;
                //             }
                //             else if(getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, true) > getAccelerationMaxPathVelocityDeriv(trajectory.back().pathPos)) {
                //                 return false;
                //             }
                //         }
                //         else {
                //             if(getMinMaxPhaseSlope(trajectory.back().pathPos, trajectory.back().pathVel, false) > getVelocityMaxPathVelocityDeriv(trajectory.back().pathPos)) {
                //                 return false;
                //             }
                //         }

                let last = self.trajectory.last().unwrap();
                if self.get_acceleration_max_path_velocity(after)
                    < self.get_velocity_max_path_velocity(after)
                {
                    if next_discontinuity.is_some() && after > next_discontinuity.unwrap().0 {
                        return Some(new_acceleration);
                    } else if self.get_min_max_phase_slope(last.path_pos, last.path_vel, true)
                        > self.get_acceleration_max_path_velocity_deriv(last.path_pos)
                    {
                        return Some(new_acceleration);
                    }
                } else if self.get_min_max_phase_slope(last.path_pos, last.path_vel, false)
                    > self.get_velocity_max_path_velocity_deriv(last.path_pos)
                {
                    return Some(new_acceleration);
                }
            }
        }
    }

    fn integrate_backward(&mut self, path_pos: f64, path_vel: f64, acceleration: f64) {
        // list<TrajectoryStep>::iterator start2 = startTrajectory.end();
        // start2--;
        // list<TrajectoryStep>::iterator start1 = start2;
        // start1--;
        // list<TrajectoryStep> trajectory;
        // double slope;
        // assert(start1->pathPos <= pathPos);

        let mut iter = self.trajectory.windows(2).rev();

        // while(start1 != startTrajectory.begin() || pathPos >= 0.0)
        // {

        while let Some(&[start1, start2]) = iter.next() {
            if path_pos < 0.0 {
                break;
            }

            //     if(start1->pathPos <= pathPos) {
            //         trajectory.push_front(TrajectoryStep(pathPos, pathVel));
            //         pathVel -= timeStep * acceleration;
            //         pathPos -= timeStep * 0.5 * (pathVel + trajectory.front().pathVel);
            //         acceleration = getMinMaxPathAcceleration(pathPos, pathVel, false);
            //         slope = (trajectory.front().pathVel - pathVel) / (trajectory.front().pathPos - pathPos);

            //         if(pathVel < 0.0) {
            //             valid = false;
            //             cout << "Error while integrating backward: Negative path velocity" << endl;
            //             endTrajectory = trajectory;
            //             return;
            //         }
            //     }
            //     else {
            //         start1--;
            //         start2--;
            //     }

            if start1.path_pos <= path_pos {
                self.trajectory
                    .insert(0, TrajectoryStep::new(path_pos, path_vel));
            }

            //     // check for intersection between current start trajectory and backward trajectory segments
            //     const double startSlope = (start2->pathVel - start1->pathVel) / (start2->pathPos - start1->pathPos);
            //     const double intersectionPathPos = (start1->pathVel - pathVel + slope * pathPos - startSlope * start1->pathPos) / (slope - startSlope);
            //     if(max(start1->pathPos, pathPos) - eps <= intersectionPathPos && intersectionPathPos <= eps + min(start2->pathPos, trajectory.front().pathPos)) {
            //         const double intersectionPathVel = start1->pathVel + startSlope * (intersectionPathPos - start1->pathPos);
            //         startTrajectory.erase(start2, startTrajectory.end());
            //         startTrajectory.push_back(TrajectoryStep(intersectionPathPos, intersectionPathVel));
            //         startTrajectory.splice(startTrajectory.end(), trajectory);
            //         return;
            //     }
            // }
        }

        // valid = false;
        // cout << "Error while integrating backward: Did not hit start trajectory" << endl;
        // endTrajectory = trajectory;

        self.valid = false;

        eprintln!("Error while integrating backward: Did not hit start trajectory");

        self.end_trajectory = Some(self.trajectory.clone());

        unimplemented!()
    }

    /// Get previous and current trajectory step for a given time
    fn get_trajectory_segment(&self, time: f64) -> (&TrajectoryStep, &TrajectoryStep) {
        // if(time >= trajectory.back().time) {
        //     list<TrajectoryStep>::const_iterator last = trajectory.end();
        //     last--;
        //     return last;
        // }
        // else {
        //     if(time < cachedTime) {
        //         cachedTrajectorySegment = trajectory.begin();
        //     }
        //     while(time >= cachedTrajectorySegment->time) {
        //         cachedTrajectorySegment++;
        //     }
        //     cachedTime = time;
        //     return cachedTrajectorySegment;
        // }

        // If we're at or past the end, get the last two elements of the vec
        if time >= self.trajectory.last().expect("No trajectory items").time {
            let len = self.trajectory.len();

            (
                self.trajectory.get(len - 2).unwrap(),
                self.trajectory.last().unwrap(),
            )
        } else {
            // TODO: Optimise
            // Find the last traj segment that has a start time gt or eq to `time` (next one will start _after_ `time`)
            let (curr_index, current) = self.trajectory
                .iter()
                .enumerate()
                .skip_while(|(idx, t)| time >= t.time)
                .next()
                .unwrap();

            let prev = self.trajectory.iter().nth(curr_index - 1).unwrap();

            (prev, current)
        }
    }

    fn get_min_max_path_acceleration(&self, path_pos: f64, path_vel: f64, max: bool) -> f64 {
        let config_deriv = self.path.get_tangent(path_pos);
        let config_deriv2 = self.path.get_curvature(path_pos);

        let factor = if max { 1.0 } else { -1.0 };

        let mut max_path_acceleration = f64::MAX;

        for i in 0..self.n {
            if config_deriv[i] != 0.0 {
                max_path_acceleration = max_path_acceleration.min(
                    self.max_acceleration[i] / config_deriv[i].abs()
                        - factor * config_deriv2[i] * path_vel * path_vel / config_deriv[i],
                );
            }
        }

        factor * max_path_acceleration
    }

    /// Get next switching point
    ///
    /// Returns (has_reached_path_end, switching_point, before_acceleration, after_acceleration)
    fn get_next_switching_point(
        &self,
        path_pos: f64,
        // next_switching_point: &TrajectoryStep,
        // before_acceleration: f64,
        // after_acceleration: f64,
    ) -> (bool, TrajectoryStep, f64, f64) {
        let mut acceleration_switching_point = TrajectoryStep::new(path_pos, 0.0);
        let mut acceleration_before_acceleration = 0.0;
        let mut acceleration_after_acceleration = 0.0;
        let mut acceleration_reached_end = false;

        loop {
            let (
                new_acceleration_reached_end,
                new_acceleration_switching_point,
                new_acceleration_before_acceleration,
                new_acceleration_after_acceleration,
            ) = self.get_next_acceleration_switching_point(path_pos);

            let acceleration_reached_end = new_acceleration_reached_end;
            let acceleration_switching_point = new_acceleration_switching_point;
            let acceleration_before_acceleration = new_acceleration_before_acceleration;
            let acceleration_after_acceleration = new_acceleration_after_acceleration;

            if acceleration_reached_end
                || acceleration_switching_point.path_vel
                    <= self.get_velocity_max_path_velocity(acceleration_switching_point.path_pos)
            {
                break;
            }
        }

        let mut velocity_switching_point = TrajectoryStep::new(path_pos, 0.0);
        let mut velocity_before_acceleration = 0.0;
        let mut velocity_after_acceleration = 0.0;
        let mut velocity_reached_end = false;

        loop {
            let (
                new_velocity_reached_end,
                new_velocity_switching_point,
                new_velocity_before_acceleration,
                new_velocity_after_acceleration,
            ) = self.get_next_velocity_switching_point(path_pos);

            let velocity_reached_end = new_velocity_reached_end;
            let velocity_switching_point = new_velocity_switching_point;
            let velocity_before_acceleration = new_velocity_before_acceleration;
            let velocity_after_acceleration = new_velocity_after_acceleration;

            if velocity_reached_end
                || velocity_switching_point.path_pos > acceleration_switching_point.path_pos
                || velocity_switching_point.path_vel
                    <= self.get_acceleration_max_path_velocity(
                        velocity_switching_point.path_pos - EPS,
                    )
                || velocity_switching_point.path_vel
                    <= self.get_acceleration_max_path_velocity(
                        velocity_switching_point.path_pos + EPS,
                    ) {
                break;
            }
        }

        if acceleration_reached_end && velocity_reached_end {
            (true, TrajectoryStep::new(0.0, 0.0), 0.0, 0.0)
        } else if !acceleration_reached_end
            && (velocity_reached_end
                || acceleration_switching_point.path_pos <= velocity_switching_point.path_pos)
        {
            (
                false,
                acceleration_switching_point,
                acceleration_before_acceleration,
                acceleration_after_acceleration,
            )
        } else {
            (
                false,
                velocity_switching_point,
                velocity_before_acceleration,
                velocity_after_acceleration,
            )
        }
    }

    fn get_next_velocity_switching_point(
        &self,
        path_pos_start: f64,
    ) -> (bool, TrajectoryStep, f64, f64) {
        let step_size = 0.001;
        let accuracy = EPS;

        let mut start = false;

        let mut path_pos = path_pos_start - step_size;

        loop {
            path_pos += step_size;

            if self.get_min_max_phase_slope(
                path_pos,
                self.get_velocity_max_path_velocity(path_pos),
                false,
            ) >= self.get_velocity_max_path_velocity_deriv(path_pos)
            {
                start = true;
            }

            if (start
                || self.get_min_max_phase_slope(
                    path_pos,
                    self.get_velocity_max_path_velocity(path_pos),
                    false,
                ) <= self.get_velocity_max_path_velocity_deriv(path_pos))
                || path_pos >= self.path.get_length()
            {
                break;
            }
        }

        if path_pos > self.path.get_length() {
            // TODO: Fix this dumb return type
            return (false, TrajectoryStep::new(0.0, 0.0), 0.0, 0.0);
        }

        let mut before_path_pos = path_pos - step_size;
        let mut after_path_pos = path_pos;

        while after_path_pos - before_path_pos > accuracy {
            path_pos = (before_path_pos + after_path_pos) / 2.0;

            if self.get_min_max_phase_slope(
                path_pos,
                self.get_velocity_max_path_velocity(path_pos),
                false,
            ) > self.get_velocity_max_path_velocity_deriv(path_pos)
            {
                before_path_pos = path_pos;
            } else {
                after_path_pos = path_pos;
            }
        }

        (
            false,
            TrajectoryStep::new(
                after_path_pos,
                self.get_velocity_max_path_velocity(after_path_pos),
            ),
            self.get_min_max_path_acceleration(
                before_path_pos,
                self.get_velocity_max_path_velocity(before_path_pos),
                false,
            ),
            self.get_min_max_path_acceleration(
                after_path_pos,
                self.get_velocity_max_path_velocity(after_path_pos),
                true,
            ),
        )
    }

    fn get_next_acceleration_switching_point(
        &self,
        path_pos: f64,
    ) -> (bool, TrajectoryStep, f64, f64) {
        let mut before_acceleration: f64 = 0.0;
        let mut after_acceleration: f64 = 0.0;
        let mut switching_path_pos = path_pos;
        let mut switching_path_vel;

        loop {
            let (new_switching_path_pos, discontinuity) =
                self.path.get_next_switching_point(switching_path_pos);

            switching_path_pos = new_switching_path_pos;

            if switching_path_pos > self.path.get_length() - EPS {
                return (
                    // TODO: (bool, TrajectoryStep) should become an Option<TrajectoryStep> I think
                    true,
                    TrajectoryStep::new(0.0, 0.0),
                    before_acceleration,
                    after_acceleration,
                );
            }

            if discontinuity {
                let before_path_vel =
                    self.get_acceleration_max_path_velocity(switching_path_pos - EPS);
                let after_path_vel =
                    self.get_acceleration_max_path_velocity(switching_path_pos + EPS);

                switching_path_vel = before_path_vel.min(after_path_vel);

                before_acceleration = self.get_min_max_path_acceleration(
                    switching_path_pos - EPS,
                    switching_path_vel,
                    false,
                );

                after_acceleration = self.get_min_max_path_acceleration(
                    switching_path_pos + EPS,
                    switching_path_vel,
                    true,
                );

                if before_path_vel > after_path_vel
                    || self.get_min_max_phase_slope(
                        switching_path_pos - EPS,
                        switching_path_vel,
                        false,
                    )
                        > self.get_acceleration_max_path_velocity_deriv(
                            switching_path_pos - 2.0 * EPS,
                        )
                        && (before_path_vel < after_path_vel
                            || self.get_min_max_phase_slope(
                                switching_path_pos + EPS,
                                switching_path_vel,
                                true,
                            )
                                < self.get_acceleration_max_path_velocity_deriv(
                                    switching_path_pos + 2.0 * EPS,
                                )) {
                    break;
                }
            } else {
                switching_path_vel = self.get_acceleration_max_path_velocity(switching_path_pos);
                before_acceleration = 0.0;
                after_acceleration = 0.0;

                if self.get_acceleration_max_path_velocity_deriv(switching_path_pos - EPS) < 0.0
                    && self.get_acceleration_max_path_velocity_deriv(switching_path_pos + EPS) > 0.0
                {
                    break;
                }
            }
        }

        (
            false,
            TrajectoryStep::new(switching_path_pos, switching_path_vel),
            0.0,
            0.0,
        )
    }

    fn get_acceleration_max_path_velocity(&self, path_pos: f64) -> f64 {
        let mut max_path_velocity = f64::INFINITY;
        let config_deriv = self.path.get_tangent(path_pos);
        let config_deriv2 = self.path.get_curvature(path_pos);

        for i in 0..self.n {
            if config_deriv[i] != 0.0 {
                for j in (i + 1)..self.n {
                    if config_deriv[j] != 0.0 {
                        let a_ij =
                            config_deriv2[i] / config_deriv[i] - config_deriv2[j] / config_deriv[j];

                        if a_ij != 0.0 {
                            max_path_velocity = max_path_velocity.min(
                                (self.max_acceleration[i] / config_deriv[i].abs()
                                    + self.max_acceleration[j] / config_deriv[j].abs())
                                    .sqrt() / a_ij.abs(),
                            );
                        }
                    }
                }
            } else if config_deriv2[i] != 0.0 {
                max_path_velocity = max_path_velocity
                    .min((self.max_acceleration[i] / config_deriv2[i].abs()).sqrt());
            }
        }

        max_path_velocity
    }

    fn get_acceleration_max_path_velocity_deriv(&self, path_pos: f64) -> f64 {
        (self.get_acceleration_max_path_velocity(path_pos + EPS)
            - self.get_acceleration_max_path_velocity(path_pos - EPS)) / (2.0 * EPS)
    }

    fn get_velocity_max_path_velocity(&self, path_pos: f64) -> f64 {
        let tangent = self.path.get_tangent(path_pos);

        let mut max_path_velocity = f64::MAX;

        for i in 0..self.n {
            max_path_velocity = max_path_velocity.min(self.max_velocity[i] / tangent[i].abs());
        }

        max_path_velocity
    }

    fn get_velocity_max_path_velocity_deriv(&self, path_pos: f64) -> f64 {
        let tangent = self.path.get_tangent(path_pos);
        let mut max_path_velocity = f64::MAX;
        let mut active_constraint: usize = 0;

        for i in 0..self.n {
            let this_max_path_velocity = self.max_velocity[i] / tangent[i].abs();

            if this_max_path_velocity < max_path_velocity {
                max_path_velocity = this_max_path_velocity;
                active_constraint = i;
            }
        }

        -(self.max_velocity[active_constraint]
            * self.path.get_curvature(path_pos)[active_constraint])
            / (tangent[active_constraint] * tangent[active_constraint].abs())
    }

    fn get_min_max_phase_slope(&self, path_pos: f64, path_vel: f64, max: bool) -> f64 {
        self.get_min_max_path_acceleration(path_pos, path_vel, max) / path_vel
    }
}
