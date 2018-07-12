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
    end_trajectory: Vec<TrajectoryStep>,
    eps: f64,
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
        let trajectory = vec![TrajectoryStep::new(0.0, 0.0)];

        unimplemented!()
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

        for t in self.end_trajectory.iter() {
            writeln!(trajectory_file, "Stuff {} {}\n", t.path_pos, t.path_vel)
                .expect("Couldn't write to file");
        }

        unimplemented!()
    }
    pub fn is_valid(&self) -> bool {
        self.valid
    }
    pub fn get_duration(&self) -> f64 {
        self.trajectory.last().unwrap().time
    }
    pub fn get_position(&self, time: f64) -> Coord {
        // list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
        // list<TrajectoryStep>::const_iterator previous = it;
        // previous--;

        // double timeStep = it->time - previous->time;
        // const double acceleration = 2.0 * (it->pathPos - previous->pathPos - timeStep * previous->pathVel) / (timeStep * timeStep);

        // timeStep = time - previous->time;
        // const double pathPos = previous->pathPos + timeStep * previous->pathVel + 0.5 * timeStep * timeStep * acceleration;

        // return path.getConfig(pathPos);

        unimplemented!()
    }
    pub fn get_velocity(&self, time: f64) -> Coord {
        // list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
        // list<TrajectoryStep>::const_iterator previous = it;
        // previous--;

        // double timeStep = it->time - previous->time;
        // const double acceleration = 2.0 * (it->pathPos - previous->pathPos - timeStep * previous->pathVel) / (timeStep * timeStep);

        // timeStep = time - previous->time;
        // const double pathPos = previous->pathPos + timeStep * previous->pathVel + 0.5 * timeStep * timeStep * acceleration;
        // const double pathVel = previous->pathVel + timeStep * acceleration;

        // return path.getTangent(pathPos) * pathVel;

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

        if time >= self.get_duration() {
            (
                self.trajectory
                    .get(self.trajectory.len() - 2)
                    .expect("Get segment out of bounds"),
                self.trajectory
                    .get(self.trajectory.len() - 3)
                    .expect("Get segment out of bounds"),
            )
        } else {
            // TODO
            unimplemented!()
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

    fn get_next_switching_point(
        &self,
        path_pos: f64,
        next_switching_point: &TrajectoryStep,
        before_acceleration: f64,
        after_acceleration: f64,
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
