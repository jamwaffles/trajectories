use path::{Continuity, Path, PathItem, SwitchingPoint};
use std;
use Coord;

/// A (position, velocity) pair
#[derive(Debug, Clone)]
struct PositionAndVelocity {
    /// Position
    position: f64,
    /// Velocity
    velocity: f64,
    /// Time
    time: f64,
}

impl PositionAndVelocity {
    pub fn new(position: f64, velocity: f64) -> Self {
        Self {
            position,
            velocity,
            time: 0.0,
        }
    }

    pub fn with_time(self, time: f64) -> Self {
        Self { time, ..self }
    }
}

impl Default for PositionAndVelocity {
    fn default() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            time: 0.0,
        }
    }
}

/// Reached end or not
#[derive(Debug, PartialEq)]
enum PathPosition {
    NotEnd,
    End,
}

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
    // TODO: Split this out into position and velocity
    pos: PositionAndVelocity,
    before_acceleration: f64,
    after_acceleration: f64,
}

impl Default for TrajectorySwitchingPoint {
    fn default() -> Self {
        TrajectorySwitchingPoint {
            pos: PositionAndVelocity::new(0.0, 0.0),
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
    timestep: f64,
    trajectory: Option<Vec<PositionAndVelocity>>,
    epsilon: f64,
}

impl Trajectory {
    /// Create a new trajectory from a given path and max velocity and acceleration
    pub fn new(path: Path, velocity_limit: Coord, acceleration_limit: Coord, epsilon: f64) -> Self {
        let mut traj = Self {
            path,
            velocity_limit,
            acceleration_limit,
            timestep: 0.001,
            trajectory: None,
            epsilon,
        };

        traj.setup();

        traj
    }

    /// Get duration of complete trajectory
    pub fn get_duration(&self) -> f64 {
        // TODO: self.trajectory shouldn't be an option
        self.trajectory.clone().unwrap().last().unwrap().time
    }

    /// Get a position in n-dimensional space given a time along the trajectory
    pub fn get_position(&self, time: f64) -> Coord {
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
    pub fn get_velocity(&self, time: f64) -> Coord {
        let (previous, current) = self.get_trajectory_segment(time);

        let mut segment_len = current.time - previous.time;
        let acceleration = 2.0
            * (current.position - previous.position - segment_len * previous.velocity)
            / segment_len.powi(2);

        let position = previous.position
            + segment_len * previous.velocity * 0.5 * segment_len.powi(2) * acceleration;
        let velocity = previous.velocity + segment_len * acceleration;

        self.path.get_tangent(position) * velocity
    }

    /// Get the (previous_segment, segment) of the trajectory that the given time lies on
    fn get_trajectory_segment(&self, time: f64) -> (PositionAndVelocity, PositionAndVelocity) {
        // TODO: Get rid of all these clones, return a reference
        let pos = self
            .trajectory
            .clone()
            .unwrap()
            .iter()
            .cloned()
            .rev()
            .position(|segment| segment.time <= time)
            // Iter is reversed, so munge index-from-end to index-from-start
            .map(|pos| self.trajectory.clone().unwrap().len() - pos - 1)
            .unwrap()
            .clone().max(1);

        let prev = self
            .trajectory
            .clone()
            .unwrap()
            .get(pos - 1)
            .unwrap_or(self.trajectory.clone().unwrap().clone().first().unwrap())
            .clone();
        let current = self
            .trajectory
            .clone()
            .unwrap()
            .clone()
            .get(pos)
            .unwrap()
            .clone();

        (prev, current)
    }

    /// Compute complete trajectory
    fn setup(&mut self) {
        let mut trajectory = vec![PositionAndVelocity::new(0.0, 0.0)];
        // let mut before_acceleration = 0.0;
        // let mut after_acceleration =
        // path.get_min_max_path_acceleration(&PositionAndVelocity::new(0.0, 0.0), MinMax::Max);
        let mut switching_point = TrajectorySwitchingPoint {
            before_acceleration: 0.0,
            after_acceleration: self
                .get_min_max_path_acceleration(&PositionAndVelocity::new(0.0, 0.0), MinMax::Max),
            pos: PositionAndVelocity::new(0.0, 0.0),
        };

        loop {
            let (fwd, pos) =
                self.integrate_forward(&trajectory, switching_point.after_acceleration);

            trajectory.extend(fwd);

            if pos == PathPosition::End {
                break;
            }

            if let Some(new_switching_point) =
                self.get_next_switching_point(trajectory.last().unwrap().position)
            {
                switching_point = new_switching_point;
            } else {
                // Break if we've reached the end of the path
                break;
            }

            if let Some((updated_traj, new_switching_point)) =
                self.integrate_backward(&trajectory, &switching_point)
            {
                trajectory = updated_traj;
            }
        }

        // Backwards integrate last section
        if let Some((updated_traj, _new_switching_point)) = self.integrate_backward(
            &trajectory,
            &TrajectorySwitchingPoint {
                pos: PositionAndVelocity::new(self.path.get_length(), 0.0),
                before_acceleration: self.get_min_max_path_acceleration(
                    &PositionAndVelocity::new(self.path.get_length(), 0.0),
                    MinMax::Min,
                ),
                after_acceleration: 0.0,
            },
        ) {
            trajectory = updated_traj;
        } else {
            panic!("Last section integrate backward failed");
        }

        let mut t = 0.0;

        // Set times on segments
        let timed = std::iter::once(PositionAndVelocity::new(0.0, 0.0).with_time(t))
            .chain(trajectory.windows(2).map(|parts| {
                if let &[ref previous, ref current] = parts {
                    t += (current.position - previous.position)
                        / ((current.velocity + previous.velocity) / 2.0);

                    // println!("TIME {}", t);
                    current.clone().with_time(t)
                } else {
                    panic!("Time windows");
                }
            })).collect::<Vec<PositionAndVelocity>>();

        self.trajectory = Some(timed);
    }

    fn integrate_forward(
        &self,
        trajectory: &Vec<PositionAndVelocity>,
        start_acceleration: f64,
    ) -> (Vec<PositionAndVelocity>, PathPosition) {
        let mut new_points = Vec::new();
        let PositionAndVelocity {
            mut position,
            mut velocity,
            ..
        } = trajectory.last().expect("Empty traj");
        let mut acceleration = start_acceleration;

        loop {
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
                new_points.push(PositionAndVelocity::new(position, velocity));

                break (new_points, PathPosition::End);
            } else if velocity < 0.0 {
                panic!("Integrate forward velocity cannot be 0");
            }

            if velocity > self.get_max_velocity_from_velocity(position)
                && self.get_min_max_phase_slope(
                    &PositionAndVelocity::new(
                        old_position,
                        self.get_max_velocity_from_velocity(old_position),
                    ),
                    MinMax::Min,
                ) <= self.get_max_velocity_from_velocity_derivative(old_position)
            {
                velocity = self.get_max_velocity_from_velocity(position);
            }

            let new_point = PositionAndVelocity::new(position, velocity);

            new_points.push(new_point.clone());

            acceleration = self.get_min_max_path_acceleration(&new_point, MinMax::Max);

            if velocity > self.get_max_velocity_from_acceleration(position)
                || velocity > self.get_max_velocity_from_velocity(position)
            {
                println!("Overshoot to vel {} at pos {}", velocity, position);
                println!(
                    "  Vel limit:   {}",
                    self.get_max_velocity_from_velocity(position)
                );
                println!(
                    "  Accel limit: {}",
                    self.get_max_velocity_from_acceleration(position)
                );

                let overshoot = new_points.pop().unwrap();
                let last_point = new_points
                    .last()
                    .unwrap_or(trajectory.last().unwrap())
                    .clone();

                let mut before = last_point.position;
                let mut before_velocity = last_point.velocity;
                let mut after = overshoot.position;
                let mut after_velocity = overshoot.velocity;
                let mut midpoint;
                let mut midpoint_velocity;

                while after - before > self.epsilon {
                    midpoint = 0.5 * (before + after);
                    midpoint_velocity = 0.5 * (before_velocity + after_velocity);

                    if midpoint_velocity > self.get_max_velocity_from_velocity(midpoint)
                        && self.get_min_max_phase_slope(
                            &PositionAndVelocity::new(
                                before,
                                self.get_max_velocity_from_velocity(before),
                            ),
                            MinMax::Min,
                        ) <= self.get_max_velocity_from_velocity_derivative(before)
                    {
                        midpoint_velocity = self.get_max_velocity_from_velocity(midpoint);
                    }

                    if midpoint_velocity > self.get_max_velocity_from_acceleration(midpoint)
                        || midpoint_velocity > self.get_max_velocity_from_velocity(midpoint)
                    {
                        after = midpoint;
                        after_velocity = midpoint_velocity;
                    } else {
                        before = midpoint;
                        before_velocity = midpoint_velocity;
                    }
                }

                println!(
                    "    overshoot resolved to vel {} at pos {}",
                    before_velocity, before
                );
                println!("    next_discontinuity {:?}", next_discontinuity);

                new_points.push(PositionAndVelocity::new(before, before_velocity));

                if self.get_max_velocity_from_acceleration(after)
                    < self.get_max_velocity_from_velocity(after)
                {
                    if next_discontinuity.is_some() && after > next_discontinuity.unwrap().position
                    {
                        println!("  break a, {} points len", new_points.len());
                        break (new_points, PathPosition::NotEnd);
                    }

                    if self.get_min_max_phase_slope(&new_points.last().unwrap(), MinMax::Max) > self
                        .get_max_velocity_from_acceleration_derivative(
                            new_points.last().unwrap().position,
                        ) {
                        println!(
                            "  break b, {} points len, last point {:?}",
                            trajectory.len() + new_points.len(),
                            new_points.last()
                        );
                        break (new_points, PathPosition::NotEnd);
                    }
                } else if self.get_min_max_phase_slope(&new_points.last().unwrap(), MinMax::Min)
                    > self.get_max_velocity_from_velocity_derivative(
                        new_points.last().unwrap().position,
                    ) {
                    println!(
                        "  break c, {} points len",
                        trajectory.len() + new_points.len()
                    );
                    break (new_points, PathPosition::NotEnd);
                }
            }
        }
    }

    fn integrate_backward(
        &self,
        start_trajectory: &Vec<PositionAndVelocity>,
        start_switching_point: &TrajectorySwitchingPoint,
    ) -> Option<(Vec<PositionAndVelocity>, TrajectorySwitchingPoint)> {
        let TrajectorySwitchingPoint {
            pos:
                PositionAndVelocity {
                    mut position,
                    mut velocity,
                    ..
                },
            mut before_acceleration,
            ..
        } = start_switching_point.clone();
        let mut slope = 0.0;
        let mut it = start_trajectory.windows(2).rev();
        let mut new_trajectory: Vec<PositionAndVelocity> = Vec::new();
        println!(
            "Integ backwards, pos {} vel {} accel {}",
            position, velocity, before_acceleration
        );
        println!("Traj len {:?}", start_trajectory.last());
        let mut parts = it.next();

        // TODO: Use iterators
        while position >= 0.0 && parts.is_some() {
            if let Some(&[ref start1, ref start2]) = parts {
                if start1.position <= position {
                    let new_point = PositionAndVelocity::new(position, velocity);

                    // TODO: Benchmark push then reverse instead of shift front
                    new_trajectory.insert(0, new_point.clone());

                    // println!(
                    //     "BEF vel {} pos {} timestep {} acceleration {}",
                    //     velocity, position, self.timestep, before_acceleration
                    // );
                    velocity -= self.timestep * before_acceleration;
                    position -= self.timestep * 0.5 * (velocity + new_point.velocity);
                    // println!(
                    //     "SUB {} {} {}",
                    //     position,
                    //     self.timestep,
                    //     (velocity + new_point.velocity)
                    // );
                    before_acceleration = self.get_min_max_path_acceleration(
                        &PositionAndVelocity::new(position, velocity),
                        MinMax::Min,
                    );
                    slope = (new_point.velocity - velocity) / (new_point.position - position);

                    // println!(
                    //     "AFT vel {} pos {} before_accel {} slope {}",
                    //     velocity, position, before_acceleration, slope
                    // );

                    if velocity < 0.0 {
                        panic!("Velocity cannot be less than zero");
                    }
                } else {
                    parts = it.next();
                }

                if let Some(&[ref start1, ref start2]) = parts {
                    let start_slope =
                        (start2.velocity - start1.velocity) / (start2.position - start1.position);

                    // Normalised position along segment where intersection occurs
                    let intersection_position = (start1.velocity - velocity + slope * position
                        - start_slope * start1.position)
                        / (slope - start_slope);

                    // println!(
                    //     "INTER POS {} start1 {}, start2 {}, accum position {}, start1 max {} start2 min {}",
                    //     intersection_position,
                    //     start1.position,
                    //     start2.position,
                    //     position,
                    //     start1.position.max(position),
                    //     start2
                    //         .position
                    //         .min(new_trajectory.first().unwrap().position),
                    // );

                    // Check for intersection between path and current segment
                    if start1.position.max(position) - self.epsilon <= intersection_position
                        && intersection_position <= self.epsilon + start2
                            .position
                            .min(new_trajectory.first().unwrap().position)
                    {
                        // println!("   INTERSECT");
                        let intersection_velocity = start1.velocity
                            + start_slope * (intersection_position - start1.position);

                        let mut ret = start_trajectory
                    .iter()
                    .cloned()
                    // Remove items in current trajectory after intersection point
                    .filter(|step| step.position < start2.position)
                    // Add intersection point
                    .chain(std::iter::once(PositionAndVelocity::new(
                        intersection_position,
                        intersection_velocity,
                    )))
                    .collect::<Vec<PositionAndVelocity>>();

                        // Append newly generated trajectory
                        ret.extend(new_trajectory);

                        return Some((
                            ret,
                            TrajectorySwitchingPoint {
                                pos: PositionAndVelocity::new(position, velocity),
                                before_acceleration,
                                ..TrajectorySwitchingPoint::default()
                            },
                        ));
                    }
                }
            } else {
                panic!("Could not get parts");
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
                pos: PositionAndVelocity::new(position_along_path, 0.0),
                ..TrajectorySwitchingPoint::default()
            });

        // Find the next acceleration switching point
        while let Some(point) = self.get_next_acceleration_switching_point(
            acceleration_switching_point
                .clone()
                .map(|p| p.pos.position)
                .unwrap_or(position_along_path),
        ) {
            println!("Sw point loop 1");

            acceleration_switching_point = Some(point.clone());

            if point.pos.velocity <= self.get_max_velocity_from_velocity(point.pos.position) {
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
            println!("Sw point loop 2");

            velocity_switching_point = Some(point.clone());

            if point.pos.position > acceleration_switching_point
                .clone()
                .expect("Accel switching point")
                .pos
                .position
                || (point.pos.velocity
                    <= self.get_max_velocity_from_acceleration(point.pos.position - self.epsilon)
                    && point.pos.velocity <= self
                        .get_max_velocity_from_acceleration(point.pos.position + self.epsilon))
            {
                break;
            }
        }

        // Return the next earliest switching point (if any)
        acceleration_switching_point.and_then(|accel_point| {
            if velocity_switching_point.is_none()
                || accel_point.pos.position
                    <= velocity_switching_point.clone().unwrap().pos.position
            {
                Some(accel_point)
            } else {
                velocity_switching_point
            }
        })
    }

    /// Find minimum or maximum acceleration at a point along path
    fn get_min_max_path_acceleration(&self, pos_vel: &PositionAndVelocity, min_max: MinMax) -> f64 {
        let &PositionAndVelocity {
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

        // println!(
        //     "MINMAX position {}, velocity {} derivative {} second_derivative {} factor {}, result {}",
        //     position, velocity, derivative, second_derivative, factor, res
        // );

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
                    if velocity[j] != 0.0 {
                        // TODO: Come up with a less mathsy name
                        let a_ij = acceleration[i] / velocity[i] - acceleration[j] / velocity[j];

                        if a_ij != 0.0 {
                            max_path_velocity = max_path_velocity.min(
                                ((self.acceleration_limit[i] / velocity[i].abs()
                                    + self.acceleration_limit[j] / velocity[j].abs())
                                    / a_ij.abs()).sqrt(),
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

    /// Get the derivative of the max velocity at a point along the path
    ///
    /// The max velocity in this case is bounded by the acceleration limits at the point
    fn get_max_velocity_from_acceleration_derivative(&self, position_along_path: f64) -> f64 {
        (self.get_max_velocity_from_acceleration(position_along_path + self.epsilon)
            - self.get_max_velocity_from_acceleration(position_along_path - self.epsilon))
            / (2.0 * self.epsilon)
    }

    /// Get the minimum or maximum phase slope for a position along the path
    ///
    /// TODO: Figure out what phase slope means in this context and give it a better name
    fn get_min_max_phase_slope(&self, pos_vel: &PositionAndVelocity, min_max: MinMax) -> f64 {
        self.get_min_max_path_acceleration(&pos_vel, min_max) / pos_vel.position
    }

    /// Get the next acceleration-bounded switching point after the current position
    fn get_next_acceleration_switching_point(
        &self,
        position_along_path: f64,
    ) -> Option<TrajectorySwitchingPoint> {
        let mut velocity;
        let mut current_point = SwitchingPoint::new(position_along_path, Continuity::Continuous);

        // TODO: Use iterators here, infinite loops suck
        loop {
            current_point = self.path.get_next_switching_point(current_point.position);
            // println!("Next accel sw point loop, next point: {:?}", current_point);

            if current_point.position > self.path.get_length() - self.epsilon {
                break None;
            }

            match current_point.continuity {
                Continuity::Discontinuous => {
                    let before_velocity = self
                        .get_max_velocity_from_acceleration(current_point.position - self.epsilon);
                    let after_velocity = self
                        .get_max_velocity_from_acceleration(current_point.position + self.epsilon);

                    velocity = before_velocity.min(after_velocity);

                    let before_point =
                        PositionAndVelocity::new(current_point.position - self.epsilon, velocity);
                    let after_point =
                        PositionAndVelocity::new(current_point.position + self.epsilon, velocity);

                    let before_acceleration =
                        self.get_min_max_path_acceleration(&before_point, MinMax::Min);
                    let after_acceleration =
                        self.get_min_max_path_acceleration(&after_point, MinMax::Max);

                    let before_phase_slope = self.get_min_max_phase_slope(
                        &PositionAndVelocity::new(current_point.position - self.epsilon, velocity),
                        MinMax::Min,
                    );
                    let after_phase_slope = self.get_min_max_phase_slope(
                        &PositionAndVelocity::new(current_point.position + self.epsilon, velocity),
                        MinMax::Max,
                    );

                    let before_max_velocity_deriv = self
                        .get_max_velocity_from_acceleration_derivative(
                            current_point.position - 2.0 * self.epsilon,
                        );
                    let after_max_velocity_deriv = self
                        .get_max_velocity_from_acceleration_derivative(
                            current_point.position + 2.0 * self.epsilon,
                        );

                    if (before_velocity > after_velocity
                        || before_phase_slope > before_max_velocity_deriv)
                        && (before_velocity < after_velocity
                            || after_phase_slope < after_max_velocity_deriv)
                    {
                        break Some(TrajectorySwitchingPoint {
                            pos: PositionAndVelocity::new(current_point.position, velocity),
                            before_acceleration,
                            after_acceleration,
                        });
                    }
                }
                Continuity::Continuous => {
                    let velocity = self.get_max_velocity_from_acceleration(current_point.position);

                    let low_deriv = self.get_max_velocity_from_acceleration_derivative(
                        current_point.position - self.epsilon,
                    );
                    let high_deriv = self.get_max_velocity_from_acceleration_derivative(
                        current_point.position + self.epsilon,
                    );

                    if low_deriv < 0.0 && high_deriv > 0.0 {
                        break Some(TrajectorySwitchingPoint {
                            pos: PositionAndVelocity::new(current_point.position, velocity),
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
        let mut position = position_along_path - step_size;
        let mut start = false;

        // Broad phase
        // TODO: Iterators, or at least refactor this fake do-while loop. The exit condition is
        // really weirdly implemented.
        while {
            position += step_size;

            if self.get_min_max_phase_slope(
                &PositionAndVelocity::new(position, self.get_max_velocity_from_velocity(position)),
                MinMax::Min,
            ) >= self.get_max_velocity_from_velocity_derivative(position)
            {
                start = true;
            }

            (!start
                || self.get_min_max_phase_slope(
                    &PositionAndVelocity::new(
                        position,
                        self.get_max_velocity_from_velocity(position),
                    ),
                    MinMax::Min,
                ) >= self.get_max_velocity_from_velocity_derivative(position))
                && position < self.path.get_length()
        } {}

        if position > self.path.get_length() {
            return None;
        }

        // Create an interval to search within to find the actual switching point
        let mut prev_position = position - step_size;
        let mut after_position = position;

        // Binary search through interval to find switching point within an epsilon
        // TODO: Iterators
        while after_position - prev_position > self.epsilon {
            position = (prev_position + after_position) / 2.0;

            if self.get_min_max_phase_slope(
                &PositionAndVelocity::new(position, self.get_max_velocity_from_velocity(position)),
                MinMax::Min,
            ) > self.get_max_velocity_from_velocity_derivative(position)
            {
                prev_position = position
            } else {
                after_position = position
            }
        }

        let before_acceleration = self.get_min_max_path_acceleration(
            &PositionAndVelocity::new(
                prev_position,
                self.get_max_velocity_from_velocity(prev_position),
            ),
            MinMax::Min,
        );
        let after_acceleration = self.get_min_max_path_acceleration(
            &PositionAndVelocity::new(
                after_position,
                self.get_max_velocity_from_velocity(after_position),
            ),
            MinMax::Max,
        );

        Some(TrajectorySwitchingPoint {
            before_acceleration,
            after_acceleration,
            pos: PositionAndVelocity::new(
                after_position,
                self.get_max_velocity_from_velocity(after_position),
            ),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_helpers::*;

    #[test]
    fn create_example_cpp_trajectory() {
        let waypoints: Vec<Coord> = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(0.0, 0.2, 1.0),
            Coord::new(0.0, 3.0, 0.5),
            Coord::new(1.1, 2.0, 0.0),
            Coord::new(1.0, 0.0, 0.0),
            Coord::new(0.0, 1.0, 0.0),
            Coord::new(0.0, 0.0, 1.0),
        ];
        let mut rows = Vec::new();

        let path = Path::from_waypoints(&waypoints, 0.001);

        // Same epsilon as Example.cpp for equal comparison
        let traj = Trajectory::new(path, Coord::repeat(1.0), Coord::repeat(1.0), 0.000001);

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

        assert_eq!(traj.trajectory.clone().unwrap().len(), 14814);
        assert_near!(duration, 14.802832847319937);
    }
}
