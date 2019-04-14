mod limit_type;
mod min_max;
mod path_position;
mod trajectory_options;
mod trajectory_step;
mod trajectory_switching_point;

pub(crate) use self::limit_type::LimitType;
pub(crate) use self::min_max::MinMax;
pub(crate) use self::path_position::PathPosition;
pub use self::trajectory_options::TrajectoryOptions;
pub(crate) use self::trajectory_step::TrajectoryStep;
pub(crate) use self::trajectory_switching_point::TrajectorySwitchingPoint;
use crate::path::{Path, PathItem};
use crate::trajectory_builder::TrajectoryBuilder;
use crate::Coord;
use nalgebra::{
    allocator::{Allocator, SameShapeVectorAllocator},
    DefaultAllocator, DimName,
};

/// Motion trajectory
#[derive(Debug)]
pub struct Trajectory<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
{
    trajectory: Vec<TrajectoryStep>,
    path: &'a Path<N>,
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
        let builder = TrajectoryBuilder::from_path(path).with_options(options);

        let trajectory = builder.into_steps()?;

        Ok(Self { path, trajectory })
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

    /// Get position and velocity at a time along the path
    ///
    /// Use this method as a more optimised way of calling both `.position()` and `.velocity()
    pub fn position_and_velocity(&self, time: f64) -> (Coord<N>, Coord<N>) {
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

        (
            self.path.position(position),
            self.path.tangent(position) * velocity,
        )
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;
    use crate::PathOptions;

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
