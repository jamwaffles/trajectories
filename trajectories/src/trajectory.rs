use super::path::Path;
use super::trajectorystep::TrajectoryStep;
use super::Coord;

pub struct Trajectory {
    path: Path,
    maxVelocity: Coord,
    maxAcceleration: Coord,
    n: u32,
    valid: bool,
    trajectory: Vec<TrajectoryStep>,
    // non-empty only if the trajectory generation failed. (Wtf does this mean?)
    end_trajectory: Vec<TrajectoryStep>,
    eps: f64,
    timestep: f64,
    cached_time: f64,
    cached_trajectory_segment: Iterator<Item = TrajectoryStep>,
}
