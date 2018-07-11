use super::path::Path;
use super::trajectorystep::TrajectoryStep;
use super::Coord;
use std::f64;

pub struct Trajectory {
    path: Path,
    max_velocity: Coord,
    max_acceleration: Coord,
    n: usize,
    valid: bool,
    trajectory: Vec<TrajectoryStep>,
    // non-empty only if the trajectory generation failed. (Wtf does this mean?)
    end_trajectory: Vec<TrajectoryStep>,
    eps: f64,
    timestep: f64,
    cached_time: f64,
    cached_trajectory_segment: Iterator<Item = TrajectoryStep>,
}

impl Trajectory {
    pub fn new(path: &Path, max_velocity: Coord, max_acceleration: Coord, timestep: f64) -> Self {
        let trajectory = vec![TrajectoryStep::new(0.0, 0.0)];

        // Self {
        //     n: max_velocity.len(),
        //     // ...
        // }

        unimplemented!()
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

    // returns true if end of path is reached.
    // bool Trajectory::getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration) {
    //     TrajectoryStep accelerationSwitchingPoint(pathPos, 0.0);
    //     double accelerationBeforeAcceleration, accelerationAfterAcceleration;
    //     bool accelerationReachedEnd;
    //     do {
    //         accelerationReachedEnd = getNextAccelerationSwitchingPoint(accelerationSwitchingPoint.pathPos, accelerationSwitchingPoint, accelerationBeforeAcceleration, accelerationAfterAcceleration);
    //         double test = getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos);
    //     } while(!accelerationReachedEnd && accelerationSwitchingPoint.pathVel > getVelocityMaxPathVelocity(accelerationSwitchingPoint.pathPos));

    //     TrajectoryStep velocitySwitchingPoint(pathPos, 0.0);
    //     double velocityBeforeAcceleration, velocityAfterAcceleration;
    //     bool velocityReachedEnd;
    //     do {
    //         velocityReachedEnd = getNextVelocitySwitchingPoint(velocitySwitchingPoint.pathPos, velocitySwitchingPoint, velocityBeforeAcceleration, velocityAfterAcceleration);
    //     } while(!velocityReachedEnd && velocitySwitchingPoint.pathPos <= accelerationSwitchingPoint.pathPos
    //         && (velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos - eps)
    //         || velocitySwitchingPoint.pathVel > getAccelerationMaxPathVelocity(velocitySwitchingPoint.pathPos + eps)));

    //     if(accelerationReachedEnd && velocityReachedEnd) {
    //         return true;
    //     }
    //     else if(!accelerationReachedEnd && (velocityReachedEnd || accelerationSwitchingPoint.pathPos <= velocitySwitchingPoint.pathPos)) {
    //         nextSwitchingPoint = accelerationSwitchingPoint;
    //         beforeAcceleration = accelerationBeforeAcceleration;
    //         afterAcceleration = accelerationAfterAcceleration;
    //         return false;
    //     }
    //     else {
    //         nextSwitchingPoint = velocitySwitchingPoint;
    //         beforeAcceleration = velocityBeforeAcceleration;
    //         afterAcceleration = velocityAfterAcceleration;
    //         return false;
    //     }
    // }

    fn get_next_switching_point(
        path_pos: f64,
        next_switching_point: &TrajectoryStep,
        before_acceleration: f64,
        after_acceleration: f64,
    ) -> bool {
        unimplemented!()
    }
}
