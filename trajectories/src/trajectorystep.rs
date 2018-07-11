pub struct TrajectoryStep {
    pub path_pos: f64,
    pub path_vel: f64,
    pub time: f64,
}

impl TrajectoryStep {
    pub fn new(path_pos: f64, path_vel: f64) -> Self {
        Self {
            path_pos,
            path_vel,
            time: 0.0,
        }
    }
}
