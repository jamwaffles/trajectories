/// Path creation options
#[derive(Debug, Copy, Clone)]
pub struct PathOptions {
    /// Maximum deviation from true ideal path
    pub max_deviation: f64,
}

impl Default for PathOptions {
    fn default() -> Self {
        Self {
            max_deviation: 0.001,
        }
    }
}
