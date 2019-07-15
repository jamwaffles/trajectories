use crate::path_segment::PathSegment;
use crate::waypoint::Waypoint;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};
use std::fmt;

#[derive(Debug, Clone)]
pub struct Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    segments: Vec<PathSegment<N>>,
}

impl<N> Path<N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    /// Create a path from only linear segments between a list of waypoints
    pub fn from_waypoints(waypoints: &[Waypoint<N>]) -> Result<Self, PathErrorKind> {
        if waypoints.len() < 2 {
            return Err(PathErrorKind::TooShort);
        }

        let segments = waypoints
            .windows(2)
            .map(|parts| match parts {
                [start, end] => PathSegment::linear(*start, *end),
                _ => unreachable!("Path requires at least two waypoints"),
            })
            .collect();

        Ok(Self { segments })
    }

    pub fn iter(&self) -> impl Iterator<Item = &PathSegment<N>> {
        self.segments.iter()
    }
}

pub enum PathErrorKind {
    TooShort,
}

// TODO: Impl Error or whatever instead for more mad Rust iomaticness
impl fmt::Debug for PathErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            PathErrorKind::TooShort => write!(
                f,
                "Path too short. Linear paths must have at least two waypoints."
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::TestCoord3;

    #[test]
    fn create_linear_path() {
        let velocity = TestCoord3::new(1.0, 1.0, 1.0);

        let waypoints = vec![
            Waypoint::new(TestCoord3::new(0.0, 0.0, 0.0), velocity),
            Waypoint::new(TestCoord3::new(1.0, 2.0, 3.0), velocity),
            Waypoint::new(TestCoord3::new(5.0, 10.0, 15.0), velocity),
        ];

        Path::from_waypoints(&waypoints).expect("Failed to generate path");
    }

    #[test]
    #[should_panic]
    fn linear_path_too_short() {
        let velocity = TestCoord3::new(1.0, 1.0, 1.0);

        let waypoints = vec![Waypoint::new(TestCoord3::new(0.0, 0.0, 0.0), velocity)];

        Path::from_waypoints(&waypoints).unwrap();
    }
}
