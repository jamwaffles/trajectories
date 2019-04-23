use super::{TrajectoryOptions, TrajectorySegment};
use crate::path::Path;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

/// A simple trajectory that uses no blending or smoothing to calculate motion along a path
///
/// TODO: Add and use a velocity per waypoint instead of slamming everything to velocity limit all
/// the time.
/// TODO: Support curved trajectory segments
pub struct LinearTrajectory<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    path: Vec<TrajectorySegment<'a, N>>,

    length: f64,
}

impl<'a, N> LinearTrajectory<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn new(path: &'a Path<N>, options: TrajectoryOptions<N>) -> Result<Self, String> {
        let path = path
            .iter()
            .scan(0.0, |time, segment| {
                let new_segment = TrajectorySegment::new(segment, options, *time);

                // TODO: Add a velocity per segment and use that instead of the max vel here
                *time += new_segment.time();

                Some(new_segment)
            })
            .collect::<Vec<TrajectorySegment<N>>>();

        let length = path.iter().fold(0.0, |acc, segment| acc + segment.len());

        Ok(Self { path, length })
    }

    pub fn len(&self) -> f64 {
        self.length
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::TestCoord3;
    use approx::assert_ulps_eq;

    #[test]
    fn create_unit_segments_with_times() {
        // All segments have a length and time of 1.0 at max velocity of 1.0
        let segments = vec![
            TestCoord3::new(1.0, 0.0, 0.0),
            TestCoord3::new(2.0, 0.0, 0.0),
            TestCoord3::new(2.0, 1.0, 0.0),
        ];

        let path = Path::from_waypoints(&segments).unwrap();

        let trajectory = LinearTrajectory::new(
            &path,
            TrajectoryOptions {
                velocity_limit: TestCoord3::new(1.0, 1.0, 1.0),
                acceleration_limit: TestCoord3::new(1.0, 1.0, 1.0),
            },
        )
        .unwrap();

        for segment in trajectory.path.iter() {
            assert_eq!(segment.len(), 1.0);
            assert_eq!(segment.time(), 1.0);
        }

        assert_eq!(trajectory.path[0].start_offset(), 0.0);
        assert_eq!(trajectory.path[1].start_offset(), 1.0);
    }

    #[test]
    fn create_segments_with_times() {
        // All segments have a length and time of 1.0 at max velocity of 1.0
        let segments = vec![
            TestCoord3::new(1.0, 1.0, 1.0),
            TestCoord3::new(2.0, 3.0, 4.0),
        ];

        let path = Path::from_waypoints(&segments).unwrap();

        let trajectory = LinearTrajectory::new(
            &path,
            TrajectoryOptions {
                velocity_limit: TestCoord3::new(1.0, 2.0, 3.0),
                acceleration_limit: TestCoord3::new(1.0, 1.0, 1.0),
            },
        )
        .unwrap();

        // Probably correct...
        assert_ulps_eq!(trajectory.path[0].len(), 3.7416573867739413);
        assert_ulps_eq!(trajectory.path[0].time(), 2.6457513110645907);
    }

    #[test]
    fn total_length() {
        // All segments have a length and time of 1.0 at max velocity of 1.0
        let segments = vec![
            TestCoord3::new(1.0, 1.0, 0.0),
            TestCoord3::new(2.0, 3.0, 0.0),
            TestCoord3::new(5.0, 10.0, 0.0),
        ];

        let path = Path::from_waypoints(&segments).unwrap();

        let trajectory = LinearTrajectory::new(
            &path,
            TrajectoryOptions {
                velocity_limit: TestCoord3::new(1.0, 2.0, 3.0),
                acceleration_limit: TestCoord3::new(1.0, 1.0, 1.0),
            },
        )
        .unwrap();

        assert_ulps_eq!(trajectory.len(), 9.851841083363698);
    }
}
