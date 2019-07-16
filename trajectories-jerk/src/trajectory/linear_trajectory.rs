use super::{TrajectoryOptions, TrajectorySegment};
use crate::path::Path;
use crate::path_segment::PathSegment;
use crate::Coord;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};
use std::cmp::Ordering;

/// A simple trajectory that uses no blending or smoothing to calculate motion along a path
///
/// TODO: Add and use a velocity per waypoint instead of slamming everything to velocity limit all
/// the time.
/// TODO: Support curved trajectory segments
#[derive(Debug)]
pub struct LinearTrajectory<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    path: Vec<TrajectorySegment<'a, N>>,

    length: f64,

    duration: f64,
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

                println!("{:?} -> {}", time, *time + new_segment.time());
                *time += new_segment.time();

                Some(new_segment)
            })
            .collect::<Vec<TrajectorySegment<N>>>();

        let length = path.iter().fold(0.0, |acc, segment| acc + segment.len());

        println!("{:#?}", path);

        let duration = path
            .last()
            .map(|l| l.start_offset() + l.time())
            .unwrap_or(0.0);

        Ok(Self {
            path,
            length,
            duration,
        })
    }

    pub fn segment_at_time(&self, time: f64) -> Option<&TrajectorySegment<N>> {
        self.path
            .binary_search_by(|segment| {
                let start = segment.start_offset();
                let end = start + segment.time();

                if start <= time && time < end {
                    Ordering::Equal
                } else if time < start {
                    Ordering::Greater
                } else if time >= end {
                    Ordering::Less
                } else {
                    unreachable!()
                }
            })
            .map_err(|e| println!("Err {:?}", e))
            .ok()
            .and_then(|idx| {
                // println!("Idx {}, t {}", idx, time);
                self.path.get(idx)
            })
    }

    pub fn len(&self) -> f64 {
        self.length
    }

    pub fn duration(&self) -> f64 {
        self.duration
    }

    /// Get position at a time along the path
    ///
    /// TODO: Meaningful error type describing why a position could not be found
    pub fn position_linear(&self, time: f64) -> Result<Coord<N>, String> {
        self.segment_at_time(time)
            .map(|segment| segment.position_unchecked(time))
            .ok_or(format!(
                "Failed to get segment at time {}. Total length {}",
                time,
                self.len()
            ))
    }

    /// Get velocity at a time along the path
    ///
    /// TODO: Meaningful error type describing why a velocity could not be found
    pub fn velocity_linear(&self, time: f64) -> Result<Coord<N>, String> {
        self.segment_at_time(time)
            .map(|segment| segment.velocity_unchecked(time))
            .ok_or(format!(
                "Failed to get segment at time {}. Total length {}",
                time,
                self.len()
            ))
    }

    pub fn velocity_s_curve(&self, time: f64) -> Result<Coord<N>, String> {
        let segment = match self
            .segment_at_time(time)
            .ok_or(format!(
                "Failed to get segment at time {}. Total length {}",
                time,
                self.len()
            ))?
            .path_segment
        {
            PathSegment::Linear(s) => s,
            // _ => unreachable!(),
        };

        println!("Segment {:?}", segment);

        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::TestCoord3;
    use crate::Waypoint;
    use approx::assert_ulps_eq;

    #[test]
    fn create_unit_segments_with_times() {
        // All segments have a length and time of 1.0 at max velocity of 1.0
        let segments = vec![
            Waypoint::new(
                TestCoord3::new(1.0, 0.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 0.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 1.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
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
            assert_eq!(segment.len(), 1.0, "Length is incorrect");
            assert_eq!(segment.time(), 1.0, "Time is incorrect");
        }

        assert_eq!(trajectory.path[0].start_offset(), 0.0);
        assert_eq!(trajectory.path[1].start_offset(), 1.0);
    }

    // #[test]
    // fn create_segments_with_times() {
    //     let segments = vec![
    //         Waypoint::new(
    //             TestCoord3::new(0.0, 0.0, 0.0),
    //             TestCoord3::new(1.0, 1.0, 1.0),
    //         ),
    //         Waypoint::new(
    //             TestCoord3::new(1.0, 1.0, 0.0),
    //             TestCoord3::new(1.0, 1.0, 1.0),
    //         ),
    //     ];

    //     let path = Path::from_waypoints(&segments).unwrap();

    //     let trajectory = LinearTrajectory::new(
    //         &path,
    //         TrajectoryOptions {
    //             velocity_limit: TestCoord3::new(1.0, 2.0, 3.0),
    //             acceleration_limit: TestCoord3::new(1.0, 1.0, 1.0),
    //         },
    //     )
    //     .unwrap();

    //     // Probably correct...
    //     assert_ulps_eq!(trajectory.path[0].len(), 1.4142135623730951);
    //     assert_ulps_eq!(trajectory.path[0].time(), 1.5811388300841895);
    // }

    #[test]
    fn total_length() {
        let segments = vec![
            Waypoint::new(
                TestCoord3::new(1.0, 1.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 3.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(5.0, 10.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
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

    #[test]
    fn get_position() {
        // All segments have a length and time of 1.0 at max velocity of 1.0
        let segments = vec![
            Waypoint::new(
                TestCoord3::new(1.0, 0.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 0.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 1.0, 0.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 1.0, 1.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
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

        // Get position half way along second segment
        let coord = trajectory.position_linear(1.5).unwrap();

        assert_eq!(coord, TestCoord3::new(2.0, 0.5, 0.0));
    }
}
