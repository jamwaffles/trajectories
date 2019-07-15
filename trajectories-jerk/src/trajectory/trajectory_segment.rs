use super::TrajectoryOptions;
use crate::path_segment::PathSegment;
use crate::Coord;
use nalgebra::{allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName};

#[derive(Debug)]
pub struct TrajectorySegment<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    // Reference to original path segment
    pub(crate) path_segment: &'a PathSegment<N>,

    /// Start time from beginning of trajectory
    start_offset: f64,

    /// The duration of this segment
    time: f64,

    /// Duration of the beginning acceleration period in this segment
    acceleration_time: f64,

    /// Acceleration to use during accel phase
    acceleration: Coord<N>,
}

impl<'a, N> TrajectorySegment<'a, N>
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    pub fn new(
        path_segment: &'a PathSegment<N>,
        options: TrajectoryOptions<N>,
        start_offset: f64,
    ) -> Self {
        match path_segment {
            PathSegment::Linear(segment) => {
                let delta_v = segment.end_velocity - segment.start_velocity;
                let delta_s = segment.end - segment.start;

                let accel_time = delta_v.component_div(&options.acceleration_limit).abs();
                let accel_distance = segment.start_velocity.component_mul(&accel_time)
                    + (0.5
                        * options
                            .acceleration_limit
                            .component_mul(&accel_time.component_mul(&accel_time)));

                let cruise_distance = (delta_s - accel_distance).abs();
                let cruise_time = cruise_distance.component_div(&segment.end_velocity);

                // Haaxxx!
                let cruise_time = if cruise_time.norm().is_infinite() {
                    Coord::repeat(0.0)
                } else {
                    cruise_time
                };

                println!("Delta_v {:?}", delta_v);
                println!("Delta_s {:?}", delta_s);

                println!(
                    "Accel time: {:?}, accel distance {:?}, total length: {}, cruise_time {:?}, cruise_distance {:?}",
                    accel_time,
                    accel_distance,
                    path_segment.len(),
                    cruise_time,
                    cruise_distance
                );

                let time = (accel_time + cruise_time).norm();

                // let naive_result = options
                //     .velocity_limit
                //     .component_mul(&(segment.end - segment.start).normalize())
                //     .norm();

                Self {
                    path_segment,
                    time,
                    start_offset,
                    acceleration_time: accel_time.norm(),
                    acceleration: options.acceleration_limit,
                }
            }
        }
    }

    pub fn start_offset(&self) -> f64 {
        self.start_offset
    }

    /// Get the duration of this segment
    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn len(&self) -> f64 {
        self.path_segment.len()
    }

    /// Get a position from a time along this segment
    ///
    /// Panics if passed time is out of bounds for the segment
    pub fn position_unchecked(&self, time: f64) -> Coord<N> {
        match self.path_segment {
            PathSegment::Linear(segment) => {
                let offset = time - self.start_offset;

                assert!(offset >= 0.0);
                assert!(offset <= self.time());

                // segment.start + ((segment.end - segment.start) * (offset / self.len()))

                if offset <= self.acceleration_time {
                    // Acceleration period
                    (segment.start * offset) + (0.5 * self.acceleration * offset)
                } else {
                    // Linear period
                    segment.start + ((segment.end - segment.start) * (offset / self.len()))
                }
            }
        }
    }

    /// Get the first derivative (velocity, gradient) of this segment
    pub fn first_derivative_unchecked(&self, _time: f64) -> Coord<N> {
        match self.path_segment {
            PathSegment::Linear(segment) => (segment.end - segment.start).normalize(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::TestCoord3;
    use crate::Waypoint;

    #[test]
    fn get_linear_position() {
        let path_segment = PathSegment::linear(
            Waypoint::new(
                TestCoord3::new(1.0, 1.0, 1.0),
                TestCoord3::new(0.0, 0.0, 0.0),
            ),
            Waypoint::new(
                TestCoord3::new(2.0, 2.0, 2.0),
                TestCoord3::new(1.0, 1.0, 1.0),
            ),
        );

        let trajectory_segment = TrajectorySegment::new(
            &path_segment,
            TrajectoryOptions {
                velocity_limit: TestCoord3::repeat(1.0),
                acceleration_limit: TestCoord3::repeat(1.0),
            },
            0.0,
        );

        let half = trajectory_segment.len() / 2.0;

        assert_eq!(half, 0.8660254037844386);

        let pos = trajectory_segment.position_unchecked(half);

        assert_eq!(pos, TestCoord3::repeat(1.5));
    }
}
