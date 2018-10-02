use super::PathItem;
use crate::Coord;
use crate::TRAJ_EPSILON;
use nalgebra::allocator::Allocator;
use nalgebra::dimension::DimName;
use nalgebra::DefaultAllocator;
use nalgebra::Real;
use nalgebra::VectorN;
use std::f64;

/// Circular path segment
///
/// Used to blend two straight path segments along a circular path. `x` and `y` form a plane on
/// on which the blend circle lies, with its center at `center`. Radius is radius.
#[derive(Clone, Debug, PartialEq)]
pub struct CircularPathSegment<N: Real, D: DimName>
where
    DefaultAllocator: Allocator<N, D>,
{
    /// Center point of circle
    pub center: VectorN<N, D>,

    /// Radius of circle
    pub radius: N,

    /// First vector along which the blend circle lies
    pub x: VectorN<N, D>,

    /// Second vector along which the blend circle lies
    pub y: VectorN<N, D>,

    /// Length of the arc in radians to use in calculating the blend
    pub arc_length: N,

    /// Path start offset
    pub start_offset: N,
}

impl<N, D> Default for CircularPathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    fn default() -> Self {
        Self {
            arc_length: nalgebra::convert(0.0),
            center: VectorN::<N, D>::zeros(),
            radius: nalgebra::convert(1.0),
            start_offset: nalgebra::convert(0.0),
            x: VectorN::<N, D>::zeros(),
            y: VectorN::<N, D>::zeros(),
        }
    }
}

impl<N, D> CircularPathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Create a blend segment for two line segments comprised of three points
    pub fn from_waypoints(
        previous: &VectorN<N, D>,
        current: &VectorN<N, D>,
        next: &VectorN<N, D>,
        max_deviation: N,
    ) -> Self {
        // If either segment is of negligible length, we don't need to blend it, however a blend
        // is still required to make the path differentiable.
        // TODO: Pass TRAJ_EPSILON as an argument
        if (current - previous).norm() < nalgebra::convert(TRAJ_EPSILON)
            || (next - current).norm() < nalgebra::convert(TRAJ_EPSILON)
        {
            // TODO: Implement Default so this section and others like it are shorter
            return CircularPathSegment {
                center: current.clone(),
                ..Self::default()
            };
        }

        // Yi
        let previous_normalised = (current - previous).normalize();
        let previous_length = (previous - current).norm();
        let previous_half_length = previous_length / nalgebra::convert(2.0);

        let next_normalised = (next - current).normalize();
        let next_length = (next - current).norm();
        let next_half_length = next_length / nalgebra::convert(2.0);

        // If segments are essentially parallel, they don't need blending, however a blend
        // is still required to make the path differentiable.
        if (previous_normalised - next_normalised).norm() < nalgebra::convert(TRAJ_EPSILON) {
            return CircularPathSegment {
                center: current.clone(),
                ..Self::default()
            };
        }

        // ⍺i (outside angle in radians, i.e. 180º - angle)
        let angle = previous_normalised.angle(&next_normalised);

        let radius_limit = (max_deviation * (angle / nalgebra::convert(2.0)).sin())
            / (nalgebra::convert::<f64, N>(1.0) - (angle / nalgebra::convert(2.0)).cos());

        // Li
        let max_blend_distance = previous_half_length.min(next_half_length).min(radius_limit);

        // Ri (radius)
        let radius = max_blend_distance / (angle / nalgebra::convert(2.0)).tan();

        // Ci (center)
        let center = current
            + (next_normalised - previous_normalised).normalize()
                * (radius / (angle / nalgebra::convert(2.0)).cos());

        // Xi (points from center of circle to point where circle touches previous segment)
        let x = (*current - previous_normalised * max_blend_distance - center).normalize();
        // Yi (direction of previous segment)
        let y = previous_normalised;

        let arc_length = angle * radius;

        CircularPathSegment {
            center,
            radius,
            x,
            y,
            arc_length,
            start_offset: nalgebra::convert(0.0),
        }
    }

    /// Clone with a start offset
    pub fn with_start_offset(&self, start_offset: N) -> Self {
        Self {
            start_offset,
            ..self.clone()
        }
    }

    /// Get switching points for circular segment
    ///
    /// A segment can have a switching point for each dimension at various points along its path.
    /// Takes into account the path's start offset
    // TODO: Trait
    pub fn get_switching_points(&self) -> Vec<N> {
        // Loop through each _component_ of unit vectors X and Y
        let mut switching_points = self
            .x
            .iter()
            .zip(self.y.iter())
            .filter_map(|(x, y)| {
                let mut switching_angle = y.atan2(*x);

                if switching_angle < nalgebra::convert(0.0) {
                    switching_angle += nalgebra::convert(f64::consts::PI);
                }

                let switching_point = switching_angle * self.radius;

                if switching_point < self.arc_length {
                    Some(switching_point)
                } else {
                    None
                }
            })
            .collect::<Vec<N>>();

        switching_points
            .sort_unstable_by(|a, b| a.partial_cmp(b).expect("Could not sort switching points"));

        switching_points
    }
}

impl<N, D> PathItem<N, D> for CircularPathSegment<N, D>
where
    N: Real,
    D: DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Get the arc length of this segment
    fn get_length(&self) -> N {
        self.arc_length
    }

    /// Get position ("robot configuration" in paper parlance) along arc from normalised distance
    /// along it (`s`)
    fn get_position(&self, distance_along_arc: N) -> VectorN<N, D> {
        let angle = (distance_along_arc - self.start_offset) / self.radius;

        self.center + ((self.x * angle.cos()) + (self.y * angle.sin())) * self.radius
    }

    /// Get derivative (tangent) of point along curve
    fn get_tangent(&self, distance_along_arc: N) -> VectorN<N, D> {
        let angle = (distance_along_arc - self.start_offset) / self.radius;

        -self.x * angle.sin() + self.y * angle.cos()
    }

    /// Get second derivative (rate of change of tangent, aka curvature) of point along curve
    fn get_curvature(&self, distance_along_arc: N) -> VectorN<N, D> {
        use std::ops::Mul;

        let angle = (distance_along_arc - self.start_offset) / self.radius;

        (self.x * angle.cos() + self.y * angle.sin())
            .mul(nalgebra::convert::<f64, N>(-1.0) / self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn it_gets_switching_points() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(1.0, 5.0, 0.0);
        let after = Coord::new(5.0, 5.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        let _thing = blend_circle.get_switching_points();

        debug_blend(
            "it_gets_switching_points",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        assert!(true);
    }

    #[test]
    fn it_gets_the_position() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, 5.0, 0.0);
        let after = Coord::new(5.0, 10.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend_position("it_gets_the_position", &blend_circle);
    }

    #[test]
    /// Compute the circular blend for an arrow head sitting on the X axis
    ///
    /// /\
    fn it_computes_right_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(5.0, 5.0, 0.0);
        let after = Coord::new(10.0, 0.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend(
            "it_computes_right_angles",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle;

        assert_near!(bc.arc_length, 0.37922377958740805);
        assert_near!(bc.center, Coord::new(5.0, 4.658578643762691, 0.0));
        assert_near!(bc.radius, 0.24142135623730956);
        assert_near!(bc.x, Coord::new(-0.70710678118654, 0.70710678118654, 0.0));
        assert_near!(bc.y, Coord::new(0.70710678118654, 0.70710678118654, 0.0));
    }

    #[test]
    /// Compute the circular blend for a unit arrow head pointing North West
    ///
    ///  _
    /// |
    fn it_computes_more_right_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, 1.0, 0.0);
        let after = Coord::new(1.0, 1.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend(
            "it_computes_more_right_angles",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle;

        assert_near!(bc.arc_length, 0.37922377958740805);
        assert_near!(bc.center, Coord::new(0.2414213562373, 0.758578643762, 0.0));
        assert_near!(bc.radius, 0.24142135623730956);
        assert_near!(bc.x, Coord::new(-1.0, 0.0, 0.0));
        assert_near!(bc.y, Coord::new(0.0, 1.0, 0.0));
    }

    #[test]
    /// Compute a 45º blend
    ///
    ///  /
    /// |
    fn it_computes_45_degree_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, 5.0, 0.0);
        let after = Coord::new(5.0, 10.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend(
            "it_computes_45_degree_angles",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle;

        assert_near!(bc.arc_length, 0.9532433417365019);
        assert_near!(
            bc.center,
            Coord::new(1.2137071184544088, 4.497266050787415, 0.0)
        );
        assert_near!(bc.radius, 1.2137071184544088);
        assert_near!(bc.x, Coord::new(-1.0, 0.0, 0.0));
        assert_near!(bc.y, Coord::new(0.0, 1.0, 0.0));
    }

    #[test]
    /// Allow large deviations
    ///
    ///  /
    /// |
    fn it_works_with_large_max_deviations() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, 5.0, 0.0);
        let after = Coord::new(5.0, 10.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 1.0);

        debug_blend(
            "it_works_with_large_max_deviations",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle;

        assert_near!(bc.arc_length, 4.740297244842599);
        assert_near!(bc.center, Coord::new(6.035533905932737, 2.5, 0.0));
        assert_near!(bc.radius, 6.035533905932737);
        assert_near!(bc.x, Coord::new(-1.0, 0.0, 0.0));
        assert_near!(bc.y, Coord::new(0.0, 1.0, 0.0));
    }

    #[test]
    /// Ignore blends for straight vertical lines
    ///
    /// |
    /// |
    fn it_computes_0_degree_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, 5.0, 0.0);
        let after = Coord::new(0.0, 10.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend(
            "it_computes_0_degree_angles",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        assert_eq!(
            blend_circle,
            CircularPathSegment {
                center: current,
                ..CircularPathSegment::default()
            }
        );
    }

    #[test]
    /// Ignore blend for straight but diagonal lines
    ///
    ///  /
    /// /
    fn it_computes_straight_diagonals() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(2.0, 2.0, 0.0);
        let after = Coord::new(4.0, 4.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend(
            "it_computes_straight_diagonals",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        assert_eq!(
            blend_circle,
            CircularPathSegment {
                center: current,
                ..CircularPathSegment::default()
            }
        );
    }

    #[test]
    /// Ignore blend for tiny lines
    ///
    /// |
    /// |
    fn it_computes_tiny_blends() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, TRAJ_EPSILON / 2.0, 0.0);
        let after = Coord::new(0.0, TRAJ_EPSILON, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        assert_eq!(
            blend_circle,
            CircularPathSegment {
                center: current,
                ..CircularPathSegment::default()
            }
        );
    }

    #[test]
    /// Really shallow angle blends
    ///
    ///  /
    /// |
    fn it_computes_blends_for_shallow_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(10.0, 7.0, 0.0);
        let after = Coord::new(20.0, 4.0, 0.0);

        let blend_circle = CircularPathSegment::from_waypoints(&before, &current, &after, 0.1);

        debug_blend(
            "it_computes_blends_for_shallow_angles",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle;

        assert_near!(bc.arc_length, 0.8117106237578);
        assert_near!(bc.center, Coord::new(10.158912720301, 6.012992370967, 0.0));
        assert_near!(bc.radius, 0.8997186166327);
        assert_near!(bc.x, Coord::new(-0.5734623443633, 0.8192319205190, 0.0));
        assert_near!(bc.y, Coord::new(0.8192319205190, 0.5734623443633, 0.0));
    }
}
