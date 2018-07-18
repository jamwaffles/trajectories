use super::{Coord, MIN_ACCURACY};

/// Circular path segment
///
/// Used to blend two straight path segments along a circular path. `x` and `y` form a plane on
/// on which the blend circle lies, with its center at `center`. Radius is radius.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CircularPathSegment {
    /// Center point of circle
    pub center: Coord,

    /// Radius of circle
    pub radius: f64,

    /// First vector along which the blend circle lies
    pub x: Coord,

    /// Second vector along which the blend circle lies
    pub y: Coord,

    /// Length of the arc to use in calculating the blend
    pub arc_length: f64,
}

impl CircularPathSegment {
    pub fn from_path_segments(
        previous: &Coord,
        current: &Coord,
        next: &Coord,
        max_deviation: f64,
    ) -> Option<Self> {
        // If either segment is of negligible length, we don't need to blend it
        if (current - previous).norm() < MIN_ACCURACY || (next - current).norm() < MIN_ACCURACY {
            return None;
        }

        // Yi
        let previous_normalised = (current - previous).normalize();
        let previous_length = (previous - current).norm();
        let previous_half_length = previous_length / 2.0;

        let next_normalised = (next - current).normalize();
        let next_length = (current - next).norm();
        let next_half_length = next_length / 2.0;

        // If segments are essentially parallel, they don't need blending
        if (previous_normalised - next_normalised).norm() < MIN_ACCURACY {
            return None;
        }

        // ⍺i (outside angle in radians, i.e. 180º - angle)
        let angle = previous_normalised.angle(&next_normalised);

        let radius_limit = (max_deviation * (angle / 2.0).sin()) / (1.0 - (angle / 2.0).cos());

        // Li
        let max_blend_distance = previous_half_length.min(next_half_length).min(radius_limit);

        // Ri (radius)
        let radius = max_blend_distance / (angle / 2.0).tan();

        // Ci (center)
        let center = current
            + (next_normalised - previous_normalised).normalize() * (radius / (angle / 2.0).cos());

        // Xi (points from center of circle to point where circle touches previous segment)
        // TODO: This seems to be perpendicular to Yi. Maybe I could optimise?
        let x = (current - max_blend_distance * previous_normalised - center).normalize();
        // Yi (direction of previous segment)
        let y = previous_normalised;

        let arc_length = angle * radius;

        // println!("\n");
        // println!("--- Yi (previous normalised) {:?}", previous_normalised);
        // println!("--- next normalised {:?}", next_normalised);
        // println!("--- Alphai (angle in radians) {:?}", angle);
        // println!(
        //     "--- Alphai (angle in degrees) {:?}",
        //     angle * (180.0 / f64::consts::PI)
        // );
        // println!(
        //     "--- Li (max deviation) {:?} from (prev_half_len {}, next_half_len {}, angle {}, rad_lim {})",
        //     max_blend_distance, previous_half_length, next_half_length, angle, radius_limit
        // );
        // println!("--- Ri (radius) {:?}", radius);
        // println!("--- Ci (center) {:?}", center);
        // println!("--- Length {:?}", length);
        // println!("--- (Xi, Yi) {:?} {:?}", x, y);
        // println!("\n");

        Some(CircularPathSegment {
            center,
            radius,
            x,
            y,
            arc_length,
        })
    }

    /// Get position ("robot configuration" in paper parlance) along arc from normalised distance
    /// along it (`s`)
    pub fn get_position(&self, distance_along_arc: f64) -> Coord {
        self.center
            + self.radius
                * ((self.x * (distance_along_arc / self.radius).cos())
                    + (self.y * (distance_along_arc / self.radius).sin()))
    }

    /// Get derivative (tangent) of point along curve
    pub fn get_tangent(&self, distance_along_arc: f64) -> Coord {
        -self.x * (distance_along_arc / self.radius).sin()
            + self.y * (distance_along_arc / self.radius).cos()
    }

    /// Get second derivative (rate of change of tangent, aka curvature) of point along curve
    pub fn get_curvature(&self, distance_along_arc: f64) -> Coord {
        -(1.0 / self.radius)
            * (self.x * (distance_along_arc / self.radius).sin()
                + self.y * (distance_along_arc / self.radius).cos())
    }
}

#[cfg(test)]
mod tests {
    use super::super::test_helpers::*;
    use super::*;

    #[test]
    fn it_gets_the_position() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, 5.0, 0.0);
        let after = Coord::new(5.0, 10.0, 0.0);

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1)
            .expect("Expected a blend circle");

        debug_blend_position("../target/it_gets_the_position.png", &blend_circle);
    }

    #[test]
    /// Compute the circular blend for an arrow head sitting on the X axis
    ///
    /// /\
    fn it_computes_right_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(5.0, 5.0, 0.0);
        let after = Coord::new(10.0, 0.0, 0.0);

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_right_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle.expect("Expected a blend circle");

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

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_more_right_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle.expect("Expected a blend circle");

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

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_45_degree_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle.expect("Expected a blend circle");

        assert_near!(bc.arc_length, 0.9532433417365019);
        assert_near!(bc.center, Coord::new(1.2137071, 4.4972660, 0.0));
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

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 1.0);

        debug_blend(
            "../target/it_works_with_large_max_deviations.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle.expect("Expected a blend circle");

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

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_0_degree_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        assert!(blend_circle.is_none());
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

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_straight_diagonals.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        assert!(blend_circle.is_none());
    }

    #[test]
    /// Ignore blend for tiny lines
    ///
    /// |
    /// |
    fn it_computes_tiny_blends() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(0.0, MIN_ACCURACY / 2.0, 0.0);
        let after = Coord::new(0.0, MIN_ACCURACY, 0.0);

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        assert!(blend_circle.is_none());
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

        let blend_circle = CircularPathSegment::from_path_segments(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_blends_for_shallow_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        let bc = blend_circle.expect("Expected a blend circle");

        assert_near!(bc.arc_length, 0.8117106237578);
        assert_near!(bc.center, Coord::new(10.158912720301, 6.012992370967, 0.0));
        assert_near!(bc.radius, 0.8997186166327);
        assert_near!(bc.x, Coord::new(-0.5734623443633, 0.8192319205190, 0.0));
        assert_near!(bc.y, Coord::new(0.8192319205190, 0.5734623443633, 0.0));
    }
}
