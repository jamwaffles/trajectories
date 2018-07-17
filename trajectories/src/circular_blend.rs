use super::{CircularPathSegment, Coord, MIN_ACCURACY};
use std::f64;

pub fn compute_circular_blend(
    previous: &Coord,
    current: &Coord,
    next: &Coord,
    max_deviation: f64,
) -> Option<CircularPathSegment> {
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

    let x = (current - max_blend_distance * previous_normalised - center).normalize();
    let y = previous_normalised;

    let length = angle * radius;

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
        length,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    use image::{Rgb, RgbImage};
    use imageproc::drawing::{
        draw_cross_mut, draw_filled_rect_mut, draw_hollow_circle_mut, draw_line_segment_mut,
    };
    use imageproc::rect::Rect;
    use std::path::Path;

    fn debug_blend(
        p: &str,
        before: &Coord,
        current: &Coord,
        after: &Coord,
        blend: &Option<CircularPathSegment>,
    ) {
        let path = Path::new(p);

        let red = Rgb([255u8, 0u8, 0u8]);
        let green = Rgb([0u8, 255u8, 0u8]);
        let blue = Rgb([0u8, 0u8, 255u8]);
        let white = Rgb([255u8, 255u8, 255u8]);

        let scale = 20.0;
        let padding = 10;
        let max_dim = (before.amax().max(current.amax()).max(after.amax()) * scale) as u32;

        let mut image = RgbImage::new(max_dim + padding * 2, max_dim + padding * 2);

        draw_filled_rect_mut(
            &mut image,
            Rect::at(0, 0).of_size(max_dim + padding * 2, max_dim + padding * 2),
            white,
        );

        let xform = |input: f64| -> f32 { (input as f32 * scale as f32) + padding as f32 };

        draw_line_segment_mut(
            &mut image,
            (xform(before.x), xform(before.y)),
            (xform(current.x), xform(current.y)),
            red,
        );

        draw_line_segment_mut(
            &mut image,
            (xform(current.x), xform(current.y)),
            (xform(after.x), xform(after.y)),
            red,
        );

        // Render blend circle if there is one, or a cross if there isn't
        if let Some(bl) = blend {
            draw_hollow_circle_mut(
                &mut image,
                (xform(bl.center.x) as i32, xform(bl.center.y) as i32),
                (bl.radius * scale) as i32,
                blue,
            );

            // Render othonormal vectors Xi and Yi (these point to the midpoints of each line segment)
            draw_line_segment_mut(
                &mut image,
                (xform(bl.center.x), xform(bl.center.y)),
                (xform(bl.center.x + bl.x.x), xform(bl.center.y + bl.x.y)),
                green,
            );
            draw_line_segment_mut(
                &mut image,
                (xform(bl.center.x), xform(bl.center.y)),
                (xform(bl.center.x + bl.y.x), xform(bl.center.y + bl.y.y)),
                green,
            );
        } else {
            draw_cross_mut(
                &mut image,
                blue,
                xform(current.x) as i32,
                xform(current.y) as i32,
            );
        }

        image.save(path).unwrap();
    }

    #[test]
    /// Compute the circular blend for an arrow head sitting on the X axis
    ///
    /// /\
    fn it_computes_right_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(5.0, 5.0, 0.0);
        let after = Coord::new(10.0, 0.0, 0.0);

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_right_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        println!("{:?}", blend_circle);
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

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_more_right_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        println!("{:?}", blend_circle);
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

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_45_degree_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );

        println!("{:?}", blend_circle);
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

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

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

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

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

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

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

        let blend_circle = compute_circular_blend(&before, &current, &after, 0.1);

        debug_blend(
            "../target/it_computes_blends_for_shallow_angles.png",
            &before,
            &current,
            &after,
            &blend_circle,
        );
    }
}
