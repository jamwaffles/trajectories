use super::{CircularPathSegment, Coord};
use std::f64;

pub fn compute_circular_blend(
    previous: &Coord,
    current: &Coord,
    next: &Coord,
    max_deviation: f64,
) -> CircularPathSegment {
    // Yi
    let previous_normalised = (current - previous).normalize();
    let previous_length = (previous - current).norm();
    let previous_half_length = previous_length / 2.0;

    let next_normalised = (next - current).normalize();
    let next_length = (current - next).norm();
    let next_half_length = next_length / 2.0;

    // ⍺i (outside angle in radians, i.e. 180º - angle)
    // let angle = f64::consts::PI - previous_normalised.angle(&next_normalised);
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

    println!("\n");
    println!("--- Yi (previous normalised) {:?}", previous_normalised);
    println!("--- next normalised {:?}", next_normalised);
    println!("--- Alphai (angle in radians) {:?}", angle);
    println!(
        "--- Alphai (angle in degrees) {:?}",
        angle * (180.0 / f64::consts::PI)
    );
    println!(
        "--- Li (max deviation) {:?} from (prev_half_len {}, next_half_len {}, angle {}, rad_lim {})",
        max_blend_distance, previous_half_length, next_half_length, angle, radius_limit
    );
    println!("--- Ri (radius) {:?}", radius);
    println!("--- Ci (center) {:?}", center);
    println!("--- (Xi, Yi) {:?} {:?}", x, y);
    println!("\n");

    CircularPathSegment::new(center, radius, x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Compute the circular blend for an arrow head sitting on the X axis
    ///
    /// /\
    fn it_computes_right_angles() {
        let before = Coord::new(0.0, 0.0, 0.0);
        let current = Coord::new(5.0, 5.0, 0.0);
        let after = Coord::new(10.0, 0.0, 0.0);

        compute_circular_blend(&before, &current, &after, 0.1);
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

        compute_circular_blend(&before, &current, &after, 0.1);
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

        compute_circular_blend(&before, &current, &after, 0.1);
    }
}
