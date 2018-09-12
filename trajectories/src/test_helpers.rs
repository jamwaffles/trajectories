//! Test helpers

use csv;
use image::{Rgb, RgbImage};
use imageproc::drawing::{
    draw_cross_mut, draw_filled_rect_mut, draw_hollow_circle_mut, draw_line_segment_mut,
};
use imageproc::rect::Rect;
use std::fmt;
use std::fs::File;
use std::path::Path;

use super::{CircularPathSegment, Coord};

/// Produce an image of the blend between two path segments
pub fn debug_blend(
    p: &str,
    before: &Coord,
    current: &Coord,
    after: &Coord,
    blend: &Option<CircularPathSegment>,
) {
    let path = Path::new(p);

    let red = Rgb([255u8, 0u8, 0u8]);
    let black = Rgb([0u8, 0, 0]);
    let green = Rgb([0u8, 255u8, 0u8]);
    let purple = Rgb([127u8, 0u8, 255u8]);
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

        // Xi (green)
        draw_line_segment_mut(
            &mut image,
            (xform(bl.center.x), xform(bl.center.y)),
            (xform(bl.center.x + bl.x.x), xform(bl.center.y + bl.x.y)),
            green,
        );
        // Yi (purple)
        draw_line_segment_mut(
            &mut image,
            (xform(bl.center.x), xform(bl.center.y)),
            (xform(bl.center.x + bl.y.x), xform(bl.center.y + bl.y.y)),
            purple,
        );
    } else {
        draw_cross_mut(
            &mut image,
            black,
            xform(current.x) as i32,
            xform(current.y) as i32,
        );
    }

    image.save(path).unwrap();
}

/// Can't remember what this does
pub fn debug_blend_position(p: &str, blend: &CircularPathSegment) {
    let path = Path::new(p);
    let mut i = 0.0;

    let scale = 100.0f64;
    let padding = 10;
    let max_dim = ((blend.radius * 2.0) * scale) as u32;

    let red = Rgb([255u8, 0u8, 0u8]);
    let green = Rgb([0u8, 255u8, 0u8]);
    let purple = Rgb([127u8, 0u8, 255u8]);
    let blue = Rgb([0u8, 0u8, 255u8]);
    let white = Rgb([255u8, 255u8, 255u8]);

    let xform_center = |input: f64| -> f32 { ((input * scale) + padding as f64) as f32 };

    let mut image = RgbImage::new(max_dim + padding * 2, max_dim + padding * 2);

    draw_filled_rect_mut(
        &mut image,
        Rect::at(0, 0).of_size(max_dim + padding * 2, max_dim + padding * 2),
        white,
    );

    draw_hollow_circle_mut(
        &mut image,
        (
            xform_center(blend.radius) as i32,
            xform_center(blend.radius) as i32,
        ),
        (blend.radius * scale) as i32,
        blue,
    );

    while i <= 1.0 {
        let pos = blend.get_position(i);

        draw_cross_mut(
            &mut image,
            red,
            (pos.x * 50.0) as i32,
            (pos.y * 50.0) as i32 - 100,
        );

        i += 0.01;
    }

    image.save(path).unwrap();
}

/// 3 element vector for testing C++ bindings
#[derive(Debug)]
pub struct TestPoint {
    x: f64,
    y: f64,
    z: f64,
}

impl From<[f64; 3]> for TestPoint {
    fn from(other: [f64; 3]) -> Self {
        TestPoint {
            x: other[0],
            y: other[1],
            z: other[2],
        }
    }
}

impl fmt::Display for TestPoint {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}, {}, {}", self.x, self.y, self.z)
    }
}

/// Container to output a debug CSV
#[derive(Serialize, Debug)]
pub struct TrajectoryStepRow {
    time: f64,
    position_x: f64,
    position_y: f64,
    position_z: f64,
    velocity_x: f64,
    velocity_y: f64,
    velocity_z: f64,
}

impl TrajectoryStepRow {
    /// Make a point from separate parts
    pub fn from_parts(time: f64, pos: &TestPoint, acc: &TestPoint) -> Self {
        Self {
            time,
            position_x: pos.x,
            position_y: pos.y,
            position_z: pos.z,
            velocity_x: acc.x,
            velocity_y: acc.y,
            velocity_z: acc.z,
        }
    }
}

/// Write a bunch of debug info to a CSV file
pub fn write_debug_csv(path: String, rows: &Vec<TrajectoryStepRow>) {
    let mut wtr = csv::Writer::from_writer(File::create(path).unwrap());

    for row in rows {
        wtr.serialize(row).expect("Could not serialize")
    }

    wtr.flush().expect("Flush");
}
