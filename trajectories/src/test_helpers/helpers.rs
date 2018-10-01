//! Test helpers

use csv;
pub use crate::path::CircularPathSegment;
use crate::path::PathItem;
use crate::path::{Continuity, Path as TrajPath, PathSegment};
use std::fmt;
use std::fs::File;
use svg;
use svg::node::element::path::Data;
use svg::node::element::{Circle, Group, Path as SvgPath, Rectangle, Text};
use svg::node::Text as TextContent;
use svg::Document;
use crate::Coord;

const PADDING: f64 = 1.0;

fn single_line(from: &Coord, to: &Coord, stroke: &str, stroke_width: u32) -> SvgPath {
    SvgPath::new()
        .set("fill", "none")
        .set("stroke", stroke)
        .set("stroke-width", stroke_width)
        .set("vector-effect", "non-scaling-stroke")
        .set(
            "d",
            Data::new().move_to((from.x, from.y)).line_to((to.x, to.y)),
        )
}

fn cross_centered_at(center: &Coord, stroke: &str, stroke_width: f32) -> SvgPath {
    let size = 0.1;

    SvgPath::new()
        .set("fill", "none")
        .set("stroke", stroke)
        .set("stroke-width", stroke_width)
        .set("vector-effect", "non-scaling-stroke")
        .set(
            "d",
            Data::new()
                .move_to((center.x, center.y - size))
                .line_by((0, size * 2.0))
                .move_to((center.x - size, center.y))
                .line_by((size * 2.0, 0)),
        )
}

fn border(top_left: &Coord, bottom_right: &Coord) -> Rectangle {
    Rectangle::new()
        .set("fill", "none")
        .set("stroke", "black")
        .set("stroke-width", 1)
        .set("vector-effect", "non-scaling-stroke")
        .set("x", top_left.x)
        .set("y", top_left.y)
        .set("width", bottom_right.x)
        .set("height", bottom_right.y)
}

fn create_document(top_left: &Coord, bottom_right: &Coord) -> Document {
    let aspect = (bottom_right.x - top_left.x) / (bottom_right.y - top_left.y);
    let width = 1024;

    Document::new()
        .set("shape-rendering", "geometricPrecision")
        .set("width", width)
        .set("height", (width as f64 * aspect) as u32)
        .set(
            "viewBox",
            (top_left.x, top_left.y, bottom_right.x, bottom_right.y),
        )
        .add(border(&top_left, &bottom_right))
}

fn save_document(suite_name: &str, doc: &Document) {
    svg::save(format!("../target/{}.svg", suite_name), doc).unwrap();
}

fn draw_blend_circle(blend: &CircularPathSegment) -> Group {
    let line_scale = 0.25;

    // Blend circle
    let circle = Circle::new()
        .set("cx", blend.center.x)
        .set("cy", blend.center.y)
        .set("stroke-width", 1)
        .set("stroke", "blue")
        .set("fill", "none")
        .set("vector-effect", "non-scaling-stroke")
        .set("r", blend.radius);

    // Xi (green)
    let xi = single_line(
        &blend.center,
        &(blend.center + blend.x * line_scale),
        "green",
        1,
    );

    // Yi (purple)
    let yi = single_line(
        &blend.center,
        &(blend.center + blend.y * line_scale),
        "purple",
        1,
    );

    Group::new().add(circle).add(xi).add(yi)
}

fn calc_bbox(coords: &[Coord]) -> (Coord, Coord) {
    (
        coords
            .iter()
            .min_by_key(|item| ((item.x + item.y + item.z) * 100.0) as u32)
            .unwrap()
            - Coord::repeat(PADDING),
        coords
            .iter()
            .max_by_key(|item| ((item.x + item.y + item.z) * 100.0) as u32)
            .unwrap()
            + Coord::repeat(PADDING * 2.0),
    )
}

/// Produce an image of the blend between two path segments
pub fn debug_blend(
    p: &str,
    before: &Coord,
    current: &Coord,
    after: &Coord,
    blend: &CircularPathSegment,
) {
    let path_before = single_line(&before, &current, "red", 1);
    let path_after = single_line(&current, &after, "red", 1);
    let blend = draw_blend_circle(blend);

    let (top_left, bottom_right) = calc_bbox(&[*before, *current, *after]);

    let document = create_document(&top_left, &bottom_right)
        .add(blend)
        .add(path_before)
        .add(path_after);

    svg::save(format!("../target/{}.svg", p), &document).unwrap();
}

/// Draw points along a blend curve
pub fn debug_blend_position(p: &str, blend: &CircularPathSegment) {
    let top_left = blend.center - Coord::repeat(blend.radius + PADDING);
    let bottom_right = blend.center + Coord::repeat(blend.radius + PADDING * 2.0);

    let circle = draw_blend_circle(&blend);

    let start = blend.get_position(0.0);

    let mut i = 0.0;
    let mut data = Data::new().move_to((start.x, start.y));

    while i <= blend.get_length() {
        i += 0.1;

        let pos = blend.get_position(i);

        data = data.line_to((pos.x, pos.y));
    }

    let path = SvgPath::new()
        .set("fill", "none")
        .set("stroke", "red")
        .set("stroke-width", 4)
        .set("vector-effect", "non-scaling-stroke")
        .set("d", data.clone());

    let document = create_document(&top_left, &bottom_right)
        .add(circle)
        .add(path);

    save_document(p, &document);
}

/// Debug an entire path
pub fn debug_path(file_path: &'static str, path: &TrajPath, waypoints: &Vec<Coord>) {
    let (top_left, bottom_right) = calc_bbox(waypoints.as_slice());

    let mut document = create_document(&top_left, &bottom_right);

    // Original waypoints in background to compare
    for parts in waypoints.windows(2) {
        if let &[curr, next] = parts {
            document = document.add(single_line(&curr, &next, "black", 1));
        } else {
            panic!("Debug blend path failed");
        }
    }

    // Generated segments with blends
    for segment in path.segments.iter() {
        match segment {
            PathSegment::Linear(ref line) => {
                document = document
                    .add(single_line(&line.start, &line.end, "red", 3))
                    // Print start offset for line
                    .add(
                        Text::new()
                            .set("x", line.start.x)
                            .set("y", line.start.y)
                            .set("fill", "red")
                            .set("style", "font-size: 0.18px; font-family: monospace")
                            .add(TextContent::new(format!(
                                "Line offs. {:.*}",
                                3, line.start_offset
                            ))),
                    )
            }

            PathSegment::Circular(ref circ) => {
                document = document
                    .add(draw_blend_circle(&circ))
                    // Print start offset for arc
                    .add({
                        let start = circ.get_position(0.0);

                        Text::new()
                            .set("x", start.x)
                            .set("y", start.y)
                            .set("fill", "blue")
                            .set("style", "font-size: 0.18px; font-family: monospace")
                            .add(TextContent::new(format!(
                                "Circ offs. {:.*}",
                                3, circ.start_offset
                            )))
                    })
            }
        }
    }

    save_document(file_path, &document);
}

/// Draw switching points on a path
pub fn debug_path_switching_points(
    file_path: &'static str,
    path: &TrajPath,
    waypoints: &Vec<Coord>,
) {
    let (top_left, bottom_right) = calc_bbox(waypoints.as_slice());

    let mut document = create_document(&top_left, &bottom_right);

    // Generated segments with blends
    for segment in path.segments.iter() {
        match segment {
            PathSegment::Linear(ref line) => {
                document = document.add(single_line(&line.start, &line.end, "red", 1))
            }

            // PathSegment::Circular(ref circ) => document = document.add(draw_blend_circle(&circ)),
            PathSegment::Circular(_) => (),
        }
    }

    // Switching points
    for point in path.get_switching_points() {
        let pos = path.get_position(point.position);

        let col = match point.continuity {
            Continuity::Continuous => "green",
            Continuity::Discontinuous => "orange",
        };

        document = document.add(cross_centered_at(&pos, col, 0.5));
    }

    save_document(file_path, &document);
}

/// Draw a complete path with a given point marked on it
pub fn debug_path_point(
    file_path: &'static str,
    path: &TrajPath,
    waypoints: &Vec<Coord>,
    point: &Coord,
) {
    let (top_left, bottom_right) = calc_bbox(&waypoints);

    let mut document = create_document(&top_left, &bottom_right);

    // Generated segments with blends
    for segment in path.segments.iter() {
        match segment {
            PathSegment::Linear(ref line) => {
                document = document
                    .add(single_line(&line.start, &line.end, "red", 3))
                    // Print start offset for line
                    .add(
                        Text::new()
                            .set("x", line.start.x)
                            .set("y", line.start.y)
                            .set("fill", "red")
                            .set("style", "font-size: 0.18px; font-family: monospace")
                            .add(TextContent::new(format!(
                                "Line offs. {:.*}",
                                3, line.start_offset
                            ))),
                    )
            }

            PathSegment::Circular(ref circ) => {
                document = document
                    .add(draw_blend_circle(&circ))
                    // Print start offset for arc
                    .add({
                        let start = circ.get_position(0.0);

                        Text::new()
                            .set("x", start.x)
                            .set("y", start.y)
                            .set("fill", "blue")
                            .set("style", "font-size: 0.18px; font-family: monospace")
                            .add(TextContent::new(format!(
                                "Circ offs. {:.*}",
                                3, circ.start_offset
                            )))
                    })
            }
        }
    }

    // Point to debug
    document = document.add(cross_centered_at(&point, "orange", 1.0));

    save_document(file_path, &document);
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
    pub fn from_parts(time: f64, pos: &TestPoint, vel: &TestPoint) -> Self {
        Self {
            time,
            position_x: pos.x,
            position_y: pos.y,
            position_z: pos.z,
            velocity_x: vel.x,
            velocity_y: vel.y,
            velocity_z: vel.z,
        }
    }

    /// Create a row from a time and position and velocity vectors
    pub fn from_coords(time: f64, pos: &Coord, vel: &Coord) -> Self {
        Self {
            time,
            position_x: pos[0],
            position_y: pos[1],
            position_z: pos[2],
            velocity_x: vel[0],
            velocity_y: vel[1],
            velocity_z: vel[2],
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
