//! Test helpers

use super::{CircularPathSegment, Coord};
use csv;
use path::{Path as TrajPath, PathSegment};
use std::fmt;
use std::fs::File;
use svg;
use svg::node::element::path::Data;
use svg::node::element::{Circle, Path as SvgPath, Rectangle, Text};
use svg::node::Text as TextContent;
use svg::Document;
use PathItem;

fn blend_circle(blend: &CircularPathSegment, width: u32) -> Circle {
    Circle::new()
        .set("cx", blend.center.x)
        .set("cy", blend.center.y)
        .set("stroke-width", width)
        .set("stroke", "blue")
        .set("fill", "none")
        .set("vector-effect", "non-scaling-stroke")
        .set("r", blend.radius)
}

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

fn cross_centered_at(center: &Coord, stroke: &str, stroke_width: u32) -> SvgPath {
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
    Document::new()
        .set("shape-rendering", "geometricPrecision")
        .set("width", 1024)
        .set("height", 768)
        .set(
            "viewBox",
            (top_left.x, top_left.y, bottom_right.x, bottom_right.y),
        ).add(border(&top_left, &bottom_right))
}

/// Produce an image of the blend between two path segments
pub fn debug_blend(
    p: &str,
    before: &Coord,
    current: &Coord,
    after: &Coord,
    blend: &CircularPathSegment,
) {
    let padding = 1.0;

    let data = Data::new()
        .move_to((0, 0))
        .line_to((before.x, before.y))
        .line_to((current.x, current.y))
        .line_to((after.x, after.y));

    let top_left = before - Coord::repeat(padding);
    let bottom_right = after + Coord::repeat(padding * 2.0);

    // The two path segments
    let path = SvgPath::new()
        .set("fill", "none")
        .set("stroke", "red")
        .set("stroke-width", 1)
        .set("vector-effect", "non-scaling-stroke")
        .set("d", data);

    // Blend circle
    let circle = blend_circle(&blend, 1);

    // Xi (green)
    let xi = single_line(&blend.center, &(blend.center + blend.x), "green", 1);

    // Yi (purple)
    let yi = single_line(&blend.center, &(blend.center + blend.y), "purple", 1);

    let document = create_document(&top_left, &bottom_right)
        .add(circle)
        .add(xi)
        .add(yi)
        .add(path);

    svg::save(format!("{}.svg", p), &document).unwrap();
}

/// Draw points along a blend curve
pub fn debug_blend_position(p: &str, blend: &CircularPathSegment) {
    let padding = 1.0;

    let top_left = blend.center - Coord::repeat(blend.radius + padding);
    let bottom_right = blend.center + Coord::repeat(blend.radius + padding * 2.0);

    let circle = blend_circle(&blend, 1);

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

    svg::save(format!("{}.svg", p), &document).unwrap();
}

/// Debug an entire path
pub fn debug_path(file_path: &'static str, path: &TrajPath, waypoints: &Vec<Coord>) {
    let padding = 1.0;

    let top_left = Coord::repeat(0.0) - Coord::repeat(padding);
    let bottom_right = Coord::repeat(10.0) + Coord::repeat(padding * 2.0);

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
                            .add(TextContent::new(format!("Line offs. {:.*}", 3, line.start_offset))),
                    )
            }

            PathSegment::Circular(ref circ) => {
                document = document
                    .add(blend_circle(&circ, 1))
                    // Print start offset for arc
                    .add({
                        let start = circ.get_position(0.0);

                        Text::new()
                            .set("x", start.x)
                            .set("y", start.y)
                            .set("fill", "blue")
                            .set("style", "font-size: 0.18px; font-family: monospace")
                            .add(TextContent::new(format!("Circ offs. {:.*}", 3, circ.start_offset)))
                    })
            }
        }
    }

    svg::save(format!("{}.svg", file_path), &document).unwrap();
}

/// Draw a complete path with a given point marked on it
pub fn debug_path_point(
    file_path: &'static str,
    path: &TrajPath,
    _waypoints: &Vec<Coord>,
    point: &Coord,
) {
    let padding = 1.0;

    let top_left = Coord::repeat(0.0) - Coord::repeat(padding);
    let bottom_right = Coord::repeat(10.0) + Coord::repeat(padding * 2.0);

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
                            .add(TextContent::new(format!("Line offs. {:.*}", 3, line.start_offset))),
                    )
            }

            PathSegment::Circular(ref circ) => {
                document = document
                    .add(blend_circle(&circ, 1))
                    // Print start offset for arc
                    .add({
                        let start = circ.get_position(0.0);

                        Text::new()
                            .set("x", start.x)
                            .set("y", start.y)
                            .set("fill", "blue")
                            .set("style", "font-size: 0.18px; font-family: monospace")
                            .add(TextContent::new(format!("Circ offs. {:.*}", 3, circ.start_offset)))
                    })
            }
        }
    }

    // Point to debug
    document = document.add(cross_centered_at(&point, "orange", 1));

    svg::save(format!("{}.svg", file_path), &document).unwrap();
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
