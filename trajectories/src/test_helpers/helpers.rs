//! Test helpers

pub use crate::path::CircularPathSegment;
use crate::path::PathItem;
use crate::path::{Continuity, Path as TrajPath, PathSegment};
use csv;
use nalgebra::allocator::Allocator;
use nalgebra::DefaultAllocator;
use nalgebra::DimName;
use nalgebra::VectorN;
use serde::Serialize;
use std::fmt;
use std::fs::File;
use svg;
use svg::node::element::path::Data;
use svg::node::element::{Circle, Group, Path as SvgPath, Rectangle, Text};
use svg::node::Text as TextContent;
use svg::Document;

const PADDING: f64 = 1.0;

fn single_line<D>(
    from: &VectorN<f64, D>,
    to: &VectorN<f64, D>,
    stroke: &str,
    stroke_width: u32,
) -> SvgPath
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    SvgPath::new()
        .set("fill", "none")
        .set("stroke", stroke)
        .set("stroke-width", stroke_width)
        .set("vector-effect", "non-scaling-stroke")
        .set(
            "d",
            Data::new()
                .move_to((from[0], from[1]))
                .line_to((to[0], to[1])),
        )
}

fn cross_centered_at<D>(center: &VectorN<f64, D>, stroke: &str, stroke_width: f64) -> SvgPath
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    let size = 0.1;

    SvgPath::new()
        .set("fill", "none")
        .set("stroke", stroke)
        .set("stroke-width", stroke_width)
        .set("vector-effect", "non-scaling-stroke")
        .set(
            "d",
            Data::new()
                .move_to((center[0], center[1] - size))
                .line_by((0, size * 2.0))
                .move_to((center[0] - size, center[1]))
                .line_by((size * 2.0, 0)),
        )
}

fn border<D>(top_left: &VectorN<f64, D>, bottom_right: &VectorN<f64, D>) -> Rectangle
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    Rectangle::new()
        .set("fill", "none")
        .set("stroke", "black")
        .set("stroke-width", 1)
        .set("vector-effect", "non-scaling-stroke")
        .set("x", top_left[1])
        .set("y", top_left[0])
        .set("width", bottom_right[1])
        .set("height", bottom_right[0])
}

fn create_document<D>(top_left: &VectorN<f64, D>, bottom_right: &VectorN<f64, D>) -> Document
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    let aspect: f64 = (bottom_right[0] - top_left[0]) / (bottom_right[0] - top_left[0]);
    let width = 1024;

    Document::new()
        .set("shape-rendering", "geometricPrecision")
        .set("width", width)
        .set("height", (width as f64 * aspect) as u32)
        .set(
            "viewBox",
            (top_left[0], top_left[0], bottom_right[0], bottom_right[0]),
        )
        .add(border(&top_left, &bottom_right))
}

fn save_document(suite_name: &str, doc: &Document) {
    svg::save(format!("../target/{}.svg", suite_name), doc).unwrap();
}

fn draw_blend_circle<D>(blend: &CircularPathSegment<f64, D>) -> Group
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    use std::ops::Mul;

    let line_scale = 0.25;

    // Blend circle
    let circle = Circle::new()
        .set("cx", blend.center[0])
        .set("cy", blend.center[0])
        .set("stroke-width", 1)
        .set("stroke", "blue")
        .set("fill", "none")
        .set("vector-effect", "non-scaling-stroke")
        .set("r", blend.radius);

    // Xi (green)
    let xi = single_line(
        &blend.center,
        &(nalgebra::convert::<f64, f64>(line_scale).mul(blend.center + blend.x)),
        "green",
        1,
    );

    // Yi (purple)
    let yi = single_line(
        &blend.center,
        &(nalgebra::convert::<f64, f64>(line_scale).mul(blend.center + blend.x)),
        "purple",
        1,
    );

    Group::new().add(circle).add(xi).add(yi)
}

fn calc_bbox<D>(coords: &[VectorN<f64, D>]) -> (VectorN<f64, D>, VectorN<f64, D>)
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    (
        coords
            .iter()
            .min_by_key(|item| ((item[0] + item[1] + item[2]) as u32 * 100))
            .unwrap()
            - VectorN::<f64, D>::repeat(nalgebra::convert(PADDING)),
        coords
            .iter()
            .max_by_key(|item| ((item[0] + item[1] + item[2]) as u32 * 100))
            .unwrap()
            + VectorN::<f64, D>::repeat(nalgebra::convert(PADDING * 2.0)),
    )
}

/// Produce an image of the blend between two path segments
pub fn debug_blend<D>(
    p: &str,
    before: &VectorN<f64, D>,
    current: &VectorN<f64, D>,
    after: &VectorN<f64, D>,
    blend: &CircularPathSegment<f64, D>,
) where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
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
pub fn debug_blend_position<D>(p: &str, blend: &CircularPathSegment<f64, D>)
where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
    let top_left = blend.center - VectorN::<f64, D>::repeat(blend.radius + PADDING);
    let bottom_right = blend.center + VectorN::<f64, D>::repeat(blend.radius + PADDING * 2.0);

    let circle = draw_blend_circle(&blend);

    let start = blend.get_position(nalgebra::convert(0.0));

    let mut i = 0.0;
    let mut data = Data::new().move_to((start[0], start[1]));

    while i <= blend.get_length() {
        i += 0.1;

        let pos = blend.get_position(i);

        data = data.line_to((pos[0], pos[1]));
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
pub fn debug_path<D>(
    file_path: &'static str,
    path: &TrajPath<f64, D>,
    waypoints: &Vec<VectorN<f64, D>>,
) where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
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
                            .set("x", line.start[0])
                            .set("y", line.start[1])
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
                        let start = circ.get_position(nalgebra::convert(0.0));

                        Text::new()
                            .set("x", start[0])
                            .set("y", start[1])
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
pub fn debug_path_switching_points<D>(
    file_path: &'static str,
    path: &TrajPath<f64, D>,
    waypoints: &Vec<VectorN<f64, D>>,
) where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
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
pub fn debug_path_point<D>(
    file_path: &'static str,
    path: &TrajPath<f64, D>,
    waypoints: &Vec<VectorN<f64, D>>,
    point: &VectorN<f64, D>,
) where
    D: DimName,
    DefaultAllocator: Allocator<f64, D>,
{
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
                            .set("x", line.start[0])
                            .set("y", line.start[1])
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
                        let start = circ.get_position(nalgebra::convert(0.0));

                        Text::new()
                            .set("x", start[0])
                            .set("y", start[1])
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
            position_x: nalgebra::convert(pos.x),
            position_y: nalgebra::convert(pos.y),
            position_z: nalgebra::convert(pos.z),
            velocity_x: nalgebra::convert(vel.x),
            velocity_y: nalgebra::convert(vel.y),
            velocity_z: nalgebra::convert(vel.z),
        }
    }

    /// Create a row from a time and position and velocity vectors
    pub fn from_coords<D>(time: f64, pos: &VectorN<f64, D>, vel: &VectorN<f64, D>) -> Self
    where
        D: DimName,
        DefaultAllocator: Allocator<f64, D>,
    {
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
