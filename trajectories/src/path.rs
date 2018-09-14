use circular_path_segment::CircularPathSegment;
use linear_path_segment::LinearPathSegment;
use Coord;
use PathItem;

#[derive(Debug, Clone)]
pub enum PathSegment {
    Linear(LinearPathSegment),
    Circular(CircularPathSegment),
}

impl PathItem for PathSegment {
    /// Get length of path
    fn get_length(&self) -> f64 {
        match self {
            PathSegment::Linear(s) => s.get_length(),
            PathSegment::Circular(s) => s.get_length(),
        }
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord {
        match self {
            PathSegment::Linear(s) => s.get_position(distance_along_line),
            PathSegment::Circular(s) => s.get_position(distance_along_line),
        }
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord {
        match self {
            PathSegment::Linear(s) => s.get_tangent(distance_along_line),
            PathSegment::Circular(s) => s.get_tangent(distance_along_line),
        }
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord {
        match self {
            PathSegment::Linear(s) => s.get_curvature(distance_along_line),
            PathSegment::Circular(s) => s.get_curvature(distance_along_line),
        }
    }
}

#[derive(Debug)]
pub struct Path {
    pub segments: Vec<PathSegment>,
    pub segments_with_offsets: Vec<(f64, PathSegment)>,
    length: f64,
}

impl Path {
    /// Create a blended path from a set of waypoints
    ///
    /// The path must be differentiable, so small blends are added between linear segments
    pub fn from_waypoints(waypoints: &Vec<Coord>, max_deviation: f64) -> Self {
        // Create a bunch of linear segments from a load of points
        let linear_segments = waypoints
            .windows(2)
            .map(|parts| {
                if let &[start, end] = parts {
                    LinearPathSegment::from_waypoints(start, end)
                } else {
                    panic!("Linear segments");
                }
            }).collect::<Vec<LinearPathSegment>>();

        let mut segments: Vec<PathSegment> = Vec::new();

        // Iterate over pairs of lines, adding a blend between each one
        linear_segments.windows(2).for_each(|parts| {
            if let &[prev, curr] = parts {
                let blend_segment = CircularPathSegment::from_waypoints(
                    &prev.start,
                    &curr.start,
                    &curr.end,
                    max_deviation,
                );

                let new_prev_end = blend_segment.get_position(0.0);
                let new_curr_start = blend_segment.get_position(blend_segment.get_length());

                let new_prev = LinearPathSegment::from_waypoints(prev.start, new_prev_end);
                let new_curr = LinearPathSegment::from_waypoints(new_curr_start, curr.end);

                // If there's a previous segment, update its end point to sit tangent to the blend
                // segment circle. If there is no previous segment, push a new one.
                segments
                    .last_mut()
                    .map(|old_prev| {
                        println!("Existing previous segment");

                        match old_prev {
                            PathSegment::Linear(old_prev) => {
                                old_prev.end = new_prev_end;
                            }
                            _ => panic!("Malformed path: previous element should be linear"),
                        }
                    }).unwrap_or_else(|| {
                        println!("New previous segment");

                        segments.push(PathSegment::Linear(new_prev));
                    });

                segments.push(PathSegment::Circular(blend_segment));
                segments.push(PathSegment::Linear(new_curr));
            } else {
                panic!("Linear segments each");
            }
        });

        // Add start offsets to the beginning of each segment
        let segments_with_offsets = segments
            .iter()
            .cloned()
            .scan(0.0, |offset, segment| {
                let ret = (*offset, segment.clone());

                *offset += segment.get_length();

                Some(ret)
            }).collect();

        let length = segments
            .iter()
            .fold(0.0, |acc, segment| acc + segment.get_length());

        Self {
            segments,
            segments_with_offsets,
            length,
        }
    }

    /// Get a path segment for a position along the entire path
    pub fn get_segment_at_position(&self, position_along_path: f64) -> Option<&(f64, PathSegment)> {
        self.segments_with_offsets
            .iter()
            .skip_while(|(offs, _)| offs < &position_along_path)
            .next()
    }
}

impl PathItem for Path {
    /// Get the length of the complete path
    fn get_length(&self) -> f64 {
        self.length
    }

    /// Get position at a point along path
    fn get_position(&self, distance_along_line: f64) -> Coord {
        self.get_segment_at_position(distance_along_line)
            .map(|(start_offset, segment)| segment.get_position(distance_along_line - start_offset))
            .expect(&format!(
                "Could not get position for path offset {}",
                distance_along_line
            ))
    }

    /// Get first derivative (tangent) at a point
    fn get_tangent(&self, distance_along_line: f64) -> Coord {
        self.get_segment_at_position(distance_along_line)
            .map(|(start_offset, segment)| segment.get_tangent(distance_along_line - start_offset))
            .expect(&format!(
                "Could not get position for path offset {}",
                distance_along_line
            ))
    }

    /// Get second derivative (curvature) at a point
    fn get_curvature(&self, distance_along_line: f64) -> Coord {
        self.get_segment_at_position(distance_along_line)
            .map(|(start_offset, segment)| {
                segment.get_curvature(distance_along_line - start_offset)
            }).expect(&format!(
                "Could not get position for path offset {}",
                distance_along_line
            ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_helpers::debug_path;

    #[test]
    fn it_creates_path_with_blends() {
        let waypoints = vec![
            Coord::new(0.0, 0.0, 0.0),
            Coord::new(1.0, 2.0, 0.0),
            Coord::new(1.5, 1.5, 0.0),
            Coord::new(3.0, 5.0, 0.0),
            Coord::new(4.0, 6.0, 0.0),
            Coord::new(5.0, 5.0, 0.0),
            Coord::new(4.0, 4.0, 0.0),
        ];

        let path = Path::from_waypoints(&waypoints, 0.1);

        // println!("{:#?}", path.segments);

        debug_path("../target/path_with_blends", &path);

        assert!(true);
    }
}
