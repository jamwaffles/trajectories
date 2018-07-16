extern crate nalgebra;
#[macro_use]
extern crate approx;

mod circular_blend;
mod path_segment;

use circular_blend::*;
use nalgebra::Vector3;
use path_segment::*;

pub type Coord = Vector3<f64>;
