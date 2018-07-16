extern crate nalgebra;
#[macro_use]
extern crate approx;

#[cfg(test)]
extern crate image;
#[cfg(test)]
extern crate imageproc;

mod circular_blend;
mod path_segment;

use circular_blend::*;
use nalgebra::Vector3;
use path_segment::*;

pub type Coord = Vector3<f64>;

pub const MIN_ACCURACY: f64 = 0.000001;
