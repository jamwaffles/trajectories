extern crate nalgebra;
#[macro_use]
extern crate approx;

#[cfg(test)]
extern crate image;
#[cfg(test)]
extern crate imageproc;

#[macro_use]
mod macros;

mod circular_path_segment;

#[cfg(test)]
mod test_helpers;

use circular_path_segment::*;
use nalgebra::Vector3;

pub type Coord = Vector3<f64>;

pub const MIN_ACCURACY: f64 = 0.000001;
