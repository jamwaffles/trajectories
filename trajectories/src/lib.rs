extern crate nalgebra;
#[macro_use]
extern crate approx;

mod path;
mod pathsegment;
mod trajectory;
mod trajectorystep;

use nalgebra::Vector3;

pub type Coord = Vector3<f64>;

pub use path::Path;

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {
//         let waypoints = vec![
//             Coord::new(0.0, 0.0, 0.0),
//             Coord::new(0.0, 0.2, 1.0),
//             Coord::new(0.0, 3.0, 0.5),
//             Coord::new(1.1, 2.0, 0.0),
//             Coord::new(1.0, 0.0, 0.0),
//             Coord::new(0.0, 1.0, 0.0),
//             Coord::new(0.0, 0.0, 1.0),
//         ];
//     }
// }
