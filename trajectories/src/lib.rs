//! Trajectory planner with acceleration/velocity limits designed for robotics/CNC motion
//! planning.
//!
//! This code is based on the paper available
//! [on Golems.org](http://www.golems.org/projects/traj.html) titled Time-Optimal Trajectory
//! Generation for Path Following with Bounded Acceleration and Velocity by Tobias Kunz and
//! Mike Stilman.

#![deny(
    bad_style, const_err, /*dead_code,*/ improper_ctypes, legacy_directory_ownership,
    non_shorthand_field_patterns, no_mangle_generic_items, overflowing_literals, path_statements,
    patterns_in_fns_without_body, plugin_as_library, private_in_public, private_no_mangle_fns,
    private_no_mangle_statics, safe_extern_statics, unconditional_recursion,
    unions_with_drop_fields, /*unused,*/ unused_allocation, unused_comparisons, unused_parens,
    while_true, missing_debug_implementations, missing_docs, trivial_casts, trivial_numeric_casts,
    unused_extern_crates, unused_import_braces, unused_qualifications, unused_results
)]

extern crate nalgebra;
#[macro_use]
extern crate approx;
extern crate csv;
extern crate image;
extern crate imageproc;
#[macro_use]
extern crate serde_derive;

#[macro_use]
mod macros;

mod circular_path_segment;

pub mod test_helpers;

use circular_path_segment::*;
use nalgebra::Vector3;

/// Type alias for all vector operations
///
/// This defines how many dimensions are supported. Change this type to add/remove dimensions
pub type Coord = Vector3<f64>;

/// Custom defined epsilon for "near enough" float comparisons and accuracy checks
pub const MIN_ACCURACY: f64 = 0.000001;
