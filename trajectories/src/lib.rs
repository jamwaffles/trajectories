//! Trajectory planner with acceleration/velocity limits designed for robotics/CNC motion
//! planning.
//!
//! This code is based on the paper available
//! [on Golems.org](http://www.golems.org/projects/traj.html) titled Time-Optimal Trajectory
//! Generation for Path Following with Bounded Acceleration and Velocity by Tobias Kunz and
//! Mike Stilman.

#![deny(
    bad_style,
    const_err,
    dead_code,
    improper_ctypes,
    legacy_directory_ownership,
    non_shorthand_field_patterns,
    no_mangle_generic_items,
    overflowing_literals,
    path_statements,
    patterns_in_fns_without_body,
    plugin_as_library,
    private_in_public,
    safe_extern_statics,
    unconditional_recursion,
    unions_with_drop_fields,
    // unused,
    unused_allocation,
    unused_comparisons,
    unused_parens,
    while_true,
    missing_debug_implementations,
    missing_docs,
    trivial_casts,
    trivial_numeric_casts,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]

#[cfg(test)]
#[macro_use]
extern crate approx;
#[macro_use]
extern crate serde_derive;
#[macro_use]
extern crate log;

#[macro_use]
mod macros;
mod path;
pub mod prelude;
#[doc(hidden)]
pub mod test_helpers;
mod trajectory;

pub use crate::path::{Path, PathOptions};
pub use crate::trajectory::{Trajectory, TrajectoryOptions};
use nalgebra::VectorN;

/// Type alias for all vector operations
pub type Coord<N> = VectorN<f64, N>;

/// Custom defined epsilon for "near enough" float comparisons and accuracy checks. This is set to
/// `(std::f64::EPSILON).sqrt()` on a Linux x64 machine with Rust 1.29 as per the
/// [answers on this StackExchange question](https://scicomp.stackexchange.com/questions/14355/choosing-epsilons)
pub const TRAJECTORY_EPSILON: f64 = 0.000_000_014_901_161_193_847_656;

/// Maximum deviation from true path
pub const MAX_DEVIATION: f64 = 0.001;
