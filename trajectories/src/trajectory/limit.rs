//! The kind of limit that exists at a certain point

/// Limited by velocity or acceleration
#[derive(Debug)]
pub enum Limit {
    Velocity,
    Acceleration,
}
