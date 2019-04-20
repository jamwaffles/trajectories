use nalgebra::VectorN;

mod path;

/// Type alias for all vector operations
pub type Coord<N> = VectorN<f64, N>;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
