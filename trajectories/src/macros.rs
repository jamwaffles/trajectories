/// Assert that two floats are near each other within an epsilon
#[cfg(test)]
#[macro_export]
macro_rules! assert_near {
    ($a:expr, $b:expr) => {
        assert_ulps_eq!($a, $b, epsilon = $crate::TRAJ_EPSILON);
    };
}
