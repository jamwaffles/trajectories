/// Assert that two floating point values are near each other within crate::TRAJECTORY_EPSILON
#[cfg(test)]
#[macro_export]
macro_rules! assert_near {
    ($a:expr, $b:expr) => {
        assert_ulps_eq!($a, $b, epsilon = $crate::TRAJECTORY_EPSILON);
    };
}
