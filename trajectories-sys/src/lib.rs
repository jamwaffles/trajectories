#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

// type Eigen_Vector3f = [f32; 3usize];

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

extern "C" {
    fn create_path(items: *const [f64; 3], len: usize, step: f64) -> *mut Path;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let waypoints: Vec<[f64; 3]> = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 3.0],
        ];

        println!("Len: {}", waypoints.len());

        let path = unsafe { create_path(waypoints.as_ptr(), waypoints.len() * 3, 0.1) };

        println!("Expected: {:?}", waypoints);
        println!("Path: {:?}", path);

        // let mut wp = std_list::new();

        // let max_velocity = [1.0, 1.0, 1.0];
        // let max_acceleration = [1.0, 1.0, 1.0];

        // let traj = Trajectory::new(
        //     &Path::new(waypoints.as_ptr(), 0.1),
        //     &max_velocity,
        //     &max_acceleration,
        //     0.1,
        // );

        assert_eq!(2 + 2, 4);
    }
}
