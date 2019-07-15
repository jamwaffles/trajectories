use crate::LinearTrajectory;
use nalgebra::{
    allocator::SameShapeVectorAllocator, storage::Owned, DefaultAllocator, DimName, Vector3,
    Vector4,
};
use std::fs::{self, File};
use std::path::PathBuf;

/// 3 dimensional dobule precision vector for use in test code
pub type TestCoord3 = Vector3<f64>;

/// 4 dimensional dobule precision vector for use in test code
pub type TestCoord4 = Vector4<f64>;

#[derive(Debug, serde_derive::Serialize)]
struct TrajDebugRecord {
    time: f64,
    pos_x: f64,
    pos_y: f64,
    pos_z: f64,
    vel_x: f64,
    vel_y: f64,
    vel_z: f64,
}

/// Log trajectory output to a CSV file for GNUPlot or other things to plot later
pub fn debug_linear_trajectory<N>(traj: &LinearTrajectory<N>, file_name: &str, time_step: f64)
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    // NOTE: Relative to lib root
    let path = PathBuf::from(format!("../target/jerk-plots/{}.csv", file_name));

    fs::create_dir_all(&path.parent().unwrap()).expect("Failed to create output dir");

    println!("Plotting to {}", path.display());

    let mut time = 0.0;

    let mut wtr = csv::Writer::from_writer(File::create(path).unwrap());

    while time <= traj.duration() {
        let pos = traj.position_linear(time).expect("Failed to get position");
        let vel = traj.velocity_linear(time).expect("Failed to get velocity");

        let row = TrajDebugRecord {
            time,
            pos_x: pos[0],
            pos_y: pos[1],
            pos_z: pos[2],
            vel_x: vel[0],
            vel_y: vel[1],
            vel_z: vel[2],
        };

        wtr.serialize(row).expect("Could not serialize");

        time += time_step;
    }

    wtr.flush().expect("Flush");
}

pub fn debug_s_curve_trajectory<N>(traj: &LinearTrajectory<N>, file_name: &str, time_step: f64)
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    Owned<f64, N>: Copy,
{
    // NOTE: Relative to lib root
    let path = PathBuf::from(format!("../target/jerk-plots/{}.csv", file_name));

    fs::create_dir_all(&path.parent().unwrap()).expect("Failed to create output dir");

    println!("Plotting to {}", path.display());

    let mut time = 0.0;

    let mut wtr = csv::Writer::from_writer(File::create(path).unwrap());

    while time <= traj.duration() {
        // let pos = traj.position_linear(time).unwrap();
        let vel = traj.velocity_s_curve(time).unwrap();

        let row = TrajDebugRecord {
            time,
            pos_x: 0.0,
            pos_y: 0.0,
            pos_z: 0.0,
            vel_x: vel[0],
            vel_y: vel[1],
            vel_z: vel[2],
        };

        wtr.serialize(row).expect("Could not serialize");

        time += time_step;
    }

    wtr.flush().expect("Flush");
}
