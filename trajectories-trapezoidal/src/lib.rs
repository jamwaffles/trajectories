use csv;
use std::fs::{self, File};
use std::path::PathBuf;

/// One-dimensional trapezoidal profile between two points (numbers) given max acceleration and
/// velocity
// fn super_simple_trapezoidal(start: f32, end: f32, max_accel: f32, max_vel: f32, time: f32) -> f32 {
//     println!("Max accel {}, max vel {}", max_accel, max_vel);
//     println!("Start {}, end {}", start, end);

//     // ta
//     let accel_time = max_vel / max_accel;

//     // L
//     let length = (end - start).abs();

//     // T
//     let duration = (length * max_accel + max_vel.powi(2)) / (max_accel * max_vel);

//     if start_time <= time && time <= start_time + accel_time {
//         println!("Accel phase");
//     } else if start_time + accel_time < time && time <= end_time - accel_time {
//         println!("Linear phase");
//     } else if end_time - accel_time < time && time <= end_time {
//         println!("Decel phase");
//     } else {
//         unreachable!(
//             "Time {} is out of bounds {} - {}",
//             time, start_time, end_time
//         );
//     }

//     0.0
// }

struct SuperSimpleTrapezoidal {
    start: f32,
    end: f32,
    max_accel: f32,
    max_vel: f32,
    accel_time: f32,
    pub length: f32,
    pub duration: f32,
    start_time: f32,
    end_time: f32,
}

impl SuperSimpleTrapezoidal {
    pub fn new(start: f32, end: f32, max_accel: f32, max_vel: f32) -> Self {
        // L
        let length = (end - start).abs();

        let has_linear_tract = length >= max_vel.powi(2) / max_accel;

        // (ta, T)
        let (accel_time, duration) = if has_linear_tract {
            (
                max_vel / max_accel,
                (length * max_accel + max_vel.powi(2)) / (max_accel * max_vel),
            )
        } else {
            let accel_time = (length / max_accel).sqrt();

            (accel_time, accel_time * 2.0)
        };

        println!("{}", has_linear_tract);

        println!("Accel time {}", accel_time);
        println!("Duration {}", duration);

        // ti
        let start_time = 0.0;

        // tf
        let end_time = start_time + duration;

        Self {
            start,
            end,
            max_accel,
            max_vel,
            accel_time,
            length,
            duration,
            start_time,
            end_time,
        }
    }

    pub fn position(&self, time: f32) -> f32 {
        if self.start_time <= time && time <= self.start_time + self.accel_time {
            println!("Accel phase");

            self.start + 0.5 * self.max_accel * (time - self.start_time).powi(2)
        } else if self.start_time + self.accel_time < time
            && time <= self.end_time - self.accel_time
        {
            println!("Linear phase");

            self.start
                + self.max_accel
                    * self.accel_time
                    * (time - self.start_time - self.accel_time / 2.0)
        } else if self.end_time - self.accel_time < time && time <= self.end_time {
            println!("Decel phase");

            self.end - 0.5 * self.max_accel * (self.end_time - time - self.start_time).powi(2)
        } else {
            unreachable!(
                "Time {} is out of bounds {} - {}",
                time, self.start_time, self.end_time
            );
        }
    }
}

fn csv_debug(name: &'static str, points: &[(f32, f32)]) {
    // NOTE: Relative to lib root
    let path = PathBuf::from(format!("../target/trapezoidal/{}.csv", name));

    fs::create_dir_all(&path.parent().unwrap()).expect("Failed to create output dir");

    println!("Plotting to {}", path.display());

    let mut wtr = csv::Writer::from_writer(File::create(path).unwrap());

    for point in points {
        wtr.serialize(point).expect("Could not serialize");
    }

    wtr.flush().expect("Flush");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple() {
        let traj = SuperSimpleTrapezoidal::new(10.0, 20.0, 1.0, 2.0);

        let mut t = 0.0;
        let mut points = Vec::new();

        while t <= traj.duration {
            let pos = traj.position(t);

            points.push((t, pos));

            t += 0.1;
        }

        csv_debug("super_simple", &points);
    }

    #[test]
    fn short() {
        let traj = SuperSimpleTrapezoidal::new(10.0, 11.0, 1.0, 2.0);

        let mut t = 0.0;
        let mut points = Vec::new();

        while t <= traj.duration {
            let pos = traj.position(t);

            points.push((t, pos));

            t += 0.1;
        }

        csv_debug("short", &points);
    }
}
