use std::collections::HashMap;
use std::fs;

/// Assert that two floating point values are near each other within crate::TRAJECTORY_EPSILON
#[cfg(test)]
#[macro_export]
macro_rules! assert_near {
    ($a:expr, $b:expr) => {
        assert_ulps_eq!($a, $b, epsilon = $crate::TRAJECTORY_EPSILON);
    };
}

/// Debug instrumentation written to CSV
#[macro_export]
macro_rules! instrument {
    ($a:expr, $b:expr) => {
        assert_ulps_eq!($a, $b, epsilon = $crate::TRAJECTORY_EPSILON);
    };
}

pub static mut INSTRUMENTATION: Option<HashMap<&'static str, fs::File>> = None;

macro_rules! instrument {
    ($name:expr, $time:expr, ( $($value:expr,)+ )) => { instrument!($name, $time, ( $($key $value),+ )) };
    ($name:expr, $time:expr, ( $($value:expr),* )) => {
        {
            use std::collections::HashMap;
            use csv;

            // Relative to lib folder
            let file_name = format!("../target/instrument_{}.csv", $name);

            match std::env::var("INSTRUMENT") {
                Ok(ref var) if var == $name => {
                    trace!("Enabling instrumentation");

                    let values = ($name, $time, $($value),+ );

                    let instrumentation = unsafe {
                        crate::macros::INSTRUMENTATION.get_or_insert(HashMap::new())
                    };

                    let log_file = instrumentation
                        .entry($name)
                        .or_insert_with(|| std::fs::File::create(file_name.clone()).expect(&format!("Could not open instrumentation file {}", file_name)));

                    let mut wtr = csv::Writer::from_writer(log_file);

                    wtr.serialize(&values).expect("Failed to serialize instrumentation record");

                    wtr.flush().expect("Failed to flush file");
                },
                _ => trace!("Not enabling tracing for {}", $name)
            }
        }
    };
}
