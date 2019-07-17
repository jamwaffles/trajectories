//! Functions to find the minimum or maximum allowed velocity or acceleration at a point along the
//! path

use super::{LimitType, MinMax, TrajectoryOptions, TrajectoryStep};
use crate::{Coord, Path, PathItem};
use nalgebra::{
    allocator::{Allocator, SameShapeVectorAllocator},
    storage::Owned,
    DefaultAllocator, DimName,
};

/// Find the maximum allowable velocity at a point, limited by either max acceleration or max
/// velocity.
pub fn max_velocity_at<N>(path: &Path<N>, position_along_path: f64, limit_type: LimitType<N>) -> f64
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
    Owned<f64, N>: Copy,
{
    match limit_type {
        LimitType::Velocity(velocity_limit) => {
            let tangent = path.tangent(position_along_path);
            let result = velocity_limit.component_div(&tangent).amin();

            instrument!("max_vel_from_vel", (position_along_path, result));

            result
        }
        LimitType::Acceleration(acceleration_limit) => {
            let (vel, acceleration) = path.tangent_and_curvature(position_along_path);
            let vel_abs = vel.abs();
            let n = nalgebra::dimension::<Coord<N>>();

            // TODO: Less dumb names
            let accel_div_vel = acceleration.component_div(&vel);
            let accel_limit_div_vel_abs = acceleration_limit.component_div(&vel_abs);
            let accel = acceleration_limit.component_div(&acceleration.abs());

            let new_res = (0..n)
                .filter_map(|i| {
                    if vel[i] != 0.0 {
                        let min = ((i + 1)..n).filter(|j| vel[*j] != 0.0).fold(
                            std::f64::INFINITY,
                            |acc, j| {
                                // TODO: Come up with a less mathsy name
                                let a_ij = (accel_div_vel[i] - accel_div_vel[j]).abs();

                                if a_ij > 0.0 {
                                    acc.min(
                                        (accel_limit_div_vel_abs[i] + accel_limit_div_vel_abs[j])
                                            / a_ij,
                                    )
                                } else {
                                    acc
                                }
                            },
                        );

                        Some(min)
                    } else if acceleration[i] != 0.0 {
                        Some(accel[i])
                    } else {
                        None
                    }
                })
                .fold(std::f64::INFINITY, |acc, x| acc.min(x))
                .sqrt();

            instrument!("max_vel_from_acc", (position_along_path, new_res));

            new_res
        }
    }
}

/// Find the maximum velocity derivative at a point
pub fn max_velocity_derivative_at<N>(
    path: &Path<N>,
    position_along_path: f64,
    limit: LimitType<N>,
    options: &TrajectoryOptions<N>,
) -> f64
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
    Owned<f64, N>: Copy,
{
    match limit {
        LimitType::Velocity(velocity_limit) => {
            let (tangent, curvature) = path.tangent_and_curvature(position_along_path);
            let tangent_abs = tangent.abs();
            let velocity = velocity_limit.component_div(&tangent_abs);

            let calc = -(velocity_limit.component_mul(&curvature))
                .component_div(&tangent.component_mul(&tangent_abs));

            // Find the component index with the smallest value
            let constraint_axis = velocity.imin();

            let result = calc[constraint_axis];

            instrument!("max_vel_vel_deriv", (position_along_path, result));

            result
        }
        LimitType::Acceleration(acceleration_limit) => {
            (max_velocity_at(
                path,
                position_along_path + options.epsilon,
                LimitType::Acceleration(acceleration_limit),
            ) - max_velocity_at(
                path,
                position_along_path - options.epsilon,
                LimitType::Acceleration(acceleration_limit),
            )) / (2.0 * options.epsilon)
        }
    }
}

/// Find minimum or maximum acceleration at a point along path
pub fn max_acceleration_at<N>(
    path: &Path<N>,
    pos_vel: &TrajectoryStep,
    min_max: MinMax,
    options: &TrajectoryOptions<N>,
) -> f64
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
    Owned<f64, N>: Copy,
{
    let &TrajectoryStep {
        position, velocity, ..
    } = pos_vel;

    let (derivative, second_derivative) = path.tangent_and_curvature(position);
    let factor = min_max.as_multiplier();

    let res = options
        .acceleration_limit
        .iter()
        .zip(derivative.iter().zip(second_derivative.iter()))
        .fold(
            std::f64::MAX,
            |acc,
             (
                acceleration_limit_component,
                (derivative_component, second_derivative_component),
            )| {
                if *derivative_component != 0.0 {
                    acc.min(
                        acceleration_limit_component / derivative_component.abs()
                            - factor * second_derivative_component * velocity.powi(2)
                                / derivative_component,
                    )
                } else {
                    acc
                }
            },
        );

    instrument!("acc_at", (position, velocity, res * factor));

    res * factor
}

/// Get the minimum or maximum phase slope for a position along the path
pub fn max_acceleration_derivative_at<N>(
    path: &Path<N>,
    pos_vel: &TrajectoryStep,
    min_max: MinMax,
    options: &TrajectoryOptions<N>,
) -> f64
where
    N: DimName + Copy,
    DefaultAllocator: SameShapeVectorAllocator<f64, N, N>,
    <DefaultAllocator as Allocator<f64, N>>::Buffer: Send + Sync,
    Owned<f64, N>: Copy,
{
    max_acceleration_at(path, &pos_vel, min_max, options) / pos_vel.velocity
}
