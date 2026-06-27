pub mod filter;

pub use filter::ComplementaryFilter;

use nalgebra::{UnitQuaternion, Vector3};

/// Integrate angular rates into an orientation quaternion using first-order
/// Rodrigues rotation (axis-angle) — cheaper than matrix exponentiation and
/// sufficient at the 200 Hz IMU rate.
///
/// `gyro_rps`  — angular rates in rad/s (body frame)
/// `dt_s`      — integration interval in seconds
/// `current_q` — orientation to update in-place
#[inline]
pub fn integrate_gyro(
    gyro_rps: Vector3<f32>,
    dt_s: f32,
    current_q: &mut UnitQuaternion<f32>,
) {
    let angle = gyro_rps.norm() * dt_s;
    if angle < 1e-9 {
        return; // no rotation this tick
    }
    let axis = nalgebra::Unit::new_normalize(gyro_rps);
    let delta_q = UnitQuaternion::from_axis_angle(&axis, angle);
    *current_q = (*current_q * delta_q).normalize();
}
