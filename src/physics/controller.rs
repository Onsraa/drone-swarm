use bevy::prelude::*;

use super::components::{DesiredAttitude, DesiredVelocity, LinearVelocity, ThrustState};
use super::constants::{
    DRONE_MASS_KG, GRAVITY, MAX_THRUST_MULTIPLE_OF_HOVER, MAX_TILT_RADIANS, VELOCITY_P_GAIN,
    YAW_TRACK_MIN_SPEED_SQ,
};

/// Take the velocity error, decide how hard and in what direction to thrust,
/// and write the resulting thrust magnitude + target body attitude.
pub fn quadcopter_controller(
    mut q: Query<(
        &Transform,
        &LinearVelocity,
        &DesiredVelocity,
        &mut ThrustState,
        &mut DesiredAttitude,
    )>,
) {
    let hover_thrust = DRONE_MASS_KG * GRAVITY;
    let max_thrust = hover_thrust * MAX_THRUST_MULTIPLE_OF_HOVER;
    let gravity_force = Vec3::new(0.0, -hover_thrust, 0.0);

    for (transform, linvel, desired_vel, mut thrust, mut attitude) in &mut q {
        let velocity_error = desired_vel.0 - linvel.0;
        let desired_accel = velocity_error * VELOCITY_P_GAIN;
        let required_force = desired_accel * DRONE_MASS_KG - gravity_force;

        let thrust_dir = clamp_tilt(required_force.normalize_or(Vec3::Y));
        thrust.magnitude = required_force.dot(thrust_dir).clamp(0.0, max_thrust);

        attitude.target_rotation = build_attitude(transform.rotation, thrust_dir, linvel.0);
    }
}

fn clamp_tilt(thrust_dir: Vec3) -> Vec3 {
    let cos_angle = thrust_dir.dot(Vec3::Y);
    let max_cos = MAX_TILT_RADIANS.cos();
    if cos_angle >= max_cos {
        return thrust_dir;
    }
    let horizontal = (thrust_dir - thrust_dir.dot(Vec3::Y) * Vec3::Y).normalize_or_zero();
    if horizontal.length_squared() < 1e-6 {
        return Vec3::Y;
    }
    (horizontal * MAX_TILT_RADIANS.sin() + Vec3::Y * max_cos).normalize()
}

/// Build a rotation whose body +Y aligns with `thrust_dir` and whose body -Z
/// (forward) aligns with the horizontal velocity direction when fast enough.
fn build_attitude(current: Quat, thrust_dir: Vec3, linear_velocity: Vec3) -> Quat {
    let horizontal_vel = Vec3::new(linear_velocity.x, 0.0, linear_velocity.z);
    let world_forward = if horizontal_vel.length_squared() > YAW_TRACK_MIN_SPEED_SQ {
        horizontal_vel.normalize()
    } else {
        let current_forward = current * Vec3::NEG_Z;
        let projected = Vec3::new(current_forward.x, 0.0, current_forward.z);
        projected.normalize_or(Vec3::NEG_Z)
    };

    let y_axis = thrust_dir;
    let z_axis = -world_forward;
    let x_axis_raw = y_axis.cross(z_axis);
    if x_axis_raw.length_squared() < 1e-6 {
        return current;
    }
    let x_axis = x_axis_raw.normalize();
    let z_axis_ortho = x_axis.cross(y_axis);
    Quat::from_mat3(&Mat3::from_cols(x_axis, y_axis, z_axis_ortho))
}
