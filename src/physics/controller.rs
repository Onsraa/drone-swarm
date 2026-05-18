use bevy::prelude::*;

use super::components::{DesiredAttitude, DesiredVelocity, LinearVelocity, ThrustState};
use super::constants::{
    DRONE_MASS_KG, FORWARD_P_GAIN, GRAVITY, HEADING_TRACK_MIN_SPEED, MAX_PITCH_RADIANS,
    MAX_THRUST_MULTIPLE_OF_HOVER, VERTICAL_P_GAIN,
};

/// Cascaded quadcopter controller modeled on a real-drone "coordinated
/// turn" loop (the simplification of a position -> velocity -> attitude
/// PID stack used by firmwares like Crazyflie / Betaflight self-level):
///
/// 1. **Yaw:** the body always points in the direction of the desired
///    horizontal velocity; the drone yaws to face its target before doing
///    anything else. This guarantees motion is along the drone's head
///    direction (no backward / sideways drift).
/// 2. **Pitch:** while facing forward, the drone pitches nose-down by an
///    amount such that thrust along body +Y produces the required forward
///    acceleration. Pitch is clamped >= 0 so the drone never tilts back-
///    ward; slowing is handled by drag.
/// 3. **Thrust magnitude:** chosen so the vertical component of thrust
///    counters gravity plus a vertical-velocity error term, giving an
///    altitude-hold + climb/descend behavior.
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

    for (transform, linvel, desired_vel, mut thrust, mut attitude) in &mut q {
        let desired_horizontal = Vec3::new(desired_vel.0.x, 0.0, desired_vel.0.z);
        let desired_speed = desired_horizontal.length();

        let current_forward_world = transform.rotation * Vec3::NEG_Z;
        let current_forward_horizontal = Vec3::new(current_forward_world.x, 0.0, current_forward_world.z)
            .normalize_or(Vec3::NEG_Z);

        let target_forward_horizontal = if desired_speed > HEADING_TRACK_MIN_SPEED {
            desired_horizontal / desired_speed
        } else {
            current_forward_horizontal
        };
        let yaw_angle = target_forward_horizontal.x.atan2(-target_forward_horizontal.z);
        let yaw_quat = Quat::from_rotation_y(yaw_angle);

        let forward_speed_actual = linvel.0.dot(current_forward_horizontal);
        let forward_speed_target = desired_speed;
        let forward_accel_target =
            ((forward_speed_target - forward_speed_actual) * FORWARD_P_GAIN).max(0.0);

        let pitch_target = (forward_accel_target / GRAVITY).atan().min(MAX_PITCH_RADIANS);
        let pitch_quat = Quat::from_rotation_x(-pitch_target);

        attitude.target_rotation = yaw_quat * pitch_quat;

        let vertical_velocity_error = desired_vel.0.y - linvel.0.y;
        let vertical_accel_target = vertical_velocity_error * VERTICAL_P_GAIN;
        let required_vertical_force = hover_thrust + vertical_accel_target * DRONE_MASS_KG;
        let pitch_cos = pitch_target.cos().max(0.1);
        let thrust_total = required_vertical_force / pitch_cos;
        thrust.magnitude = thrust_total.clamp(0.0, max_thrust);
    }
}
