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
/// 2. **Pitch:** signed forward-velocity error drives pitch. Positive
///    error (too slow) → pitch nose-down so body +Y thrust accelerates
///    the drone forward. Negative error (too fast / past the goal) →
///    pitch nose-up so the same thrust decelerates it. Clamped to
///    `±MAX_PITCH_RADIANS` either way. This gives active braking
///    instead of relying purely on drag.
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
        // Bevy is Y-up right-handed. `Quat::from_rotation_y(θ)` applied
        // to `NEG_Z` (the default forward) gives `(-sin θ, 0, -cos θ)`.
        // To make the drone face `(fx, 0, fz)`, solve sin θ = -fx,
        // cos θ = -fz → θ = atan2(-fx, -fz). Missing the X negation
        // (the original code) flipped pursuit on the X axis: drones
        // headed east instead of west and vice versa.
        let yaw_angle = (-target_forward_horizontal.x).atan2(-target_forward_horizontal.z);
        let yaw_quat = Quat::from_rotation_y(yaw_angle);

        let forward_speed_actual = linvel.0.dot(current_forward_horizontal);
        let forward_speed_target = desired_speed;
        // Signed error → signed pitch. Negative pitch tilts the nose
        // up so body +Y has a backward horizontal component, which
        // brakes the drone when it's faster than the requested speed
        // (typically as it nears the arrival ramp).
        let forward_accel_target =
            (forward_speed_target - forward_speed_actual) * FORWARD_P_GAIN;

        let pitch_target = (forward_accel_target / GRAVITY)
            .atan()
            .clamp(-MAX_PITCH_RADIANS, MAX_PITCH_RADIANS);
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
