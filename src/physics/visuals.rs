use bevy::prelude::*;

use crate::drone::Drone;

use super::components::LinearVelocity;
use super::constants::{
    COSMETIC_LERP_RATE, COSMETIC_MIN_SPEED, COSMETIC_PITCH_FACTOR, COSMETIC_PITCH_MAX,
    COSMETIC_ROLL_FACTOR, COSMETIC_ROLL_MAX,
};

/// Component on each drone holding the previous-frame linvel so the
/// cosmetic system can compute lateral acceleration (for banking).
/// Stays here in the physics module since it's part of the visual
/// state, not the physical state.
#[derive(Component, Default)]
pub struct PrevLinvel(pub Vec3);

/// Drone body rotation derived from `linvel`. Pure cosmetic — no
/// physics system reads this. Yaw points along horizontal velocity,
/// pitch leans nose-down with forward speed, roll banks into lateral
/// acceleration. Below `COSMETIC_MIN_SPEED` the rotation holds steady
/// so a hovering drone doesn't whip around on tiny noise.
pub fn update_drone_visuals(
    time: Res<Time>,
    mut q: Query<(&mut Transform, &LinearVelocity, &mut PrevLinvel), With<Drone>>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let alpha = (COSMETIC_LERP_RATE * dt).min(1.0);

    for (mut transform, linvel, mut prev) in &mut q {
        let horiz = Vec3::new(linvel.0.x, 0.0, linvel.0.z);
        let speed = horiz.length();

        let target_rot = if speed > COSMETIC_MIN_SPEED {
            let yaw_dir = horiz / speed;
            // Same convention as the cluster steering: in Bevy's Y-up
            // RH coords, `Quat::from_rotation_y(θ)` rotates NEG_Z to
            // (-sin θ, 0, -cos θ). To face (dx, 0, dz) we need
            // θ = atan2(-dx, -dz).
            let yaw = (-yaw_dir.x).atan2(-yaw_dir.z);

            // Nose-down pitch scales with forward speed. Cap at
            // COSMETIC_PITCH_MAX so a sudden velocity spike doesn't
            // flip the model.
            let pitch = -(speed * COSMETIC_PITCH_FACTOR).clamp(0.0, COSMETIC_PITCH_MAX);

            // Lateral accel = component of dv/dt perpendicular to
            // current horizontal heading. Right-perpendicular in
            // Bevy's Y-up RH coords = yaw_dir × Y.
            let accel = (linvel.0 - prev.0) / dt;
            let accel_horiz = Vec3::new(accel.x, 0.0, accel.z);
            let right = Vec3::Y.cross(yaw_dir).normalize_or_zero();
            let lateral = accel_horiz.dot(right);
            // Roll into the turn: banking the same direction the
            // drone is accelerating. Positive lateral (right turn) →
            // roll right (negative Z rotation in body frame).
            let roll =
                (-lateral * COSMETIC_ROLL_FACTOR).clamp(-COSMETIC_ROLL_MAX, COSMETIC_ROLL_MAX);

            Quat::from_rotation_y(yaw)
                * Quat::from_rotation_x(pitch)
                * Quat::from_rotation_z(roll)
        } else {
            // Below the minimum speed, keep current yaw but level out
            // pitch + roll. Decompose current rotation to extract yaw
            // only.
            let (yaw, _, _) = transform.rotation.to_euler(EulerRot::YXZ);
            Quat::from_rotation_y(yaw)
        };

        transform.rotation = transform.rotation.slerp(target_rot, alpha);
        prev.0 = linvel.0;
    }
}
