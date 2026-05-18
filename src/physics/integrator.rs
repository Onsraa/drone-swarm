use bevy::prelude::*;

use crate::world::WorldConfig;

use super::components::{DesiredAttitude, LinearVelocity, ThrustState};
use super::constants::{
    ATTITUDE_LERP_RATE, BOUND_REPULSION_K, BOUND_SOFT_MARGIN_METERS, DRONE_MASS_KG, GRAVITY,
    LINEAR_DRAG_COEF,
};

/// Apply thrust + gravity + drag + bound repulsion, integrate to position,
/// then slerp orientation toward the controller's target attitude.
pub fn integrate_forces(
    time: Res<Time>,
    config: Res<WorldConfig>,
    mut q: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &ThrustState,
        &DesiredAttitude,
    )>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let gravity_force = Vec3::new(0.0, -DRONE_MASS_KG * GRAVITY, 0.0);
    let world_size = config.world_size();

    for (mut transform, mut linvel, thrust, attitude) in &mut q {
        let thrust_world = thrust.magnitude * (transform.rotation * Vec3::Y);
        let drag_force = -LINEAR_DRAG_COEF * linvel.0;
        let bound_force = bound_repulsion_force(transform.translation, world_size);
        let total_force = thrust_world + gravity_force + drag_force + bound_force;
        let accel = total_force / DRONE_MASS_KG;

        linvel.0 += accel * dt;
        transform.translation += linvel.0 * dt;
        clamp_to_world(&mut transform.translation, &mut linvel.0, world_size);

        let alpha = (ATTITUDE_LERP_RATE * dt).min(1.0);
        transform.rotation = transform.rotation.slerp(attitude.target_rotation, alpha);
    }
}

fn bound_repulsion_force(pos: Vec3, world_size: Vec3) -> Vec3 {
    Vec3::new(
        axis_repulsion(pos.x, world_size.x),
        axis_repulsion(pos.y, world_size.y),
        axis_repulsion(pos.z, world_size.z),
    )
}

fn axis_repulsion(position: f32, world_extent: f32) -> f32 {
    let low_intrusion = (BOUND_SOFT_MARGIN_METERS - position).max(0.0);
    let high_intrusion = (BOUND_SOFT_MARGIN_METERS - (world_extent - position)).max(0.0);
    (low_intrusion - high_intrusion) * BOUND_REPULSION_K
}

fn clamp_to_world(position: &mut Vec3, velocity: &mut Vec3, world_size: Vec3) {
    clamp_axis(&mut position.x, &mut velocity.x, 0.0, world_size.x);
    clamp_axis(&mut position.y, &mut velocity.y, 0.0, world_size.y);
    clamp_axis(&mut position.z, &mut velocity.z, 0.0, world_size.z);
}

fn clamp_axis(position: &mut f32, velocity: &mut f32, lo: f32, hi: f32) {
    if *position < lo {
        *position = lo;
        if *velocity < 0.0 {
            *velocity = 0.0;
        }
    } else if *position > hi {
        *position = hi;
        if *velocity > 0.0 {
            *velocity = 0.0;
        }
    }
}
