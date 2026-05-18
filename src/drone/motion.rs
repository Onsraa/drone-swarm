use bevy::prelude::*;

use crate::world::WorldConfig;

use super::components::{Drone, Velocity, WalkTimer};
use super::constants::{BOUND_MARGIN_METERS, DRONE_SPEED_METERS_PER_SEC, ROTATION_LERP_RATE};
use super::spawn::random_unit_dir;

pub fn random_walk(time: Res<Time>, mut q: Query<(&mut Velocity, &mut WalkTimer), With<Drone>>) {
    let mut rng = rand::rng();
    for (mut velocity, mut walk_timer) in &mut q {
        walk_timer.0.tick(time.delta());
        if walk_timer.0.just_finished() {
            velocity.0 = random_unit_dir(&mut rng) * DRONE_SPEED_METERS_PER_SEC;
        }
    }
}

pub fn integrate_motion(
    time: Res<Time>,
    config: Res<WorldConfig>,
    mut q: Query<(&mut Transform, &mut Velocity), With<Drone>>,
) {
    let world_size = config.world_size();
    let lo = Vec3::splat(BOUND_MARGIN_METERS);
    let hi = world_size - Vec3::splat(BOUND_MARGIN_METERS);
    let dt = time.delta_secs();

    for (mut transform, mut velocity) in &mut q {
        let next = transform.translation + velocity.0 * dt;
        let (px, vx) = reflect_axis(next.x, lo.x, hi.x, velocity.0.x);
        let (py, vy) = reflect_axis(next.y, lo.y, hi.y, velocity.0.y);
        let (pz, vz) = reflect_axis(next.z, lo.z, hi.z, velocity.0.z);
        transform.translation = Vec3::new(px, py, pz);
        velocity.0 = Vec3::new(vx, vy, vz);

        let dir = velocity.0.normalize_or_zero();
        if dir.length_squared() > 0.0 {
            let mut target = *transform;
            target.look_to(dir, Vec3::Y);
            let alpha = (ROTATION_LERP_RATE * dt).min(1.0);
            transform.rotation = transform.rotation.slerp(target.rotation, alpha);
        }
    }
}

fn reflect_axis(p: f32, lo: f32, hi: f32, v: f32) -> (f32, f32) {
    if p < lo {
        (lo, v.abs())
    } else if p > hi {
        (hi, -v.abs())
    } else {
        (p, v)
    }
}
