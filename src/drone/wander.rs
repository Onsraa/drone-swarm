use bevy::prelude::*;
use rand::Rng;

use crate::exploration::{FrontierTarget, Path};
use crate::physics::DesiredVelocity;

use super::components::{Drone, WanderTarget, WanderTimer};
use super::constants::{
    CRUISE_SPEED_MPS, VERTICAL_SPEED_FACTOR, WANDER_LERP_RATE,
};
use super::spawn::random_unit_dir;

/// Pick a fresh wander target each interval and smoothly lerp the drone's
/// `DesiredVelocity` toward it. Only fires when the drone has neither a
/// `FrontierTarget` nor a populated `Path` — wander is a cold-start /
/// fully-isolated fallback, not a constant noise source competing with
/// the steering pipeline.
pub fn wander(
    time: Res<Time>,
    mut q: Query<
        (
            &mut WanderTimer,
            &mut WanderTarget,
            &mut DesiredVelocity,
            &FrontierTarget,
            &Path,
        ),
        With<Drone>,
    >,
) {
    let mut rng = rand::rng();
    let dt = time.delta_secs();

    for (mut timer, mut target, mut desired, frontier, path) in &mut q {
        if frontier.pos.is_some() || !path.waypoints.is_empty() {
            // Drone has a real goal. Don't inject random noise on top
            // of the steering signal; the steer system owns DesiredVelocity.
            continue;
        }
        timer.0.tick(time.delta());
        if timer.0.just_finished() {
            target.0 = random_target_velocity(&mut rng);
        }
        let alpha = (WANDER_LERP_RATE * dt).min(1.0);
        desired.0 = desired.0.lerp(target.0, alpha);
    }
}

fn random_target_velocity(rng: &mut impl Rng) -> Vec3 {
    let mut dir = random_unit_dir(rng);
    dir.y *= VERTICAL_SPEED_FACTOR;
    let unit = dir.normalize_or_zero();
    if unit == Vec3::ZERO {
        return Vec3::ZERO;
    }
    unit * CRUISE_SPEED_MPS
}
