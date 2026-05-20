use bevy::prelude::*;

use crate::world::WorldConfig;

use super::components::{DesiredVelocity, LinearVelocity};
use super::constants::VEL_TRACK_GAIN;

/// Point-mass velocity tracker. Lerps `linvel` toward `desired`, then
/// integrates the resulting velocity into `Transform.translation`.
/// Replaces the old quadcopter controller + integrator. No body
/// orientation involved — cosmetic visuals live in a separate system.
pub fn track_velocity(
    time: Res<Time>,
    config: Res<WorldConfig>,
    mut q: Query<(&mut Transform, &mut LinearVelocity, &DesiredVelocity)>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let alpha = (VEL_TRACK_GAIN * dt).min(1.0);
    let world_size = config.world_size();
    for (mut transform, mut linvel, desired) in &mut q {
        linvel.0 = linvel.0.lerp(desired.0, alpha);
        transform.translation += linvel.0 * dt;
        clamp_to_world(&mut transform.translation, &mut linvel.0, world_size);
    }
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
