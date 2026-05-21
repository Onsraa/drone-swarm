mod constants;
mod field;

use bevy::prelude::*;

pub use field::{Channel, PheromoneField};

use crate::drone::Drone;
use crate::exploration::Role;

use constants::{
    DECAY_RATE, DEPOSIT_ANCHOR_PER_FRAME, DEPOSIT_MAPPER_PER_FRAME, DEPOSIT_NEIGHBOR_FRACTION,
    DEPOSIT_SCOUT_PER_FRAME, DIFFUSION_RATE,
};
use field::{ensure_pheromone_sized, CHANNEL_COUNT};

/// Pheromone-field plugin. Maintains a two-channel CPU scalar grid
/// (Scout / Mapper) the swarm uses as shared memory: drones deposit
/// each frame on their role's channel, the field decays + diffuses
/// continuously, and `apply_role_steering` reads channel gradients to
/// steer scouts (anti-sum), mappers (pro-scout − anti-mapper), and to
/// drive heatmap visualisation (sum).
pub struct PheromonePlugin;

impl Plugin for PheromonePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PheromoneField>().add_systems(
            Update,
            (
                ensure_pheromone_sized,
                decay_pheromone,
                diffuse_pheromone,
                deposit_pheromone,
            )
                .chain(),
        );
    }
}

/// Per-second exponential decay of every channel. Half-life is set by
/// `DECAY_RATE` in constants.rs.
fn decay_pheromone(time: Res<Time>, mut field: ResMut<PheromoneField>) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let factor = (-DECAY_RATE * dt).exp();
    for ch in 0..CHANNEL_COUNT {
        for v in field.channels[ch].iter_mut() {
            *v *= factor;
        }
    }
}

/// Per-frame Laplacian diffusion step on every channel. Smooths sharp
/// deposit boundaries so mappers don't lose the gradient on thin
/// trails. `scratch` is a Local so the read-buffer is reused frame to
/// frame.
fn diffuse_pheromone(mut field: ResMut<PheromoneField>, mut scratch: Local<Vec<f32>>) {
    field.diffuse(DIFFUSION_RATE, &mut scratch);
}

/// Each drone drops a deposit at its current position into its role's
/// channel. Scouts → Scout channel. Mappers → Mapper channel. Anchors
/// hover so a deposit would just pile up; their amount is 0.
fn deposit_pheromone(
    mut field: ResMut<PheromoneField>,
    q: Query<(&Transform, &Role), With<Drone>>,
) {
    for (transform, role) in &q {
        let (channel, amount) = match role {
            Role::Scout => (Channel::Scout, DEPOSIT_SCOUT_PER_FRAME),
            Role::Mapper => (Channel::Mapper, DEPOSIT_MAPPER_PER_FRAME),
            Role::Anchor => (Channel::Scout, DEPOSIT_ANCHOR_PER_FRAME),
        };
        if amount <= 0.0 {
            continue;
        }
        field.deposit_at_channel(
            transform.translation,
            channel,
            amount,
            DEPOSIT_NEIGHBOR_FRACTION,
        );
    }
}
