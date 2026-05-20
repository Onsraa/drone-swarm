mod constants;
mod field;

use bevy::prelude::*;

pub use field::PheromoneField;

use crate::drone::Drone;
use crate::exploration::Role;

use constants::{
    DECAY_RATE, DEPOSIT_ANCHOR_PER_FRAME, DEPOSIT_MAPPER_PER_FRAME, DEPOSIT_NEIGHBOR_FRACTION,
    DEPOSIT_SCOUT_PER_FRAME,
};
use field::ensure_pheromone_sized;

/// Pheromone-field plugin. Maintains a CPU scalar grid the swarm uses
/// as a shared memory: drones deposit each frame, the field decays
/// continuously, and `apply_role_steering` reads the local gradient to
/// steer scouts away from explored zones + mappers toward them.
pub struct PheromonePlugin;

impl Plugin for PheromonePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PheromoneField>().add_systems(
            Update,
            (
                ensure_pheromone_sized,
                decay_pheromone,
                deposit_pheromone,
            )
                .chain(),
        );
    }
}

/// Per-second exponential decay of the entire field. Half-life is set
/// by `DECAY_RATE` in constants.rs.
fn decay_pheromone(time: Res<Time>, mut field: ResMut<PheromoneField>) {
    let dt = time.delta_secs();
    if dt <= 0.0 || field.cells.is_empty() {
        return;
    }
    let factor = (-DECAY_RATE * dt).exp();
    for v in field.cells.iter_mut() {
        *v *= factor;
    }
}

/// Each drone drops a pheromone deposit at its current position.
/// Per-role amount: scouts deposit heavily (they're the trail-blazers),
/// mappers + anchors don't deposit.
fn deposit_pheromone(
    mut field: ResMut<PheromoneField>,
    q: Query<(&Transform, &Role), With<Drone>>,
) {
    if field.cells.is_empty() {
        return;
    }
    for (transform, role) in &q {
        let amount = match role {
            Role::Scout => DEPOSIT_SCOUT_PER_FRAME,
            Role::Mapper => DEPOSIT_MAPPER_PER_FRAME,
            Role::Anchor => DEPOSIT_ANCHOR_PER_FRAME,
        };
        if amount <= 0.0 {
            continue;
        }
        field.deposit_at(transform.translation, amount, DEPOSIT_NEIGHBOR_FRACTION);
    }
}
