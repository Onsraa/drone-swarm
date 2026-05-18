use std::collections::HashSet;

use bevy::prelude::*;

use crate::drone::Drone;

use super::components::{LocalMapVoxel, OwnedByDrone};

/// When drones despawn (e.g. on a count change), their owned local-map
/// cubes are now orphaned. This listens for `Drone` component removals
/// and despawns the matching cubes.
pub fn cleanup_orphan_local_voxels(
    mut commands: Commands,
    mut removed: RemovedComponents<Drone>,
    voxels_q: Query<(Entity, &OwnedByDrone), With<LocalMapVoxel>>,
) {
    let removed_set: HashSet<Entity> = removed.read().collect();
    if removed_set.is_empty() {
        return;
    }
    for (entity, owner) in &voxels_q {
        if removed_set.contains(&owner.0) {
            commands.entity(entity).despawn();
        }
    }
}
