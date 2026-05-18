use bevy::prelude::*;

use crate::drone::Drone;
use crate::map::{GlobalMap, LocalMap, VoxelMap};
use crate::world::WorldConfig;

use super::resources::MergeTimer;

pub fn init_global_map(mut commands: Commands, config: Res<WorldConfig>) {
    commands.insert_resource(GlobalMap(VoxelMap::new(config.size)));
}

/// At each tick, upgrade every cell of the global map from each drone's
/// local observations. The map's own `upgrade` rule (Occupied sticky,
/// Free overrides only Unknown) handles conflict resolution, so the merge
/// itself is a straight cell-by-cell fold.
pub fn merge_local_into_global(
    time: Res<Time>,
    mut timer: ResMut<MergeTimer>,
    global: Option<ResMut<GlobalMap>>,
    drones_q: Query<&LocalMap, With<Drone>>,
) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let Some(mut global) = global else {
        return;
    };
    for local in &drones_q {
        for (cell, state) in local.0.iter_known() {
            global.0.upgrade(cell, state);
        }
    }
}
