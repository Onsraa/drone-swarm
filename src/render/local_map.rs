use bevy::prelude::*;

use crate::drone::Drone;
use crate::map::{CellState, LocalMap};
use crate::world::WorldConfig;

use super::assets::VoxelAssets;
use super::components::{LocalMapRender, LocalMapVoxel};

pub fn ensure_local_render(
    mut commands: Commands,
    drones_q: Query<Entity, (With<Drone>, With<LocalMap>, Without<LocalMapRender>)>,
) {
    for entity in &drones_q {
        commands.entity(entity).insert(LocalMapRender::default());
    }
}

pub fn sync_local_maps(
    mut commands: Commands,
    assets: Option<Res<VoxelAssets>>,
    config: Res<WorldConfig>,
    mut drones_q: Query<(&LocalMap, &mut LocalMapRender), With<Drone>>,
) {
    let Some(assets) = assets else {
        return;
    };
    let s = config.voxel_size;
    let half = Vec3::splat(s * 0.5);

    for (local_map, mut render) in &mut drones_q {
        for (cell, state) in local_map.0.iter_known() {
            if state == CellState::Occupied && !render.spawned.contains_key(&cell) {
                let position = cell.as_vec3() * s + half;
                let entity = commands
                    .spawn((
                        LocalMapVoxel,
                        Mesh3d(assets.cube.clone()),
                        MeshMaterial3d(assets.local_occupied_mat.clone()),
                        Transform::from_translation(position),
                    ))
                    .id();
                render.spawned.insert(cell, entity);
            }
        }
    }
}
