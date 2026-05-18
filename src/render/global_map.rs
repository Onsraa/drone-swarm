use bevy::prelude::*;

use crate::map::{CellState, GlobalMap};
use crate::world::WorldConfig;

use super::assets::VoxelAssets;
use super::components::GlobalMapVoxel;
use super::resources::GlobalMapRender;

pub fn sync_global_map(
    mut commands: Commands,
    assets: Option<Res<VoxelAssets>>,
    config: Res<WorldConfig>,
    global: Option<Res<GlobalMap>>,
    mut render: ResMut<GlobalMapRender>,
) {
    let (Some(assets), Some(global)) = (assets, global) else {
        return;
    };
    let s = config.voxel_size;
    let half = Vec3::splat(s * 0.5);

    for (cell, state) in global.0.iter_known() {
        if state == CellState::Occupied && !render.spawned.contains_key(&cell) {
            let position = cell.as_vec3() * s + half;
            let entity = commands
                .spawn((
                    GlobalMapVoxel,
                    Mesh3d(assets.cube.clone()),
                    MeshMaterial3d(assets.global_occupied_mat.clone()),
                    Transform::from_translation(position),
                ))
                .id();
            render.spawned.insert(cell, entity);
        }
    }
}
