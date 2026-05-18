use bevy::prelude::*;

use crate::map::{CellState, GlobalMap};
use crate::world::WorldConfig;

use super::assets::VoxelAssets;
use super::components::GlobalMapVoxel;
use super::mesh_builder::{build_voxel_chunk_mesh, empty_voxel_mesh};
use super::resources::GlobalMapRender;

/// One chunk mesh for the entire global map. Rebuilt only on frames when
/// the `GlobalMap` resource was written to (i.e. merge ticks).
pub fn sync_global_map(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    assets: Option<Res<VoxelAssets>>,
    config: Res<WorldConfig>,
    global: Option<Res<GlobalMap>>,
    mut render: ResMut<GlobalMapRender>,
) {
    let (Some(assets), Some(global)) = (assets, global) else {
        return;
    };
    if !global.is_changed() && render.handle.is_some() {
        return;
    }

    let occupied: Vec<IVec3> = global
        .0
        .iter_known()
        .filter_map(|(cell, state)| (state == CellState::Occupied).then_some(cell))
        .collect();
    let mesh = if occupied.is_empty() {
        empty_voxel_mesh()
    } else {
        build_voxel_chunk_mesh(occupied, config.voxel_size)
    };

    match render.handle.as_ref() {
        Some(handle) => {
            if let Some(asset) = meshes.get_mut(handle) {
                *asset = mesh;
            }
        }
        None => {
            let handle = meshes.add(mesh);
            commands.spawn((
                GlobalMapVoxel,
                Mesh3d(handle.clone()),
                MeshMaterial3d(assets.global_occupied_mat.clone()),
                Transform::IDENTITY,
            ));
            render.handle = Some(handle);
        }
    }
}
