use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::map::{CellState, GlobalMap};
use crate::world::WorldConfig;

use super::components::GlobalMapVoxel;
use super::constants::GLOBAL_OCCUPIED_INSTANCE_COLOR;
use super::instancing::{InstanceData, InstancedVoxelLayer};
use super::resources::CubeMesh;

/// One instanced layer for the entire global map. Rebuild the instance
/// buffer only on frames where the `GlobalMap` resource was written to.
pub fn sync_global_map(
    mut commands: Commands,
    cube: Res<CubeMesh>,
    config: Res<WorldConfig>,
    global: Option<Res<GlobalMap>>,
    mut layer_q: Query<&mut InstancedVoxelLayer, With<GlobalMapVoxel>>,
    layer_exists_q: Query<(), With<GlobalMapVoxel>>,
) {
    let Some(global) = global else {
        return;
    };
    if !global.is_changed() && !layer_exists_q.is_empty() {
        return;
    }

    let instances = build_instances(&global, config.voxel_size);

    if let Ok(mut layer) = layer_q.single_mut() {
        layer.replace(instances);
    } else if !instances.is_empty() {
        commands.spawn((
            GlobalMapVoxel,
            Mesh3d(cube.0.clone()),
            InstancedVoxelLayer::new(instances),
            NoFrustumCulling,
            Transform::IDENTITY,
            Visibility::default(),
        ));
    }
}

fn build_instances(global: &GlobalMap, voxel_size: f32) -> Vec<InstanceData> {
    let half = voxel_size * 0.5;
    global
        .0
        .iter_known()
        .filter_map(|(cell, state)| {
            (state == CellState::Occupied).then(|| {
                let pos = cell.as_vec3() * voxel_size + Vec3::splat(half);
                InstanceData {
                    pos_scale: [pos.x, pos.y, pos.z, voxel_size],
                    color: GLOBAL_OCCUPIED_INSTANCE_COLOR,
                }
            })
        })
        .collect()
}
