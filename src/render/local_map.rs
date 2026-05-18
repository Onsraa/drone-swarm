use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::map::{CellState, LocalMap};
use crate::world::WorldConfig;

use super::components::LocalMapVoxel;
use super::constants::{LOCAL_MAP_ALPHA, LOCAL_MAP_COLOR_FACTOR};
use super::instancing::{InstanceData, InstancedVoxelLayer};
use super::resources::CubeMesh;

/// One instanced layer aggregating every drone's local map. Per-instance
/// color comes from each drone's `DroneColor` so all drones share a
/// single draw call. Rebuilt every frame any drone's `LocalMap` was
/// mutated.
pub fn sync_local_maps(
    mut commands: Commands,
    cube: Res<CubeMesh>,
    config: Res<WorldConfig>,
    drones_q: Query<(&LocalMap, &DroneColor), With<Drone>>,
    changed_q: Query<(), (With<Drone>, Changed<LocalMap>)>,
    mut layer_q: Query<&mut InstancedVoxelLayer, With<LocalMapVoxel>>,
    layer_exists_q: Query<(), With<LocalMapVoxel>>,
) {
    let no_change_yet = changed_q.is_empty() && !layer_exists_q.is_empty();
    if no_change_yet {
        return;
    }

    let mut instances: Vec<InstanceData> = Vec::new();
    let voxel_size = config.voxel_size;
    let half = voxel_size * 0.5;
    for (local_map, color) in &drones_q {
        let linear = color.0.to_linear();
        let instance_color = [
            (linear.red * LOCAL_MAP_COLOR_FACTOR).min(1.0),
            (linear.green * LOCAL_MAP_COLOR_FACTOR).min(1.0),
            (linear.blue * LOCAL_MAP_COLOR_FACTOR).min(1.0),
            LOCAL_MAP_ALPHA,
        ];
        for (cell, state) in local_map.0.iter_known() {
            if state == CellState::Occupied {
                let pos = cell.as_vec3() * voxel_size + Vec3::splat(half);
                instances.push(InstanceData {
                    pos_scale: [pos.x, pos.y, pos.z, voxel_size],
                    color: instance_color,
                });
            }
        }
    }

    if let Ok(mut layer) = layer_q.single_mut() {
        layer.0 = instances;
    } else if !instances.is_empty() {
        commands.spawn((
            LocalMapVoxel,
            Mesh3d(cube.0.clone()),
            InstancedVoxelLayer(instances),
            NoFrustumCulling,
            Transform::IDENTITY,
            Visibility::default(),
        ));
    }
}
