use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::map::{unflatten, CellState, LocalMap};
use crate::world::WorldConfig;

use super::components::LocalMapVoxel;
use super::constants::{LOCAL_MAP_ALPHA, LOCAL_MAP_COLOR_FACTOR};
use super::instancing::{InstanceData, InstancedVoxelLayer};
use super::resources::CubeMesh;

/// One instanced layer aggregating every drone's local map. Per-instance
/// color comes from each drone's `DroneColor`.
///
/// Steady state: each frame we drain each drone's `dirty_occupied` queue
/// (cells flipped to `Occupied` since the last drain) and append matching
/// instances to the layer — `Occupied` is sticky, so the buffer is purely
/// additive. The persistent GPU buffer then streams only the new tail.
///
/// Full rebuild happens only when the drone count changes (respawn). At
/// that point the previous layer's contents are stale (drones were
/// despawned) so we `replace` and bump generation, forcing the GPU buffer
/// to re-upload from offset 0.
pub fn sync_local_maps(
    mut commands: Commands,
    cube: Res<CubeMesh>,
    config: Res<WorldConfig>,
    mut drones_q: Query<(&mut LocalMap, &DroneColor), With<Drone>>,
    mut layer_q: Query<&mut InstancedVoxelLayer, With<LocalMapVoxel>>,
    mut prev_count: Local<u32>,
) {
    let count = drones_q.iter().count() as u32;
    let count_changed = count != *prev_count;
    *prev_count = count;

    let voxel_size = config.voxel_size;
    let half = voxel_size * 0.5;

    if count_changed {
        let mut instances: Vec<InstanceData> = Vec::new();
        for (mut local, color) in &mut drones_q {
            let instance_color = drone_instance_color(color);
            // Drain pending dirty cells so the next append cycle starts clean.
            let _ = local.0.drain_dirty_occupied().count();
            for (cell, state) in local.0.iter_known() {
                if state == CellState::Occupied {
                    instances.push(make_instance(cell, voxel_size, half, instance_color));
                }
            }
        }
        if let Ok(mut layer) = layer_q.single_mut() {
            layer.replace(instances);
        } else if !instances.is_empty() {
            info!("spawning local-map instanced layer ({} instances)", instances.len());
            commands.spawn((
                LocalMapVoxel,
                Mesh3d(cube.0.clone()),
                InstancedVoxelLayer::new(instances),
                NoFrustumCulling,
                Transform::IDENTITY,
                Visibility::default(),
            ));
        }
        return;
    }

    let mut appended: Vec<InstanceData> = Vec::new();
    for (mut local, color) in &mut drones_q {
        if !local.0.has_dirty_occupied() {
            continue;
        }
        let instance_color = drone_instance_color(color);
        let dims = local.0.dims;
        for flat in local.0.drain_dirty_occupied() {
            let cell = unflatten(flat, dims);
            appended.push(make_instance(cell, voxel_size, half, instance_color));
        }
    }
    if appended.is_empty() {
        return;
    }
    if let Ok(mut layer) = layer_q.single_mut() {
        layer.append(appended);
    } else {
        info!("spawning local-map instanced layer ({} instances)", appended.len());
        commands.spawn((
            LocalMapVoxel,
            Mesh3d(cube.0.clone()),
            InstancedVoxelLayer::new(appended),
            NoFrustumCulling,
            Transform::IDENTITY,
            Visibility::default(),
        ));
    }
}

fn drone_instance_color(color: &DroneColor) -> [f32; 4] {
    let linear = color.0.to_linear();
    [
        (linear.red * LOCAL_MAP_COLOR_FACTOR).min(1.0),
        (linear.green * LOCAL_MAP_COLOR_FACTOR).min(1.0),
        (linear.blue * LOCAL_MAP_COLOR_FACTOR).min(1.0),
        LOCAL_MAP_ALPHA,
    ]
}

fn make_instance(cell: IVec3, voxel_size: f32, half: f32, color: [f32; 4]) -> InstanceData {
    let pos = cell.as_vec3() * voxel_size + Vec3::splat(half);
    InstanceData {
        pos_scale: [pos.x, pos.y, pos.z, voxel_size],
        color,
    }
}
