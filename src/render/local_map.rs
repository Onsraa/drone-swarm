use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::map::{CellState, LocalMap};
use crate::world::WorldConfig;

use super::components::{DroneMaterial, LocalMapMeshHandle, LocalMapVoxel, OwnedByDrone};
use super::constants::LOCAL_MAP_EMISSIVE_FACTOR;
use super::mesh_builder::{build_voxel_chunk_mesh, empty_voxel_mesh};

/// On each new drone, lazily create its per-drone material + per-drone
/// mesh entity, and link them via `LocalMapMeshHandle`. The render module
/// is the sole owner of `StandardMaterial` and `Mesh` for the drone's
/// local-map visualization, so the drone module never touches those.
pub fn ensure_local_render(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    drones_q: Query<
        (Entity, &DroneColor),
        (With<Drone>, With<LocalMap>, Without<LocalMapMeshHandle>),
    >,
) {
    for (drone_entity, color) in &drones_q {
        let linear = color.0.to_linear();
        let material = materials.add(StandardMaterial {
            base_color: color.0,
            emissive: LinearRgba::rgb(
                linear.red * LOCAL_MAP_EMISSIVE_FACTOR,
                linear.green * LOCAL_MAP_EMISSIVE_FACTOR,
                linear.blue * LOCAL_MAP_EMISSIVE_FACTOR,
            ),
            alpha_mode: AlphaMode::Blend,
            ..Default::default()
        });
        let mesh_handle = meshes.add(empty_voxel_mesh());
        commands.spawn((
            LocalMapVoxel,
            OwnedByDrone(drone_entity),
            Mesh3d(mesh_handle.clone()),
            MeshMaterial3d(material.clone()),
            Transform::IDENTITY,
        ));
        commands.entity(drone_entity).insert((
            DroneMaterial(material),
            LocalMapMeshHandle { mesh: mesh_handle },
        ));
    }
}

/// Rebuild only the meshes whose owning drone's `LocalMap` was mutated
/// this frame (scoped via `Changed<LocalMap>`).
pub fn sync_local_maps(
    mut meshes: ResMut<Assets<Mesh>>,
    config: Res<WorldConfig>,
    drones_q: Query<(&LocalMap, &LocalMapMeshHandle), (With<Drone>, Changed<LocalMap>)>,
) {
    for (local_map, handle) in &drones_q {
        let occupied: Vec<IVec3> = local_map
            .0
            .iter_known()
            .filter_map(|(cell, state)| (state == CellState::Occupied).then_some(cell))
            .collect();
        let new_mesh = if occupied.is_empty() {
            empty_voxel_mesh()
        } else {
            build_voxel_chunk_mesh(occupied, config.voxel_size)
        };
        if let Some(asset) = meshes.get_mut(&handle.mesh) {
            *asset = new_mesh;
        }
    }
}
