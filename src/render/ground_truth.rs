use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::world::{GroundTruthMap, WorldConfig};

use super::components::GroundTruthVoxel;
use super::constants::GROUND_TRUTH_CUBE_COLOR;
use super::cube_instancing::{CubeMeshHandle, InstancedCubeLayer};
use super::instancing::InstanceData;

/// Update-tick spawn: builds the ground-truth voxel cube layer whenever
/// a `GroundTruthMap` exists and no `GroundTruthVoxel` is currently in
/// the world. Renders as transparent unit cubes so the local + global
/// map billboard layers can paint inside them.
pub fn spawn_ground_truth_layer(
    mut commands: Commands,
    cube_mesh: Option<Res<CubeMeshHandle>>,
    config: Option<Res<WorldConfig>>,
    map: Option<Res<GroundTruthMap>>,
    existing: Query<(), With<GroundTruthVoxel>>,
) {
    if !existing.is_empty() {
        return;
    }
    let (Some(cube_mesh), Some(config), Some(map)) = (cube_mesh, config, map) else {
        return;
    };
    let instances = build_instances(&map, config.voxel_size);
    let count = instances.len();
    commands.spawn((
        GroundTruthVoxel,
        Mesh3d(cube_mesh.0.clone()),
        InstancedCubeLayer::new(instances),
        NoFrustumCulling,
        Transform::IDENTITY,
        Visibility::default(),
    ));
    info!("ground truth: {} instanced voxel cubes (single draw call)", count);
}

fn build_instances(map: &GroundTruthMap, voxel_size: f32) -> Vec<InstanceData> {
    let half = voxel_size * 0.5;
    map.iter_occupied()
        .map(|cell| {
            let pos = cell.as_vec3() * voxel_size + Vec3::splat(half);
            // `pos_scale.w` is the cube side length in meters.
            InstanceData {
                pos_scale: [pos.x, pos.y, pos.z, voxel_size],
                color: GROUND_TRUTH_CUBE_COLOR,
            }
        })
        .collect()
}
