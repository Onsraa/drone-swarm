use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::world::{GroundTruthMap, WorldConfig};

use super::components::GroundTruthVoxel;
use super::constants::{GROUND_TRUTH_INSTANCE_COLOR, GROUND_TRUTH_SCALE_FACTOR};
use super::instancing::{InstanceData, InstancedVoxelLayer};
use super::resources::CubeMesh;

/// Update-tick spawn: builds the ground-truth instance entity whenever
/// a `GroundTruthMap` exists and no `GroundTruthVoxel` is currently in
/// the world. Lets the map-swap path despawn the entity and have it
/// reappear automatically once the new map is in place.
pub fn spawn_ground_truth_layer(
    mut commands: Commands,
    cube: Option<Res<CubeMesh>>,
    config: Option<Res<WorldConfig>>,
    map: Option<Res<GroundTruthMap>>,
    existing: Query<(), With<GroundTruthVoxel>>,
) {
    if !existing.is_empty() {
        return;
    }
    let (Some(cube), Some(config), Some(map)) = (cube, config, map) else {
        return;
    };
    let instances = build_instances(&map, config.voxel_size);
    let count = instances.len();
    commands.spawn((
        GroundTruthVoxel,
        Mesh3d(cube.0.clone()),
        InstancedVoxelLayer::new(instances),
        NoFrustumCulling,
        Transform::IDENTITY,
        Visibility::default(),
    ));
    info!("ground truth: {} instanced voxels (single draw call)", count);
}

fn build_instances(map: &GroundTruthMap, voxel_size: f32) -> Vec<InstanceData> {
    let half = voxel_size * 0.5;
    map.iter_occupied()
        .map(|cell| {
            let pos = cell.as_vec3() * voxel_size + Vec3::splat(half);
            InstanceData {
                pos_scale: [pos.x, pos.y, pos.z, voxel_size * GROUND_TRUTH_SCALE_FACTOR],
                color: GROUND_TRUTH_INSTANCE_COLOR,
            }
        })
        .collect()
}
