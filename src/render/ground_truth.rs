use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::world::{GroundTruthMap, WorldConfig};

use super::components::GroundTruthVoxel;
use super::constants::GROUND_TRUTH_INSTANCE_COLOR;
use super::instancing::{InstanceData, InstancedVoxelLayer};
use super::resources::CubeMesh;

pub fn spawn_ground_truth_layer(
    mut commands: Commands,
    cube: Res<CubeMesh>,
    config: Res<WorldConfig>,
    map: Res<GroundTruthMap>,
) {
    let instances = build_instances(&map, config.voxel_size);
    let count = instances.len();
    commands.spawn((
        GroundTruthVoxel,
        Mesh3d(cube.0.clone()),
        InstancedVoxelLayer(instances),
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
                pos_scale: [pos.x, pos.y, pos.z, voxel_size],
                color: GROUND_TRUTH_INSTANCE_COLOR,
            }
        })
        .collect()
}
