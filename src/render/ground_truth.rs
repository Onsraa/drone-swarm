use bevy::prelude::*;

use crate::world::{GroundTruthMap, WorldConfig};

use super::assets::VoxelAssets;
use super::components::GroundTruthVoxel;

pub fn spawn_ground_truth_voxels(
    mut commands: Commands,
    assets: Res<VoxelAssets>,
    config: Res<WorldConfig>,
    map: Res<GroundTruthMap>,
) {
    let s = config.voxel_size;
    let half = Vec3::splat(s * 0.5);
    let mut count = 0;
    for cell in map.iter_occupied() {
        let position = cell.as_vec3() * s + half;
        commands.spawn((
            GroundTruthVoxel,
            Mesh3d(assets.cube.clone()),
            MeshMaterial3d(assets.ground_mat.clone()),
            Transform::from_translation(position),
        ));
        count += 1;
    }
    info!("spawned {} voxel cubes for ground truth", count);
}
