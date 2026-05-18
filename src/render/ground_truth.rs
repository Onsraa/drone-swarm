use bevy::prelude::*;

use crate::world::{GroundTruthMap, WorldConfig};

use super::assets::VoxelAssets;
use super::components::GroundTruthVoxel;
use super::mesh_builder::build_voxel_chunk_mesh;

pub fn spawn_ground_truth_voxels(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    assets: Res<VoxelAssets>,
    config: Res<WorldConfig>,
    map: Res<GroundTruthMap>,
) {
    let count = map.count_occupied();
    let mesh = build_voxel_chunk_mesh(map.iter_occupied(), config.voxel_size);
    let handle = meshes.add(mesh);
    commands.spawn((
        GroundTruthVoxel,
        Mesh3d(handle),
        MeshMaterial3d(assets.ground_mat.clone()),
        Transform::IDENTITY,
    ));
    info!(
        "built ground-truth voxel chunk: {} cells in a single mesh",
        count
    );
}
