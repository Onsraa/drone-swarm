use bevy::prelude::*;

use super::constants::{
    GLOBAL_OCCUPIED_BASE_COLOR, GLOBAL_OCCUPIED_EMISSIVE, GROUND_TRUTH_BASE_COLOR,
    GROUND_TRUTH_ROUGHNESS,
};

/// Materials shared across all voxel layers. Per-drone local-map materials
/// live on each drone via `DroneMaterial`. With the batched-mesh renderer,
/// no shared cube mesh is needed — each layer builds its own chunk mesh.
#[derive(Resource)]
pub struct VoxelAssets {
    pub ground_mat: Handle<StandardMaterial>,
    pub global_occupied_mat: Handle<StandardMaterial>,
}

pub fn init_voxel_assets(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let ground_mat = materials.add(StandardMaterial {
        base_color: GROUND_TRUTH_BASE_COLOR,
        perceptual_roughness: GROUND_TRUTH_ROUGHNESS,
        ..Default::default()
    });
    let global_occupied_mat = materials.add(StandardMaterial {
        base_color: GLOBAL_OCCUPIED_BASE_COLOR,
        emissive: GLOBAL_OCCUPIED_EMISSIVE,
        alpha_mode: AlphaMode::Blend,
        ..Default::default()
    });

    commands.insert_resource(VoxelAssets {
        ground_mat,
        global_occupied_mat,
    });
}
