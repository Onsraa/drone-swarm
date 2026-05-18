use bevy::prelude::*;

use crate::world::WorldConfig;

use super::constants::{
    GLOBAL_OCCUPIED_BASE_COLOR, GLOBAL_OCCUPIED_EMISSIVE, GROUND_TRUTH_BASE_COLOR,
    GROUND_TRUTH_ROUGHNESS,
};

/// Shared mesh + non-drone-specific materials. Per-drone local-map
/// materials live on each drone via `DroneMaterial`.
#[derive(Resource)]
pub struct VoxelAssets {
    pub cube: Handle<Mesh>,
    pub ground_mat: Handle<StandardMaterial>,
    pub global_occupied_mat: Handle<StandardMaterial>,
}

pub fn init_voxel_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<WorldConfig>,
) {
    let s = config.voxel_size;
    let cube = meshes.add(Cuboid::new(s, s, s));

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
        cube,
        ground_mat,
        global_occupied_mat,
    });
}
