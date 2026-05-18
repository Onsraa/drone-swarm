use bevy::prelude::*;

use crate::world::WorldConfig;

use super::constants::{
    GROUND_TRUTH_BASE_COLOR, GROUND_TRUTH_ROUGHNESS, LOCAL_OCCUPIED_BASE_COLOR,
    LOCAL_OCCUPIED_EMISSIVE,
};

#[derive(Resource)]
pub struct VoxelAssets {
    pub cube: Handle<Mesh>,
    pub ground_mat: Handle<StandardMaterial>,
    pub local_occupied_mat: Handle<StandardMaterial>,
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
    let local_occupied_mat = materials.add(StandardMaterial {
        base_color: LOCAL_OCCUPIED_BASE_COLOR,
        emissive: LOCAL_OCCUPIED_EMISSIVE,
        alpha_mode: AlphaMode::Blend,
        ..Default::default()
    });

    commands.insert_resource(VoxelAssets {
        cube,
        ground_mat,
        local_occupied_mat,
    });
}
