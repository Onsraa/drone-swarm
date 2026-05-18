mod assets;
mod components;
mod constants;
mod ground_truth;
mod local_map;

use bevy::prelude::*;

pub use components::{GroundTruthVoxel, LocalMapVoxel};

use assets::init_voxel_assets;
use ground_truth::spawn_ground_truth_voxels;
use local_map::{ensure_local_render, sync_local_maps};

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Startup,
            (
                init_voxel_assets,
                spawn_ground_truth_voxels.after(init_voxel_assets),
            ),
        )
        .add_systems(Update, (ensure_local_render, sync_local_maps).chain());
    }
}
