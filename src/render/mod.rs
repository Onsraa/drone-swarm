mod assets;
mod cleanup;
mod components;
mod constants;
mod global_map;
mod ground_truth;
mod local_map;
mod resources;

use bevy::prelude::*;

pub use components::{GlobalMapVoxel, GroundTruthVoxel, LocalMapVoxel};

use assets::init_voxel_assets;
use cleanup::cleanup_orphan_local_voxels;
use global_map::sync_global_map;
use ground_truth::spawn_ground_truth_voxels;
use local_map::{ensure_local_render, sync_local_maps};
use resources::GlobalMapRender;

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GlobalMapRender>()
            .add_systems(
                Startup,
                (
                    init_voxel_assets,
                    spawn_ground_truth_voxels.after(init_voxel_assets),
                ),
            )
            .add_systems(
                Update,
                (
                    ensure_local_render,
                    sync_local_maps,
                    sync_global_map,
                    cleanup_orphan_local_voxels,
                )
                    .chain(),
            );
    }
}
