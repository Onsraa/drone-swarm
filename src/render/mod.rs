mod assets;
mod components;
pub mod constants;
mod global_map;
mod gpu_local_map;
mod ground_truth;
mod instancing;
mod local_map;
mod resources;

use bevy::prelude::*;

pub use components::{GlobalMapVoxel, GpuLocalMapVoxel, GroundTruthVoxel, LocalMapVoxel};

use assets::init_voxel_assets;
use global_map::sync_global_map;
use gpu_local_map::GpuLocalMapPlugin;
use ground_truth::spawn_ground_truth_layer;
use instancing::InstancedVoxelPlugin;

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InstancedVoxelPlugin)
            .add_plugins(GpuLocalMapPlugin)
            .add_systems(
                Startup,
                (
                    init_voxel_assets,
                    spawn_ground_truth_layer.after(init_voxel_assets),
                ),
            )
            // The CPU `sync_local_maps` path is replaced by the GPU
            // build pass + `GpuLocalMapPlugin`. `sync_global_map` still
            // drives the central map renderer until Tier 3 follow-up.
            .add_systems(Update, sync_global_map);
    }
}
