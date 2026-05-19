mod assets;
mod components;
pub mod constants;
mod gpu_global_map;
mod gpu_local_map;
mod ground_truth;
mod instancing;
mod resources;

use bevy::prelude::*;

pub use components::{GpuGlobalMapVoxel, GpuLocalMapVoxel, GroundTruthVoxel};

use assets::init_voxel_assets;
use gpu_global_map::GpuGlobalMapPlugin;
use gpu_local_map::GpuLocalMapPlugin;
use ground_truth::spawn_ground_truth_layer;
use instancing::InstancedVoxelPlugin;

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InstancedVoxelPlugin)
            .add_plugins(GpuLocalMapPlugin)
            .add_plugins(GpuGlobalMapPlugin)
            .add_systems(Startup, init_voxel_assets)
            .add_systems(Update, spawn_ground_truth_layer);
    }
}
