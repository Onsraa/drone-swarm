mod assets;
mod components;
pub mod constants;
mod cube_instancing;
mod gpu_global_map;
mod gpu_lidar_points;
mod gpu_local_map;
mod ground_truth;
mod instancing;
mod raycast_viz;
mod resources;

use bevy::prelude::*;

pub use components::{GpuGlobalMapVoxel, GpuLocalMapVoxel, GroundTruthVoxel, LidarPointVoxel};

use assets::init_voxel_assets;
use cube_instancing::InstancedCubePlugin;
use gpu_global_map::GpuGlobalMapPlugin;
use gpu_lidar_points::GpuLidarPointsPlugin;
use gpu_local_map::GpuLocalMapPlugin;
use ground_truth::spawn_ground_truth_layer;
use instancing::InstancedVoxelPlugin;
use raycast_viz::RaycastVizPlugin;

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InstancedVoxelPlugin)
            .add_plugins(InstancedCubePlugin)
            .add_plugins(GpuLocalMapPlugin)
            .add_plugins(GpuGlobalMapPlugin)
            .add_plugins(GpuLidarPointsPlugin)
            .add_plugins(RaycastVizPlugin)
            .add_systems(Startup, init_voxel_assets)
            .add_systems(Update, spawn_ground_truth_layer);
    }
}
