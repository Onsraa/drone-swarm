mod assets;
mod components;
pub mod constants;
mod gpu_global_map;
mod gpu_lidar_points;
mod gpu_local_map;
mod instancing;
mod pheromone_render;
mod raycast_viz;
mod resources;

use bevy::prelude::*;

pub use components::{GpuGlobalMapVoxel, GpuLocalMapVoxel, LidarPointVoxel, PheromoneVoxel};

use assets::init_voxel_assets;
use gpu_global_map::GpuGlobalMapPlugin;
use gpu_lidar_points::GpuLidarPointsPlugin;
use gpu_local_map::GpuLocalMapPlugin;
use instancing::InstancedVoxelPlugin;
use pheromone_render::PheromoneRenderPlugin;
use raycast_viz::RaycastVizPlugin;

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InstancedVoxelPlugin)
            .add_plugins(GpuLocalMapPlugin)
            .add_plugins(GpuGlobalMapPlugin)
            .add_plugins(GpuLidarPointsPlugin)
            .add_plugins(RaycastVizPlugin)
            .add_plugins(PheromoneRenderPlugin)
            .add_systems(Startup, init_voxel_assets);
    }
}
