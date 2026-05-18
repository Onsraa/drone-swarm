mod assets;
mod components;
mod constants;
mod global_map;
mod ground_truth;
mod instancing;
mod local_map;
mod resources;

use bevy::prelude::*;

pub use components::{GlobalMapVoxel, GroundTruthVoxel, LocalMapVoxel};

use assets::init_voxel_assets;
use global_map::sync_global_map;
use ground_truth::spawn_ground_truth_layer;
use instancing::InstancedVoxelPlugin;
use local_map::sync_local_maps;

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(InstancedVoxelPlugin)
            .add_systems(
                Startup,
                (
                    init_voxel_assets,
                    spawn_ground_truth_layer.after(init_voxel_assets),
                ),
            )
            .add_systems(Update, (sync_local_maps, sync_global_map));
    }
}
