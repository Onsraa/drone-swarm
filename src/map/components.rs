use bevy::prelude::*;

use super::voxel_map::VoxelMap;

#[derive(Component)]
pub struct LocalMap(pub VoxelMap);
