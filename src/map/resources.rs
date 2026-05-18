use bevy::prelude::*;

use super::voxel_map::VoxelMap;

/// Centralized map that aggregates each drone's `LocalMap` via the merge
/// system. Same dims and indexing as `LocalMap` and `GroundTruthMap`.
#[derive(Resource)]
pub struct GlobalMap(pub VoxelMap);
