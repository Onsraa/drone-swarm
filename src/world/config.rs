use bevy::prelude::*;

use super::constants::{DEFAULT_VOXEL_SIZE_METERS, DEFAULT_WORLD_DIMS};

/// Bevy is right-handed Y-up: X = width, Y = height (up), Z = depth.
/// `size` is in voxel cells; multiply by `voxel_size` for world meters.
#[derive(Resource, Clone)]
pub struct WorldConfig {
    pub size: UVec3,
    pub voxel_size: f32,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            size: DEFAULT_WORLD_DIMS,
            voxel_size: DEFAULT_VOXEL_SIZE_METERS,
        }
    }
}

impl WorldConfig {
    pub fn world_size(&self) -> Vec3 {
        self.size.as_vec3() * self.voxel_size
    }

    pub fn center(&self) -> Vec3 {
        self.world_size() * 0.5
    }
}
