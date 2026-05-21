mod config;
pub mod constants;
pub mod mesh;

use bevy::prelude::*;

pub use config::WorldConfig;
pub use mesh::{
    ground_altitude, raycast_bvh, MeshGroundTruthConfig, MeshGroundTruthPlugin, WorldBvh,
};

/// Inserts the default `WorldConfig` (640×24×640 at 1 m voxels) so
/// every downstream plugin can depend on the resource existing at app
/// build time. Ground truth lives in the mesh BVH (`WorldBvh`); the
/// `WorldConfig` here only describes the drone-discovery voxel grid
/// resolution + sim bounds.
pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldConfig::default());
    }
}
