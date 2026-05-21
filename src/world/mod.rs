mod config;
pub mod constants;
mod ground_truth;
pub mod mesh;

use bevy::prelude::*;

pub use config::WorldConfig;
pub use ground_truth::GroundTruthMap;
pub use mesh::{
    ground_altitude, raycast_bvh, MeshGroundTruthConfig, MeshGroundTruthPlugin, WorldBvh,
};

/// Inserts an initial `WorldConfig` placeholder so other plugins can
/// safely depend on the resource existing at app build time. The real
/// dims + voxel_size land via the `MapSwapRequested` pipeline in
/// `crate::maps` once the first map asset loads.
pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldConfig::default());
    }
}
