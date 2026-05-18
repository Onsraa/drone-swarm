mod config;
mod constants;
mod ground_truth;
mod scene_builder;

use bevy::prelude::*;

pub use config::WorldConfig;
pub use ground_truth::GroundTruthMap;

use scene_builder::build_test_scene;

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldConfig::default())
            .add_systems(Startup, build_test_scene);
    }
}
