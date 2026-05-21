use bevy::prelude::*;

use super::constants::DEFAULT_SCENE_PATH;

#[derive(Resource, Debug, Clone)]
#[allow(dead_code)]
pub struct MeshGroundTruthConfig {
    pub scene_asset_path: String,
    pub visible: bool,
    pub spawned: bool,
}

impl Default for MeshGroundTruthConfig {
    fn default() -> Self {
        Self {
            scene_asset_path: DEFAULT_SCENE_PATH.to_string(),
            visible: true,
            spawned: false,
        }
    }
}
