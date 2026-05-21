use bevy::prelude::*;

use super::constants::{DEFAULT_SCENE_PATH, DEFAULT_SCENE_POS, DEFAULT_SCENE_SCALE};

#[derive(Resource, Debug, Clone)]
#[allow(dead_code)]
pub struct MeshGroundTruthConfig {
    pub scene_asset_path: String,
    pub visible: bool,
    pub spawned: bool,
    /// World-space translation applied to the SceneRoot at spawn.
    /// Default puts the scene at the centre of the 640×24×640 world.
    pub translation: Vec3,
    /// Uniform scale applied to the SceneRoot.
    pub scale: f32,
    /// UI sets `true` on Apply-button click; the invalidate system
    /// despawns the current scene + clears `WorldBvh` so the next frame
    /// rebuilds with the new transform.
    pub apply_requested: bool,
    /// Last (translation, scale) that was actually spawned. Used by the
    /// invalidate system to detect divergence from the current values.
    pub applied_transform: Option<(Vec3, f32)>,
}

impl Default for MeshGroundTruthConfig {
    fn default() -> Self {
        Self {
            scene_asset_path: DEFAULT_SCENE_PATH.to_string(),
            visible: true,
            spawned: false,
            translation: DEFAULT_SCENE_POS,
            scale: DEFAULT_SCENE_SCALE,
            apply_requested: false,
            applied_transform: None,
        }
    }
}
