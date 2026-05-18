mod components;
mod constants;
mod systems;

use bevy::prelude::*;

use systems::{orbit_input, spawn_camera, sync_camera_transform};

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_camera)
            .add_systems(Update, (orbit_input, sync_camera_transform).chain());
    }
}
