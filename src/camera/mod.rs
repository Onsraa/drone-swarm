mod components;
mod constants;
mod resources;
mod systems;

use bevy::prelude::*;

pub use resources::CameraMode;

use systems::{
    freefly_input, orbit_input, spawn_camera, sync_camera_transform, toggle_camera_mode,
};

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraMode>()
            .add_systems(Startup, spawn_camera)
            .add_systems(
                Update,
                (
                    toggle_camera_mode,
                    orbit_input,
                    freefly_input,
                    sync_camera_transform,
                )
                    .chain(),
            );
    }
}
