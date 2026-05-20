#[allow(dead_code)]
mod centering;
mod components;
mod constants;
mod resources;
mod spawn;
#[allow(dead_code)]
mod wander;

use bevy::prelude::*;

pub use components::{Drone, DroneColor, DroneId};
pub use constants::{MAX_DRONE_COUNT, MIN_DRONE_COUNT};
pub use resources::DroneSpawnConfig;

use spawn::{init_drone_body_assets, respawn_drones_if_needed, sync_color_to_role};

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        // Body mesh + per-role materials get built once at startup.
        // `respawn_drones_if_needed` waits for that resource before
        // spawning any drones. GLB pipeline retired in this commit —
        // drones are now flat role-tinted cuboids.
        app.init_resource::<DroneSpawnConfig>()
            .add_systems(Startup, init_drone_body_assets)
            .add_systems(Update, (respawn_drones_if_needed, sync_color_to_role));
    }
}
