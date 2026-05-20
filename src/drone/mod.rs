mod components;
mod constants;
mod resources;
mod spawn;

use bevy::prelude::*;

pub use components::{Drone, DroneColor, DroneId};
pub use constants::{MAX_DRONE_COUNT, MIN_DRONE_COUNT};
pub use resources::DroneSpawnConfig;

use spawn::{init_drone_body_assets, respawn_drones_if_needed, sync_color_to_role};

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DroneSpawnConfig>()
            .add_systems(Startup, init_drone_body_assets)
            .add_systems(Update, (respawn_drones_if_needed, sync_color_to_role));
    }
}
