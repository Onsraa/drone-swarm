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

use centering::recenter_visuals;
use spawn::{respawn_drones_if_needed, sync_color_to_role};

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        // `wander` is no longer scheduled. `apply_role_steering` in
        // `ExplorationPlugin` owns `DesiredVelocity` for every drone
        // now (per-role pheromone-gradient + separation + terrain
        // repulsion). Wander module is kept around as dead code for
        // one or two more commits in case we want to revive a
        // cold-start fallback; Phase 6 of the foraging-colony plan
        // deletes it.
        app.init_resource::<DroneSpawnConfig>().add_systems(
            Update,
            (respawn_drones_if_needed, recenter_visuals, sync_color_to_role),
        );
    }
}
