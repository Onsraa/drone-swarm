mod centering;
mod components;
mod constants;
mod resources;
mod spawn;
mod wander;

use bevy::prelude::*;

pub use components::{Drone, DroneColor};
pub use constants::{MAX_DRONE_COUNT, MIN_DRONE_COUNT};
pub use resources::DroneSpawnConfig;

use crate::physics::PhysicsSet;

use centering::recenter_visuals;
use spawn::respawn_drones_if_needed;
use wander::wander;

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DroneSpawnConfig>().add_systems(
            Update,
            (
                respawn_drones_if_needed,
                recenter_visuals,
                wander.before(PhysicsSet::Control),
            ),
        );
    }
}
