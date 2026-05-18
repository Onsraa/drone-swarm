use bevy::prelude::*;

use super::constants::DEFAULT_DRONE_COUNT;

/// Desired number of drones in the world. The respawn system observes
/// changes here and rebuilds the swarm accordingly.
#[derive(Resource)]
pub struct DroneSpawnConfig {
    pub target_count: u32,
}

impl Default for DroneSpawnConfig {
    fn default() -> Self {
        Self {
            target_count: DEFAULT_DRONE_COUNT,
        }
    }
}
