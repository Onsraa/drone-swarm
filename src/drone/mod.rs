mod centering;
mod components;
mod constants;
mod spawn;
mod wander;

use bevy::prelude::*;

pub use components::{Drone, DroneColor};

use crate::physics::PhysicsSet;

use centering::recenter_visuals;
use spawn::spawn_drones;
use wander::wander;

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_drones).add_systems(
            Update,
            (recenter_visuals, wander.before(PhysicsSet::Control)),
        );
    }
}
