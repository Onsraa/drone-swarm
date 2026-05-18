mod centering;
mod components;
mod constants;
mod motion;
mod spawn;

use bevy::prelude::*;

pub use components::Drone;

use centering::recenter_visuals;
use motion::{integrate_motion, random_walk};
use spawn::spawn_drone;

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_drone).add_systems(
            Update,
            (recenter_visuals, random_walk, integrate_motion).chain(),
        );
    }
}
