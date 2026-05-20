mod components;
mod constants;
mod tracker;
mod visuals;

use bevy::prelude::*;

pub use components::{DesiredVelocity, LinearVelocity};
pub use visuals::PrevLinvel;

use tracker::track_velocity;
use visuals::update_drone_visuals;

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum PhysicsSet {
    /// Steering systems write `DesiredVelocity` before this set.
    Control,
    /// `track_velocity` lerps `LinearVelocity` toward `DesiredVelocity`
    /// and integrates the transform.
    Integrate,
}

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(Update, (PhysicsSet::Control, PhysicsSet::Integrate).chain())
            .add_systems(Update, track_velocity.in_set(PhysicsSet::Integrate))
            .add_systems(Update, update_drone_visuals.after(PhysicsSet::Integrate));
    }
}
