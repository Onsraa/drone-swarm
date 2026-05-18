mod components;
mod constants;
mod controller;
mod integrator;

use bevy::prelude::*;

pub use components::{DesiredAttitude, DesiredVelocity, LinearVelocity, ThrustState};

use controller::quadcopter_controller;
use integrator::integrate_forces;

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum PhysicsSet {
    Control,
    Integrate,
}

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(Update, (PhysicsSet::Control, PhysicsSet::Integrate).chain())
            .add_systems(Update, quadcopter_controller.in_set(PhysicsSet::Control))
            .add_systems(Update, integrate_forces.in_set(PhysicsSet::Integrate));
    }
}
