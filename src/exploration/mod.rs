pub mod anchor_planner;
pub mod components;
pub mod constants;
pub mod frontier;
mod resources;
pub mod role;
pub mod steering;
pub mod supervisor;
pub mod systems;

use bevy::prelude::*;

pub use anchor_planner::AnchorPlannerPlugin;
pub use components::{GhostMemory, ScoutGradientEma, Trail};
pub use frontier::FrontierPlugin;
pub use role::{Role, RoleParams};
pub use supervisor::{LastRoleChange, SupervisorTimer};

use crate::physics::PhysicsSet;
use systems::{
    apply_role_steering, draw_anchor_gizmos, draw_frontier_gizmos, draw_trail_gizmos, sample_trails,
};

pub struct ExplorationPlugin;

impl Plugin for ExplorationPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(SupervisorTimer::new())
            .add_plugins((FrontierPlugin, AnchorPlannerPlugin))
            .add_systems(Update, supervisor::supervisor_tick)
            .add_systems(Update, apply_role_steering.before(PhysicsSet::Control))
            .add_systems(
                Update,
                (
                    sample_trails,
                    draw_trail_gizmos,
                    draw_frontier_gizmos,
                    draw_anchor_gizmos,
                ),
            );
    }
}
