pub mod cluster;
pub mod components;
pub mod constants;
pub mod planner;
pub mod resources;
pub mod role;
pub mod scoring;
pub mod steering;
pub mod supervisor;
pub mod systems;

use bevy::prelude::*;

pub use components::{FrontierTarget, MovementHealth, Path, Trail};
pub use resources::{FrontierClusters, PlannerGrid};
pub use role::{Role, RoleParams};
pub use supervisor::{LastRoleChange, SupervisorTimer};

use crate::physics::PhysicsSet;
use systems::{apply_role_steering, draw_trail_gizmos, sample_trails};

pub struct ExplorationPlugin;

impl Plugin for ExplorationPlugin {
    fn build(&self, app: &mut App) {
        // FrontierClusters + PlannerGrid resources kept alive for now
        // even though their producer systems are unwired — they exist
        // as cosmetic targets for the side-panel HUD and as scaffolding
        // for the upcoming Phase 4 anchor work. Phase 6 of the
        // foraging-colony plan deletes them.
        app.init_resource::<FrontierClusters>()
            .init_resource::<PlannerGrid>()
            .insert_resource(SupervisorTimer::new())
            .add_systems(Update, supervisor::supervisor_tick)
            .add_systems(
                Update,
                apply_role_steering.before(PhysicsSet::Control),
            )
            .add_systems(Update, (sample_trails, draw_trail_gizmos));
    }
}
