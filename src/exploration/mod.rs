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

pub use components::{FrontierTarget, MovementHealth, Path};
pub use resources::{CoarseCell, FrontierCluster, FrontierClusters, PlannerGrid};
pub use role::{Role, RoleParams};
pub use supervisor::{LastRoleChange, SupervisorTimer};
pub use systems::ReplanTimer;

use crate::physics::PhysicsSet;
use systems::{
    assign_targets, compute_frontier_clusters, enforce_anchor_hover, rebuild_planner_grid,
    reactive_avoid, replan_paths, steer_along_path, stuck_recovery, update_movement_health,
};

pub struct ExplorationPlugin;

impl Plugin for ExplorationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FrontierClusters>()
            .init_resource::<PlannerGrid>()
            .insert_resource(SupervisorTimer::new())
            .add_systems(Update, supervisor::supervisor_tick)
            .add_systems(
                Update,
                (
                    rebuild_planner_grid,
                    compute_frontier_clusters,
                    assign_targets,
                    replan_paths,
                    update_movement_health,
                    stuck_recovery,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (steer_along_path, reactive_avoid)
                    .after(replan_paths)
                    .after(crate::drone::wander)
                    .before(PhysicsSet::Control),
            )
            .add_systems(
                Update,
                enforce_anchor_hover
                    .after(steer_along_path)
                    .after(reactive_avoid)
                    .before(PhysicsSet::Control),
            );
    }
}
