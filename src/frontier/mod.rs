mod components;
mod constants;
mod resources;
mod systems;

use bevy::prelude::*;

pub use components::FrontierTarget;
pub use resources::FrontierCandidates;

use crate::drone::wander;
use crate::physics::PhysicsSet;

use systems::{assign_frontier_targets, compute_frontiers, seek_frontier};

/// Frontier-based exploration: rescans the merged global occupancy
/// bitset on a slow timer, hands each drone the nearest Unknown-adjacent-
/// to-Free cell as its DesiredVelocity target, and falls back to random
/// wander whenever no candidates are available.
pub struct FrontierPlugin;

impl Plugin for FrontierPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FrontierCandidates>().add_systems(
            Update,
            (
                compute_frontiers,
                assign_frontier_targets.after(compute_frontiers),
                seek_frontier
                    .after(wander)
                    .after(assign_frontier_targets)
                    .before(PhysicsSet::Control),
            ),
        );
    }
}
