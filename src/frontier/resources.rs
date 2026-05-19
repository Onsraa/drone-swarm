use bevy::prelude::*;

/// World-space centers of cells currently sitting on the frontier
/// (Unknown cells adjacent to a Free cell in the merged global map).
/// Refreshed by `compute_frontiers` on a 1 Hz cadence; assignment to
/// drones reads this list every frame.
#[derive(Resource, Default)]
pub struct FrontierCandidates {
    pub cells: Vec<Vec3>,
}
