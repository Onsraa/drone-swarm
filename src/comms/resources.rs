use bevy::prelude::*;

use crate::drone::MAX_DRONE_COUNT;

use super::constants::DEFAULT_COMMS_RANGE_M;

/// Sentinel parent ID meaning "this drone connects directly to base".
pub const PARENT_BASE: i16 = -1;
/// Sentinel parent ID meaning "this drone is disconnected".
pub const PARENT_NONE: i16 = -2;

/// Runtime-tunable comms parameters wired to egui sliders.
#[derive(Resource, Clone, Copy, Debug)]
pub struct CommsSettings {
    pub enabled: bool,
    pub range_m: f32,
    pub show_links: bool,
}

impl Default for CommsSettings {
    fn default() -> Self {
        Self {
            // Knowledge-to-central gating is core to the simulation
            // narrative (drones can only contribute to the central map
            // when physically chain-connected back to the base), so
            // this defaults ON. The slider still allows toggling for
            // A/B testing.
            enabled: true,
            range_m: DEFAULT_COMMS_RANGE_M,
            show_links: true,
        }
    }
}

/// Output of the per-frame connectivity solve. `connected_mask` is the
/// 64-bit per-drone mask uploaded into `BuildLocalParams`; bit `i` set
/// means drone id `i` is reachable from the base via a chain of peers
/// each within `CommsSettings.range_m`. `bfs_parent[i]` records the
/// drone id that first reached drone `i` during the BFS, or
/// `PARENT_BASE` if drone `i` is directly within range of base, or
/// `PARENT_NONE` if disconnected. The anchor planner walks this tree
/// to find stretched edges where adding a relay would extend the
/// reachable swarm.
#[derive(Resource, Clone, Copy, Debug)]
pub struct CommsState {
    pub connected_mask: [u32; 2],
    pub connected_count: usize,
    pub total_count: usize,
    pub base_pos: Vec3,
    pub bfs_parent: [i16; MAX_DRONE_COUNT as usize],
}

impl Default for CommsState {
    fn default() -> Self {
        Self {
            connected_mask: [0u32; 2],
            connected_count: 0,
            total_count: 0,
            base_pos: Vec3::ZERO,
            bfs_parent: [PARENT_NONE; MAX_DRONE_COUNT as usize],
        }
    }
}
