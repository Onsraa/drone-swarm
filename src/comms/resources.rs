use bevy::prelude::*;

use super::constants::DEFAULT_COMMS_RANGE_M;

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
/// each within `CommsSettings.range_m`. `connected_count` + `base_pos`
/// are for the side-panel readout + gizmo rendering.
#[derive(Resource, Clone, Copy, Debug, Default)]
pub struct CommsState {
    pub connected_mask: [u32; 2],
    pub connected_count: usize,
    pub total_count: usize,
    pub base_pos: Vec3,
}
