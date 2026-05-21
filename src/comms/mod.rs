mod constants;
mod resources;
mod systems;

use bevy::prelude::*;

pub use constants::{MAX_COMMS_RANGE_M, MIN_COMMS_RANGE_M};
pub use resources::{CommsSettings, CommsState, PARENT_BASE, PARENT_NONE};

use systems::{compute_connectivity, draw_comms_gizmos};

/// Real-world radio constraint on the merged central map. By default
/// every drone contributes; when `CommsSettings.enabled` is set, only
/// drones within `range_m` of the base or transitively reachable via
/// peers within range stay in the contribution mask. Output is a 64-bit
/// `CommsState.connected_mask` consumed by `merge_global.wgsl`.
pub struct CommsPlugin;

impl Plugin for CommsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CommsSettings>()
            .init_resource::<CommsState>()
            .add_systems(Update, (compute_connectivity, draw_comms_gizmos).chain());
    }
}
