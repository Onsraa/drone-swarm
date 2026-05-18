use bevy::prelude::*;

use crate::drone::Drone;
use crate::lidar::LastScanRays;

use super::constants::RAY_GIZMO_COLOR;
use super::resources::UiState;

pub fn draw_ray_gizmos(
    state: Res<UiState>,
    mut gizmos: Gizmos,
    rays_q: Query<&LastScanRays, With<Drone>>,
) {
    if !state.show_rays {
        return;
    }
    for rays in &rays_q {
        for (start, end) in &rays.0 {
            gizmos.line(*start, *end, RAY_GIZMO_COLOR);
        }
    }
}
