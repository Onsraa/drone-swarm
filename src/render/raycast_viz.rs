//! Two gizmo-line systems showing per-role sensors:
//!
//! - `draw_detector_rays` — short-range collision-probe rays. READS
//!   the per-drone `DetectorHits` component populated by
//!   `sensors::update_detector_hits` (same data that drives
//!   `apply_role_steering`'s terrain repulsion). Drawn GREY.
//! - `draw_lidar_rays` — longer-range mapping cone. Per-role
//!   fibonacci cone cast against ground truth, drawn in the drone's
//!   color. Anchors have `rays_per_scan = 0` → no lidar lines.

use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::exploration::{Role, RoleParams};
use crate::lidar::sampling::{fibonacci_cone, SCOUT_LIDAR_TILT_DOWN_RAD};
use crate::sensors::DetectorHits;
use crate::ui::UiState;
use crate::world::{raycast_bvh, WorldBvh, WorldConfig};

/// Per-role mapping-lidar viz dirs (BODY frame, forward = -Z). Smaller
/// ray count than the GPU mapping lidar so the gizmo overlay stays
/// readable. Scout's set is tilted down by
/// `SCOUT_LIDAR_TILT_DOWN_RAD` to match the mapping cone.
#[derive(Resource)]
pub struct LidarVizRays {
    pub scout: Vec<Vec3>,
    pub mapper: Vec<Vec3>,
    pub anchor: Vec<Vec3>,
}

impl FromWorld for LidarVizRays {
    fn from_world(_world: &mut World) -> Self {
        let scout_cone = RoleParams::for_role(Role::Scout)
            .cone_half_angle_deg
            .to_radians();
        let mut scout_dirs = fibonacci_cone(12, scout_cone);
        let tilt = Quat::from_rotation_x(SCOUT_LIDAR_TILT_DOWN_RAD);
        for d in scout_dirs.iter_mut() {
            *d = (tilt * *d).normalize_or_zero();
        }

        let mapper_dirs = fibonacci_cone(
            48,
            RoleParams::for_role(Role::Mapper)
                .cone_half_angle_deg
                .to_radians(),
        );
        // Anchor has rays_per_scan = 0 in role params (no mapping).
        // Empty viz set so `draw_lidar_rays` draws nothing for anchors.
        let anchor_dirs: Vec<Vec3> = Vec::new();
        Self {
            scout: scout_dirs,
            mapper: mapper_dirs,
            anchor: anchor_dirs,
        }
    }
}

pub struct RaycastVizPlugin;

impl Plugin for RaycastVizPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LidarVizRays>()
            .add_systems(Update, (draw_detector_rays, draw_lidar_rays));
    }
}

pub fn draw_detector_rays(
    ui_state: Res<UiState>,
    mut gizmos: Gizmos,
    drones: Query<(&Transform, &DetectorHits), With<Drone>>,
) {
    if !ui_state.show_detector_rays {
        return;
    }
    let detector_color = Color::linear_rgba(0.6, 0.62, 0.65, 0.55);
    for (transform, det) in &drones {
        let origin = transform.translation;
        for ep in det.endpoints.iter() {
            gizmos.line(origin, *ep, detector_color);
        }
    }
}

pub fn draw_lidar_rays(
    ui_state: Res<UiState>,
    bvh: Option<Res<WorldBvh>>,
    config: Option<Res<WorldConfig>>,
    rays: Res<LidarVizRays>,
    mut gizmos: Gizmos,
    drones: Query<(&Transform, &Role, &DroneColor), With<Drone>>,
) {
    if !ui_state.show_lidar_rays {
        return;
    }
    let (Some(bvh), Some(config)) = (bvh, config) else {
        return;
    };
    let voxel = config.voxel_size;

    for (transform, role, color) in &drones {
        let dirs = match role {
            Role::Scout => &rays.scout,
            Role::Mapper => &rays.mapper,
            Role::Anchor => &rays.anchor,
        };
        if dirs.is_empty() {
            continue;
        }
        let max_range = RoleParams::for_role(*role).max_range_cells as f32 * voxel;
        let origin = transform.translation;
        let rot = transform.rotation;

        let base = color.0.to_linear();
        let line_color = Color::linear_rgba(base.red, base.green, base.blue, 0.40);

        for d in dirs.iter() {
            let dir_world = rot * (*d);
            let (endpoint, _hit) = raycast_bvh(&bvh, origin, dir_world, max_range);
            gizmos.line(origin, endpoint, line_color);
        }
    }
}
