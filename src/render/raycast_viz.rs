//! Gizmo-line raycast visualization. Each drone fires a small set of
//! visualization rays at its current orientation, against the
//! GROUND-TRUTH map (CPU side), and draws a line from the drone to
//! the first hit or to its max range. Separate from the GPU lidar
//! pipeline — this is a "what would the lidar see right now" overlay
//! used to make per-role sensor shape visible.
//!
//! Ray density is intentionally lower than the GPU mapping lidar so
//! the gizmo lines stay legible at swarm scale.

use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::exploration::{Role, RoleParams};
use crate::lidar::sampling::fibonacci_cone;
use crate::ui::UiState;
use crate::world::{GroundTruthMap, WorldConfig};

/// Per-role visualization ray dirs in BODY frame (forward = -Z). Built
/// once at startup. Different from the GPU mapping rays — fewer, since
/// these get drawn as gizmo lines.
#[derive(Resource)]
pub struct VizRayDirs {
    pub scout: Vec<Vec3>,
    pub mapper: Vec<Vec3>,
    pub anchor: Vec<Vec3>,
}

impl FromWorld for VizRayDirs {
    fn from_world(_world: &mut World) -> Self {
        Self {
            // Scout: 12 rays in its narrow forward cone — enough to
            // see the shape, not so many they overlap visually.
            scout: fibonacci_cone(
                12,
                RoleParams::for_role(Role::Scout)
                    .cone_half_angle_deg
                    .to_radians(),
            ),
            // Mapper: dense sphere so the 360° spray reads.
            mapper: fibonacci_cone(
                48,
                RoleParams::for_role(Role::Mapper)
                    .cone_half_angle_deg
                    .to_radians(),
            ),
            // Anchor: medium hemispheric coverage.
            anchor: fibonacci_cone(
                24,
                RoleParams::for_role(Role::Anchor)
                    .cone_half_angle_deg
                    .to_radians(),
            ),
        }
    }
}

pub struct RaycastVizPlugin;

impl Plugin for RaycastVizPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VizRayDirs>()
            .add_systems(Update, draw_lidar_rays);
    }
}

pub fn draw_lidar_rays(
    ui_state: Res<UiState>,
    ground: Option<Res<GroundTruthMap>>,
    config: Option<Res<WorldConfig>>,
    rays: Res<VizRayDirs>,
    mut gizmos: Gizmos,
    drones: Query<(&Transform, &Role, &DroneColor), With<Drone>>,
) {
    if !ui_state.show_raycast_lines {
        return;
    }
    let (Some(ground), Some(config)) = (ground, config) else {
        return;
    };
    let voxel = config.voxel_size;

    for (transform, role, color) in &drones {
        let dirs = match role {
            Role::Scout => &rays.scout,
            Role::Mapper => &rays.mapper,
            Role::Anchor => &rays.anchor,
        };
        let max_range = RoleParams::for_role(*role).max_range_cells as f32 * voxel;
        let origin = transform.translation;
        let rot = transform.rotation;

        let base = color.0.to_linear();
        let line_color = Color::linear_rgba(base.red, base.green, base.blue, 0.35);

        for d in dirs.iter() {
            let dir_world = rot * (*d);
            let hit = raycast_dda(origin, dir_world, max_range, &ground, voxel);
            gizmos.line(origin, hit, line_color);
        }
    }
}

/// Step-and-check DDA against the ground-truth grid. Step is
/// `voxel * 0.5` so we don't skip over thin walls. Returns the hit
/// point in world coords, or `origin + dir * max_dist` if no hit.
fn raycast_dda(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    ground: &GroundTruthMap,
    voxel: f32,
) -> Vec3 {
    let step = voxel * 0.5;
    let n_steps = (max_dist / step) as usize + 1;
    for i in 1..=n_steps {
        let t = i as f32 * step;
        if t > max_dist {
            return origin + dir * max_dist;
        }
        let p = origin + dir * t;
        let cell = (p / voxel).floor().as_ivec3();
        if ground.get(cell) {
            return p;
        }
    }
    origin + dir * max_dist
}
