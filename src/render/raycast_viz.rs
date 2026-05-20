//! Two gizmo-line systems showing per-role sensors:
//!
//! - `draw_detector_rays` — short-range collision-probe rays, drawn
//!   GREY. Per-role direction set + range encodes the safety bubble.
//! - `draw_lidar_rays` — longer-range mapping cone, drawn in the
//!   drone's color. Per-role cone shape + ray count.
//!
//! Both raycast against `GroundTruthMap` via a half-voxel-step DDA.
//! Anchors have NO lidar rays (`rays_per_scan = 0` in role params).

use std::f32::consts::FRAC_PI_4;

use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::exploration::{Role, RoleParams};
use crate::lidar::sampling::{fibonacci_cone, SCOUT_LIDAR_TILT_DOWN_RAD};
use crate::ui::UiState;
use crate::world::{GroundTruthMap, WorldConfig};

/// Per-role mapping-lidar viz dirs (BODY frame, forward = -Z). Smaller
/// ray count than the GPU mapping lidar so the gizmo overlay stays
/// readable. Scout's set is also tilted down by
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

/// Per-role detector rays (BODY frame). Short-range collision-probe
/// directions a drone uses to decide which way is safe to move.
/// Distinct from the mapping lidar — detectors are about NOT crashing.
#[derive(Resource)]
pub struct DetectorRays {
    pub scout: Vec<Vec3>,
    pub mapper: Vec<Vec3>,
    pub anchor: Vec<Vec3>,
}

impl FromWorld for DetectorRays {
    fn from_world(_world: &mut World) -> Self {
        // Scout: dense 10-ray fan (6 horizontal + 2 forward-down +
        // 2 vertical) so it can move fast and still steer around
        // obstacles in any direction.
        let scout: Vec<Vec3> = {
            let mut v = Vec::with_capacity(10);
            // 6 horizontal (every 60°).
            for i in 0..6 {
                let a = i as f32 * std::f32::consts::TAU / 6.0;
                v.push(Vec3::new(a.sin(), 0.0, -a.cos()).normalize());
            }
            // 2 forward-down probes (45° down to either side).
            v.push(Vec3::new(0.0, -1.0, -1.0).normalize());
            v.push(Vec3::new(0.3, -0.7, -0.65).normalize());
            // 2 vertical (up + down).
            v.push(Vec3::Y);
            v.push(Vec3::NEG_Y);
            v
        };

        // Mapper: minimal 4-cardinal + down. It moves slow so safety
        // can be sparser.
        let mapper: Vec<Vec3> = vec![
            Vec3::NEG_Z, // forward
            Vec3::Z,     // back
            Vec3::X,     // right
            Vec3::NEG_X, // left
            Vec3::NEG_Y, // down
        ];

        // Anchor: 6 face-cardinal (no lidar, this is its only sensor).
        let anchor: Vec<Vec3> = vec![
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::Z,
            Vec3::NEG_Z,
            // Plus two diagonals for slightly better coverage when
            // anchor needs to dodge a fast-moving scout.
            Vec3::new(0.7, 0.0, -0.7).normalize(),
            Vec3::new(-0.7, 0.0, -0.7).normalize(),
        ];

        // Suppress the unused FRAC_PI_4 warning by binding it (kept
        // available in case later tweaks want a clean 45° literal).
        let _ = FRAC_PI_4;

        Self { scout, mapper, anchor }
    }
}

/// Per-role detector range in meters. Short enough that the gizmo
/// lines don't reach to terrain features at swarm scale.
pub fn detector_range_for(role: Role) -> f32 {
    match role {
        Role::Scout => 6.0,
        Role::Mapper => 4.0,
        Role::Anchor => 5.0,
    }
}

pub struct RaycastVizPlugin;

impl Plugin for RaycastVizPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LidarVizRays>()
            .init_resource::<DetectorRays>()
            .add_systems(Update, (draw_detector_rays, draw_lidar_rays));
    }
}

pub fn draw_detector_rays(
    ui_state: Res<UiState>,
    ground: Option<Res<GroundTruthMap>>,
    config: Option<Res<WorldConfig>>,
    rays: Res<DetectorRays>,
    mut gizmos: Gizmos,
    drones: Query<(&Transform, &Role), With<Drone>>,
) {
    if !ui_state.show_detector_rays {
        return;
    }
    let (Some(ground), Some(config)) = (ground, config) else {
        return;
    };
    let voxel = config.voxel_size;
    let detector_color = Color::linear_rgba(0.6, 0.62, 0.65, 0.55);

    for (transform, role) in &drones {
        let dirs = match role {
            Role::Scout => &rays.scout,
            Role::Mapper => &rays.mapper,
            Role::Anchor => &rays.anchor,
        };
        let max_range = detector_range_for(*role);
        let origin = transform.translation;
        let rot = transform.rotation;
        for d in dirs.iter() {
            let dir_world = rot * (*d);
            let hit = raycast_dda(origin, dir_world, max_range, &ground, voxel);
            gizmos.line(origin, hit, detector_color);
        }
    }
}

pub fn draw_lidar_rays(
    ui_state: Res<UiState>,
    ground: Option<Res<GroundTruthMap>>,
    config: Option<Res<WorldConfig>>,
    rays: Res<LidarVizRays>,
    mut gizmos: Gizmos,
    drones: Query<(&Transform, &Role, &DroneColor), With<Drone>>,
) {
    if !ui_state.show_lidar_rays {
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
