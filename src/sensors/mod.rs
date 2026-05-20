//! Per-role short-range collision-probe rays ("detectors"). Shared
//! between the visualization layer (grey gizmo lines per ray) and the
//! behavior layer (terrain repulsion in `apply_role_steering`). Same
//! ray set, same ground-truth DDA cast, one frame: what you see is
//! what physically pushes the drone.
//!
//! The mapping lidar is separate — it lives on the GPU and writes to
//! occupancy SSBOs. Detectors are pure CPU + ground-truth-aware
//! (they cheat a bit by reading the real map directly, but every
//! real-drone short-range ToF / ultrasonic sensor "cheats" the same
//! way: it just measures distance to the actual wall).

use bevy::prelude::*;

use crate::drone::Drone;
use crate::exploration::Role;
use crate::world::{GroundTruthMap, WorldConfig};

pub struct SensorsPlugin;

impl Plugin for SensorsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DetectorRays>()
            .add_systems(Update, update_detector_hits);
    }
}

/// Per-role detector ray dirs in BODY frame (forward = -Z). Short-
/// range collision probes; built once at startup so every drone in a
/// given role shares the same body-frame ray set.
#[derive(Resource)]
pub struct DetectorRays {
    pub scout: Vec<Vec3>,
    pub mapper: Vec<Vec3>,
    pub anchor: Vec<Vec3>,
}

impl FromWorld for DetectorRays {
    fn from_world(_world: &mut World) -> Self {
        // Scout: 10 rays — 6 horizontal cardinal+diagonal + 2 forward-
        // down "sniffer" probes + 2 vertical.
        let scout: Vec<Vec3> = {
            let mut v = Vec::with_capacity(10);
            for i in 0..6 {
                let a = i as f32 * std::f32::consts::TAU / 6.0;
                v.push(Vec3::new(a.sin(), 0.0, -a.cos()).normalize());
            }
            v.push(Vec3::new(0.0, -1.0, -1.0).normalize());
            v.push(Vec3::new(0.3, -0.7, -0.65).normalize());
            v.push(Vec3::Y);
            v.push(Vec3::NEG_Y);
            v
        };
        // Mapper: 4 cardinal + down. Moves slow, simpler probes.
        let mapper: Vec<Vec3> = vec![
            Vec3::NEG_Z,
            Vec3::Z,
            Vec3::X,
            Vec3::NEG_X,
            Vec3::NEG_Y,
        ];
        // Anchor: 6 face cardinals + 2 forward diagonals.
        let anchor: Vec<Vec3> = vec![
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::Z,
            Vec3::NEG_Z,
            Vec3::new(0.7, 0.0, -0.7).normalize(),
            Vec3::new(-0.7, 0.0, -0.7).normalize(),
        ];
        Self { scout, mapper, anchor }
    }
}

/// Per-role detector range in meters. Short — drones don't react to
/// faraway walls, only imminent ones.
pub fn detector_range_for(role: Role) -> f32 {
    match role {
        Role::Scout => 6.0,
        Role::Mapper => 4.0,
        Role::Anchor => 5.0,
    }
}

/// Per-drone detector ray results. `endpoints[i]` is ray `i`'s
/// terminus — either the first ground-truth Occupied cell along the
/// ray (`is_hit[i] = true`) or `origin + dir * max_range` (miss).
///
/// The behavior layer (`apply_role_steering`) iterates only the hit
/// entries as terrain obstacles. The viz layer
/// (`render::raycast_viz`) draws all entries as gizmo lines.
#[derive(Component, Default, Debug)]
pub struct DetectorHits {
    pub endpoints: Vec<Vec3>,
    pub is_hit: Vec<bool>,
}

pub fn dirs_for_role<'a>(role: Role, rays: &'a DetectorRays) -> &'a [Vec3] {
    match role {
        Role::Scout => &rays.scout,
        Role::Mapper => &rays.mapper,
        Role::Anchor => &rays.anchor,
    }
}

fn update_detector_hits(
    ground: Option<Res<GroundTruthMap>>,
    config: Option<Res<WorldConfig>>,
    rays: Res<DetectorRays>,
    mut q: Query<(&Transform, &Role, &mut DetectorHits), With<Drone>>,
) {
    let (Some(ground), Some(config)) = (ground, config) else {
        return;
    };
    let voxel = config.voxel_size;
    for (transform, role, mut hits) in &mut q {
        let dirs = dirs_for_role(*role, &rays);
        let max_range = detector_range_for(*role);
        let origin = transform.translation;
        let rot = transform.rotation;
        hits.endpoints.clear();
        hits.is_hit.clear();
        hits.endpoints.reserve(dirs.len());
        hits.is_hit.reserve(dirs.len());
        for d in dirs {
            let dir_world = rot * (*d);
            let (endpoint, hit) =
                raycast_dda(origin, dir_world, max_range, &ground, voxel);
            hits.endpoints.push(endpoint);
            hits.is_hit.push(hit);
        }
    }
}

/// Half-voxel-step DDA. Returns `(endpoint, hit)` — endpoint is the
/// first Occupied cell along the ray or `origin + dir * max_dist` on
/// miss; hit is whether we actually struck.
pub fn raycast_dda(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    ground: &GroundTruthMap,
    voxel: f32,
) -> (Vec3, bool) {
    let step = voxel * 0.5;
    let n_steps = (max_dist / step) as usize + 1;
    for i in 1..=n_steps {
        let t = i as f32 * step;
        if t > max_dist {
            return (origin + dir * max_dist, false);
        }
        let p = origin + dir * t;
        let cell = (p / voxel).floor().as_ivec3();
        if ground.get(cell) {
            return (p, true);
        }
    }
    (origin + dir * max_dist, false)
}
