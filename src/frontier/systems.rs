use bevy::prelude::*;

use crate::drone::{Drone, CRUISE_SPEED_MPS};
use crate::lidar::gpu::GpuGlobalOccupancyMirror;
use crate::physics::DesiredVelocity;
use crate::world::WorldConfig;

use super::components::FrontierTarget;
use super::constants::{
    FRONTIER_LERP_RATE, FRONTIER_REACHED_DIST, FRONTIER_REFRESH_SECS, MAX_FRONTIER_CANDIDATES,
};
use super::resources::FrontierCandidates;

/// Decode the 2-bit-per-cell global occupancy bitset and emit one
/// candidate world position for each Unknown cell sitting next to a Free
/// cell in the 6-neighborhood. Skipping all-zero words keeps cold-start
/// frames cheap; the soft cap bounds work once exploration is well underway.
pub fn compute_frontiers(
    time: Res<Time>,
    mut timer: Local<f32>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    config: Res<WorldConfig>,
    mut candidates: ResMut<FrontierCandidates>,
) {
    *timer += time.delta_secs();
    if *timer < FRONTIER_REFRESH_SECS {
        return;
    }
    *timer = 0.0;

    if mirror.data.is_empty() {
        return;
    }

    let dims = config.size;
    let voxel_size = config.voxel_size;
    let total_cells = (dims.x * dims.y * dims.z) as usize;
    let dx = dims.x as i32;
    let dy = dims.y as i32;
    let dz = dims.z as i32;
    let plane = dims.x * dims.y;
    let data = &mirror.data;

    let read_state = |cell: u32| -> u32 {
        let w = (cell / 16) as usize;
        if w >= data.len() {
            return 0;
        }
        let b = (cell % 16) * 2;
        (data[w] >> b) & 0b11
    };

    let mut out: Vec<Vec3> = Vec::new();
    let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();

    'outer: for w in 0..data.len() {
        let word = data[w];
        if word == 0 {
            continue;
        }
        for slot in 0..16u32 {
            let state = (word >> (slot * 2)) & 0b11;
            if state != 1 {
                continue;
            }
            let cell = (w * 16) as u32 + slot;
            if cell as usize >= total_cells {
                break;
            }
            let z = (cell / plane) as i32;
            let rem = cell % plane;
            let y = (rem / dims.x) as i32;
            let x = (rem % dims.x) as i32;

            let neighbors = [
                (x - 1, y, z),
                (x + 1, y, z),
                (x, y - 1, z),
                (x, y + 1, z),
                (x, y, z - 1),
                (x, y, z + 1),
            ];
            for (nx, ny, nz) in neighbors {
                if nx < 0 || ny < 0 || nz < 0 || nx >= dx || ny >= dy || nz >= dz {
                    continue;
                }
                let nidx = (nx as u32) + (ny as u32) * dims.x + (nz as u32) * plane;
                if read_state(nidx) != 0 {
                    continue;
                }
                if seen.insert(nidx) {
                    let half = voxel_size * 0.5;
                    let pos = Vec3::new(nx as f32, ny as f32, nz as f32) * voxel_size
                        + Vec3::splat(half);
                    out.push(pos);
                    if out.len() >= MAX_FRONTIER_CANDIDATES {
                        break 'outer;
                    }
                }
            }
        }
    }

    candidates.cells = out;
}

/// Assign each drone the nearest unclaimed frontier candidate. A drone
/// keeps its current target until it gets within `FRONTIER_REACHED_DIST`
/// of it, at which point the next call picks a fresh one.
pub fn assign_frontier_targets(
    candidates: Res<FrontierCandidates>,
    mut q: Query<(&Transform, &mut FrontierTarget), With<Drone>>,
) {
    if candidates.cells.is_empty() {
        return;
    }
    for (transform, mut target) in &mut q {
        let pos = transform.translation;
        let need_new = match target.0 {
            None => true,
            Some(t) => pos.distance(t) < FRONTIER_REACHED_DIST,
        };
        if !need_new {
            continue;
        }
        let mut best: Option<(f32, Vec3)> = None;
        for &c in &candidates.cells {
            let d = pos.distance_squared(c);
            if best.map(|(bd, _)| d < bd).unwrap_or(true) {
                best = Some((d, c));
            }
        }
        target.0 = best.map(|(_, p)| p);
    }
}

/// Override DesiredVelocity with a vector pointing at the drone's
/// FrontierTarget. Runs after `wander`, so the random fallback drives
/// drones only while no target is assigned (cold start or empty
/// candidates).
pub fn seek_frontier(
    time: Res<Time>,
    mut q: Query<(&Transform, &FrontierTarget, &mut DesiredVelocity), With<Drone>>,
) {
    let dt = time.delta_secs();
    for (transform, target, mut desired) in &mut q {
        let Some(t) = target.0 else { continue };
        let to_target = t - transform.translation;
        let dist = to_target.length();
        if dist < 1e-3 {
            continue;
        }
        let target_vel = (to_target / dist) * CRUISE_SPEED_MPS;
        let alpha = (FRONTIER_LERP_RATE * dt).min(1.0);
        desired.0 = desired.0.lerp(target_vel, alpha);
    }
}
