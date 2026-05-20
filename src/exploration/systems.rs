// src/exploration/systems.rs
use bevy::platform::collections::HashSet;
use bevy::prelude::*;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneId};
use crate::lidar::gpu::GpuGlobalOccupancyMirror;
use crate::physics::{DesiredVelocity, LinearVelocity};

use super::cluster::build_clusters;
use super::components::{FrontierTarget, MovementHealth, Path};
use super::constants::{
    AVOID_RADIUS_M, FRONTIER_REACHED_DIST, MAX_FRONTIER_CANDIDATES, PATH_FOLLOW_LERP_RATE,
    PLANNER_DOWNSAMPLE, SCORE_UPGRADE_RATIO, STUCK_ESCALATION_WINDOW_SECS, STUCK_SECS,
    STUCK_VEL_MPS,
};

/// Cap how many A* calls run per frame to amortise the cost of large
/// target-changed events (e.g. first frame after spawn, or right after
/// a map swap when every drone simultaneously gets a fresh target).
/// Remaining drones get their replan on the next frame.
const MAX_REPLANS_PER_FRAME: usize = 4;
use super::planner::{plan, PlanScratch};
use super::resources::{FrontierClusters, PlannerGrid};
use super::steering::{pure_pursuit, reactive_force};
use super::scoring::{crowding_for, score, ScoringWeights};
use super::role::{Role, RoleParams};
use rand::RngExt;

pub fn assign_targets(
    clusters: Res<FrontierClusters>,
    comms: Res<CommsState>,
    mut peers_buf: Local<Vec<(u32, Vec3, Option<u32>)>>,
    mut visible_buf: Local<Vec<(Vec3, Option<u32>)>>,
    mut scored_buf: Local<Vec<(f32, u32, Vec3)>>,
    mut q: Query<(&DroneId, &Transform, &Role, &mut FrontierTarget), With<Drone>>,
) {
    if clusters.entries.is_empty() {
        return;
    }
    // Snapshot peer positions + targets keyed by id for crowding lookups.
    // Role is not needed for peer crowding — only position + cluster_id matter.
    peers_buf.clear();
    peers_buf.extend(
        q.iter()
            .map(|(id, t, _role, ft)| (id.0, t.translation, ft.cluster_id)),
    );

    for (id, transform, role, mut target) in &mut q {
        // Anchors hold position; supervisor assigns them — skip scoring.
        if *role == Role::Anchor {
            continue;
        }

        let role_params = RoleParams::for_role(*role);
        let weights = ScoringWeights {
            info: role_params.info_weight,
            distance: role_params.distance_weight,
            distance_bias: role_params.distance_bias,
            crowding: role_params.crowding_weight,
        };
        let drone_pos = transform.translation;
        // Filter peers to the comms cluster of the deciding drone.
        let half = (id.0 >= 32) as usize;
        let i_am_connected = (comms.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        visible_buf.clear();
        if i_am_connected {
            visible_buf.extend(peers_buf.iter().filter_map(|(pid, p, t)| {
                if *pid == id.0 {
                    return None;
                }
                let h = (*pid >= 32) as usize;
                if (comms.connected_mask[h] >> (pid % 32)) & 1 == 1 {
                    Some((*p, *t))
                } else {
                    None
                }
            }));
        }

        // Score all clusters once.
        scored_buf.clear();
        scored_buf.extend(clusters.entries.iter().map(|c| {
            let crowding = crowding_for(c, &visible_buf, 0.5);
            (score(c, drone_pos, crowding, &weights), c.id, c.centroid)
        }));

        let best = scored_buf
            .iter()
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let Some(&(best_score, best_id, best_centroid)) = best else {
            target.pos = None;
            target.cluster_id = None;
            continue;
        };

        // Stickiness: keep current unless reached, vanished, or upgrade by 1.5x.
        let keep = match target.cluster_id {
            None => false,
            Some(cur_id) => {
                let cur_alive = clusters.entries.iter().any(|c| c.id == cur_id);
                if !cur_alive {
                    false
                } else if let Some(cur_pos) = target.pos {
                    if cur_pos.distance(drone_pos) < FRONTIER_REACHED_DIST {
                        false
                    } else {
                        let cur_score = scored_buf
                            .iter()
                            .find(|(_, id, _)| *id == cur_id)
                            .map(|s| s.0)
                            .unwrap_or(0.0);
                        best_score <= cur_score * SCORE_UPGRADE_RATIO
                    }
                } else {
                    false
                }
            }
        };
        if !keep {
            target.cluster_id = Some(best_id);
            target.pos = Some(best_centroid);
        }
    }
}

/// Words processed per frame in the time-sliced 1 Hz scans. At 614K
/// words for a 640³ map this finishes a full scan in ~20 frames =
/// ~170 ms at 120 FPS. Below 0.5 ms per frame so the freeze is gone.
const SCAN_WORDS_PER_FRAME: usize = 32_768;

#[derive(Default)]
pub struct PlannerScanState {
    pub snapshot: Vec<u32>,
    pub dims: UVec3,
    pub voxel_size: f32,
    pub occ: Vec<u32>,
    pub free: Vec<u32>,
    pub coarse_dims: UVec3,
    pub word_cursor: usize,
    pub active: bool,
    pub cooldown_secs: f32,
}

/// Gap between planner-grid rebuilds. Without it the scan restarts the
/// instant it finishes (~160 ms at 120 FPS) → 6 finalisations per
/// second. A 0.5 s cooldown caps it at ~1.5 Hz which is plenty for A*
/// replans that fire on target-change anyway.
const PLANNER_SCAN_COOLDOWN_SECS: f32 = 0.5;

pub fn rebuild_planner_grid(
    time: Res<Time>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    mut state: Local<PlannerScanState>,
    mut grid: ResMut<PlannerGrid>,
) {
    if mirror.data.is_empty() {
        return;
    }

    // Start a fresh scan when the previous one finished.
    if !state.active {
        state.cooldown_secs -= time.delta_secs();
        if state.cooldown_secs > 0.0 {
            return;
        }
        let dims = world.size;
        let coarse_dims = UVec3::new(
            dims.x.div_ceil(PLANNER_DOWNSAMPLE),
            dims.y.div_ceil(PLANNER_DOWNSAMPLE),
            dims.z.div_ceil(PLANNER_DOWNSAMPLE),
        );
        let total = (coarse_dims.x * coarse_dims.y * coarse_dims.z) as usize;
        state.snapshot.clone_from(&mirror.data);
        state.dims = dims;
        state.voxel_size = world.voxel_size;
        state.coarse_dims = coarse_dims;
        state.occ.clear();
        state.occ.resize(total, 0);
        state.free.clear();
        state.free.resize(total, 0);
        state.word_cursor = 0;
        state.active = true;
    }

    let dims = state.dims;
    let coarse_dims = state.coarse_dims;
    let plane = dims.x * dims.y;
    let total_cells = dims.x * dims.y * dims.z;
    let downsample = PLANNER_DOWNSAMPLE;

    let end = (state.word_cursor + SCAN_WORDS_PER_FRAME).min(state.snapshot.len());
    for w_idx in state.word_cursor..end {
        let word = state.snapshot[w_idx];
        if word == 0 {
            continue;
        }
        let base_cell = (w_idx as u32) * 16;
        for slot in 0..16u32 {
            let cell = base_cell + slot;
            if cell >= total_cells {
                break;
            }
            let s = (word >> (slot * 2)) & 0b11;
            if s == 0 {
                continue;
            }
            let z = cell / plane;
            let rem = cell % plane;
            let y = rem / dims.x;
            let x = rem % dims.x;
            let cx = x / downsample;
            let cy = y / downsample;
            let cz = z / downsample;
            let idx = ((cz * coarse_dims.y + cy) * coarse_dims.x + cx) as usize;
            if s & 0b10 != 0 {
                state.occ[idx] += 1;
            } else if s & 0b01 != 0 {
                state.free[idx] += 1;
            }
        }
    }
    state.word_cursor = end;

    if state.word_cursor < state.snapshot.len() {
        return;
    }

    // Scan complete — publish the new grid + reset state for the next round.
    let total = state.occ.len();
    let coarse: Vec<super::resources::CoarseCell> = (0..total)
        .map(|i| {
            if state.occ[i] > state.free[i] {
                super::resources::CoarseCell::Blocked
            } else if state.free[i] > state.occ[i] {
                super::resources::CoarseCell::Free
            } else {
                super::resources::CoarseCell::Unknown
            }
        })
        .collect();
    *grid = PlannerGrid {
        coarse,
        dims: coarse_dims,
        voxel_size: state.voxel_size,
        downsample,
    };
    state.active = false;
    state.cooldown_secs = PLANNER_SCAN_COOLDOWN_SECS;
}

#[derive(Default)]
pub struct ClusterScanState {
    pub snapshot: Vec<u32>,
    pub dims: UVec3,
    pub candidates: HashSet<UVec3>,
    pub word_cursor: usize,
    pub active: bool,
    pub cooldown_secs: f32,
}

/// Minimum gap between cluster-scan finalisations. Caps the `build_clusters`
/// spike rate to ~1/s regardless of frame rate. assign_targets re-scores
/// every frame, so cluster freshness above 1 Hz is wasted CPU.
const CLUSTER_SCAN_COOLDOWN_SECS: f32 = 1.0;

pub fn compute_frontier_clusters(
    time: Res<Time>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    mut state: Local<ClusterScanState>,
    mut clusters: ResMut<FrontierClusters>,
) {
    if mirror.data.is_empty() {
        return;
    }
    if !state.active {
        state.cooldown_secs -= time.delta_secs();
        if state.cooldown_secs > 0.0 {
            return;
        }
        state.snapshot.clone_from(&mirror.data);
        state.dims = world.size;
        state.candidates.clear();
        state.word_cursor = 0;
        state.active = true;
    }

    let dims = state.dims;
    let total_cells = dims.x * dims.y * dims.z;
    let plane = dims.x * dims.y;
    let end = (state.word_cursor + SCAN_WORDS_PER_FRAME).min(state.snapshot.len());

    let read = |snapshot: &[u32], cell: u32| -> u32 {
        let w = (cell / 16) as usize;
        if w >= snapshot.len() {
            return 0;
        }
        let b = (cell % 16) * 2;
        (snapshot[w] >> b) & 0b11
    };

    for w_idx in state.word_cursor..end {
        let word = state.snapshot[w_idx];
        if word == 0 {
            continue;
        }
        let base_cell = (w_idx as u32) * 16;
        for slot in 0..16u32 {
            let s = (word >> (slot * 2)) & 0b11;
            if s != 0b01 {
                continue;
            }
            let cell = base_cell + slot;
            if cell >= total_cells {
                break;
            }
            let z = cell / plane;
            let rem = cell % plane;
            let y = rem / dims.x;
            let x = rem % dims.x;
            let ix = x as i32;
            let iy = y as i32;
            let iz = z as i32;
            for d in [
                IVec3::new(-1, 0, 0),
                IVec3::new(1, 0, 0),
                IVec3::new(0, -1, 0),
                IVec3::new(0, 1, 0),
                IVec3::new(0, 0, -1),
                IVec3::new(0, 0, 1),
            ] {
                let nx = ix + d.x;
                let ny = iy + d.y;
                let nz = iz + d.z;
                if nx < 0 || ny < 0 || nz < 0 {
                    continue;
                }
                if nx as u32 >= dims.x || ny as u32 >= dims.y || nz as u32 >= dims.z {
                    continue;
                }
                let nflat = nx as u32 + ny as u32 * dims.x + nz as u32 * plane;
                if state.candidates.len() >= MAX_FRONTIER_CANDIDATES {
                    continue;
                }
                if read(&state.snapshot, nflat) == 0 {
                    state.candidates.insert(UVec3::new(nx as u32, ny as u32, nz as u32));
                }
            }
        }
    }
    state.word_cursor = end;

    if state.word_cursor < state.snapshot.len() {
        return;
    }

    // Scan complete — publish the new cluster list, restart after cooldown.
    clusters.entries = build_clusters(&state.candidates, &mut clusters.next_id);
    state.active = false;
    state.cooldown_secs = CLUSTER_SCAN_COOLDOWN_SECS;
}

pub fn replan_paths(
    grid: Res<PlannerGrid>,
    mut scratch: Local<PlanScratch>,
    mut q: Query<(&Transform, Ref<FrontierTarget>, &mut Path), With<Drone>>,
) {
    if grid.dims == UVec3::ZERO {
        return;
    }
    let cell_size = grid.voxel_size * grid.downsample as f32;
    let mut budget = MAX_REPLANS_PER_FRAME;
    for (transform, target, mut path) in &mut q {
        let Some(target_pos) = target.pos else {
            if !path.waypoints.is_empty() {
                path.waypoints.clear();
                path.cursor = 0;
            }
            continue;
        };
        // Event-driven replan: target changed, or path empty.
        // Stuck recovery clears `path` directly, which trips the
        // path-empty branch on the next call.
        let need_replan = path.waypoints.is_empty() || target.is_changed();
        if !need_replan {
            continue;
        }
        if budget == 0 {
            // Leave this drone's stale path in place; next frame's
            // budget will replan it. Path stays valid enough to keep
            // the drone moving in roughly the right direction.
            break;
        }
        budget -= 1;

        let drone_pos = transform.translation;
        let start = (drone_pos / cell_size).floor().as_uvec3();
        let goal = (target_pos / cell_size).floor().as_uvec3();
        match plan(&grid, start, goal, &mut scratch) {
            Some(cells) => {
                path.waypoints.clear();
                path.waypoints.extend(cells.iter().map(|c| grid.world_pos_of(*c)));
                path.cursor = 0;
            }
            None => {
                path.waypoints.clear();
                path.cursor = 0;
            }
        }
    }
}
pub fn update_movement_health(
    time: Res<Time>,
    mut q: Query<(&LinearVelocity, &mut MovementHealth), With<Drone>>,
) {
    let dt = time.delta_secs();
    for (linvel, mut health) in &mut q {
        if linvel.0.length() < STUCK_VEL_MPS {
            health.slow_secs += dt;
        } else {
            health.slow_secs = 0.0;
        }
    }
}

pub fn stuck_recovery(
    time: Res<Time>,
    world: Res<crate::world::WorldConfig>,
    mut q: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &mut MovementHealth,
        &mut Path,
    ), With<Drone>>,
) {
    let now = time.elapsed_secs();
    let mut rng = rand::rng();
    for (mut transform, mut linvel, mut health, mut path) in &mut q {
        if health.slow_secs < STUCK_SECS {
            continue;
        }
        health.slow_secs = 0.0;

        // Force replan by clearing the path.
        path.waypoints.clear();
        path.cursor = 0;

        // Apply random kick (small impulse) to escape local minima.
        let kick = Vec3::new(
            rng.random_range(-2.0..2.0),
            rng.random_range(-0.5..0.5),
            rng.random_range(-2.0..2.0),
        );
        linvel.0 += kick;

        // Bookkeeping for escalation.
        let window_open = now - health.window_start_secs < STUCK_ESCALATION_WINDOW_SECS;
        if window_open {
            health.escalations_in_window += 1;
        } else {
            health.window_start_secs = now;
            health.escalations_in_window = 1;
        }

        if health.escalations_in_window >= 3 {
            // Final fallback: teleport to world center.
            warn!("drone stuck after 3 escalations — teleporting to world center");
            transform.translation = world.center();
            linvel.0 = Vec3::ZERO;
            health.escalations_in_window = 0;
        }
    }
}
pub fn steer_along_path(
    time: Res<Time>,
    mut q: Query<(&Transform, &Role, &mut Path, &mut DesiredVelocity), With<Drone>>,
) {
    let dt = time.delta_secs();
    for (transform, role, mut path, mut desired) in &mut q {
        let cruise = RoleParams::for_role(*role).cruise_speed_mps;
        if cruise <= 0.0 {
            // Anchors don't move via the planner.
            continue;
        }
        let Some(waypoint) = pure_pursuit(&mut path, transform.translation) else {
            continue;
        };
        let to_wp = waypoint - transform.translation;
        let dist = to_wp.length();
        if dist < 1e-3 {
            continue;
        }
        let target_vel = (to_wp / dist) * cruise;
        let alpha = (PATH_FOLLOW_LERP_RATE * dt).min(1.0);
        desired.0 = desired.0.lerp(target_vel, alpha);
    }
}
#[allow(clippy::too_many_arguments)]
pub fn reactive_avoid(
    mirror: Res<GpuGlobalOccupancyMirror>,
    comms: Res<CommsState>,
    world: Res<crate::world::WorldConfig>,
    mut q_self: Query<(&DroneId, &Transform, &Role, &mut DesiredVelocity), With<Drone>>,
    q_peers: Query<(&DroneId, &Transform), With<Drone>>,
    mut hits_buf: Local<Vec<Vec3>>,
    mut peers_buf: Local<Vec<Vec3>>,
    mut peer_snapshot: Local<Vec<(u32, Vec3)>>,
) {
    if mirror.data.is_empty() {
        return;
    }
    let dims = world.size;
    let voxel_size = world.voxel_size;
    let data = &mirror.data;
    let read = |cell: UVec3| -> u32 {
        let flat = cell.x + cell.y * dims.x + cell.z * dims.x * dims.y;
        let w = (flat / 16) as usize;
        if w >= data.len() {
            return 0;
        }
        let b = (flat % 16) * 2;
        (data[w] >> b) & 0b11
    };

    let radius_cells = (AVOID_RADIUS_M / voxel_size).ceil() as i32;
    peer_snapshot.clear();
    peer_snapshot.extend(q_peers.iter().map(|(id, t)| (id.0, t.translation)));

    for (id, transform, role, mut desired) in &mut q_self {
        let pos = transform.translation;
        let drone_cell = (pos / voxel_size).floor().as_ivec3();
        hits_buf.clear();
        for dz in -radius_cells..=radius_cells {
            for dy in -radius_cells..=radius_cells {
                for dx in -radius_cells..=radius_cells {
                    let c = drone_cell + IVec3::new(dx, dy, dz);
                    if c.x < 0 || c.y < 0 || c.z < 0 {
                        continue;
                    }
                    let u = UVec3::new(c.x as u32, c.y as u32, c.z as u32);
                    if u.x >= dims.x || u.y >= dims.y || u.z >= dims.z {
                        continue;
                    }
                    let state = read(u);
                    if state & 0b10 != 0 {
                        let wp = Vec3::new(u.x as f32, u.y as f32, u.z as f32) * voxel_size
                            + Vec3::splat(voxel_size * 0.5);
                        hits_buf.push(wp);
                    }
                }
            }
        }
        // Filter peer list to comms-connected peers.
        peers_buf.clear();
        let half = (id.0 >= 32) as usize;
        let connected = (comms.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        if connected {
            for (pid, p) in peer_snapshot.iter() {
                if *pid == id.0 {
                    continue;
                }
                let h = (*pid >= 32) as usize;
                if (comms.connected_mask[h] >> (pid % 32)) & 1 == 1 {
                    peers_buf.push(*p);
                }
            }
        }
        let avoid_k = RoleParams::for_role(*role).avoid_k;
        let force = reactive_force(pos, &hits_buf, &peers_buf, avoid_k);
        desired.0 += force;
    }
}

/// Anchors hover. Runs after every other steering input so it
/// authoritatively zeroes `DesiredVelocity` for any Role::Anchor drone
/// before the physics controller sees it.
pub fn enforce_anchor_hover(
    mut q: Query<(&Role, &mut DesiredVelocity), With<Drone>>,
) {
    for (role, mut desired) in &mut q {
        if *role == Role::Anchor {
            desired.0 = Vec3::ZERO;
        }
    }
}
