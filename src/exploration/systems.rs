// src/exploration/systems.rs
//
// Transitional state during the Foraging-Colony plan (see
// docs/.../perfect-ok-so-now-foamy-meerkat.md). `apply_role_steering`
// at the bottom of the file is the live system; the older
// `assign_targets` + A* + path-follow chain is unwired but kept around
// for one or two commits so the diff is reviewable. Phase 6 deletes
// them.
#![allow(dead_code, unused_imports)]

use bevy::platform::collections::HashSet;
use bevy::prelude::*;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneColor, DroneId};
use crate::lidar::gpu::GpuGlobalOccupancyMirror;
use crate::pheromone::PheromoneField;
use crate::physics::{DesiredVelocity, LinearVelocity};

use super::cluster::build_clusters;
use super::components::{FrontierTarget, GhostMemory, GhostPeer, MovementHealth, Path, Trail};
use super::constants::{
    ARRIVAL_RADIUS_M, AVOID_RADIUS_M, FRONTIER_REACHED_DIST, MAX_FRONTIER_CANDIDATES,
    PLANNER_DOWNSAMPLE, SCORE_UPGRADE_RATIO, STUCK_ESCALATION_WINDOW_SECS, STUCK_SECS,
    STUCK_VEL_MPS, TRAIL_MAX_POINTS, TRAIL_SAMPLE_INTERVAL_SECS,
};

/// Cap how many A* calls run per frame to amortise the cost of large
/// target-changed events (e.g. first frame after spawn, or right after
/// a map swap when every drone simultaneously gets a fresh target).
/// Remaining drones get their replan on the next frame.
const MAX_REPLANS_PER_FRAME: usize = 4;
use super::planner::{plan, PlanScratch};
use super::resources::{FrontierClusters, PlannerGrid};
use super::steering::{pure_pursuit, reactive_force, reactive_force_peers};
use super::scoring::{score, ScoringWeights};
use super::role::{peer_repulsion_for, Role, RoleParams};
use rand::RngExt;

pub fn assign_targets(
    clusters: Res<FrontierClusters>,
    mut snapshot: Local<Vec<(u32, Vec3, Role, Option<u32>, Option<Vec3>)>>,
    mut scored_buf: Local<Vec<(f32, u32, Vec3)>>,
    mut claimed: Local<HashSet<u32>>,
    mut assignments: Local<Vec<(u32, Option<u32>, Option<Vec3>)>>,
    mut q: Query<(&DroneId, &Transform, &Role, &mut FrontierTarget), With<Drone>>,
) {
    if clusters.entries.is_empty() {
        return;
    }

    // Snapshot every drone's state, then sort by id so the auction
    // assigns clusters deterministically. Scouts and mappers compete
    // for the same pool; the score per role still pulls each toward
    // its preferred kind of cluster (Scout = info-heavy + far,
    // Mapper = crowding-heavy + close). Greedy auction with hard
    // claim guarantees no two drones target the same cluster
    // (provided #clusters >= #drones; otherwise late drones share).
    snapshot.clear();
    snapshot.extend(q.iter().map(|(id, t, r, ft)| {
        (id.0, t.translation, *r, ft.cluster_id, ft.pos)
    }));
    snapshot.sort_by_key(|(id, ..)| *id);

    claimed.clear();
    assignments.clear();
    assignments.reserve(snapshot.len());

    for (id, drone_pos, role, cur_id, cur_pos) in snapshot.iter().copied() {
        if role == Role::Anchor {
            // Anchors hold position; supervisor pipeline owns their
            // target. Preserve whatever target they had.
            assignments.push((id, cur_id, cur_pos));
            if let Some(c) = cur_id {
                claimed.insert(c);
            }
            continue;
        }

        let role_params = RoleParams::for_role(role);
        let weights = ScoringWeights {
            info: role_params.info_weight,
            distance: role_params.distance_weight,
            distance_bias: role_params.distance_bias,
            crowding: role_params.crowding_weight,
        };

        // Score all clusters. Crowding-from-peer-position now comes
        // for free from the claim set: a cluster already taken by an
        // earlier drone in the auction gets a hard 90% score penalty,
        // which beats out the cost-utility math for anything but the
        // most lopsided cluster.
        scored_buf.clear();
        scored_buf.extend(clusters.entries.iter().map(|c| {
            let base = score(c, drone_pos, 0, &weights);
            let s = if claimed.contains(&c.id) { base * 0.1 } else { base };
            (s, c.id, c.centroid)
        }));

        let best = scored_buf
            .iter()
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let Some(&(best_score, best_id, best_centroid)) = best else {
            assignments.push((id, None, None));
            continue;
        };

        // Stickiness: keep current target unless reached, vanished,
        // or an UNCLAIMED cluster scores ≥ 1.5x the current one.
        let keep = match cur_id {
            None => false,
            Some(cur) => {
                let cur_alive = clusters.entries.iter().any(|c| c.id == cur);
                if !cur_alive {
                    false
                } else if let Some(cp) = cur_pos {
                    if cp.distance(drone_pos) < FRONTIER_REACHED_DIST {
                        false
                    } else {
                        let cur_score = scored_buf
                            .iter()
                            .find(|(_, cid, _)| *cid == cur)
                            .map(|s| s.0)
                            .unwrap_or(0.0);
                        best_score <= cur_score * SCORE_UPGRADE_RATIO
                    }
                } else {
                    false
                }
            }
        };
        if keep {
            assignments.push((id, cur_id, cur_pos));
            if let Some(c) = cur_id {
                claimed.insert(c);
            }
        } else {
            assignments.push((id, Some(best_id), Some(best_centroid)));
            claimed.insert(best_id);
        }
    }

    // Apply assignments — only write `FrontierTarget` when it
    // actually changed, to avoid burning a change-detection tick
    // every frame and re-firing replan_paths.
    for (id, _t, _r, mut target) in &mut q {
        let Some(&(_, new_id, new_pos)) =
            assignments.iter().find(|(aid, _, _)| *aid == id.0)
        else {
            continue;
        };
        if target.cluster_id != new_id || target.pos != new_pos {
            target.cluster_id = new_id;
            target.pos = new_pos;
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
    mut q: Query<
        (
            &Transform,
            &Role,
            &mut Path,
            &FrontierTarget,
            &mut DesiredVelocity,
        ),
        With<Drone>,
    >,
) {
    for (transform, role, mut path, frontier, mut desired) in &mut q {
        let cruise = RoleParams::for_role(*role).cruise_speed_mps;
        if cruise <= 0.0 {
            // Anchors don't move via the planner. `anchor_seek` handles
            // their motion separately.
            continue;
        }
        let pos = transform.translation;
        // Prefer the A* waypoint when one exists; otherwise fall back
        // to a straight-line vector toward `frontier.pos` so the drone
        // doesn't sit drifting for the 1-13 frames it takes the replan
        // budget to compute its path.
        let goal = match pure_pursuit(&mut path, pos) {
            Some(wp) => wp,
            None => match frontier.pos {
                Some(p) => p,
                None => continue,
            },
        };
        let to_goal = goal - pos;
        let dist = to_goal.length();
        if dist < 1e-3 {
            continue;
        }
        // Arrival ramp keys off distance to the FINAL target (last
        // waypoint or frontier.pos), not the next A* waypoint. A*
        // waypoints are spaced ~8 m apart, so braking on every
        // intermediate waypoint would chop the drone into baby steps
        // and it would never reach cruise. Only the last leg should
        // ramp down.
        let final_pos = path
            .waypoints
            .last()
            .copied()
            .or(frontier.pos)
            .unwrap_or(goal);
        let dist_to_final = (final_pos - pos).length();
        let arrival_scale = (dist_to_final / ARRIVAL_RADIUS_M).clamp(0.0, 1.0);
        // Direct write — no lerp here. The velocity tracker in
        // `physics::track_velocity` already lerps `linvel` toward
        // `desired`; doubling the lag delays pursuit by a full extra
        // VEL_TRACK_GAIN time constant for no benefit.
        desired.0 = (to_goal / dist) * cruise * arrival_scale;
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
        // Cap the combined steering + avoidance command so that a
        // near-singular peer-bubble repulsion can't shoot the lerp
        // tracker past its stable rate. `cruise * 1.5` is plenty to
        // sidestep a peer at a brisk pace.
        let cruise = RoleParams::for_role(*role).cruise_speed_mps;
        let max_mag = (cruise * 1.5).max(2.0);
        desired.0 = (desired.0 + force).clamp_length_max(max_mag);
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

/// Push the drone's current position into its `Trail` ring buffer at a
/// fixed sample interval. The interval keeps the line readable at low
/// flight speeds (per-frame sampling would clump points within a few
/// cm at 120 FPS).
pub fn sample_trails(
    time: Res<Time>,
    mut q: Query<(&Transform, &mut Trail), With<Drone>>,
) {
    let now = time.elapsed_secs();
    for (transform, mut trail) in &mut q {
        if now - trail.last_sample_secs < TRAIL_SAMPLE_INTERVAL_SECS {
            continue;
        }
        trail.last_sample_secs = now;
        trail.points.push_back(transform.translation);
        while trail.points.len() > TRAIL_MAX_POINTS {
            trail.points.pop_front();
        }
    }
}

/// Draw the recent-position trail behind each drone as a gizmo
/// polyline. Color = drone tint with a per-segment alpha ramp so older
/// samples fade toward transparent.
pub fn draw_trail_gizmos(
    ui_state: Res<crate::ui::UiState>,
    mut gizmos: Gizmos,
    q: Query<(&Transform, &DroneColor, &Trail), With<Drone>>,
) {
    if !ui_state.show_trails {
        return;
    }
    for (transform, color, trail) in &q {
        if trail.points.len() < 2 {
            continue;
        }
        let base = color.0.to_linear();
        let n = trail.points.len();
        for (i, window) in trail.points.iter().collect::<Vec<_>>().windows(2).enumerate() {
            // Alpha ramps from ~0.1 at the oldest segment to 1.0 at
            // the newest — makes the head of the trail pop while the
            // tail dissolves into the world.
            let t = (i + 1) as f32 / n as f32;
            let alpha = 0.1 + 0.9 * t;
            let c = Color::linear_rgba(base.red, base.green, base.blue, alpha);
            gizmos.line(*window[0], *window[1], c);
        }
        // Final segment from the last sample to the live position so
        // the trail tip stays glued to the drone.
        if let Some(last) = trail.points.back() {
            let c = Color::linear_rgba(base.red, base.green, base.blue, 1.0);
            gizmos.line(*last, transform.translation, c);
        }
    }
}

/// Draw the planned A* polyline ahead of each drone plus a final
/// dashed line to the frontier target centroid. Different colors for
/// "path ahead" vs "target lock" so they're distinguishable.
pub fn draw_path_gizmos(
    ui_state: Res<crate::ui::UiState>,
    mut gizmos: Gizmos,
    q: Query<(&Transform, &DroneColor, &Path, &FrontierTarget), With<Drone>>,
) {
    if !ui_state.show_paths {
        return;
    }
    for (transform, color, path, target) in &q {
        let base = color.0.to_linear();
        // Path polyline: drone -> next waypoint -> ... -> last waypoint.
        if !path.waypoints.is_empty() {
            let c = Color::linear_rgba(base.red, base.green, base.blue, 0.55);
            let mut prev = transform.translation;
            for wp in &path.waypoints[path.cursor..] {
                gizmos.line(prev, *wp, c);
                prev = *wp;
            }
        }
        // Target lock: faint white tag at the cluster centroid.
        if let Some(target_pos) = target.pos {
            gizmos.sphere(
                Isometry3d::from_translation(target_pos),
                1.5,
                Color::linear_rgba(1.0, 1.0, 1.0, 0.35),
            );
        }
    }
}

/// Per-role gradient steering. Replaces the old `assign_targets` +
/// A* + `pure_pursuit` + `steer_along_path` chain.
///
/// Each drone reads the local pheromone gradient and combines it with
/// per-role rules:
///
/// - **Scout** flies down the gradient (toward LOW pheromone). When
///   there's no gradient yet (cold start, fresh map), falls back to a
///   radial-out vector from the world center so scouts fan out instead
///   of piling on the spawn ring.
/// - **Mapper** flies up the gradient (toward HIGH pheromone). Tracks
///   scout trails to refine them with its wide-cone scan.
/// - **Anchor** stays put for now; Phase 4 of the foraging-colony plan
///   replaces this with a geometric-median relay-positioning algorithm.
///
/// Plus a peer separation force (`reactive_force` over peers only) and
/// a lidar-hit terrain repulsion (`reactive_force` over hits in a small
/// cube around the drone). Final `desired` is clamped to `1.5×cruise`
/// so the singular peer-bubble term in `reactive_force` can't shake the
/// physics tracker.
#[allow(clippy::too_many_arguments)]
pub fn apply_role_steering(
    time: Res<Time>,
    pheromone: Res<PheromoneField>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    comms_state: Res<CommsState>,
    comms_settings: Res<crate::comms::CommsSettings>,
    mut q_self: Query<
        (
            &DroneId,
            &Transform,
            &Role,
            &mut DesiredVelocity,
            &mut GhostMemory,
        ),
        With<Drone>,
    >,
    q_peers: Query<(&DroneId, &Transform, &Role), With<Drone>>,
    mut hits_buf: Local<Vec<Vec3>>,
    mut peers_buf: Local<Vec<(Vec3, f32)>>,
    mut peer_snap: Local<Vec<(u32, Vec3, Role)>>,
) {
    peer_snap.clear();
    peer_snap.extend(q_peers.iter().map(|(id, t, r)| (id.0, t.translation, *r)));
    let now = time.elapsed_secs();
    let comms_range = comms_settings.range_m;
    let central_pos = comms_state.base_pos;

    let data = &mirror.data;
    let dims = world.size;
    let voxel_size = world.voxel_size;
    let world_center = world.center();
    let radius_cells = (AVOID_RADIUS_M / voxel_size).ceil() as i32;

    for (id, transform, role, mut desired, mut ghost) in &mut q_self {
        let pos = transform.translation;
        let cruise = RoleParams::for_role(*role).cruise_speed_mps;
        let avoid_k = RoleParams::for_role(*role).avoid_k;

        // Peer separation — pair-wise stiffness. Scouts barely brake
        // for Mappers (k = 1) but Mappers actively yield to Scouts
        // (k = 28). Anchors don't move at all. See
        // `peer_repulsion_for` in `role.rs`.
        peers_buf.clear();
        for (pid, p, peer_role) in peer_snap.iter() {
            if *pid == id.0 {
                continue;
            }
            let k = peer_repulsion_for(*role, *peer_role);
            if k > 0.0 {
                peers_buf.push((*p, k));
            }
        }
        let separation = reactive_force_peers(pos, &peers_buf);

        // Terrain repulsion from lidar hits in a small cube around
        // the drone. Reads the global occupancy mirror (CPU copy of
        // the GPU bitset). Skips when the mirror hasn't populated yet.
        hits_buf.clear();
        if !data.is_empty() {
            let drone_cell = (pos / voxel_size).floor().as_ivec3();
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
                        let flat = u.x + u.y * dims.x + u.z * dims.x * dims.y;
                        let w = (flat / 16) as usize;
                        if w >= data.len() {
                            continue;
                        }
                        let b = (flat % 16) * 2;
                        let state = (data[w] >> b) & 0b11;
                        if state & 0b10 != 0 {
                            let wp = Vec3::new(u.x as f32, u.y as f32, u.z as f32)
                                * voxel_size
                                + Vec3::splat(voxel_size * 0.5);
                            hits_buf.push(wp);
                        }
                    }
                }
            }
        }
        let terrain = reactive_force(pos, &hits_buf, &[], avoid_k);

        // Per-role attractor.
        let role_force = match role {
            Role::Scout => {
                let grad = pheromone.gradient_at(pos);
                let anti = -grad;
                let dir = anti.normalize_or_zero();
                if dir == Vec3::ZERO {
                    // No gradient yet (just spawned, no pheromone
                    // anywhere). Fall back to a radial-out vector from
                    // the world center so scouts fan out.
                    let outward = (pos - world_center).normalize_or_zero();
                    if outward == Vec3::ZERO {
                        Vec3::new(1.0, 0.0, 0.0) * cruise
                    } else {
                        outward * cruise
                    }
                } else {
                    dir * cruise
                }
            }
            Role::Mapper => {
                let grad = pheromone.gradient_at(pos);
                let dir = grad.normalize_or_zero();
                if dir == Vec3::ZERO {
                    // No trail to follow yet — hover near spawn until
                    // a scout lays one down.
                    Vec3::ZERO
                } else {
                    dir * cruise
                }
            }
            Role::Anchor => {
                // Update ghost memory: any peer currently in comms
                // range refreshes its last-known position; ghosts
                // older than GHOST_FORGET_SECS get dropped.
                let range_sq = comms_range * comms_range;
                for (pid, p, _r) in peer_snap.iter() {
                    if *pid == id.0 {
                        continue;
                    }
                    if pos.distance_squared(*p) <= range_sq {
                        ghost.peers.insert(
                            *pid,
                            super::components::GhostPeer {
                                last_pos: *p,
                                last_seen_secs: now,
                            },
                        );
                    }
                }
                ghost.peers.retain(|_, g| now - g.last_seen_secs < GHOST_FORGET_SECS);

                // Connection state vs central.
                let half = (id.0 >= 32) as usize;
                let in_chain =
                    (comms_state.connected_mask[half] >> (id.0 % 32)) & 1 == 1;

                // Decide target: out-of-chain → move toward central;
                // peer near range limit → move toward that peer's
                // last-known position; otherwise hover at the
                // geometric median of remembered peers.
                let target = if !comms_settings.enabled || ghost.peers.is_empty() {
                    // Comms gating off OR no peers ever seen — just
                    // sit at central as a passive relay.
                    central_pos
                } else if !in_chain {
                    central_pos
                } else {
                    let critical_radius = comms_range * 0.85;
                    let critical = ghost
                        .peers
                        .values()
                        .filter(|g| pos.distance(g.last_pos) > critical_radius)
                        .max_by(|a, b| {
                            pos.distance(a.last_pos)
                                .partial_cmp(&pos.distance(b.last_pos))
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|g| g.last_pos);
                    match critical {
                        Some(p) => p,
                        None => geometric_median_of_ghosts(&ghost.peers, pos),
                    }
                };

                let to_target = target - pos;
                let dist = to_target.length();
                if dist < 0.5 {
                    Vec3::ZERO
                } else {
                    // Arrival ramp: full cruise outside 5 m, taper
                    // linearly to zero at the target.
                    let scale = (dist / 5.0).clamp(0.0, 1.0);
                    (to_target / dist) * cruise * scale
                }
            }
        };

        // Cap each contributor independently so a giant separation
        // term can't drag a slow Mapper up to Scout-speed. role_force
        // is always magnitude `cruise` (or 0 for hover), so cap it
        // there. Repulsion forces get their own cap proportional to
        // role cruise too — fast roles can dodge harder, slow roles
        // can't be flung. Final magnitude clamp on the sum keeps the
        // tracker stable.
        let role_capped = role_force.clamp_length_max(cruise);
        let avoid_cap = (cruise * 1.5).max(2.0);
        let avoid_combined = (separation + terrain).clamp_length_max(avoid_cap);
        let total = role_capped + avoid_combined;
        desired.0 = total.clamp_length_max((cruise + avoid_cap).max(2.0));
    }
}

/// Forget peer ghosts older than this many seconds. Anchors don't
/// trust position estimates from observations more than a few seconds
/// stale — comms range is the spec, and the swarm moves fast enough
/// that 5 s is already several body-lengths of drift.
const GHOST_FORGET_SECS: f32 = 5.0;

/// Weiszfeld iteration for the geometric median (the point minimizing
/// the sum of L2 distances to a set of input points). Anchors use this
/// to hover "between" their visible peers rather than at the centroid
/// (which gets pulled by clumps). 5 iterations is plenty to converge
/// within sub-meter accuracy for the ≤ 50 peers we ever see.
fn geometric_median_of_ghosts(
    peers: &bevy::platform::collections::HashMap<u32, GhostPeer>,
    fallback: Vec3,
) -> Vec3 {
    if peers.is_empty() {
        return fallback;
    }
    let n = peers.len() as f32;
    let mut y: Vec3 = peers.values().fold(Vec3::ZERO, |a, g| a + g.last_pos) / n;
    for _ in 0..5 {
        let mut num = Vec3::ZERO;
        let mut den = 0.0;
        for g in peers.values() {
            let d = (y - g.last_pos).length().max(0.01);
            let w = 1.0 / d;
            num += g.last_pos * w;
            den += w;
        }
        if den > 0.0 {
            y = num / den;
        }
    }
    y
}
