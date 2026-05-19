// src/exploration/systems.rs
use bevy::prelude::*;
use std::collections::HashSet;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneId, CRUISE_SPEED_MPS};
use crate::lidar::gpu::GpuGlobalOccupancyMirror;
use crate::physics::{DesiredVelocity, LinearVelocity};

use super::cluster::build_clusters;
use super::components::{FrontierTarget, MovementHealth, Path};
use super::constants::{
    AVOID_K_DEFAULT, AVOID_RADIUS_M, FRONTIER_REACHED_DIST, FRONTIER_REFRESH_SECS,
    PATH_FOLLOW_LERP_RATE, PLANNER_DOWNSAMPLE, REPLAN_MIN_INTERVAL_SECS, SCORE_UPGRADE_RATIO,
    STUCK_ESCALATION_WINDOW_SECS, STUCK_SECS, STUCK_VEL_MPS,
};
use super::planner::plan;
use super::resources::{FrontierClusters, PlannerGrid};
use super::steering::{pure_pursuit, reactive_force};
use super::scoring::{crowding_for, score, ScoringWeights};
use rand::RngExt;

pub fn assign_targets(
    clusters: Res<FrontierClusters>,
    comms: Res<CommsState>,
    mut q_self: Query<(&DroneId, &Transform, &mut FrontierTarget), With<Drone>>,
    q_peers: Query<(&DroneId, &Transform, &FrontierTarget), With<Drone>>,
) {
    if clusters.entries.is_empty() {
        return;
    }
    // Snapshot peer positions + targets keyed by id for crowding lookups.
    let peers: Vec<(u32, Vec3, Option<u32>)> = q_peers
        .iter()
        .map(|(id, t, ft)| (id.0, t.translation, ft.cluster_id))
        .collect();

    let weights = ScoringWeights::default();

    for (id, transform, mut target) in &mut q_self {
        let drone_pos = transform.translation;
        // Filter peers to the comms cluster of the deciding drone.
        let half = (id.0 >= 32) as usize;
        let i_am_connected = (comms.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        let visible_peers: Vec<(Vec3, Option<u32>)> = if i_am_connected {
            peers
                .iter()
                .filter(|(pid, _, _)| {
                    if *pid == id.0 {
                        return false;
                    }
                    let h = (*pid >= 32) as usize;
                    (comms.connected_mask[h] >> (pid % 32)) & 1 == 1
                })
                .map(|(_, p, t)| (*p, *t))
                .collect()
        } else {
            Vec::new()
        };

        // Score all clusters once.
        let scored: Vec<(f32, u32, Vec3)> = clusters
            .entries
            .iter()
            .map(|c| {
                let crowding = crowding_for(c, &visible_peers, 0.5);
                (score(c, drone_pos, crowding, &weights), c.id, c.centroid)
            })
            .collect();

        let best = scored
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
                        let cur_score = scored
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

pub fn rebuild_planner_grid(
    time: Res<Time>,
    mut timer: Local<f32>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    mut grid: ResMut<PlannerGrid>,
) {
    *timer += time.delta_secs();
    if *timer < FRONTIER_REFRESH_SECS {
        return;
    }
    *timer = 0.0;
    if mirror.data.is_empty() {
        return;
    }
    *grid = PlannerGrid::downsample_from_bitset(
        world.size,
        world.voxel_size,
        &mirror.data,
        PLANNER_DOWNSAMPLE,
    );
}

pub fn compute_frontier_clusters(
    time: Res<Time>,
    mut timer: Local<f32>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    mut clusters: ResMut<FrontierClusters>,
) {
    *timer += time.delta_secs();
    if *timer < FRONTIER_REFRESH_SECS {
        return;
    }
    *timer = 0.0;
    if mirror.data.is_empty() {
        return;
    }
    let dims = world.size;
    let total = dims.x * dims.y * dims.z;
    let data = &mirror.data;
    let read = |cell: u32| -> u32 {
        let w = (cell / 16) as usize;
        if w >= data.len() {
            return 0;
        }
        let b = (cell % 16) * 2;
        (data[w] >> b) & 0b11
    };
    let mut candidates: HashSet<UVec3> = HashSet::new();
    let plane = dims.x * dims.y;
    for cell in 0..total {
        if read(cell) != 0b01 {
            continue;
        }
        // Free cell — push Unknown 6-neighbours.
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
            if read(nflat) == 0 {
                candidates.insert(UVec3::new(nx as u32, ny as u32, nz as u32));
            }
        }
    }
    clusters.entries = build_clusters(&candidates, &mut clusters.next_id);
}

#[derive(Component, Default, Debug)]
pub struct ReplanTimer(pub f32);

pub fn replan_paths(
    time: Res<Time>,
    grid: Res<PlannerGrid>,
    mut q: Query<
        (&Transform, &FrontierTarget, &mut Path, &mut ReplanTimer),
        With<Drone>,
    >,
) {
    if grid.dims == UVec3::ZERO {
        return;
    }
    let dt = time.delta_secs();
    for (transform, target, mut path, mut rt) in &mut q {
        rt.0 += dt;
        let Some(target_pos) = target.pos else {
            path.waypoints.clear();
            path.cursor = 0;
            continue;
        };
        let need_replan =
            path.waypoints.is_empty() || rt.0 >= REPLAN_MIN_INTERVAL_SECS;
        if !need_replan {
            continue;
        }
        rt.0 = 0.0;

        let drone_pos = transform.translation;
        let cell_size = grid.voxel_size * grid.downsample as f32;
        let start = (drone_pos / cell_size).floor().as_uvec3();
        let goal = (target_pos / cell_size).floor().as_uvec3();
        match plan(&grid, start, goal) {
            Some(cells) => {
                path.waypoints = cells.iter().map(|c| grid.world_pos_of(*c)).collect();
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
    mut q: Query<(&Transform, &mut Path, &mut DesiredVelocity), With<Drone>>,
) {
    let dt = time.delta_secs();
    for (transform, mut path, mut desired) in &mut q {
        let Some(waypoint) = pure_pursuit(&mut path, transform.translation) else {
            continue;
        };
        let to_wp = waypoint - transform.translation;
        let dist = to_wp.length();
        if dist < 1e-3 {
            continue;
        }
        let target_vel = (to_wp / dist) * CRUISE_SPEED_MPS;
        let alpha = (PATH_FOLLOW_LERP_RATE * dt).min(1.0);
        desired.0 = desired.0.lerp(target_vel, alpha);
    }
}
pub fn reactive_avoid(
    mirror: Res<GpuGlobalOccupancyMirror>,
    comms: Res<CommsState>,
    world: Res<crate::world::WorldConfig>,
    mut q_self: Query<(&DroneId, &Transform, &mut DesiredVelocity), With<Drone>>,
    q_peers: Query<(&DroneId, &Transform), With<Drone>>,
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
    let peer_snapshot: Vec<(u32, Vec3)> =
        q_peers.iter().map(|(id, t)| (id.0, t.translation)).collect();

    for (id, transform, mut desired) in &mut q_self {
        let pos = transform.translation;
        let drone_cell = (pos / voxel_size).floor().as_ivec3();
        let mut hits = Vec::new();
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
                        // Occupied cell center in world coords.
                        let wp = Vec3::new(u.x as f32, u.y as f32, u.z as f32) * voxel_size
                            + Vec3::splat(voxel_size * 0.5);
                        hits.push(wp);
                    }
                }
            }
        }
        // Filter peer list to comms-connected peers.
        let half = (id.0 >= 32) as usize;
        let connected = (comms.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        let peers: Vec<Vec3> = if connected {
            peer_snapshot
                .iter()
                .filter(|(pid, _)| {
                    if *pid == id.0 {
                        return false;
                    }
                    let h = (*pid >= 32) as usize;
                    (comms.connected_mask[h] >> (pid % 32)) & 1 == 1
                })
                .map(|(_, p)| *p)
                .collect()
        } else {
            Vec::new()
        };
        let force = reactive_force(pos, &hits, &peers, AVOID_K_DEFAULT);
        desired.0 += force;
    }
}
