// src/exploration/systems.rs
use bevy::prelude::*;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneId};
use crate::physics::LinearVelocity;

use super::components::{FrontierTarget, MovementHealth, Path};
use super::constants::{
    FRONTIER_REACHED_DIST, SCORE_UPGRADE_RATIO, STUCK_ESCALATION_WINDOW_SECS, STUCK_SECS,
    STUCK_VEL_MPS,
};
use super::resources::FrontierClusters;
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

pub fn compute_frontier_clusters() {
    // Wired in Task 10.
}
pub fn rebuild_planner_grid() {}
pub fn replan_paths() {}
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
pub fn steer_along_path() {}
pub fn reactive_avoid() {}
