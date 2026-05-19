// src/exploration/systems.rs
use bevy::prelude::*;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneId};

use super::components::{FrontierTarget, MovementHealth, Path};
use super::constants::{FRONTIER_REACHED_DIST, SCORE_UPGRADE_RATIO};
use super::resources::FrontierClusters;
use super::scoring::{crowding_for, score, ScoringWeights};

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
pub fn update_movement_health() {}
pub fn stuck_recovery() {}
pub fn steer_along_path() {}
pub fn reactive_avoid() {}
