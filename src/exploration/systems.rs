// src/exploration/systems.rs
//
// Live behaviour: pheromone-driven per-role steering, trail sampling
// + drawing, anchor relay via ghost-memory + geometric median. The
// old planner / A* / cluster / auction / wander pipeline is gone
// (Phase 6 of the foraging-colony plan).

use bevy::prelude::*;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneColor, DroneId};
use crate::pheromone::PheromoneField;
use crate::physics::DesiredVelocity;
use crate::sensors::DetectorHits;

use super::components::{GhostMemory, GhostPeer, Trail};
use super::constants::{PEER_BUBBLE_RADIUS_M, TRAIL_MAX_POINTS, TRAIL_SAMPLE_INTERVAL_SECS};
use super::role::{peer_repulsion_for, Role, RoleParams};
use super::steering::{reactive_force, reactive_force_peers};

/// Push the drone's current position into its `Trail` ring buffer at a
/// fixed sample interval. The interval keeps the line readable at low
/// flight speeds (per-frame sampling would clump points within a few
/// cm at 120 FPS).
pub fn sample_trails(time: Res<Time>, mut q: Query<(&Transform, &mut Trail), With<Drone>>) {
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
            let t = (i + 1) as f32 / n as f32;
            let alpha = 0.1 + 0.9 * t;
            let c = Color::linear_rgba(base.red, base.green, base.blue, alpha);
            gizmos.line(*window[0], *window[1], c);
        }
        if let Some(last) = trail.points.back() {
            let c = Color::linear_rgba(base.red, base.green, base.blue, 1.0);
            gizmos.line(*last, transform.translation, c);
        }
    }
}

/// Per-role pheromone-gradient steering. Each drone composes:
/// - role attractor (scout = anti-pheromone, mapper = pro-pheromone,
///   anchor = relay-positioning via ghost memory)
/// - peer separation (pair-wise stiffness from `peer_repulsion_for`)
/// - terrain repulsion (cube scan of the global occupancy mirror)
/// - final clamp on each contributor so a singular peer-bubble term
///   can't drag a slow Mapper to Scout-speed.
#[allow(clippy::too_many_arguments)]
pub fn apply_role_steering(
    time: Res<Time>,
    pheromone: Res<PheromoneField>,
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
            &DetectorHits,
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
    let world_center = world.center();

    for (id, transform, role, mut desired, mut ghost, detector) in &mut q_self {
        let pos = transform.translation;
        let cruise = RoleParams::for_role(*role).cruise_speed_mps;
        let avoid_k = RoleParams::for_role(*role).avoid_k;

        // Peer separation — pair-wise stiffness. Scouts barely brake
        // for Mappers (k = 1) but Mappers actively yield to Scouts
        // (k = 28). Anchors don't react to peer forces.
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

        // Terrain repulsion from THIS drone's detector ray hits —
        // same rays the grey gizmo viz draws. What you see is what
        // pushes the drone. Misses (`is_hit = false`) get skipped.
        hits_buf.clear();
        for (i, ep) in detector.endpoints.iter().enumerate() {
            if detector.is_hit.get(i).copied().unwrap_or(false) {
                hits_buf.push(*ep);
            }
        }
        let terrain = reactive_force(pos, &hits_buf, &[], avoid_k);

        let role_force = match role {
            Role::Scout => {
                let grad = pheromone.gradient_at(pos);
                let anti = -grad;
                let dir = anti.normalize_or_zero();
                if dir == Vec3::ZERO {
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
                    Vec3::ZERO
                } else {
                    dir * cruise
                }
            }
            Role::Anchor => {
                let range_sq = comms_range * comms_range;
                for (pid, p, _r) in peer_snap.iter() {
                    if *pid == id.0 {
                        continue;
                    }
                    if pos.distance_squared(*p) <= range_sq {
                        ghost.peers.insert(
                            *pid,
                            GhostPeer {
                                last_pos: *p,
                                last_seen_secs: now,
                            },
                        );
                    }
                }
                ghost.peers.retain(|_, g| now - g.last_seen_secs < GHOST_FORGET_SECS);

                let half = (id.0 >= 32) as usize;
                let in_chain = (comms_state.connected_mask[half] >> (id.0 % 32)) & 1 == 1;

                let target = if !comms_settings.enabled || ghost.peers.is_empty() {
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
                    let scale = (dist / 5.0).clamp(0.0, 1.0);
                    (to_target / dist) * cruise * scale
                }
            }
        };

        // Wall-slide: for each close detector hit, strip the component
        // of `role_force` that points INTO the wall. Net effect — the
        // drone keeps its tangential intent (slides along the wall)
        // instead of being braked head-on. Only consider hits within
        // `PEER_BUBBLE_RADIUS_M` so distant walls don't constrain
        // long-range planning.
        let mut role_force = role_force;
        for (i, ep) in detector.endpoints.iter().enumerate() {
            if !detector.is_hit.get(i).copied().unwrap_or(false) {
                continue;
            }
            let to_drone = pos - *ep;
            let d = to_drone.length();
            if d < 1e-3 || d > PEER_BUBBLE_RADIUS_M {
                continue;
            }
            let n = to_drone / d;
            let inward = role_force.dot(n);
            if inward < 0.0 {
                role_force -= n * inward;
            }
        }

        // Cap terrain magnitude so even if projection misses something
        // (rare edge case — multiple obstacles, weird normals), the
        // role force can still overpower and squirt the drone past.
        let terrain = terrain.clamp_length_max(cruise * 0.8);

        let role_capped = role_force.clamp_length_max(cruise);
        let avoid_cap = (cruise * 1.5).max(2.0);
        let avoid_combined = (separation + terrain).clamp_length_max(avoid_cap);
        let total = role_capped + avoid_combined;
        desired.0 = total.clamp_length_max((cruise + avoid_cap).max(2.0));
    }
}

/// Forget peer ghosts older than this many seconds. Anchors don't
/// trust position estimates more than a few seconds stale — comms
/// range is the spec, and the swarm moves fast enough that 5 s is
/// already several body-lengths of drift.
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
