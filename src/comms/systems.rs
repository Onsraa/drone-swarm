use bevy::color::palettes::tailwind;
use bevy::prelude::*;

use crate::drone::{Drone, DroneId};
use crate::world::{ground_altitude, WorldBvh, WorldConfig};

use super::constants::BASE_DEFAULT_HEIGHT_M;
use super::resources::{CommsSettings, CommsState, PARENT_BASE, PARENT_NONE};
use crate::drone::MAX_DRONE_COUNT;

const COMMS_SKY_CAST_Y: f32 = 2000.0;
const COMMS_BASE_CLEARANCE_M: f32 = 1.0;

/// BFS over the drone positions. Seed with drones within `range_m` of
/// the base, then walk the peer graph (each edge = pair of drones within
/// `range_m` of each other) to mark every reachable drone. Output is a
/// 64-bit mask written to `CommsState.connected_mask`.
pub fn compute_connectivity(
    settings: Res<CommsSettings>,
    world: Res<WorldConfig>,
    bvh: Option<Res<WorldBvh>>,
    drones: Query<(&DroneId, &Transform), With<Drone>>,
    mut state: ResMut<CommsState>,
) {
    let center = world.center();
    // Base altitude: BVH sky-cast (when the mesh ground truth is
    // built); fall back to BASE_DEFAULT_HEIGHT_M during the brief
    // startup window before the first BVH lands.
    let base_y = bvh
        .as_deref()
        .and_then(|b| ground_altitude(b, center.x, center.z, COMMS_SKY_CAST_Y))
        .map(|gy| gy + COMMS_BASE_CLEARANCE_M)
        .unwrap_or(BASE_DEFAULT_HEIGHT_M);
    let base_pos = Vec3::new(center.x, base_y, center.z);
    state.base_pos = base_pos;
    state.total_count = drones.iter().count();
    state.bfs_parent = [PARENT_NONE; MAX_DRONE_COUNT as usize];

    if !settings.enabled {
        state.connected_mask = [u32::MAX, u32::MAX];
        state.connected_count = state.total_count;
        // With comms disabled every drone is treated as connected
        // directly to the base; record that so downstream consumers
        // see a flat tree instead of a stale parent map.
        for (id, _) in &drones {
            if (id.0 as u32) < MAX_DRONE_COUNT {
                state.bfs_parent[id.0 as usize] = PARENT_BASE;
            }
        }
        return;
    }

    let r2 = settings.range_m * settings.range_m;
    let mut entries: Vec<(u32, Vec3)> = drones
        .iter()
        .map(|(id, t)| (id.0, t.translation))
        .collect();
    entries.sort_by_key(|(id, _)| *id);

    let n = entries.len();
    let mut connected = vec![false; n];
    // Parent in the BFS tree, indexed by entries[]. -3 = no parent yet
    // (workspace sentinel; rewritten as PARENT_NONE for disconnected
    // drones in the final pass below). PARENT_BASE for the seeded
    // frontier, otherwise the drone id of whoever discovered this one.
    let mut entry_parent: Vec<i16> = vec![-3; n];
    let mut frontier: Vec<usize> = Vec::new();

    for (i, (_, pos)) in entries.iter().enumerate() {
        if pos.distance_squared(base_pos) <= r2 {
            connected[i] = true;
            entry_parent[i] = PARENT_BASE;
            frontier.push(i);
        }
    }

    while let Some(i) = frontier.pop() {
        let pi = entries[i].1;
        let parent_id = entries[i].0 as i16;
        for j in 0..n {
            if connected[j] {
                continue;
            }
            if pi.distance_squared(entries[j].1) <= r2 {
                connected[j] = true;
                entry_parent[j] = parent_id;
                frontier.push(j);
            }
        }
    }

    let mut mask = [0u32; 2];
    let mut count = 0usize;
    for (i, (id, _)) in entries.iter().enumerate() {
        if (*id as u32) < MAX_DRONE_COUNT {
            state.bfs_parent[*id as usize] = if connected[i] {
                entry_parent[i]
            } else {
                PARENT_NONE
            };
        }
        if !connected[i] {
            continue;
        }
        count += 1;
        let half = (*id >= 32) as usize;
        mask[half] |= 1u32 << (id % 32);
    }
    state.connected_mask = mask;
    state.connected_count = count;
}

/// Draw the active comms graph as gizmos: green sphere at the base, a
/// line from the base to every connected drone, and a thinner line
/// between every connected peer pair within range. Disconnected drones
/// get a red ring under them to make their isolation visible.
pub fn draw_comms_gizmos(
    settings: Res<CommsSettings>,
    state: Res<CommsState>,
    drones: Query<(&DroneId, &Transform), With<Drone>>,
    mut gizmos: Gizmos,
) {
    if !settings.enabled || !settings.show_links {
        return;
    }
    let r2 = settings.range_m * settings.range_m;
    let base = state.base_pos;

    gizmos.sphere(
        Isometry3d::from_translation(base),
        2.0,
        Color::from(tailwind::EMERALD_400),
    );

    let mut connected_positions: Vec<(u32, Vec3)> = Vec::new();
    for (id, transform) in &drones {
        let half = (id.0 >= 32) as usize;
        let is_connected = (state.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        if is_connected {
            connected_positions.push((id.0, transform.translation));
            if transform.translation.distance_squared(base) <= r2 {
                gizmos.line(
                    base,
                    transform.translation,
                    Color::from(tailwind::EMERALD_300),
                );
            }
        } else {
            gizmos.circle(
                Isometry3d::from_translation(transform.translation),
                3.0,
                Color::from(tailwind::RED_400),
            );
        }
    }

    for i in 0..connected_positions.len() {
        for j in (i + 1)..connected_positions.len() {
            let a = connected_positions[i].1;
            let b = connected_positions[j].1;
            if a.distance_squared(b) <= r2 {
                gizmos.line(a, b, Color::from(tailwind::SKY_300));
            }
        }
    }
}
