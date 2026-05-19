use bevy::color::palettes::tailwind;
use bevy::prelude::*;

use crate::drone::{Drone, DroneId};
use crate::world::WorldConfig;

use super::constants::BASE_DEFAULT_HEIGHT_M;
use super::resources::{CommsSettings, CommsState};

/// BFS over the drone positions. Seed with drones within `range_m` of
/// the base, then walk the peer graph (each edge = pair of drones within
/// `range_m` of each other) to mark every reachable drone. Output is a
/// 64-bit mask written to `CommsState.connected_mask`.
pub fn compute_connectivity(
    settings: Res<CommsSettings>,
    world: Res<WorldConfig>,
    drones: Query<(&DroneId, &Transform), With<Drone>>,
    mut state: ResMut<CommsState>,
) {
    let center = world.center();
    let base_pos = Vec3::new(center.x, BASE_DEFAULT_HEIGHT_M, center.z);
    state.base_pos = base_pos;
    state.total_count = drones.iter().count();

    if !settings.enabled {
        state.connected_mask = [u32::MAX, u32::MAX];
        state.connected_count = state.total_count;
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
    let mut frontier: Vec<usize> = Vec::new();

    for (i, (_, pos)) in entries.iter().enumerate() {
        if pos.distance_squared(base_pos) <= r2 {
            connected[i] = true;
            frontier.push(i);
        }
    }

    while let Some(i) = frontier.pop() {
        let pi = entries[i].1;
        for j in 0..n {
            if connected[j] {
                continue;
            }
            if pi.distance_squared(entries[j].1) <= r2 {
                connected[j] = true;
                frontier.push(j);
            }
        }
    }

    let mut mask = [0u32; 2];
    let mut count = 0usize;
    for (i, (id, _)) in entries.iter().enumerate() {
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
