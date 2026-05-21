//! Active anchor planner.
//!
//! Reads the comms BFS parent tree (`CommsState.bfs_parent`) + drone
//! positions, builds the list of parent→child edges, filters to edges
//! whose length exceeds a fraction of the comms range, and assigns
//! each Anchor to the midpoint of the longest unclaimed stretched
//! edge nearest to it. Worst gaps are repaired first.
//!
//! Where the previous critical-radius spring was a passive drag, this
//! is an active goal that pulls anchors into the gaps between peers
//! before the chain breaks.

use bevy::platform::collections::HashMap;
use bevy::prelude::*;

use crate::comms::{CommsSettings, CommsState, PARENT_BASE, PARENT_NONE};
use crate::drone::{Drone, DroneId, MAX_DRONE_COUNT};

use super::role::Role;

/// Fraction of the comms range at which an edge counts as "stretched"
/// and needs a relay. 0.8 means an edge longer than 80% of the comms
/// range is a candidate for repair.
pub const STRETCH_THRESHOLD_FRAC: f32 = 0.8;

/// Per-anchor target position. Anchors not in the map fall back to the
/// existing critical-peer / median / base logic.
#[derive(Resource, Default, Debug)]
pub struct AnchorAssignments {
    pub targets: HashMap<u32, Vec3>,
}

pub struct AnchorPlannerPlugin;

impl Plugin for AnchorPlannerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AnchorAssignments>()
            .add_systems(Update, assign_anchor_targets);
    }
}

#[derive(Clone, Copy, Debug)]
struct StretchedEdge {
    midpoint: Vec3,
    length: f32,
}

fn assign_anchor_targets(
    settings: Res<CommsSettings>,
    state: Res<CommsState>,
    drones: Query<(&DroneId, &Transform, &Role), With<Drone>>,
    mut assignments: ResMut<AnchorAssignments>,
) {
    assignments.targets.clear();
    if !settings.enabled {
        return;
    }
    let stretch_min = settings.range_m * STRETCH_THRESHOLD_FRAC;
    let stretch_min_sq = stretch_min * stretch_min;

    // Snapshot positions by id for O(1) parent lookup.
    let mut pos_by_id: [Option<Vec3>; MAX_DRONE_COUNT as usize] =
        [None; MAX_DRONE_COUNT as usize];
    let mut anchors: Vec<(u32, Vec3)> = Vec::new();
    for (id, transform, role) in &drones {
        let i = id.0 as usize;
        if i < pos_by_id.len() {
            pos_by_id[i] = Some(transform.translation);
        }
        if matches!(role, Role::Anchor) {
            anchors.push((id.0, transform.translation));
        }
    }
    if anchors.is_empty() {
        return;
    }

    // Walk every connected drone, compose its tree edge, keep edges
    // whose length crosses the stretched threshold.
    let mut stretched: Vec<StretchedEdge> = Vec::new();
    for i in 0..MAX_DRONE_COUNT as usize {
        let Some(child_pos) = pos_by_id[i] else { continue };
        let parent = state.bfs_parent[i];
        if parent == PARENT_NONE {
            continue;
        }
        let parent_pos = if parent == PARENT_BASE {
            state.base_pos
        } else {
            let pi = parent as usize;
            if pi >= pos_by_id.len() {
                continue;
            }
            let Some(p) = pos_by_id[pi] else { continue };
            p
        };
        let delta = child_pos - parent_pos;
        let len_sq = delta.length_squared();
        if len_sq <= stretch_min_sq {
            continue;
        }
        stretched.push(StretchedEdge {
            midpoint: (child_pos + parent_pos) * 0.5,
            length: len_sq.sqrt(),
        });
    }
    if stretched.is_empty() {
        return;
    }

    // Worst-first: repair the longest gaps before borderline ones, so
    // a single anchor isn't wasted on a marginal edge when the chain
    // has a real break elsewhere.
    stretched.sort_by(|a, b| b.length.partial_cmp(&a.length).unwrap_or(std::cmp::Ordering::Equal));

    let mut used: Vec<bool> = vec![false; anchors.len()];
    for edge in stretched.iter() {
        let mut best: Option<(usize, f32)> = None;
        for (ai, (_id, apos)) in anchors.iter().enumerate() {
            if used[ai] {
                continue;
            }
            let d = apos.distance_squared(edge.midpoint);
            match best {
                Some((_, bd)) if d >= bd => {}
                _ => best = Some((ai, d)),
            }
        }
        let Some((ai, _)) = best else { break };
        used[ai] = true;
        assignments.targets.insert(anchors[ai].0, edge.midpoint);
        if used.iter().all(|&u| u) {
            break;
        }
    }
}
