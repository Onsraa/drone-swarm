use bevy::prelude::*;
use std::time::Duration;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneId};
use crate::world::WorldConfig;

use super::resources::FrontierClusters;
use super::role::Role;

#[derive(Debug, Clone, Copy)]
pub struct SwarmTelemetry {
    pub total_drones: u32,
    pub comms_components: u32,
    pub comms_density: f32,
    pub farthest_frontier_m: f32,
    pub known_free_ratio: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct TargetRatio {
    pub scouts: f32,
    pub mappers: f32,
    pub anchors: f32,
}

/// Decide the target role ratio for the current swarm telemetry.
/// Sums to 1.0 after normalisation.
pub fn decide_ratio(t: &SwarmTelemetry) -> TargetRatio {
    let mut scouts = 0.6;
    let mut mappers = 0.3;
    let mut anchors = 0.1;
    if t.comms_components >= 2 {
        anchors += 0.1;
    }
    if t.comms_density < 0.4 {
        anchors += 0.05;
    }
    if t.farthest_frontier_m > 300.0 {
        scouts += 0.1;
    }
    if t.known_free_ratio > 0.7 {
        mappers += 0.1;
    }
    let sum = scouts + mappers + anchors;
    TargetRatio {
        scouts: scouts / sum,
        mappers: mappers / sum,
        anchors: anchors / sum,
    }
}

pub fn role_for_ratio(idx: u32, total: u32, ratio: TargetRatio) -> Role {
    let scout_cutoff = (total as f32 * ratio.scouts).round() as u32;
    let mapper_cutoff = scout_cutoff + (total as f32 * ratio.mappers).round() as u32;
    if idx < scout_cutoff {
        Role::Scout
    } else if idx < mapper_cutoff {
        Role::Mapper
    } else {
        Role::Anchor
    }
}

#[derive(Resource, Default)]
pub struct SupervisorTimer(pub Timer);

impl SupervisorTimer {
    pub fn new() -> Self {
        Self(Timer::new(Duration::from_millis(2000), TimerMode::Repeating))
    }
}

#[derive(Component, Default)]
pub struct LastRoleChange(pub f32);

pub fn supervisor_tick(
    time: Res<Time>,
    mut timer: ResMut<SupervisorTimer>,
    comms: Res<CommsState>,
    clusters: Res<FrontierClusters>,
    world: Res<WorldConfig>,
    mut drones: Query<(&DroneId, &Transform, &mut Role, &mut LastRoleChange), With<Drone>>,
) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let now = time.elapsed_secs();

    let total = drones.iter().count() as u32;
    if total == 0 {
        return;
    }

    // Telemetry estimate.
    let comms_components = if comms.total_count == 0 || comms.connected_count == comms.total_count {
        1
    } else {
        2
    };
    let comms_density = if total <= 1 {
        1.0
    } else {
        comms.connected_count as f32 / total as f32
    };
    let farthest_frontier_m = clusters
        .entries
        .iter()
        .map(|c| c.centroid.distance(world.center()))
        .fold(0.0_f32, f32::max);
    let known_free_ratio = 0.3; // Placeholder until coverage telemetry wired.

    let ratio = decide_ratio(&SwarmTelemetry {
        total_drones: total,
        comms_components,
        comms_density,
        farthest_frontier_m,
        known_free_ratio,
    });

    // Snapshot ids immutably, then apply mutations in a separate pass.
    let ids: Vec<u32> = {
        let mut v: Vec<u32> = drones.iter().map(|(id, _, _, _)| id.0).collect();
        v.sort_unstable();
        v
    };

    for (i, &target_id) in ids.iter().enumerate() {
        let desired = role_for_ratio(i as u32, total, ratio);
        for (id, _, mut role, mut last_change) in drones.iter_mut() {
            if id.0 != target_id {
                continue;
            }
            if *role == desired {
                break;
            }
            if now - last_change.0 < 5.0 {
                break; // smoothing window
            }
            *role = desired;
            last_change.0 = now;
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> SwarmTelemetry {
        SwarmTelemetry {
            total_drones: 10,
            comms_components: 1,
            comms_density: 0.8,
            farthest_frontier_m: 100.0,
            known_free_ratio: 0.3,
        }
    }

    #[test]
    fn default_ratio_when_healthy() {
        let r = decide_ratio(&base());
        assert!((r.scouts - 0.6).abs() < 0.01);
        assert!((r.mappers - 0.3).abs() < 0.01);
        assert!((r.anchors - 0.1).abs() < 0.01);
    }

    #[test]
    fn fragmented_comms_bumps_anchors() {
        let mut t = base();
        t.comms_components = 2;
        let r = decide_ratio(&t);
        assert!(r.anchors > 0.1);
    }

    #[test]
    fn distant_frontier_bumps_scouts() {
        let mut t = base();
        t.farthest_frontier_m = 500.0;
        let r = decide_ratio(&t);
        assert!(r.scouts > 0.6);
    }

    #[test]
    fn well_explored_bumps_mappers() {
        let mut t = base();
        t.known_free_ratio = 0.8;
        let r = decide_ratio(&t);
        assert!(r.mappers > 0.3);
    }
}
