use bevy::prelude::*;

/// World-space target the drone is currently flying toward. `None` means
/// no frontier assigned (cold start). Replaces the previous `frontier::FrontierTarget`.
#[derive(Component, Default, Debug)]
pub struct FrontierTarget {
    pub pos: Option<Vec3>,
    pub cluster_id: Option<u32>,
}

/// Planned waypoint sequence in world coords. Empty = no plan; `wander`
/// fallback drives the drone. Pure-pursuit consumes `waypoints` + `cursor`
/// directly; no helper methods needed.
#[derive(Component, Default, Debug)]
pub struct Path {
    pub waypoints: Vec<Vec3>,
    pub cursor: usize,
}

/// Stuck detector state per drone.
#[derive(Component, Default, Debug)]
pub struct MovementHealth {
    pub slow_secs: f32,
    pub escalations_in_window: u32,
    pub window_start_secs: f32,
}

/// Sampled past positions of a drone, drawn as a gizmo trail. Bounded
/// ring of `TRAIL_MAX_POINTS`; older samples fall off the front when
/// new ones push in. Sampled at a small interval so the line stays
/// readable at swarm scale.
#[derive(Component, Default, Debug)]
pub struct Trail {
    pub points: std::collections::VecDeque<Vec3>,
    pub last_sample_secs: f32,
}
