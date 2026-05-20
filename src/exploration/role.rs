use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Scout,
    Mapper,
    Anchor,
}

impl Default for Role {
    fn default() -> Self {
        Role::Scout
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RoleParams {
    pub cruise_speed_mps: f32,
    pub cone_half_angle_deg: f32,
    pub max_range_cells: u32,
    pub rays_per_scan: u32,
    pub scan_interval_frames: u32,
    pub info_weight: f32,
    pub distance_weight: f32,
    pub distance_bias: f32,
    pub crowding_weight: f32,
    pub avoid_k: f32,
    pub tint: [f32; 4], // linear RGBA before alpha
}

/// Per-pair peer-repulsion stiffness. `peer_repulsion_for(self, peer)`
/// = "how strongly a drone of role `self` is pushed by a peer of role
/// `peer`". Asymmetric: Scout barely cares about Mapper bubbles (k=1)
/// but Mapper yields hard to incoming Scouts (k=28). Anchors don't
/// move so their value is 0 across the row.
pub fn peer_repulsion_for(self_role: Role, peer_role: Role) -> f32 {
    match (self_role, peer_role) {
        (Role::Scout, Role::Scout) => 8.0,
        (Role::Scout, Role::Mapper) => 1.0,
        (Role::Scout, Role::Anchor) => 2.0,
        (Role::Mapper, Role::Scout) => 28.0,
        (Role::Mapper, Role::Mapper) => 12.0,
        (Role::Mapper, Role::Anchor) => 6.0,
        (Role::Anchor, _) => 0.0,
    }
}

impl RoleParams {
    pub fn for_role(role: Role) -> Self {
        match role {
            Role::Scout => Self {
                cruise_speed_mps: 15.0,
                cone_half_angle_deg: 15.0,
                max_range_cells: 160,
                rays_per_scan: 32,
                scan_interval_frames: 2,
                info_weight: 1.0,
                distance_weight: 0.3,
                distance_bias: 1.0,
                // 10x the old weight so two scouts targeting the same
                // cluster see ~half the score versus an empty alternative
                // of comparable distance. Drives target diversification.
                crowding_weight: 8.0,
                avoid_k: 12.0,
                tint: [1.0, 0.85, 0.2, 0.85],
            },
            Role::Mapper => Self {
                // Mapper is the slow, thorough scanner: 360° spherical
                // lidar (half-angle = 180° produces a full sphere in
                // `fibonacci_cone`), high ray density, scan every
                // frame. Visibly different from Scout's narrow
                // forward cone. 1 m/s = 15x slower than the Scout.
                cruise_speed_mps: 1.0,
                cone_half_angle_deg: 180.0,
                max_range_cells: 64,
                rays_per_scan: 192,
                scan_interval_frames: 1,
                info_weight: 1.5,
                distance_weight: 1.0,
                distance_bias: 1.0,
                crowding_weight: 12.0,
                avoid_k: 16.0,
                tint: [0.3, 0.8, 0.35, 0.85],
            },
            Role::Anchor => Self {
                cruise_speed_mps: 0.0,
                cone_half_angle_deg: 180.0,
                max_range_cells: 128,
                // Anchor doesn't map. `rays_per_scan = 0` makes the
                // lidar compute shader iterate zero times for this
                // role, so anchor's local + central occupancy
                // contributions are nil. Visually: no lidar rays on
                // anchors.
                rays_per_scan: 0,
                scan_interval_frames: 3,
                info_weight: 0.0,
                distance_weight: 0.0,
                distance_bias: 1.0,
                crowding_weight: 0.0,
                avoid_k: 20.0,
                tint: [0.92, 0.95, 1.0, 0.85],
            },
        }
    }
}
