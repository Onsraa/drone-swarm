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

impl RoleParams {
    pub fn for_role(role: Role) -> Self {
        match role {
            Role::Scout => Self {
                cruise_speed_mps: 8.0,
                cone_half_angle_deg: 15.0,
                max_range_cells: 160,
                rays_per_scan: 32,
                scan_interval_frames: 2,
                info_weight: 1.0,
                distance_weight: 0.3,
                distance_bias: 1.0,
                crowding_weight: 0.5,
                avoid_k: 4.0,
                tint: [1.0, 0.85, 0.2, 0.85],
            },
            Role::Mapper => Self {
                cruise_speed_mps: 3.0,
                cone_half_angle_deg: 90.0,
                max_range_cells: 64,
                rays_per_scan: 128,
                scan_interval_frames: 1,
                info_weight: 1.5,
                distance_weight: 1.0,
                distance_bias: 1.0,
                crowding_weight: 1.5,
                avoid_k: 6.0,
                tint: [0.3, 0.8, 0.35, 0.85],
            },
            Role::Anchor => Self {
                cruise_speed_mps: 0.0,
                cone_half_angle_deg: 180.0,
                max_range_cells: 128,
                rays_per_scan: 64,
                scan_interval_frames: 3,
                info_weight: 0.0,
                distance_weight: 0.0,
                distance_bias: 1.0,
                crowding_weight: 0.0,
                avoid_k: 10.0,
                tint: [0.92, 0.95, 1.0, 0.85],
            },
        }
    }
}
