use bevy::math::{IVec3, UVec3};

pub const DEFAULT_WORLD_DIMS: UVec3 = UVec3::new(640, 24, 640);
pub const DEFAULT_VOXEL_SIZE_METERS: f32 = 1.0;

pub const FLOOR_Y: i32 = 0;

// Obstacle clusters scattered across the larger world. Horizontal coords
// are ~10x the original tiny-world layout; Y stays bounded by the
// 24-cell height.
pub const CLUSTER_A_LO: IVec3 = IVec3::new(80, 1, 80);
pub const CLUSTER_A_HI: IVec3 = IVec3::new(140, 6, 140);

pub const CLUSTER_B_LO: IVec3 = IVec3::new(440, 1, 440);
pub const CLUSTER_B_HI: IVec3 = IVec3::new(500, 18, 500);

pub const CLUSTER_C_LO: IVec3 = IVec3::new(280, 1, 180);
pub const CLUSTER_C_HI: IVec3 = IVec3::new(380, 5, 260);

pub const CLUSTER_D_LO: IVec3 = IVec3::new(120, 1, 380);
pub const CLUSTER_D_HI: IVec3 = IVec3::new(180, 12, 440);

pub const CLUSTER_E_LO: IVec3 = IVec3::new(480, 1, 80);
pub const CLUSTER_E_HI: IVec3 = IVec3::new(560, 4, 140);

pub const CLUSTER_F_LO: IVec3 = IVec3::new(200, 1, 520);
pub const CLUSTER_F_HI: IVec3 = IVec3::new(400, 3, 560);
