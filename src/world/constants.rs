use bevy::math::{IVec3, UVec3};

pub const DEFAULT_WORLD_DIMS: UVec3 = UVec3::new(64, 24, 64);
pub const DEFAULT_VOXEL_SIZE_METERS: f32 = 1.0;

pub const FLOOR_Y: i32 = 0;

// Obstacle clusters scattered across the larger world.
pub const CLUSTER_A_LO: IVec3 = IVec3::new(8, 1, 8);
pub const CLUSTER_A_HI: IVec3 = IVec3::new(14, 6, 14);

pub const CLUSTER_B_LO: IVec3 = IVec3::new(44, 1, 44);
pub const CLUSTER_B_HI: IVec3 = IVec3::new(50, 18, 50);

pub const CLUSTER_C_LO: IVec3 = IVec3::new(28, 1, 18);
pub const CLUSTER_C_HI: IVec3 = IVec3::new(38, 5, 26);

pub const CLUSTER_D_LO: IVec3 = IVec3::new(12, 1, 38);
pub const CLUSTER_D_HI: IVec3 = IVec3::new(18, 12, 44);

pub const CLUSTER_E_LO: IVec3 = IVec3::new(48, 1, 8);
pub const CLUSTER_E_HI: IVec3 = IVec3::new(56, 4, 14);

pub const CLUSTER_F_LO: IVec3 = IVec3::new(20, 1, 52);
pub const CLUSTER_F_HI: IVec3 = IVec3::new(40, 3, 56);
