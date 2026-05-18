use bevy::math::{IVec3, UVec3};

pub const DEFAULT_WORLD_DIMS: UVec3 = UVec3::new(32, 16, 32);
pub const DEFAULT_VOXEL_SIZE_METERS: f32 = 1.0;

pub const FLOOR_Y: i32 = 0;
pub const CLUSTER_A_LO: IVec3 = IVec3::new(6, 1, 6);
pub const CLUSTER_A_HI: IVec3 = IVec3::new(10, 6, 10);
pub const CLUSTER_B_LO: IVec3 = IVec3::new(22, 1, 22);
pub const CLUSTER_B_HI: IVec3 = IVec3::new(26, 12, 26);
pub const CLUSTER_C_LO: IVec3 = IVec3::new(16, 1, 14);
pub const CLUSTER_C_HI: IVec3 = IVec3::new(22, 4, 18);
