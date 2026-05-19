use bevy::prelude::*;

#[derive(Component)]
pub struct GroundTruthVoxel;

/// Marker for the single render entity whose vertex buffer is the
/// GPU-built local-map instance buffer.
#[derive(Component)]
pub struct GpuLocalMapVoxel;

/// Marker for the single render entity backed by the GPU-built central
/// (global) map instance buffer.
#[derive(Component)]
pub struct GpuGlobalMapVoxel;

/// Marker for the per-frame lidar point-spray render entity. The vertex
/// buffer comes from `LidarPointVecBuffer` and the instance count from
/// a Readback over `LidarPointCountBuffer`.
#[derive(Component)]
pub struct LidarPointVoxel;
