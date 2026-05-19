use bevy::prelude::*;

#[derive(Component)]
pub struct GroundTruthVoxel;

#[derive(Component)]
pub struct LocalMapVoxel;

#[derive(Component)]
pub struct GlobalMapVoxel;

/// Marker for the single render entity whose vertex buffer is the GPU-
/// built local-map instance buffer. Stage 9C of Tier 3 #9; the CPU
/// `LocalMapVoxel` path is bypassed when this entity exists.
#[derive(Component)]
pub struct GpuLocalMapVoxel;
