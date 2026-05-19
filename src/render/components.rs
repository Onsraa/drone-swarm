use bevy::prelude::*;

#[derive(Component)]
pub struct GroundTruthVoxel;

#[derive(Component)]
pub struct GlobalMapVoxel;

/// Marker for the single render entity whose vertex buffer is the
/// GPU-built local-map instance buffer.
#[derive(Component)]
pub struct GpuLocalMapVoxel;

/// Marker for the single render entity backed by the GPU-built central
/// (global) map instance buffer.
#[derive(Component)]
pub struct GpuGlobalMapVoxel;
