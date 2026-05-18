use bevy::prelude::*;
use bytemuck::{Pod, Zeroable};

/// One GPU instance: cube center + uniform scale, plus linear RGBA color.
/// Packed for direct upload into a vertex buffer with `Float32x4 + Float32x4`.
#[derive(Clone, Copy, Pod, Zeroable, Debug)]
#[repr(C)]
pub struct InstanceData {
    pub pos_scale: [f32; 4],
    pub color: [f32; 4],
}

/// Holds the per-instance buffer source for one rendered voxel layer
/// (ground truth, global map, or all local maps aggregated).
#[derive(Component, Clone, Deref)]
pub struct InstancedVoxelLayer(pub Vec<InstanceData>);
