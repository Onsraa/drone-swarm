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
///
/// `generation` bumps whenever `data` is rewritten in place (`replace`).
/// Append-only growth (`append`) keeps the generation stable so the GPU
/// uploader can stream the new tail instead of re-uploading from offset 0.
#[derive(Component, Clone)]
pub struct InstancedVoxelLayer {
    pub data: Vec<InstanceData>,
    pub generation: u32,
}

impl InstancedVoxelLayer {
    pub fn new(data: Vec<InstanceData>) -> Self {
        Self { data, generation: 1 }
    }

    /// Full rewrite. Bumps generation so the GPU buffer re-uploads from 0.
    pub fn replace(&mut self, data: Vec<InstanceData>) {
        self.data = data;
        self.generation = self.generation.wrapping_add(1);
    }

    /// Append-only. Generation unchanged — uploader streams the tail.
    pub fn append<I: IntoIterator<Item = InstanceData>>(&mut self, items: I) {
        self.data.extend(items);
    }
}
