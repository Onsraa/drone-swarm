use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{BufferUsages, ShaderType};
use bevy::render::storage::ShaderStorageBuffer;

use super::resources::MAX_DRONES_GPU;

#[derive(ShaderType, Clone, Copy, Debug, Default)]
pub struct DroneScanParams {
    pub ray_offset: u32,
    pub ray_count: u32,
    pub max_steps: u32,
    pub scan_interval: u32,
}

#[derive(Resource, ExtractResource, Clone)]
pub struct DroneScanParamsBuffer(pub Handle<ShaderStorageBuffer>);

pub fn allocate_buffer(
    buffers: &mut Assets<ShaderStorageBuffer>,
) -> Handle<ShaderStorageBuffer> {
    let init: Vec<DroneScanParams> = vec![DroneScanParams::default(); MAX_DRONES_GPU as usize];
    let mut buf = ShaderStorageBuffer::from(init);
    buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    buffers.add(buf)
}
