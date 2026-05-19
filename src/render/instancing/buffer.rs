use std::mem::size_of;

use bevy::prelude::*;
use bevy::render::render_resource::{Buffer, BufferDescriptor, BufferUsages};
use bevy::render::renderer::{RenderDevice, RenderQueue};

use super::components::{InstanceData, InstancedVoxelLayer};

#[derive(Component)]
pub struct InstanceBuffer {
    pub buffer: Buffer,
    pub length: usize,
    /// Allocated GPU capacity in bytes. We grow with a 1.5x strategy so
    /// steady-state instance counts don't pay for a reallocation each
    /// frame; only crossing the high-water mark allocates.
    pub capacity_bytes: u64,
    /// Generation of the source layer the GPU contents currently reflect.
    /// Matches `InstancedVoxelLayer::generation`. When matched, additional
    /// data is uploaded as a tail-only append; on mismatch we re-upload
    /// from offset 0.
    pub last_generation: u32,
}

pub fn prepare_instance_buffers(
    mut commands: Commands,
    layers_q: Query<(Entity, &InstancedVoxelLayer, Option<&InstanceBuffer>)>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    let stride = size_of::<InstanceData>() as u64;
    for (entity, layer, existing) in &layers_q {
        if layer.data.is_empty() {
            commands.entity(entity).remove::<InstanceBuffer>();
            continue;
        }

        let bytes: &[u8] = bytemuck::cast_slice::<InstanceData, u8>(layer.data.as_slice());
        let needed_capacity = bytes.len() as u64;
        let new_length = layer.data.len();
        let new_generation = layer.generation;

        let (buffer, capacity_bytes) = match existing {
            Some(buf) if buf.capacity_bytes >= needed_capacity => {
                if buf.last_generation == new_generation {
                    if new_length > buf.length {
                        let tail_offset = (buf.length as u64) * stride;
                        let tail_start = buf.length * size_of::<InstanceData>();
                        render_queue.write_buffer(&buf.buffer, tail_offset, &bytes[tail_start..]);
                    }
                    // new_length == buf.length: no-op. new_length < buf.length only
                    // happens with the same generation if someone mutated the layer
                    // in a non-append way; the shorter draw range hides any stale
                    // tail data and the next gen bump will rewrite it.
                } else {
                    render_queue.write_buffer(&buf.buffer, 0, bytes);
                }
                (buf.buffer.clone(), buf.capacity_bytes)
            }
            _ => {
                let new_capacity = grow_capacity(needed_capacity);
                let new_buffer = render_device.create_buffer(&BufferDescriptor {
                    label: Some("voxel instance buffer"),
                    size: new_capacity,
                    usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                render_queue.write_buffer(&new_buffer, 0, bytes);
                (new_buffer, new_capacity)
            }
        };

        commands.entity(entity).insert(InstanceBuffer {
            buffer,
            length: new_length,
            capacity_bytes,
            last_generation: new_generation,
        });
    }
}

fn grow_capacity(needed: u64) -> u64 {
    needed.saturating_mul(3) / 2
}
