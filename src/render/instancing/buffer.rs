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
}

pub fn prepare_instance_buffers(
    mut commands: Commands,
    layers_q: Query<(Entity, &InstancedVoxelLayer, Option<&InstanceBuffer>)>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    for (entity, layer, existing) in &layers_q {
        if layer.0.is_empty() {
            commands.entity(entity).remove::<InstanceBuffer>();
            continue;
        }

        let bytes: &[u8] = bytemuck::cast_slice::<InstanceData, u8>(layer.0.as_slice());
        let needed_capacity = bytes.len() as u64;

        let (buffer, capacity_bytes) = match existing {
            Some(buf) if buf.capacity_bytes >= needed_capacity => {
                render_queue.write_buffer(&buf.buffer, 0, bytes);
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
            length: layer.0.len(),
            capacity_bytes,
        });
    }
}

fn grow_capacity(needed: u64) -> u64 {
    needed.saturating_mul(3) / 2
}
