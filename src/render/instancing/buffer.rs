use bevy::prelude::*;
use bevy::render::render_resource::{BufferInitDescriptor, BufferUsages};
use bevy::render::renderer::RenderDevice;

use super::components::InstancedVoxelLayer;

#[derive(Component)]
pub struct InstanceBuffer {
    pub buffer: bevy::render::render_resource::Buffer,
    pub length: usize,
}

pub fn prepare_instance_buffers(
    mut commands: Commands,
    layers_q: Query<(Entity, &InstancedVoxelLayer)>,
    render_device: Res<RenderDevice>,
) {
    for (entity, layer) in &layers_q {
        if layer.0.is_empty() {
            commands.entity(entity).remove::<InstanceBuffer>();
            continue;
        }
        let buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("voxel instance buffer"),
            contents: bytemuck::cast_slice(layer.0.as_slice()),
            usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
        });
        commands.entity(entity).insert(InstanceBuffer {
            buffer,
            length: layer.0.len(),
        });
    }
}
