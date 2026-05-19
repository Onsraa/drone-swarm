use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::BufferUsages;
use bevy::render::storage::ShaderStorageBuffer;

use crate::world::GroundTruthMap;

/// Handle to the storage buffer holding the packed ground-truth bitset.
/// Created once at startup; mirrored into the render world via
/// `ExtractResourcePlugin`.
#[derive(Resource, ExtractResource, Clone)]
pub struct GroundTruthBuffer(pub Handle<ShaderStorageBuffer>);

/// Handle to the Stage-2 sanity output buffer (single `u32` = number of
/// occupied cells the compute shader counted). Replaced in later stages by
/// the per-(drone, ray) hit buffer.
#[derive(Resource, ExtractResource, Clone)]
pub struct LidarCountBuffer(pub Handle<ShaderStorageBuffer>);

/// One-shot startup system: packs the CPU ground truth to a `u32` bitset,
/// uploads it as a `ShaderStorageBuffer` asset, and creates the matching
/// output buffer. Both handles are inserted as resources so the render
/// world can pick them up via `ExtractResource`.
pub fn upload_ground_truth_to_gpu(
    mut commands: Commands,
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    ground: Res<GroundTruthMap>,
) {
    let bitset = ground.pack_bitset();
    let bitset_len = bitset.len();
    let mut ground_buffer = ShaderStorageBuffer::from(bitset);
    ground_buffer.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let ground_handle = buffers.add(ground_buffer);

    let mut count_buffer = ShaderStorageBuffer::from(vec![0u32]);
    count_buffer.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let count_handle = buffers.add(count_buffer);

    info!(
        "uploaded ground-truth bitset to GPU: {} words for dims {:?}",
        bitset_len, ground.dims
    );

    commands.insert_resource(GroundTruthBuffer(ground_handle));
    commands.insert_resource(LidarCountBuffer(count_handle));
}
