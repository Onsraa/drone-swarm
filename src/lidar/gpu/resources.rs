use bevy::prelude::*;
use bevy::render::render_resource::{Buffer, BufferUsages};
use bevy::render::renderer::RenderDevice;
use bevy::render::Extract;

use crate::world::GroundTruthMap;

/// Wrapper around the GPU storage buffer holding the packed ground-truth
/// bitset. Held in the render world. Lives in a `Slot` so the upload can
/// happen lazily on the first frame the main-world `GroundTruthMap`
/// resource is observed.
///
/// Fields are read by the compute pipeline once it lands; suppress the
/// dead-code warning until then.
#[allow(dead_code)]
pub struct GroundTruthGpu {
    pub buffer: Buffer,
    pub dims: UVec3,
    pub bitset_words: u32,
}

#[derive(Resource, Default)]
pub struct GroundTruthGpuSlot {
    pub gpu: Option<GroundTruthGpu>,
}

pub fn ensure_ground_truth_gpu(
    mut slot: ResMut<GroundTruthGpuSlot>,
    render_device: Res<RenderDevice>,
    ground: Extract<Option<Res<GroundTruthMap>>>,
) {
    if slot.gpu.is_some() {
        return;
    }
    let Some(ground) = ground.as_deref() else {
        return;
    };
    let bitset = ground.pack_bitset();
    let bytes: &[u8] = bytemuck::cast_slice(&bitset);
    let buffer = render_device.create_buffer_with_data(&bevy::render::render_resource::BufferInitDescriptor {
        label: Some("ground truth bitset"),
        contents: bytes,
        usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
    });
    info!(
        "uploaded ground-truth bitset to GPU: {} bytes, {} words for dims {:?}",
        bytes.len(),
        bitset.len(),
        ground.dims
    );
    slot.gpu = Some(GroundTruthGpu {
        buffer,
        dims: ground.dims,
        bitset_words: bitset.len() as u32,
    });
}
