use bevy::prelude::*;
use bevy::render::render_resource::{
    binding_types::{storage_buffer, storage_buffer_read_only},
    BindGroupLayoutDescriptor, BindGroupLayoutEntries, CachedComputePipelineId,
    ComputePipelineDescriptor, PipelineCache, ShaderStages,
};

use super::per_drone_scan::DroneScanParams;
use super::resources::LidarParams;

const SHADER_ASSET_PATH: &str = "shaders/lidar_compute.wgsl";

#[derive(Resource)]
pub struct ComputeLidarPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_compute_lidar_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "compute lidar layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // 0: ground-truth bitset
                storage_buffer_read_only::<Vec<u32>>(false),
                // 1: lidar params (single struct)
                storage_buffer_read_only::<LidarParams>(false),
                // 2: drone positions (Vec<Vec4>)
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 3: ray dirs (Vec<Vec4>)
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 4: drone orientations (Vec<Vec4>, quaternion xyzw)
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 5: per-drone local-map occupancy (2 bits/cell, atomic)
                storage_buffer::<Vec<u32>>(false),
                // 6: drone colors (Vec<Vec4>) for tinting emitted points
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 7: lidar point counter (atomic u32)
                storage_buffer::<Vec<u32>>(false),
                // 8: lidar point buffer (Vec<Vec4> pairs: pos_scale + color)
                storage_buffer::<Vec<Vec4>>(false),
                // 9: per-drone scan params (read)
                storage_buffer_read_only::<Vec<DroneScanParams>>(false),
                // 10: global occupancy bitset (atomic); lidar writes
                // here directly (comms-gated) so merge_global is
                // retired.
                storage_buffer::<Vec<u32>>(false),
                // 11: per-drone active-cell list (cell flat-indices
                // appended on first Unknown->Occupied transition).
                storage_buffer::<Vec<u32>>(false),
                // 12: per-drone active-cell count (atomic).
                storage_buffer::<Vec<u32>>(false),
                // 13: global active-cell list.
                storage_buffer::<Vec<u32>>(false),
                // 14: global active-cell count (atomic).
                storage_buffer::<Vec<u32>>(false),
            ),
        ),
    );
    let shader: Handle<Shader> = asset_server.load(SHADER_ASSET_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("compute lidar".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("lidar".into()),
        ..default()
    });
    commands.insert_resource(ComputeLidarPipeline { layout, pipeline });
}
