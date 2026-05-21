use bevy::prelude::*;
use bevy::render::render_resource::{
    binding_types::{storage_buffer, storage_buffer_read_only},
    BindGroupLayoutDescriptor, BindGroupLayoutEntries, CachedComputePipelineId,
    ComputePipelineDescriptor, PipelineCache, ShaderStages,
};

use super::per_drone_scan::DroneScanParams;
use super::resources::LidarParams;

const BVH_SHADER_ASSET_PATH: &str = "shaders/lidar_bvh.wgsl";

/// BVH lidar compute pipeline + bind-group layout. 17 entries: every
/// per-drone / per-ray buffer the old DDA path used, plus the three
/// new BVH SSBOs (nodes, primitive indices, triangle vertices).
#[derive(Resource)]
pub struct ComputeLidarBvhPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_compute_lidar_bvh_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "compute lidar bvh layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // 0: lidar params
                storage_buffer_read_only::<LidarParams>(false),
                // 1: drone positions
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 2: ray dirs
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 3: drone orientations
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 4: per-drone local-map occupancy (atomic)
                storage_buffer::<Vec<u32>>(false),
                // 5: drone colors
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 6: lidar point counter (atomic)
                storage_buffer::<Vec<u32>>(false),
                // 7: lidar point buffer
                storage_buffer::<Vec<Vec4>>(false),
                // 8: per-drone scan params
                storage_buffer_read_only::<Vec<DroneScanParams>>(false),
                // 9: global occupancy (atomic)
                storage_buffer::<Vec<u32>>(false),
                // 10: local active cells list
                storage_buffer::<Vec<u32>>(false),
                // 11: local active count (atomic)
                storage_buffer::<Vec<u32>>(false),
                // 12: global active cells list
                storage_buffer::<Vec<u32>>(false),
                // 13: global active count (atomic)
                storage_buffer::<Vec<u32>>(false),
                // 14: CWBVH8 nodes (20 × u32 per node, bytemuck-cast).
                storage_buffer_read_only::<Vec<u32>>(false),
                // 15: primitive indices.
                storage_buffer_read_only::<Vec<u32>>(false),
                // 16: unindexed triangle vertex positions (3 × vec4 per
                // triangle).
                storage_buffer_read_only::<Vec<Vec4>>(false),
            ),
        ),
    );
    let shader: Handle<Shader> = asset_server.load(BVH_SHADER_ASSET_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("compute lidar bvh".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("lidar_bvh".into()),
        ..default()
    });
    commands.insert_resource(ComputeLidarBvhPipeline { layout, pipeline });
}
