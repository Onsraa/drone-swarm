use bevy::prelude::*;
use bevy::render::render_resource::{
    binding_types::{storage_buffer, storage_buffer_read_only},
    BindGroupLayoutDescriptor, BindGroupLayoutEntries, CachedComputePipelineId,
    ComputePipelineDescriptor, PipelineCache, ShaderStages,
};

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
                // 4: hits output
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
