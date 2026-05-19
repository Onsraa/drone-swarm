use bevy::prelude::*;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_graph::{
    self, NodeRunError, RenderGraph, RenderGraphContext, RenderLabel,
};
use bevy::render::render_resource::{
    binding_types::{storage_buffer, storage_buffer_read_only},
    BindGroup, BindGroupEntries, BindGroupLayoutDescriptor, BindGroupLayoutEntries,
    CachedComputePipelineId, ComputePassDescriptor, ComputePipelineDescriptor, PipelineCache,
    ShaderStages,
};
use bevy::render::renderer::{RenderContext, RenderDevice};
use bevy::render::storage::GpuShaderStorageBuffer;

use super::merge_pass::MergeGlobalNodeLabel;
use super::resources::{
    BuildLocalParams, BuildLocalParamsBuffer, GlobalInstanceCountBuffer, GlobalInstanceVecBuffer,
    GlobalOccupancyBuffer,
};

const SHADER_ASSET_PATH: &str = "shaders/build_global_instances.wgsl";

#[derive(Resource)]
pub struct BuildGlobalPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_build_global_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "build global instances layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // 0: global occupancy SSBO (read)
                storage_buffer_read_only::<Vec<u32>>(false),
                // 1: build params
                storage_buffer_read_only::<BuildLocalParams>(false),
                // 2: instance counter
                storage_buffer::<Vec<u32>>(false),
                // 3: instance buffer (pos_scale + color pairs)
                storage_buffer::<Vec<Vec4>>(false),
            ),
        ),
    );
    let shader: Handle<Shader> = asset_server.load(SHADER_ASSET_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("build global instances".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("build_global".into()),
        ..default()
    });
    commands.insert_resource(BuildGlobalPipeline { layout, pipeline });
}

#[derive(Resource)]
pub struct BuildGlobalBindGroup(pub BindGroup);

pub fn prepare_build_global_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<BuildGlobalPipeline>,
    pipeline_cache: Res<PipelineCache>,
    occupancy: Res<GlobalOccupancyBuffer>,
    params: Res<BuildLocalParamsBuffer>,
    count: Res<GlobalInstanceCountBuffer>,
    instances: Res<GlobalInstanceVecBuffer>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let Some(occupancy_buf) = buffers.get(&occupancy.0) else { return; };
    let Some(params_buf) = buffers.get(&params.0) else { return; };
    let Some(count_buf) = buffers.get(&count.0) else { return; };
    let Some(instances_buf) = buffers.get(&instances.0) else { return; };

    let bind_group = render_device.create_bind_group(
        "build global instances bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            occupancy_buf.buffer.as_entire_buffer_binding(),
            params_buf.buffer.as_entire_buffer_binding(),
            count_buf.buffer.as_entire_buffer_binding(),
            instances_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(BuildGlobalBindGroup(bind_group));
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
pub struct BuildGlobalNodeLabel;

#[derive(Default)]
pub struct BuildGlobalNode;

impl render_graph::Node for BuildGlobalNode {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let pipeline_cache = world.resource::<PipelineCache>();
        let pipeline = world.resource::<BuildGlobalPipeline>();
        let Some(bind_group) = world.get_resource::<BuildGlobalBindGroup>() else {
            return Ok(());
        };
        let Some(count_handle) = world.get_resource::<GlobalInstanceCountBuffer>() else {
            return Ok(());
        };
        let Some(buffers) = world.get_resource::<RenderAssets<GpuShaderStorageBuffer>>() else {
            return Ok(());
        };
        let Some(count_buf) = buffers.get(&count_handle.0) else {
            return Ok(());
        };
        let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
            return Ok(());
        };

        let dims = crate::world::WorldConfig::default().size;
        let cells_per_drone = dims.x * dims.y * dims.z;
        let groups_x = cells_per_drone.div_ceil(256);

        let encoder = render_context.command_encoder();
        encoder.clear_buffer(&count_buf.buffer, 0, None);
        {
            let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
                label: Some("build global instances pass"),
                ..default()
            });
            pass.set_bind_group(0, &bind_group.0, &[]);
            pass.set_pipeline(compute_pipeline);
            pass.dispatch_workgroups(groups_x, 1, 1);
        }
        Ok(())
    }
}

pub fn add_build_global_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(BuildGlobalNodeLabel, BuildGlobalNode);
    render_graph.add_node_edge(MergeGlobalNodeLabel, BuildGlobalNodeLabel);
}
