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

use super::dispatch::ComputeLidarNodeLabel;
use super::resources::{BuildIndirectBuffer, GlobalActiveCountBuffer, LocalActiveCountBuffer};

const SHADER_ASSET_PATH: &str = "shaders/prepare_build_indirect.wgsl";

#[derive(Resource)]
pub struct PrepareBuildIndirectPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_prepare_build_indirect_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "prepare build indirect layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // 0: per-drone local active-cell counts (atomic read).
                storage_buffer_read_only::<Vec<u32>>(false),
                // 1: global active-cell count (atomic read).
                storage_buffer_read_only::<Vec<u32>>(false),
                // 2: build dispatch indirect args (write).
                storage_buffer::<Vec<u32>>(false),
            ),
        ),
    );
    let shader: Handle<Shader> = asset_server.load(SHADER_ASSET_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("prepare build indirect".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("prepare".into()),
        ..default()
    });
    commands.insert_resource(PrepareBuildIndirectPipeline { layout, pipeline });
}

#[derive(Resource)]
pub struct PrepareBuildIndirectBindGroup(pub BindGroup);

pub fn prepare_build_indirect_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<PrepareBuildIndirectPipeline>,
    pipeline_cache: Res<PipelineCache>,
    local_count: Option<Res<LocalActiveCountBuffer>>,
    global_count: Option<Res<GlobalActiveCountBuffer>>,
    indirect: Option<Res<BuildIndirectBuffer>>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let (Some(local_count), Some(global_count), Some(indirect)) =
        (local_count, global_count, indirect)
    else {
        return;
    };
    let Some(local_buf) = buffers.get(&local_count.0) else { return; };
    let Some(global_buf) = buffers.get(&global_count.0) else { return; };
    let Some(indirect_buf) = buffers.get(&indirect.0) else { return; };

    let bind_group = render_device.create_bind_group(
        "prepare build indirect bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            local_buf.buffer.as_entire_buffer_binding(),
            global_buf.buffer.as_entire_buffer_binding(),
            indirect_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(PrepareBuildIndirectBindGroup(bind_group));
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
pub struct PrepareBuildIndirectNodeLabel;

#[derive(Default)]
pub struct PrepareBuildIndirectNode;

impl render_graph::Node for PrepareBuildIndirectNode {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let pipeline_cache = world.resource::<PipelineCache>();
        let pipeline = world.resource::<PrepareBuildIndirectPipeline>();
        let Some(bind_group) = world.get_resource::<PrepareBuildIndirectBindGroup>() else {
            return Ok(());
        };
        let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
            return Ok(());
        };
        let encoder = render_context.command_encoder();
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("prepare build indirect pass"),
            ..default()
        });
        pass.set_bind_group(0, &bind_group.0, &[]);
        pass.set_pipeline(compute_pipeline);
        pass.dispatch_workgroups(1, 1, 1);
        Ok(())
    }
}

pub fn add_prepare_build_indirect_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(PrepareBuildIndirectNodeLabel, PrepareBuildIndirectNode);
    // After lidar writes the active-cell counts, before build_local +
    // build_global read the indirect args.
    render_graph.add_node_edge(ComputeLidarNodeLabel, PrepareBuildIndirectNodeLabel);
}
