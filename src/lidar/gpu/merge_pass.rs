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
use super::resources::{
    BuildLocalParams, BuildLocalParamsBuffer, GlobalOccupancyBuffer, LocalOccupancyBuffer,
};

const SHADER_ASSET_PATH: &str = "shaders/merge_global.wgsl";

#[derive(Resource)]
pub struct MergeGlobalPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_merge_global_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "merge global occupancy layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                storage_buffer_read_only::<Vec<u32>>(false),
                storage_buffer_read_only::<BuildLocalParams>(false),
                storage_buffer::<Vec<u32>>(false),
            ),
        ),
    );
    let shader: Handle<Shader> = asset_server.load(SHADER_ASSET_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("merge global occupancy".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("merge_global".into()),
        ..default()
    });
    commands.insert_resource(MergeGlobalPipeline { layout, pipeline });
}

#[derive(Resource)]
pub struct MergeGlobalBindGroup(pub BindGroup);

pub fn prepare_merge_global_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<MergeGlobalPipeline>,
    pipeline_cache: Res<PipelineCache>,
    occupancy: Res<LocalOccupancyBuffer>,
    params: Res<BuildLocalParamsBuffer>,
    global_occupancy: Res<GlobalOccupancyBuffer>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let Some(occupancy_buf) = buffers.get(&occupancy.0) else { return; };
    let Some(params_buf) = buffers.get(&params.0) else { return; };
    let Some(global_buf) = buffers.get(&global_occupancy.0) else { return; };

    let bind_group = render_device.create_bind_group(
        "merge global occupancy bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            occupancy_buf.buffer.as_entire_buffer_binding(),
            params_buf.buffer.as_entire_buffer_binding(),
            global_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(MergeGlobalBindGroup(bind_group));
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
pub struct MergeGlobalNodeLabel;

#[derive(Default)]
pub struct MergeGlobalNode;

impl render_graph::Node for MergeGlobalNode {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let pipeline_cache = world.resource::<PipelineCache>();
        let pipeline = world.resource::<MergeGlobalPipeline>();
        let Some(bind_group) = world.get_resource::<MergeGlobalBindGroup>() else {
            return Ok(());
        };
        let Some(global_handle) = world.get_resource::<GlobalOccupancyBuffer>() else {
            return Ok(());
        };
        let Some(buffers) = world.get_resource::<RenderAssets<GpuShaderStorageBuffer>>() else {
            return Ok(());
        };
        let Some(global_buf) = buffers.get(&global_handle.0) else {
            return Ok(());
        };
        let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
            return Ok(());
        };

        // Same world-size assumption as the build pass: shader bounds-
        // checks `w >= words_per_drone`, so the dispatch grid can be
        // sized for the default dims without reading WorldConfig from
        // the render world.
        let dims = crate::world::WorldConfig::default().size;
        let cells_per_drone = dims.x * dims.y * dims.z;
        let words_per_drone = cells_per_drone.div_ceil(16);
        let groups_x = words_per_drone.div_ceil(64);

        let encoder = render_context.command_encoder();
        // Reset the global SSBO each frame; the merge OR-folds fresh
        // contents from per-drone buffers (which themselves are sticky).
        encoder.clear_buffer(&global_buf.buffer, 0, None);
        {
            let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
                label: Some("merge global occupancy pass"),
                ..default()
            });
            pass.set_bind_group(0, &bind_group.0, &[]);
            pass.set_pipeline(compute_pipeline);
            pass.dispatch_workgroups(groups_x, 1, 1);
        }
        Ok(())
    }
}

pub fn add_merge_global_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(MergeGlobalNodeLabel, MergeGlobalNode);
    // Only one edge needed: lidar must run first so the per-drone
    // occupancy has this frame's writes. `build_local` doesn't read
    // the global SSBO, so no edge there.
    render_graph.add_node_edge(ComputeLidarNodeLabel, MergeGlobalNodeLabel);
}
