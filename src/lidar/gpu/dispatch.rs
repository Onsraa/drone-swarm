use bevy::prelude::*;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_graph::{
    self, NodeRunError, RenderGraph, RenderGraphContext, RenderLabel,
};
use bevy::render::render_resource::{
    BindGroup, BindGroupEntries, ComputePassDescriptor, PipelineCache,
};
use bevy::render::renderer::{RenderContext, RenderDevice};
use bevy::render::storage::GpuShaderStorageBuffer;

use super::pipeline::ComputeLidarPipeline;
use super::resources::{GroundTruthBuffer, LidarCountBuffer};

#[derive(Resource)]
pub struct LidarBindGroup(pub BindGroup);

/// Builds the bind group once both storage buffers exist on the GPU.
/// Runs every frame but the resource-existence guard makes it a one-shot
/// in practice — only re-fires if `LidarBindGroup` gets removed.
pub fn prepare_lidar_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<ComputeLidarPipeline>,
    pipeline_cache: Res<PipelineCache>,
    ground: Res<GroundTruthBuffer>,
    output: Res<LidarCountBuffer>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let Some(ground_buf) = buffers.get(&ground.0) else {
        return;
    };
    let Some(output_buf) = buffers.get(&output.0) else {
        return;
    };
    let bind_group = render_device.create_bind_group(
        "compute lidar bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            ground_buf.buffer.as_entire_buffer_binding(),
            output_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(LidarBindGroup(bind_group));
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
pub struct ComputeLidarNodeLabel;

#[derive(Default)]
pub struct ComputeLidarNode;

impl render_graph::Node for ComputeLidarNode {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let pipeline_cache = world.resource::<PipelineCache>();
        let pipeline = world.resource::<ComputeLidarPipeline>();
        let Some(bind_group) = world.get_resource::<LidarBindGroup>() else {
            return Ok(());
        };
        let Some(output_handle) = world.get_resource::<LidarCountBuffer>() else {
            return Ok(());
        };
        let Some(buffers) = world.get_resource::<RenderAssets<GpuShaderStorageBuffer>>() else {
            return Ok(());
        };
        let Some(output_buf) = buffers.get(&output_handle.0) else {
            return Ok(());
        };
        let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
            return Ok(());
        };
        let ground_handle = world.resource::<GroundTruthBuffer>();
        let Some(ground_buf) = buffers.get(&ground_handle.0) else {
            return Ok(());
        };
        let bitset_words = (ground_buf.buffer.size() / 4) as u32;

        let encoder = render_context.command_encoder();
        encoder.clear_buffer(&output_buf.buffer, 0, None);
        {
            let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
                label: Some("compute lidar count pass"),
                ..default()
            });
            pass.set_bind_group(0, &bind_group.0, &[]);
            pass.set_pipeline(compute_pipeline);
            let workgroups = bitset_words.div_ceil(64).max(1);
            pass.dispatch_workgroups(workgroups, 1, 1);
        }
        Ok(())
    }
}

pub fn add_compute_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(ComputeLidarNodeLabel, ComputeLidarNode);
}
