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
use super::resources::{
    DroneOrientationsBuffer, DronePositionsBuffer, GroundTruthBuffer, LidarParamsBuffer,
    LocalOccupancyBuffer, RayDirsBuffer, MAX_DRONES_GPU,
};
use super::super::constants::RAYS_PER_SCAN;

#[derive(Resource)]
pub struct LidarBindGroup(pub BindGroup);

pub fn prepare_lidar_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<ComputeLidarPipeline>,
    pipeline_cache: Res<PipelineCache>,
    ground: Res<GroundTruthBuffer>,
    params: Res<LidarParamsBuffer>,
    positions: Res<DronePositionsBuffer>,
    orientations: Res<DroneOrientationsBuffer>,
    dirs: Res<RayDirsBuffer>,
    occupancy: Res<LocalOccupancyBuffer>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let Some(ground_buf) = buffers.get(&ground.0) else { return; };
    let Some(params_buf) = buffers.get(&params.0) else { return; };
    let Some(positions_buf) = buffers.get(&positions.0) else { return; };
    let Some(orientations_buf) = buffers.get(&orientations.0) else { return; };
    let Some(dirs_buf) = buffers.get(&dirs.0) else { return; };
    let Some(occupancy_buf) = buffers.get(&occupancy.0) else { return; };

    let bind_group = render_device.create_bind_group(
        "compute lidar bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            ground_buf.buffer.as_entire_buffer_binding(),
            params_buf.buffer.as_entire_buffer_binding(),
            positions_buf.buffer.as_entire_buffer_binding(),
            dirs_buf.buffer.as_entire_buffer_binding(),
            orientations_buf.buffer.as_entire_buffer_binding(),
            occupancy_buf.buffer.as_entire_buffer_binding(),
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
        let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
            return Ok(());
        };

        let encoder = render_context.command_encoder();
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("compute lidar pass"),
            ..default()
        });
        pass.set_bind_group(0, &bind_group.0, &[]);
        pass.set_pipeline(compute_pipeline);
        let group_x = MAX_DRONES_GPU.div_ceil(8);
        let group_y = (RAYS_PER_SCAN as u32).div_ceil(8);
        pass.dispatch_workgroups(group_x, group_y, 1);
        Ok(())
    }
}

pub fn add_compute_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(ComputeLidarNodeLabel, ComputeLidarNode);
}
