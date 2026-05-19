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

use super::per_drone_scan::DroneScanParamsBuffer;
use super::pipeline::ComputeLidarPipeline;
use super::resources::{
    DroneColorsBuffer, DroneOrientationsBuffer, DronePositionsBuffer, GlobalOccupancyBuffer,
    GroundTruthBuffer, LidarParamsBuffer, LidarPointCountBuffer, LidarPointVecBuffer,
    LocalOccupancyBuffer, RayDirsBuffer, MAX_DRONES_GPU,
};
use crate::lidar::{LidarFrameCounter, LidarSettings};

#[derive(Resource)]
pub struct LidarBindGroup(pub BindGroup);

#[allow(clippy::too_many_arguments)]
pub fn prepare_lidar_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<ComputeLidarPipeline>,
    pipeline_cache: Res<PipelineCache>,
    ground: Option<Res<GroundTruthBuffer>>,
    params: Option<Res<LidarParamsBuffer>>,
    positions: Option<Res<DronePositionsBuffer>>,
    orientations: Option<Res<DroneOrientationsBuffer>>,
    dirs: Option<Res<RayDirsBuffer>>,
    occupancy: Option<Res<LocalOccupancyBuffer>>,
    colors: Option<Res<DroneColorsBuffer>>,
    point_count: Option<Res<LidarPointCountBuffer>>,
    point_vec: Option<Res<LidarPointVecBuffer>>,
    scan_params: Option<Res<DroneScanParamsBuffer>>,
    global_occupancy: Option<Res<GlobalOccupancyBuffer>>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let (
        Some(ground),
        Some(params),
        Some(positions),
        Some(orientations),
        Some(dirs),
        Some(occupancy),
        Some(colors),
        Some(point_count),
        Some(point_vec),
        Some(scan_params),
        Some(global_occupancy),
    ) = (
        ground,
        params,
        positions,
        orientations,
        dirs,
        occupancy,
        colors,
        point_count,
        point_vec,
        scan_params,
        global_occupancy,
    ) else {
        return;
    };
    let Some(ground_buf) = buffers.get(&ground.0) else { return; };
    let Some(params_buf) = buffers.get(&params.0) else { return; };
    let Some(positions_buf) = buffers.get(&positions.0) else { return; };
    let Some(orientations_buf) = buffers.get(&orientations.0) else { return; };
    let Some(dirs_buf) = buffers.get(&dirs.0) else { return; };
    let Some(occupancy_buf) = buffers.get(&occupancy.0) else { return; };
    let Some(colors_buf) = buffers.get(&colors.0) else { return; };
    let Some(point_count_buf) = buffers.get(&point_count.0) else { return; };
    let Some(point_vec_buf) = buffers.get(&point_vec.0) else { return; };
    let Some(scan_params_buf) = buffers.get(&scan_params.0) else { return; };
    let Some(global_occupancy_buf) = buffers.get(&global_occupancy.0) else { return; };

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
            colors_buf.buffer.as_entire_buffer_binding(),
            point_count_buf.buffer.as_entire_buffer_binding(),
            point_vec_buf.buffer.as_entire_buffer_binding(),
            scan_params_buf.buffer.as_entire_buffer_binding(),
            global_occupancy_buf.buffer.as_entire_buffer_binding(),
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
        // Scan rate: only dispatch every `scan_interval_frames` frames.
        // The point counter is also gated on a real dispatch so the
        // visible spray sticks for `interval - 1` frames between scans.
        let settings = world
            .get_resource::<LidarSettings>()
            .copied()
            .unwrap_or_default();
        let frame = world
            .get_resource::<LidarFrameCounter>()
            .map(|c| c.0)
            .unwrap_or(0);
        let interval = settings.scan_interval_frames.max(1);
        if frame % interval != 0 {
            return Ok(());
        }
        // Live mode zeroes the point counter every active-scan frame
        // so the cloud reflects "this scan's hits". Sticky mode keeps
        // the counter monotonic so each scan appends to the buffer.
        let buffers = world.resource::<RenderAssets<GpuShaderStorageBuffer>>();
        let Some(point_count_handle) = world.get_resource::<LidarPointCountBuffer>() else {
            return Ok(());
        };
        let Some(point_count_buf) = buffers.get(&point_count_handle.0) else {
            return Ok(());
        };

        let encoder = render_context.command_encoder();
        if !settings.sticky_spray {
            encoder.clear_buffer(&point_count_buf.buffer, 0, None);
        }
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("compute lidar pass"),
            ..default()
        });
        pass.set_bind_group(0, &bind_group.0, &[]);
        pass.set_pipeline(compute_pipeline);
        let group_x = MAX_DRONES_GPU.div_ceil(8);
        let group_y = settings.rays_per_scan.max(1).div_ceil(8);
        pass.dispatch_workgroups(group_x, group_y, 1);
        Ok(())
    }
}

pub fn add_compute_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(ComputeLidarNodeLabel, ComputeLidarNode);
}
