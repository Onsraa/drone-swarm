use bevy::ecs::system::SystemParam;
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
use super::pipeline::{ComputeLidarBvhPipeline, ComputeLidarPipeline};
use super::prepare_indirect::PrepareBuildIndirectNodeLabel;
use super::resources::{
    BvhNodesBuffer, BvhPrimitiveIndicesBuffer, BvhTriangleVerticesBuffer, DroneColorsBuffer,
    DroneOrientationsBuffer, DronePositionsBuffer, GlobalActiveCellsBuffer, GlobalActiveCountBuffer,
    GlobalOccupancyBuffer, GroundTruthBuffer, LidarParamsBuffer, LidarPointCountBuffer,
    LidarPointVecBuffer, LocalActiveCellsBuffer, LocalActiveCountBuffer, LocalOccupancyBuffer,
    RayDirsBuffer, MAX_DRONES_GPU,
};
use crate::lidar::{LidarFrameCounter, LidarSettings, LidarSourceMode};

/// Bundles the active-list buffer handles + point-cloud buffers so
/// `prepare_lidar_bind_group` stays within Bevy's 16-parameter system
/// limit (the lidar bind group has 15 SSBO bindings + RenderDevice +
/// pipeline + cache + commands + buffers + ...).
#[derive(SystemParam)]
pub(crate) struct LidarExtraBuffers<'w> {
    pub point_count: Option<Res<'w, LidarPointCountBuffer>>,
    pub point_vec: Option<Res<'w, LidarPointVecBuffer>>,
    pub scan_params: Option<Res<'w, DroneScanParamsBuffer>>,
    pub global_occupancy: Option<Res<'w, GlobalOccupancyBuffer>>,
    pub local_active_cells: Option<Res<'w, LocalActiveCellsBuffer>>,
    pub local_active_count: Option<Res<'w, LocalActiveCountBuffer>>,
    pub global_active_cells: Option<Res<'w, GlobalActiveCellsBuffer>>,
    pub global_active_count: Option<Res<'w, GlobalActiveCountBuffer>>,
}

#[derive(Resource)]
pub struct LidarBindGroup(pub BindGroup);

/// BVH SSBO handles bundled to keep `prepare_lidar_bvh_bind_group`
/// inside Bevy's 16-parameter system limit.
#[derive(SystemParam)]
pub(crate) struct BvhBuffers<'w> {
    pub nodes: Option<Res<'w, BvhNodesBuffer>>,
    pub primitive_indices: Option<Res<'w, BvhPrimitiveIndicesBuffer>>,
    pub triangle_vertices: Option<Res<'w, BvhTriangleVerticesBuffer>>,
}

/// Render-world resource holding the BVH-path bind group. Built each
/// frame from the same buffer handles as the DDA path plus the three
/// BVH SSBOs. Phase 2b doesn't dispatch this — the bind group + group
/// layout existing is enough to flush out pipeline compilation issues.
#[derive(Resource)]
#[allow(dead_code)]
pub struct LidarBvhBindGroup(pub BindGroup);

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
    extras: LidarExtraBuffers,
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
        Some(local_active_cells),
        Some(local_active_count),
        Some(global_active_cells),
        Some(global_active_count),
    ) = (
        ground,
        params,
        positions,
        orientations,
        dirs,
        occupancy,
        colors,
        extras.point_count,
        extras.point_vec,
        extras.scan_params,
        extras.global_occupancy,
        extras.local_active_cells,
        extras.local_active_count,
        extras.global_active_cells,
        extras.global_active_count,
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
    let Some(local_active_cells_buf) = buffers.get(&local_active_cells.0) else { return; };
    let Some(local_active_count_buf) = buffers.get(&local_active_count.0) else { return; };
    let Some(global_active_cells_buf) = buffers.get(&global_active_cells.0) else { return; };
    let Some(global_active_count_buf) = buffers.get(&global_active_count.0) else { return; };

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
            local_active_cells_buf.buffer.as_entire_buffer_binding(),
            local_active_count_buf.buffer.as_entire_buffer_binding(),
            global_active_cells_buf.buffer.as_entire_buffer_binding(),
            global_active_count_buf.buffer.as_entire_buffer_binding(),
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
        // Phase 2c: gate on the source-mode toggle. BVH path is owned
        // by ComputeLidarBvhNode; this DDA path only runs when Dda.
        let mode = world
            .get_resource::<LidarSourceMode>()
            .copied()
            .unwrap_or_default();
        if mode != LidarSourceMode::Dda {
            return Ok(());
        }
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

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
pub struct ComputeLidarBvhNodeLabel;

#[derive(Default)]
pub struct ComputeLidarBvhNode;

impl render_graph::Node for ComputeLidarBvhNode {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let mode = world
            .get_resource::<LidarSourceMode>()
            .copied()
            .unwrap_or_default();
        if mode != LidarSourceMode::Bvh {
            return Ok(());
        }
        let pipeline_cache = world.resource::<PipelineCache>();
        let Some(pipeline) = world.get_resource::<ComputeLidarBvhPipeline>() else {
            return Ok(());
        };
        let Some(bind_group) = world.get_resource::<LidarBvhBindGroup>() else {
            return Ok(());
        };
        let Some(compute_pipeline) = pipeline_cache.get_compute_pipeline(pipeline.pipeline) else {
            return Ok(());
        };
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
            label: Some("compute lidar bvh pass"),
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

pub fn add_compute_lidar_bvh_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(ComputeLidarBvhNodeLabel, ComputeLidarBvhNode);
    // Run after the DDA node (no real dependency since only one fires
    // per frame, but keeps render-graph order deterministic) and
    // before PrepareBuildIndirect so the active-cell counts written
    // by BVH are visible to it.
    render_graph.add_node_edge(ComputeLidarNodeLabel, ComputeLidarBvhNodeLabel);
    render_graph.add_node_edge(ComputeLidarBvhNodeLabel, PrepareBuildIndirectNodeLabel);
}

/// Build the BVH-shader bind group from the same 15 SSBOs the DDA
/// path uses plus the three BVH SSBOs (nodes, prim_indices, verts).
/// Mirror of `prepare_lidar_bind_group` against `ComputeLidarBvhPipeline`.
#[allow(clippy::too_many_arguments)]
pub fn prepare_lidar_bvh_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Option<Res<ComputeLidarBvhPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    ground: Option<Res<GroundTruthBuffer>>,
    params: Option<Res<LidarParamsBuffer>>,
    positions: Option<Res<DronePositionsBuffer>>,
    orientations: Option<Res<DroneOrientationsBuffer>>,
    dirs: Option<Res<RayDirsBuffer>>,
    occupancy: Option<Res<LocalOccupancyBuffer>>,
    colors: Option<Res<DroneColorsBuffer>>,
    extras: LidarExtraBuffers,
    bvh: BvhBuffers,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let (
        Some(pipeline),
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
        Some(local_active_cells),
        Some(local_active_count),
        Some(global_active_cells),
        Some(global_active_count),
        Some(bvh_nodes),
        Some(bvh_prim_idx),
        Some(bvh_verts),
    ) = (
        pipeline,
        ground,
        params,
        positions,
        orientations,
        dirs,
        occupancy,
        colors,
        extras.point_count,
        extras.point_vec,
        extras.scan_params,
        extras.global_occupancy,
        extras.local_active_cells,
        extras.local_active_count,
        extras.global_active_cells,
        extras.global_active_count,
        bvh.nodes,
        bvh.primitive_indices,
        bvh.triangle_vertices,
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
    let Some(local_active_cells_buf) = buffers.get(&local_active_cells.0) else { return; };
    let Some(local_active_count_buf) = buffers.get(&local_active_count.0) else { return; };
    let Some(global_active_cells_buf) = buffers.get(&global_active_cells.0) else { return; };
    let Some(global_active_count_buf) = buffers.get(&global_active_count.0) else { return; };
    let Some(bvh_nodes_buf) = buffers.get(&bvh_nodes.0) else { return; };
    let Some(bvh_prim_idx_buf) = buffers.get(&bvh_prim_idx.0) else { return; };
    let Some(bvh_verts_buf) = buffers.get(&bvh_verts.0) else { return; };

    let bind_group = render_device.create_bind_group(
        "compute lidar bvh bind group",
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
            local_active_cells_buf.buffer.as_entire_buffer_binding(),
            local_active_count_buf.buffer.as_entire_buffer_binding(),
            global_active_cells_buf.buffer.as_entire_buffer_binding(),
            global_active_count_buf.buffer.as_entire_buffer_binding(),
            bvh_nodes_buf.buffer.as_entire_buffer_binding(),
            bvh_prim_idx_buf.buffer.as_entire_buffer_binding(),
            bvh_verts_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(LidarBvhBindGroup(bind_group));
}
