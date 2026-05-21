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
use super::pipeline::ComputeLidarBvhPipeline;
use super::resources::{
    BvhAtlasBuffer, BvhMaterialPaletteBuffer, BvhMaterialRectsBuffer, BvhNodesBuffer,
    BvhPrimitiveIndicesBuffer, BvhTriMaterialsBuffer, BvhTriUvsBuffer, BvhTriangleVerticesBuffer,
    DroneColorsBuffer, DroneOrientationsBuffer, DronePositionsBuffer, GlobalActiveCellsBuffer,
    GlobalActiveCountBuffer, GlobalOccupancyBuffer, LidarParamsBuffer, LidarPointCountBuffer,
    LidarPointVecBuffer, LocalActiveCellsBuffer, LocalActiveCountBuffer, LocalOccupancyBuffer,
    RayDirsBuffer, MAX_DRONES_GPU,
};
use crate::lidar::{LidarFrameCounter, LidarSettings};

/// Per-frame SystemParam bundle for the BVH bind-group prep system.
/// Keeps the function under Bevy's 16-parameter system limit by
/// collapsing the eight "tail" buffers + the BVH triplet into two
/// nested structs.
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

#[derive(SystemParam)]
pub(crate) struct BvhBuffers<'w> {
    pub nodes: Option<Res<'w, BvhNodesBuffer>>,
    pub primitive_indices: Option<Res<'w, BvhPrimitiveIndicesBuffer>>,
    pub triangle_vertices: Option<Res<'w, BvhTriangleVerticesBuffer>>,
    pub tri_materials: Option<Res<'w, BvhTriMaterialsBuffer>>,
    pub material_palette: Option<Res<'w, BvhMaterialPaletteBuffer>>,
    pub tri_uvs: Option<Res<'w, BvhTriUvsBuffer>>,
    pub material_rects: Option<Res<'w, BvhMaterialRectsBuffer>>,
    pub atlas: Option<Res<'w, BvhAtlasBuffer>>,
}

#[derive(Resource)]
pub struct LidarBvhBindGroup(pub BindGroup);

#[allow(clippy::too_many_arguments)]
pub fn prepare_lidar_bvh_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Option<Res<ComputeLidarBvhPipeline>>,
    pipeline_cache: Res<PipelineCache>,
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
        Some(bvh_tri_mats),
        Some(bvh_palette),
        Some(bvh_tri_uvs),
        Some(bvh_mat_rects),
        Some(bvh_atlas),
    ) = (
        pipeline,
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
        bvh.tri_materials,
        bvh.material_palette,
        bvh.tri_uvs,
        bvh.material_rects,
        bvh.atlas,
    )
    else {
        return;
    };
    let Some(params_buf) = buffers.get(&params.0) else { return };
    let Some(positions_buf) = buffers.get(&positions.0) else { return };
    let Some(orientations_buf) = buffers.get(&orientations.0) else { return };
    let Some(dirs_buf) = buffers.get(&dirs.0) else { return };
    let Some(occupancy_buf) = buffers.get(&occupancy.0) else { return };
    let Some(colors_buf) = buffers.get(&colors.0) else { return };
    let Some(point_count_buf) = buffers.get(&point_count.0) else { return };
    let Some(point_vec_buf) = buffers.get(&point_vec.0) else { return };
    let Some(scan_params_buf) = buffers.get(&scan_params.0) else { return };
    let Some(global_occupancy_buf) = buffers.get(&global_occupancy.0) else { return };
    let Some(local_active_cells_buf) = buffers.get(&local_active_cells.0) else { return };
    let Some(local_active_count_buf) = buffers.get(&local_active_count.0) else { return };
    let Some(global_active_cells_buf) = buffers.get(&global_active_cells.0) else { return };
    let Some(global_active_count_buf) = buffers.get(&global_active_count.0) else { return };
    let Some(bvh_nodes_buf) = buffers.get(&bvh_nodes.0) else { return };
    let Some(bvh_prim_idx_buf) = buffers.get(&bvh_prim_idx.0) else { return };
    let Some(bvh_verts_buf) = buffers.get(&bvh_verts.0) else { return };
    let Some(bvh_tri_mats_buf) = buffers.get(&bvh_tri_mats.0) else { return };
    let Some(bvh_palette_buf) = buffers.get(&bvh_palette.0) else { return };
    let Some(bvh_tri_uvs_buf) = buffers.get(&bvh_tri_uvs.0) else { return };
    let Some(bvh_mat_rects_buf) = buffers.get(&bvh_mat_rects.0) else { return };
    let Some(bvh_atlas_buf) = buffers.get(&bvh_atlas.0) else { return };

    let bind_group = render_device.create_bind_group(
        "compute lidar bvh bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
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
            bvh_tri_mats_buf.buffer.as_entire_buffer_binding(),
            bvh_palette_buf.buffer.as_entire_buffer_binding(),
            bvh_tri_uvs_buf.buffer.as_entire_buffer_binding(),
            bvh_mat_rects_buf.buffer.as_entire_buffer_binding(),
            bvh_atlas_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(LidarBvhBindGroup(bind_group));
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
    // Edge to PrepareBuildIndirect is wired by `prepare_indirect::
    // add_prepare_build_indirect_render_graph_node` after both nodes
    // exist.
}
