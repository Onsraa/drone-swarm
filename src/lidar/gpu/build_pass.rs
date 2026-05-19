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
    BuildLocalParams, BuildLocalParamsBuffer, DroneColorsBuffer, LocalInstanceCountBuffer,
    LocalInstanceVecBuffer, LocalOccupancyBuffer,
};

const SHADER_ASSET_PATH: &str = "shaders/build_local_instances.wgsl";

#[derive(Resource)]
pub struct BuildLocalPipeline {
    pub layout: BindGroupLayoutDescriptor,
    pub pipeline: CachedComputePipelineId,
}

pub fn init_build_local_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    pipeline_cache: Res<PipelineCache>,
) {
    let layout = BindGroupLayoutDescriptor::new(
        "build local instances layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                // 0: occupancy SSBO (read)
                storage_buffer_read_only::<Vec<u32>>(false),
                // 1: build params struct
                storage_buffer_read_only::<BuildLocalParams>(false),
                // 2: drone colors (Vec<Vec4>)
                storage_buffer_read_only::<Vec<Vec4>>(false),
                // 3: instance counter (atomic u32)
                storage_buffer::<Vec<u32>>(false),
                // 4: instance buffer (Vec<Vec4>, pairs of pos_scale + color)
                storage_buffer::<Vec<Vec4>>(false),
            ),
        ),
    );
    let shader: Handle<Shader> = asset_server.load(SHADER_ASSET_PATH);
    let pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("build local instances".into()),
        layout: vec![layout.clone()],
        shader,
        entry_point: Some("build".into()),
        ..default()
    });
    commands.insert_resource(BuildLocalPipeline { layout, pipeline });
}

#[derive(Resource)]
pub struct BuildLocalBindGroup(pub BindGroup);

pub fn prepare_build_local_bind_group(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline: Res<BuildLocalPipeline>,
    pipeline_cache: Res<PipelineCache>,
    occupancy: Res<LocalOccupancyBuffer>,
    params: Res<BuildLocalParamsBuffer>,
    colors: Res<DroneColorsBuffer>,
    count: Res<LocalInstanceCountBuffer>,
    instances: Res<LocalInstanceVecBuffer>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
) {
    let Some(occupancy_buf) = buffers.get(&occupancy.0) else { return; };
    let Some(params_buf) = buffers.get(&params.0) else { return; };
    let Some(colors_buf) = buffers.get(&colors.0) else { return; };
    let Some(count_buf) = buffers.get(&count.0) else { return; };
    let Some(instances_buf) = buffers.get(&instances.0) else { return; };

    let bind_group = render_device.create_bind_group(
        "build local instances bind group",
        &pipeline_cache.get_bind_group_layout(&pipeline.layout),
        &BindGroupEntries::sequential((
            occupancy_buf.buffer.as_entire_buffer_binding(),
            params_buf.buffer.as_entire_buffer_binding(),
            colors_buf.buffer.as_entire_buffer_binding(),
            count_buf.buffer.as_entire_buffer_binding(),
            instances_buf.buffer.as_entire_buffer_binding(),
        )),
    );
    commands.insert_resource(BuildLocalBindGroup(bind_group));
}

#[derive(Debug, Hash, PartialEq, Eq, Clone, RenderLabel)]
pub struct BuildLocalNodeLabel;

#[derive(Default)]
pub struct BuildLocalNode;

impl render_graph::Node for BuildLocalNode {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let pipeline_cache = world.resource::<PipelineCache>();
        let pipeline = world.resource::<BuildLocalPipeline>();
        let Some(bind_group) = world.get_resource::<BuildLocalBindGroup>() else {
            return Ok(());
        };
        let Some(count_handle) = world.get_resource::<LocalInstanceCountBuffer>() else {
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
        let Some(world_config) = world.get_resource::<crate::world::WorldConfig>() else {
            return Ok(());
        };

        let dims = world_config.size;
        let cells_per_drone = dims.x * dims.y * dims.z;
        // Workgroup_size_x = 64; one workgroup per 64 cells per drone.
        let groups_x = cells_per_drone.div_ceil(64);
        let groups_y = super::resources::MAX_DRONES_GPU;

        let encoder = render_context.command_encoder();
        encoder.clear_buffer(&count_buf.buffer, 0, None);
        {
            let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
                label: Some("build local instances pass"),
                ..default()
            });
            pass.set_bind_group(0, &bind_group.0, &[]);
            pass.set_pipeline(compute_pipeline);
            pass.dispatch_workgroups(groups_x, groups_y, 1);
        }
        Ok(())
    }
}

pub fn add_build_local_render_graph_node(mut render_graph: ResMut<RenderGraph>) {
    render_graph.add_node(BuildLocalNodeLabel, BuildLocalNode);
    // Ensure the build pass runs after the lidar pass so the occupancy
    // SSBO already reflects this frame's hits before we sweep over it.
    render_graph.add_node_edge(ComputeLidarNodeLabel, BuildLocalNodeLabel);
}
