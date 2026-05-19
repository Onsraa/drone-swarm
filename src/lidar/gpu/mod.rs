mod build_global_pass;
mod build_pass;
mod dispatch;
mod merge_pass;
mod pipeline;
mod resources;

pub use resources::{
    GlobalInstanceCountBuffer, GlobalInstanceVecBuffer, GlobalOccupancyBuffer,
    LocalInstanceCountBuffer, LocalInstanceVecBuffer,
};

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::storage::ShaderStorageBuffer;
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

use crate::drone::{Drone, DroneColor, DroneId};
use crate::world::WorldConfig;

use build_global_pass::{
    add_build_global_render_graph_node, init_build_global_pipeline,
    prepare_build_global_bind_group,
};
use build_pass::{
    add_build_local_render_graph_node, init_build_local_pipeline, prepare_build_local_bind_group,
};
use dispatch::{add_compute_render_graph_node, prepare_lidar_bind_group};
use merge_pass::{
    add_merge_global_render_graph_node, init_merge_global_pipeline, prepare_merge_global_bind_group,
};
use pipeline::init_compute_lidar_pipeline;
use resources::{
    setup_gpu_lidar_assets, BuildLocalParams, BuildLocalParamsBuffer, DroneColorsBuffer,
    DroneOrientationsBuffer, DronePositionsBuffer, GroundTruthBuffer, LidarParams,
    LidarParamsBuffer, RayDirsBuffer, MAX_DRONES_GPU, MAX_LOCAL_INSTANCES, MAX_STEPS_PER_RAY,
};

use super::constants::RAYS_PER_SCAN;

/// CPU-side mirror of the global occupancy counts. Filled in by a
/// Readback observer over `GlobalOccupancyBuffer`; the side panel reads
/// it to display central-map coverage.
#[derive(Resource, Default, Clone, Copy, Debug)]
pub struct GpuGlobalStats {
    pub free: usize,
    pub occupied: usize,
}

/// Owns all the GPU lidar storage buffers, compute pipelines, and the
/// render-graph nodes that schedule them (lidar -> merge_global ->
/// build_global; lidar -> build_local). All map state lives on the GPU;
/// the CPU only uploads drone positions/orientations/colors and reads
/// back the global-occupancy counts for the side panel.
pub struct GpuLidarPlugin;

impl Plugin for GpuLidarPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GpuGlobalStats>()
            .add_plugins(ExtractResourcePlugin::<GroundTruthBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarParamsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DronePositionsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DroneOrientationsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<RayDirsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<resources::LocalOccupancyBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<GlobalOccupancyBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<BuildLocalParamsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DroneColorsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LocalInstanceCountBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LocalInstanceVecBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<GlobalInstanceCountBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<GlobalInstanceVecBuffer>::default())
            .add_systems(
                Update,
                (
                    setup_gpu_lidar_assets
                        .run_if(resource_exists::<crate::world::GroundTruthMap>)
                        .run_if(not(resource_exists::<GroundTruthBuffer>)),
                    upload_drone_state.run_if(resource_exists::<DronePositionsBuffer>),
                    upload_build_params_and_colors
                        .run_if(resource_exists::<BuildLocalParamsBuffer>),
                    spawn_global_stats_readback
                        .run_if(resource_exists::<GlobalOccupancyBuffer>),
                ),
            );

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .add_systems(
                RenderStartup,
                (
                    init_compute_lidar_pipeline,
                    add_compute_render_graph_node,
                    init_build_local_pipeline,
                    init_merge_global_pipeline,
                    init_build_global_pipeline,
                    // Edges: lidar -> {merge_global, build_local}.
                    //        merge_global -> build_global.
                    add_merge_global_render_graph_node
                        .after(add_compute_render_graph_node),
                    add_build_local_render_graph_node
                        .after(add_compute_render_graph_node),
                    add_build_global_render_graph_node
                        .after(add_merge_global_render_graph_node),
                ),
            )
            .add_systems(
                Render,
                // `set_data` each frame re-prepares the storage buffers as
                // brand-new GPU Buffer handles, so the bind group must
                // rebuild every frame to point at the live ones.
                (
                    prepare_lidar_bind_group,
                    prepare_merge_global_bind_group,
                    prepare_build_local_bind_group,
                    prepare_build_global_bind_group,
                )
                    .in_set(RenderSystems::PrepareBindGroups),
            );
    }
}

fn upload_drone_state(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    positions_handle: Res<DronePositionsBuffer>,
    orientations_handle: Res<DroneOrientationsBuffer>,
    params_handle: Res<LidarParamsBuffer>,
    config: Res<WorldConfig>,
    drones: Query<(&DroneId, &Transform), With<Drone>>,
) {
    let voxel_size = config.voxel_size;
    let mut sorted: Vec<(u32, Vec3, Quat)> = drones
        .iter()
        .map(|(id, t)| (id.0, t.translation, t.rotation))
        .collect();
    sorted.sort_by_key(|(id, _, _)| *id);

    let max = MAX_DRONES_GPU as usize;
    let mut positions = vec![Vec4::ZERO; max];
    let mut orientations = vec![Vec4::new(0.0, 0.0, 0.0, 1.0); max];
    let count = sorted.len().min(max) as u32;
    for (i, (_, pos, rot)) in sorted.iter().take(max).enumerate() {
        let g = *pos / voxel_size;
        positions[i] = Vec4::new(g.x, g.y, g.z, 0.0);
        orientations[i] = Vec4::new(rot.x, rot.y, rot.z, rot.w);
    }

    if let Some(buf) = buffers.get_mut(&positions_handle.0) {
        buf.set_data(positions);
    }
    if let Some(buf) = buffers.get_mut(&orientations_handle.0) {
        buf.set_data(orientations);
    }
    if let Some(buf) = buffers.get_mut(&params_handle.0) {
        let params = LidarParams {
            dims: UVec4::new(config.size.x, config.size.y, config.size.z, 0),
            max_steps: MAX_STEPS_PER_RAY,
            rays_per_scan: RAYS_PER_SCAN as u32,
            drone_count: count,
            _pad: 0,
        };
        buf.set_data(params);
    }
}

fn upload_build_params_and_colors(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    colors_handle: Res<DroneColorsBuffer>,
    params_handle: Res<BuildLocalParamsBuffer>,
    config: Res<WorldConfig>,
    drones: Query<(&DroneId, &DroneColor), With<Drone>>,
) {
    let mut sorted: Vec<(u32, Vec4)> = drones
        .iter()
        .map(|(id, color)| {
            let linear = color.0.to_linear();
            (
                id.0,
                Vec4::new(
                    (linear.red * crate::render::constants::LOCAL_MAP_COLOR_FACTOR).min(1.0),
                    (linear.green * crate::render::constants::LOCAL_MAP_COLOR_FACTOR).min(1.0),
                    (linear.blue * crate::render::constants::LOCAL_MAP_COLOR_FACTOR).min(1.0),
                    crate::render::constants::LOCAL_MAP_ALPHA,
                ),
            )
        })
        .collect();
    sorted.sort_by_key(|(id, _)| *id);

    let max = MAX_DRONES_GPU as usize;
    let mut colors = vec![Vec4::ZERO; max];
    let count = sorted.len().min(max) as u32;
    for (i, (_, color)) in sorted.iter().take(max).enumerate() {
        colors[i] = *color;
    }
    if let Some(buf) = buffers.get_mut(&colors_handle.0) {
        buf.set_data(colors);
    }
    if let Some(buf) = buffers.get_mut(&params_handle.0) {
        let params = BuildLocalParams {
            dims: UVec4::new(config.size.x, config.size.y, config.size.z, 0),
            drone_count: count,
            voxel_size: config.voxel_size,
            scale_factor: crate::render::constants::LOCAL_MAP_SCALE_FACTOR,
            max_instances: MAX_LOCAL_INSTANCES,
        };
        buf.set_data(params);
    }
}

/// One Readback over the global occupancy SSBO, counting Free/Occupied
/// 2-bit slots into `GpuGlobalStats`. The panel reads the resource;
/// this is the last CPU consumer of global voxel state.
fn spawn_global_stats_readback(
    mut commands: Commands,
    occupancy: Option<Res<GlobalOccupancyBuffer>>,
    mut spawned: Local<bool>,
) {
    if *spawned {
        return;
    }
    let Some(occupancy) = occupancy else {
        return;
    };
    *spawned = true;
    commands
        .spawn(Readback::buffer(occupancy.0.clone()))
        .observe(
            |event: On<ReadbackComplete>, mut stats: ResMut<GpuGlobalStats>| {
                let data: Vec<u32> = event.to_shader_type();
                let mut free = 0usize;
                let mut occupied = 0usize;
                for &word in &data {
                    for slot in 0..16u32 {
                        let state = (word >> (slot * 2)) & 0b11;
                        match state {
                            1 => free += 1,
                            2 | 3 => occupied += 1,
                            _ => {}
                        }
                    }
                }
                stats.free = free;
                stats.occupied = occupied;
            },
        );
}
