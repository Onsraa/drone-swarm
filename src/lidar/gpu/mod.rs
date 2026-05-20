mod build_global_pass;
mod build_pass;
mod dispatch;
mod per_drone_scan;
mod pipeline;
mod resources;

pub use per_drone_scan::{DroneScanParams, DroneScanParamsBuffer};
pub use resources::{
    BuildLocalParamsBuffer, DroneColorsBuffer, DroneOrientationsBuffer, DronePositionsBuffer,
    GlobalActiveCellsBuffer, GlobalActiveCountBuffer, GlobalInstanceCountBuffer,
    GlobalInstanceVecBuffer, GlobalOccupancyBuffer, GroundTruthBuffer, LidarParamsBuffer,
    LidarPointCountBuffer, LidarPointVecBuffer, LocalActiveCellsBuffer, LocalActiveCountBuffer,
    LocalInstanceCountBuffer, LocalInstanceVecBuffer, LocalOccupancyBuffer, RayDirsBuffer,
    RoleConeRanges,
};

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::storage::ShaderStorageBuffer;
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

use crate::comms::CommsState;
use crate::drone::{Drone, DroneColor, DroneId};
use crate::exploration::{Role, RoleParams};
use crate::lidar::sampling::RoleConeRange;
use crate::lidar::{LidarFrameCounter, LidarSettings};
use crate::world::WorldConfig;

use build_global_pass::{
    add_build_global_render_graph_node, init_build_global_pipeline,
    prepare_build_global_bind_group,
};
use build_pass::{
    add_build_local_render_graph_node, init_build_local_pipeline, prepare_build_local_bind_group,
};
use dispatch::{add_compute_render_graph_node, prepare_lidar_bind_group};
use pipeline::init_compute_lidar_pipeline;
use resources::{
    setup_gpu_lidar_assets, BuildLocalParams, LidarParams, MAX_DRONES_GPU, MAX_LIDAR_POINTS,
    MAX_LOCAL_INSTANCES,
};

use super::sampling::fibonacci_cone;

/// CPU-side mirror of the global occupancy counts. Filled in by a
/// Readback observer over `GlobalOccupancyBuffer`; the side panel reads
/// it to display central-map coverage.
#[derive(Resource, Default, Clone, Copy, Debug)]
pub struct GpuGlobalStats {
    pub free: usize,
    pub occupied: usize,
}

/// CPU-side mirror of the raw global occupancy bitset (2 bits per cell,
/// 16 cells per u32 word). Updated by the same Readback observer that
/// drives `GpuGlobalStats`; downstream consumers (frontier exploration)
/// decode the 2-bit states to find Unknown/Free transitions.
#[derive(Resource, Default, Clone, Debug)]
pub struct GpuGlobalOccupancyMirror {
    pub data: Vec<u32>,
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
            .init_resource::<GpuGlobalOccupancyMirror>()
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
            .add_plugins(ExtractResourcePlugin::<LidarPointCountBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarPointVecBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DroneScanParamsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarSettings>::default())
            .add_plugins(ExtractResourcePlugin::<LidarFrameCounter>::default())
            .add_plugins(ExtractResourcePlugin::<LocalActiveCellsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LocalActiveCountBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<GlobalActiveCellsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<GlobalActiveCountBuffer>::default())
            .add_systems(
                Update,
                (
                    setup_gpu_lidar_assets
                        .run_if(resource_exists::<crate::world::GroundTruthMap>)
                        .run_if(not(resource_exists::<GroundTruthBuffer>)),
                    upload_drone_state.run_if(resource_exists::<DronePositionsBuffer>),
                    upload_build_params_and_colors
                        .run_if(resource_exists::<BuildLocalParamsBuffer>),
                    upload_ray_dirs.run_if(resource_exists::<RayDirsBuffer>),
                    upload_drone_scan_params
                        .run_if(resource_exists::<DroneScanParamsBuffer>),
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
                    init_build_global_pipeline,
                    // Edges: lidar -> {build_local, build_global}. The
                    // old merge_global pass is gone — lidar now writes
                    // directly into global_occupancy with a comms-mask
                    // gate, so build_global can run straight after.
                    add_build_local_render_graph_node
                        .after(add_compute_render_graph_node),
                    add_build_global_render_graph_node
                        .after(add_compute_render_graph_node),
                ),
            )
            // `set_data` each frame re-prepares the storage buffers as
            // brand-new GPU Buffer handles, so the bind group must
            // rebuild every frame to point at the live ones.
            .add_systems(
                Render,
                prepare_lidar_bind_group.in_set(RenderSystems::PrepareBindGroups),
            )
            .add_systems(
                Render,
                prepare_build_local_bind_group.in_set(RenderSystems::PrepareBindGroups),
            )
            .add_systems(
                Render,
                prepare_build_global_bind_group.in_set(RenderSystems::PrepareBindGroups),
            );
    }
}

#[allow(clippy::too_many_arguments)]
fn upload_drone_state(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    positions_handle: Res<DronePositionsBuffer>,
    orientations_handle: Res<DroneOrientationsBuffer>,
    params_handle: Res<LidarParamsBuffer>,
    config: Res<WorldConfig>,
    ui_state: Res<crate::ui::UiState>,
    settings: Res<LidarSettings>,
    comms: Res<CommsState>,
    mut sorted: Local<Vec<(u32, Vec3, Quat)>>,
    drones: Query<(&DroneId, &Transform), With<Drone>>,
) {
    let voxel_size = config.voxel_size;
    sorted.clear();
    sorted.extend(drones.iter().map(|(id, t)| (id.0, t.translation, t.rotation)));
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
            max_steps: settings.max_steps_per_ray,
            rays_per_scan: settings.rays_per_scan,
            drone_count: count,
            voxel_size: config.voxel_size,
            drone_mask_lo: ui_state.drone_mask[0],
            drone_mask_hi: ui_state.drone_mask[1],
            max_points: MAX_LIDAR_POINTS,
            connected_mask_lo: comms.connected_mask[0],
            connected_mask_hi: comms.connected_mask[1],
            _pad0: 0,
            _pad1: 0,
        };
        buf.set_data(params);
    }
}

/// Rebuild the fibonacci cone whenever `LidarSettings` changes and
/// stream the new directions into `RayDirsBuffer`. The buffer is
/// allocated at `MAX_RAYS_PER_SCAN` slots; trailing slots stay zero
/// when `rays_per_scan` < max. The shader only iterates
/// `params.rays_per_scan` rays so padding is harmless.
fn upload_ray_dirs(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    dirs_handle: Res<RayDirsBuffer>,
    settings: Res<LidarSettings>,
    mut last_settings: Local<Option<LidarSettings>>,
) {
    let needs_update = match *last_settings {
        None => true,
        Some(prev) => {
            prev.rays_per_scan != settings.rays_per_scan
                || prev.cone_half_angle_deg != settings.cone_half_angle_deg
        }
    };
    if !needs_update {
        return;
    }

    let n = settings.rays_per_scan as usize;
    let dirs = fibonacci_cone(n, settings.cone_half_angle_deg.to_radians());
    let mut padded: Vec<Vec4> =
        vec![Vec4::ZERO; super::constants::MAX_RAYS_PER_SCAN as usize];
    for (i, d) in dirs.iter().enumerate() {
        if i >= padded.len() {
            break;
        }
        padded[i] = Vec4::new(d.x, d.y, d.z, 0.0);
    }
    if let Some(buf) = buffers.get_mut(&dirs_handle.0) {
        buf.set_data(padded);
    }
    *last_settings = Some(*settings);
}

#[allow(clippy::too_many_arguments)]
fn upload_build_params_and_colors(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    colors_handle: Res<DroneColorsBuffer>,
    params_handle: Res<BuildLocalParamsBuffer>,
    config: Res<WorldConfig>,
    ui_state: Res<crate::ui::UiState>,
    comms: Res<CommsState>,
    mut sorted: Local<Vec<(u32, Vec4)>>,
    drones: Query<(&DroneId, &DroneColor), With<Drone>>,
) {
    sorted.clear();
    sorted.extend(drones.iter().map(|(id, color)| {
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
    }));
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
        let mask_visual = if ui_state.show_local_maps {
            ui_state.drone_mask
        } else {
            [0, 0]
        };
        let params = BuildLocalParams {
            dims: UVec4::new(config.size.x, config.size.y, config.size.z, 0),
            drone_count: count,
            voxel_size: config.voxel_size,
            scale_factor: crate::render::constants::LOCAL_MAP_POINT_PX,
            max_instances: MAX_LOCAL_INSTANCES,
            drone_mask_lo: mask_visual[0],
            drone_mask_hi: mask_visual[1],
            connected_mask_lo: comms.connected_mask[0],
            connected_mask_hi: comms.connected_mask[1],
        };
        buf.set_data(params);
    }
}

fn role_range(ranges: &[RoleConeRange; 3], role: Role) -> RoleConeRange {
    ranges
        .iter()
        .find(|r| r.role == role)
        .copied()
        .expect("role missing from concatenated ray buffer")
}

fn upload_drone_scan_params(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    handle: Res<DroneScanParamsBuffer>,
    ranges: Res<RoleConeRanges>,
    mut sorted: Local<Vec<(u32, Role)>>,
    drones: Query<(&DroneId, &Role), With<Drone>>,
) {
    sorted.clear();
    sorted.extend(drones.iter().map(|(id, r)| (id.0, *r)));
    sorted.sort_by_key(|(id, _)| *id);
    let max = MAX_DRONES_GPU as usize;
    let mut out = vec![DroneScanParams::default(); max];
    for (i, (_, role)) in sorted.iter().take(max).enumerate() {
        let p = RoleParams::for_role(*role);
        let r = role_range(&ranges.0, *role);
        out[i] = DroneScanParams {
            ray_offset: r.offset,
            ray_count: r.count.min(p.rays_per_scan),
            max_steps: p.max_range_cells,
            scan_interval: p.scan_interval_frames,
        };
    }
    if let Some(buf) = buffers.get_mut(&handle.0) {
        buf.set_data(out);
    }
}

/// Marker on the Readback observer entity over the global occupancy
/// SSBO. `apply_map_swap` despawns this entity (its handle would point
/// at the stale pre-swap buffer); the system below respawns it once a
/// fresh `GlobalOccupancyBuffer` is allocated for the new map.
#[derive(Component)]
pub struct GlobalOccupancyReadbackTag;

/// One Readback over the global occupancy SSBO, counting Free/Occupied
/// 2-bit slots into `GpuGlobalStats`. The panel reads the resource;
/// this is the last CPU consumer of global voxel state.
fn spawn_global_stats_readback(
    mut commands: Commands,
    occupancy: Option<Res<GlobalOccupancyBuffer>>,
    existing: Query<(), With<GlobalOccupancyReadbackTag>>,
) {
    if !existing.is_empty() {
        return;
    }
    let Some(occupancy) = occupancy else {
        return;
    };
    commands
        .spawn((
            Readback::buffer(occupancy.0.clone()),
            GlobalOccupancyReadbackTag,
        ))
        .observe(
            |event: On<ReadbackComplete>,
             mut stats: ResMut<GpuGlobalStats>,
             mut mirror: ResMut<GpuGlobalOccupancyMirror>,
             mut throttle: Local<u32>| {
                // Throttle: only decode every 4th readback (~15 Hz at
                // 60 FPS, ~30 Hz at 120 FPS). Skipping early avoids
                // the 2.4 MB mirror copy entirely on the dropped
                // firings. Downstream consumers (frontier scan,
                // planner grid, reactive_avoid) tolerate the ~67 ms
                // lag.
                *throttle = throttle.wrapping_add(1);
                if *throttle % 4 != 0 {
                    return;
                }
                // Zero-alloc decode: reinterpret the raw byte buffer
                // as &[u32]. Bypasses `to_shader_type`'s encase reader
                // which would `default()` a Vec then resize it (one
                // 2.4 MB allocation per fire). Bitwise + popcount:
                // each u32 holds 16 cells, 2 bits each, even bits =
                // Free flag, odd = Occupied. Occupied wins when both
                // set (state == 0b11).
                let words: &[u32] = bytemuck::cast_slice(&event.data);
                let mut free = 0usize;
                let mut occupied = 0usize;
                for &word in words {
                    let occ_bits = word & 0xAAAAAAAA;
                    let free_bits = word & 0x55555555;
                    let free_only = free_bits & !(occ_bits >> 1);
                    occupied += occ_bits.count_ones() as usize;
                    free += free_only.count_ones() as usize;
                }
                stats.free = free;
                stats.occupied = occupied;
                // Reuse the existing mirror Vec — `extend_from_slice`
                // memcpys into the existing allocation instead of
                // dropping + reallocating it.
                mirror.data.clear();
                mirror.data.extend_from_slice(words);
            },
        );
}
