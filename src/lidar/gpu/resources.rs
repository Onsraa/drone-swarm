use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{BufferUsages, ShaderType};
use bevy::render::storage::ShaderStorageBuffer;

use crate::world::GroundTruthMap;

use super::super::constants::RAYS_PER_SCAN;
use super::super::sampling::LidarRayDirs;

pub const MAX_STEPS_PER_RAY: u32 = 96;
pub const MAX_DRONES_GPU: u32 = 64;

/// Stage 9B output buffer capacity: max number of Occupied-cell instances
/// the build pass can emit across all drones in a single dispatch. 1M
/// instances at 32 bytes each = 32 MB. Steady state in a 50-drone session
/// is hundreds of thousands; this is generous headroom.
pub const MAX_LOCAL_INSTANCES: u32 = 1_000_000;

/// Stage 9Eb output capacity. The merged central map has at most one
/// instance per cell (`cells_per_drone` <= 98K for the default world),
/// so 100K slots is plenty. 3.2 MB at 32 bytes each.
pub const MAX_GLOBAL_INSTANCES: u32 = 100_000;

/// Per-drone local-map occupancy is packed as 2 bits per cell into a
/// `u32` storage buffer: bit 0 = Free flag, bit 1 = Occupied flag. Both
/// flags are sticky under `atomicOr`. `Unknown` is the all-zero default.
pub fn occupancy_words_per_drone(dims: UVec3) -> usize {
    let cells = (dims.x * dims.y * dims.z) as usize;
    cells.div_ceil(16)
}

/// Mirrors the WGSL `LidarParams` struct. Stage 3 uses a stub drone count
/// of 1 to validate the traversal shader in isolation; Stage 4 replaces
/// these values with per-tick uploads driven by the real drone query.
#[derive(ShaderType, Clone, Copy, Debug)]
pub struct LidarParams {
    pub dims: UVec4,
    pub max_steps: u32,
    pub rays_per_scan: u32,
    pub drone_count: u32,
    pub _pad: u32,
}

/// Mirrors the WGSL `BuildParams` struct used by `build_local_instances`.
#[derive(ShaderType, Clone, Copy, Debug)]
pub struct BuildLocalParams {
    pub dims: UVec4,
    pub drone_count: u32,
    pub voxel_size: f32,
    pub scale_factor: f32,
    pub max_instances: u32,
}

#[derive(Resource, ExtractResource, Clone)]
pub struct GroundTruthBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LidarParamsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct DronePositionsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct DroneOrientationsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct RayDirsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LidarHitsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalOccupancyBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalOccupancyBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct BuildLocalParamsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct DroneColorsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalInstanceCountBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalInstanceVecBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalInstanceCountBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalInstanceVecBuffer(pub Handle<ShaderStorageBuffer>);

/// Stash for the latest GPU hits buffer. The Readback observer writes
/// here from the main world; `apply_lidar_hits` drains and feeds each
/// drone's trail into its `LocalMap` via `upgrade()`.
#[derive(Resource, Default)]
pub struct PendingLidarHits(pub Option<Vec<u32>>);

/// One-shot startup: packs the CPU ground truth and allocates every
/// lidar input/output buffer. Positions and params start zeroed; the
/// per-frame `upload_drone_positions` system fills them with real data.
pub fn setup_gpu_lidar_assets(
    mut commands: Commands,
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    ground: Res<GroundTruthMap>,
    ray_dirs_res: Res<LidarRayDirs>,
) {
    let bitset = ground.pack_bitset();
    let mut ground_buf = ShaderStorageBuffer::from(bitset);
    ground_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let ground_handle = buffers.add(ground_buf);

    let params = LidarParams {
        dims: UVec4::new(ground.dims.x, ground.dims.y, ground.dims.z, 0),
        max_steps: MAX_STEPS_PER_RAY,
        rays_per_scan: RAYS_PER_SCAN as u32,
        drone_count: 0,
        _pad: 0,
    };
    let mut params_buf = ShaderStorageBuffer::from(params);
    params_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let params_handle = buffers.add(params_buf);

    let drone_positions: Vec<Vec4> = vec![Vec4::ZERO; MAX_DRONES_GPU as usize];
    let mut positions_buf = ShaderStorageBuffer::from(drone_positions);
    positions_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let positions_handle = buffers.add(positions_buf);

    // Identity quaternion = (0, 0, 0, 1).
    let drone_orientations: Vec<Vec4> =
        vec![Vec4::new(0.0, 0.0, 0.0, 1.0); MAX_DRONES_GPU as usize];
    let mut orientations_buf = ShaderStorageBuffer::from(drone_orientations);
    orientations_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let orientations_handle = buffers.add(orientations_buf);

    let ray_dirs: Vec<Vec4> = ray_dirs_res
        .0
        .iter()
        .map(|d| Vec4::new(d.x, d.y, d.z, 0.0))
        .collect();
    let mut dirs_buf = ShaderStorageBuffer::from(ray_dirs);
    dirs_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let dirs_handle = buffers.add(dirs_buf);

    let hits_len =
        (MAX_DRONES_GPU * params.rays_per_scan * params.max_steps) as usize;
    let mut hits_buf = ShaderStorageBuffer::from(vec![0u32; hits_len]);
    hits_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let hits_handle = buffers.add(hits_buf);

    let words_per_drone = occupancy_words_per_drone(ground.dims);
    let occupancy_len = (MAX_DRONES_GPU as usize) * words_per_drone;
    let mut occupancy_buf = ShaderStorageBuffer::from(vec![0u32; occupancy_len]);
    occupancy_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let occupancy_handle = buffers.add(occupancy_buf);

    let mut global_occupancy_buf = ShaderStorageBuffer::from(vec![0u32; words_per_drone]);
    global_occupancy_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let global_occupancy_handle = buffers.add(global_occupancy_buf);

    let build_params = BuildLocalParams {
        dims: UVec4::new(ground.dims.x, ground.dims.y, ground.dims.z, 0),
        drone_count: 0,
        voxel_size: 1.0,
        scale_factor: 1.0,
        max_instances: MAX_LOCAL_INSTANCES,
    };
    let mut build_params_buf = ShaderStorageBuffer::from(build_params);
    build_params_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let build_params_handle = buffers.add(build_params_buf);

    let drone_colors: Vec<Vec4> = vec![Vec4::ZERO; MAX_DRONES_GPU as usize];
    let mut drone_colors_buf = ShaderStorageBuffer::from(drone_colors);
    drone_colors_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let drone_colors_handle = buffers.add(drone_colors_buf);

    let mut count_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    count_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let count_handle = buffers.add(count_buf);

    // Two vec4 per instance (pos_scale + color). Also flag for vertex use
    // so Stage 9C's render pipeline can bind the same buffer.
    let instance_vec_len = (MAX_LOCAL_INSTANCES as usize) * 2;
    let mut instance_vec_buf = ShaderStorageBuffer::from(vec![Vec4::ZERO; instance_vec_len]);
    instance_vec_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST | BufferUsages::VERTEX;
    let instance_vec_handle = buffers.add(instance_vec_buf);

    let mut global_count_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    global_count_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let global_count_handle = buffers.add(global_count_buf);

    let global_instance_vec_len = (MAX_GLOBAL_INSTANCES as usize) * 2;
    let mut global_instance_vec_buf =
        ShaderStorageBuffer::from(vec![Vec4::ZERO; global_instance_vec_len]);
    global_instance_vec_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST | BufferUsages::VERTEX;
    let global_instance_vec_handle = buffers.add(global_instance_vec_buf);

    info!(
        "GPU lidar buffers allocated: {} drone slots, {} rays/scan, {} steps/ray, {} hit u32s, {} occupancy u32s ({} words/drone)",
        MAX_DRONES_GPU,
        params.rays_per_scan,
        params.max_steps,
        hits_len,
        occupancy_len,
        words_per_drone,
    );

    commands.insert_resource(GroundTruthBuffer(ground_handle));
    commands.insert_resource(LidarParamsBuffer(params_handle));
    commands.insert_resource(DronePositionsBuffer(positions_handle));
    commands.insert_resource(DroneOrientationsBuffer(orientations_handle));
    commands.insert_resource(RayDirsBuffer(dirs_handle));
    commands.insert_resource(LidarHitsBuffer(hits_handle));
    commands.insert_resource(LocalOccupancyBuffer(occupancy_handle));
    commands.insert_resource(GlobalOccupancyBuffer(global_occupancy_handle));
    commands.insert_resource(BuildLocalParamsBuffer(build_params_handle));
    commands.insert_resource(DroneColorsBuffer(drone_colors_handle));
    commands.insert_resource(LocalInstanceCountBuffer(count_handle));
    commands.insert_resource(LocalInstanceVecBuffer(instance_vec_handle));
    commands.insert_resource(GlobalInstanceCountBuffer(global_count_handle));
    commands.insert_resource(GlobalInstanceVecBuffer(global_instance_vec_handle));
}
