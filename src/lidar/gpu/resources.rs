use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{BufferUsages, ShaderType};
use bevy::render::storage::ShaderStorageBuffer;

use crate::world::GroundTruthMap;

use super::super::constants::RAYS_PER_SCAN;
use super::super::sampling::LidarRayDirs;

pub const MAX_STEPS_PER_RAY: u32 = 96;
pub const MAX_DRONES_GPU: u32 = 64;

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
}
