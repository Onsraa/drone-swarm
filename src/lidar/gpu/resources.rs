use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{BufferUsages, ShaderType};
use bevy::render::storage::ShaderStorageBuffer;

use crate::world::GroundTruthMap;

use super::super::constants::RAYS_PER_SCAN;
use super::super::sampling::LidarRayDirs;

pub const MAX_STEPS_PER_RAY: u32 = 96;
pub const MAX_DRONES_GPU: u32 = 64;

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
pub struct RayDirsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LidarHitsBuffer(pub Handle<ShaderStorageBuffer>);

/// One-shot startup: packs the CPU ground truth, allocates the lidar
/// inputs/outputs as `ShaderStorageBuffer` assets, and seeds Stage-3 stub
/// data — one drone hovering at world-space (32.5, 5.5, 32.5) so a few
/// rays should hit the floor and the obstacle clusters.
pub fn setup_gpu_lidar_assets(
    mut commands: Commands,
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    ground: Res<GroundTruthMap>,
    ray_dirs_res: Res<LidarRayDirs>,
) {
    // Ground-truth bitset.
    let bitset = ground.pack_bitset();
    let mut ground_buf = ShaderStorageBuffer::from(bitset);
    ground_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let ground_handle = buffers.add(ground_buf);

    // Params (stub: one drone).
    let params = LidarParams {
        dims: UVec4::new(ground.dims.x, ground.dims.y, ground.dims.z, 0),
        max_steps: MAX_STEPS_PER_RAY,
        rays_per_scan: RAYS_PER_SCAN as u32,
        drone_count: 1,
        _pad: 0,
    };
    let mut params_buf = ShaderStorageBuffer::from(params);
    params_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let params_handle = buffers.add(params_buf);

    // Drone positions (stub: one drone at the floor's clear airspace).
    let drone_positions: Vec<Vec4> = (0..MAX_DRONES_GPU)
        .map(|i| {
            if i == 0 {
                Vec4::new(32.5, 5.5, 32.5, 0.0)
            } else {
                Vec4::ZERO
            }
        })
        .collect();
    let mut positions_buf = ShaderStorageBuffer::from(drone_positions);
    positions_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let positions_handle = buffers.add(positions_buf);

    // Ray dirs (cached fibonacci sphere, padded to vec4).
    let ray_dirs: Vec<Vec4> = ray_dirs_res
        .0
        .iter()
        .map(|d| Vec4::new(d.x, d.y, d.z, 0.0))
        .collect();
    let mut dirs_buf = ShaderStorageBuffer::from(ray_dirs);
    dirs_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let dirs_handle = buffers.add(dirs_buf);

    // Hits output: drones * rays * steps.
    let hits_len =
        (MAX_DRONES_GPU * params.rays_per_scan * params.max_steps) as usize;
    let mut hits_buf = ShaderStorageBuffer::from(vec![0u32; hits_len]);
    hits_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let hits_handle = buffers.add(hits_buf);

    info!(
        "GPU lidar buffers allocated: {} drone slots, {} rays/scan, {} steps/ray, {} hit u32s",
        MAX_DRONES_GPU, params.rays_per_scan, params.max_steps, hits_len
    );

    commands.insert_resource(GroundTruthBuffer(ground_handle));
    commands.insert_resource(LidarParamsBuffer(params_handle));
    commands.insert_resource(DronePositionsBuffer(positions_handle));
    commands.insert_resource(RayDirsBuffer(dirs_handle));
    commands.insert_resource(LidarHitsBuffer(hits_handle));
}
