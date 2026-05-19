mod dispatch;
mod pipeline;
mod resources;

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::storage::ShaderStorageBuffer;
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

use crate::drone::{Drone, DroneId};
use crate::map::{unflatten, CellState, LocalMap};
use crate::world::WorldConfig;

use dispatch::{add_compute_render_graph_node, prepare_lidar_bind_group, LidarBindGroup};
use pipeline::init_compute_lidar_pipeline;
use resources::{
    setup_gpu_lidar_assets, DroneOrientationsBuffer, DronePositionsBuffer, GroundTruthBuffer,
    LidarHitsBuffer, LidarParams, LidarParamsBuffer, PendingLidarHits, RayDirsBuffer,
    MAX_DRONES_GPU, MAX_STEPS_PER_RAY,
};

use super::constants::RAYS_PER_SCAN;

/// Stage 4: GPU lidar wired to the real per-tick drone query. Each frame
/// we upload the drones' grid positions, orientations, and current count
/// to storage buffers; the render-graph node dispatches the WGSL kernel;
/// the Readback observer stashes the hits and `apply_lidar_hits` folds
/// them into each drone's `LocalMap` via `upgrade()`.
pub struct GpuLidarPlugin;

impl Plugin for GpuLidarPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PendingLidarHits>()
            .add_plugins(ExtractResourcePlugin::<GroundTruthBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarParamsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DronePositionsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DroneOrientationsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<RayDirsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarHitsBuffer>::default())
            .add_systems(
                Update,
                (
                    setup_gpu_lidar_assets
                        .run_if(resource_exists::<crate::world::GroundTruthMap>)
                        .run_if(not(resource_exists::<GroundTruthBuffer>)),
                    spawn_lidar_readback.run_if(resource_exists::<LidarHitsBuffer>),
                    upload_drone_state
                        .run_if(resource_exists::<DronePositionsBuffer>),
                    apply_lidar_hits.after(upload_drone_state),
                ),
            );

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .add_systems(
                RenderStartup,
                (init_compute_lidar_pipeline, add_compute_render_graph_node),
            )
            .add_systems(
                Render,
                prepare_lidar_bind_group
                    .in_set(RenderSystems::PrepareBindGroups)
                    .run_if(not(resource_exists::<LidarBindGroup>)),
            );
    }
}

/// Spawn the persistent Readback entity once the hits buffer exists. The
/// observer stashes the latest result into `PendingLidarHits`; the
/// main-world `apply_lidar_hits` system drains it.
fn spawn_lidar_readback(
    mut commands: Commands,
    hits: Res<LidarHitsBuffer>,
    mut spawned: Local<bool>,
) {
    if *spawned {
        return;
    }
    *spawned = true;
    commands
        .spawn(Readback::buffer(hits.0.clone()))
        .observe(|event: On<ReadbackComplete>, mut pending: ResMut<PendingLidarHits>| {
            pending.0 = Some(event.to_shader_type());
        });
}

/// Sort drones by `DroneId`, write their grid positions, orientations,
/// and count into the storage buffers the GPU lidar reads. Runs each
/// frame; the per-frame cost is tiny (≤50 drones × 32 bytes).
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

/// Drain the latest GPU hits and fold them into each drone's `LocalMap`.
/// Each `(drone_idx, ray, step)` entry packs state and flat-cell index;
/// the trail terminates at the first `0` sentinel or `Occupied` entry.
fn apply_lidar_hits(
    mut pending: ResMut<PendingLidarHits>,
    mut drones: Query<(&DroneId, &mut LocalMap), With<Drone>>,
    config: Res<WorldConfig>,
) {
    let Some(hits) = pending.0.take() else {
        return;
    };
    let dims = config.size;
    let rays = RAYS_PER_SCAN;
    let max_steps = MAX_STEPS_PER_RAY as usize;

    for (id, mut local) in drones.iter_mut() {
        let drone_idx = id.0 as usize;
        if drone_idx >= MAX_DRONES_GPU as usize {
            continue;
        }
        for ray_idx in 0..rays {
            let base = (drone_idx * rays + ray_idx) * max_steps;
            for step in 0..max_steps {
                let entry = hits[base + step];
                if entry == 0 {
                    break;
                }
                let state = entry >> 30;
                let flat = entry & 0x3FFF_FFFF;
                let cell = unflatten(flat, dims);
                let cs = match state {
                    1 => CellState::Free,
                    2 => CellState::Occupied,
                    _ => continue,
                };
                local.0.upgrade(cell, cs);
                if state == 2 {
                    break;
                }
            }
        }
    }
}
