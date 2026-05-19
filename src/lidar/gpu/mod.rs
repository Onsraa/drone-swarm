mod dispatch;
mod pipeline;
mod resources;

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

use dispatch::{add_compute_render_graph_node, prepare_lidar_bind_group, LidarBindGroup};
use pipeline::init_compute_lidar_pipeline;
use resources::{
    setup_gpu_lidar_assets, DronePositionsBuffer, GroundTruthBuffer, LidarHitsBuffer,
    LidarParamsBuffer, RayDirsBuffer,
};

use super::constants::RAYS_PER_SCAN;
use resources::MAX_STEPS_PER_RAY;

/// Stage 3: GPU lidar with stub one-drone input. WGSL Amanatides-Woo
/// writes per-(drone, ray) hit trails into the hits storage buffer.
/// Stage 4 will swap the stub for per-tick drone uploads and feed the
/// hits back into `LocalMap` via `upgrade()`.
pub struct GpuLidarPlugin;

impl Plugin for GpuLidarPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ExtractResourcePlugin::<GroundTruthBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarParamsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<DronePositionsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<RayDirsBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarHitsBuffer>::default())
            .add_systems(
                Update,
                (
                    setup_gpu_lidar_assets
                        .run_if(resource_exists::<crate::world::GroundTruthMap>)
                        .run_if(not(resource_exists::<GroundTruthBuffer>)),
                    observe_lidar_readback.run_if(resource_exists::<LidarHitsBuffer>),
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

/// Stage-3 sanity readback. Spawns a `Readback` on the hits buffer, logs
/// the first ray's trail of the first drone, then despawns itself.
fn observe_lidar_readback(
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
        .observe(|event: On<ReadbackComplete>, mut commands: Commands| {
            let data: Vec<u32> = event.to_shader_type();
            // Skip frames where the buffer is still all-zeros.
            if data.iter().take(MAX_STEPS_PER_RAY as usize).all(|&v| v == 0) {
                return;
            }
            describe_first_ray(&data);
            commands.entity(event.entity).despawn();
        });
}

fn describe_first_ray(data: &[u32]) {
    let max_steps = MAX_STEPS_PER_RAY as usize;
    let ray_count = RAYS_PER_SCAN;
    let mut log = String::from("GPU lidar stub: drone 0 trails:");
    for ray in 0..3.min(ray_count) {
        let base = ray * max_steps;
        log.push_str(&format!("\n  ray {}: ", ray));
        for step in 0..max_steps {
            let entry = data[base + step];
            if entry == 0 {
                log.push_str(&format!("[end at step {}]", step));
                break;
            }
            let state = entry >> 30;
            let flat = entry & 0x3FFFFFFF;
            let tag = match state {
                1 => "F",
                2 => "O",
                _ => "?",
            };
            log.push_str(&format!("{}{} ", tag, flat));
            if state == 2 {
                break;
            }
        }
    }
    info!("{}", log);
}
