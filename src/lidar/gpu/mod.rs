mod dispatch;
mod pipeline;
mod resources;

use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResourcePlugin;
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

use dispatch::{
    add_compute_render_graph_node, prepare_lidar_bind_group, LidarBindGroup,
};
use pipeline::init_compute_lidar_pipeline;
use resources::{upload_ground_truth_to_gpu, GroundTruthBuffer, LidarCountBuffer};

/// Mirrors the ground-truth map onto the GPU as a packed `u32` bitset and
/// runs a sanity compute pass that counts its set bits each frame. The
/// count is read back via Bevy's `Readback` component; the first non-zero
/// reading is logged once and the entity despawns to stop further reads.
///
/// Foundation for Tier 3 #8: subsequent stages swap the count shader for
/// Amanatides-Woo lidar traversal while reusing the same bind-group shape.
pub struct GpuLidarPlugin;

impl Plugin for GpuLidarPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ExtractResourcePlugin::<GroundTruthBuffer>::default())
            .add_plugins(ExtractResourcePlugin::<LidarCountBuffer>::default())
            .add_systems(
                Update,
                (
                    upload_ground_truth_to_gpu
                        .run_if(resource_exists::<crate::world::GroundTruthMap>)
                        .run_if(not(resource_exists::<GroundTruthBuffer>)),
                    observe_count_readback.run_if(resource_exists::<LidarCountBuffer>),
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

/// Spawn the Readback entity after the count buffer exists. Logs the
/// first non-zero reading, then despawns so we don't keep polling.
fn observe_count_readback(
    mut commands: Commands,
    count: Res<LidarCountBuffer>,
    mut spawned: Local<bool>,
) {
    if *spawned {
        return;
    }
    *spawned = true;
    commands
        .spawn(Readback::buffer(count.0.clone()))
        .observe(|event: On<ReadbackComplete>, mut commands: Commands| {
            let data: Vec<u32> = event.to_shader_type();
            if data.first().copied().unwrap_or(0) == 0 {
                return;
            }
            info!(
                "compute lidar sanity: GPU counted {} set bits in the ground-truth bitset",
                data[0]
            );
            commands.entity(event.entity).despawn();
        });
}
