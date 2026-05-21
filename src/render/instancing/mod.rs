mod buffer;
mod components;
mod draw;
mod pipeline;
mod queue;

use bevy::core_pipeline::core_3d::Transparent3d;
use bevy::prelude::*;
use bevy::render::render_phase::AddRenderCommand;
use bevy::render::render_resource::SpecializedMeshPipelines;
use bevy::render::sync_world::{RenderEntity, SyncToRenderWorld};
use bevy::render::{Extract, ExtractSchedule, Render, RenderApp, RenderStartup, RenderSystems};

pub use buffer::InstanceBuffer;
pub use components::InstancedVoxelLayer;
pub use draw::DrawVoxelInstanced;
pub use pipeline::VoxelInstancedPipeline;

use buffer::prepare_instance_buffers;
use pipeline::init_voxel_instanced_pipeline;
use queue::queue_voxel_instanced;

pub struct InstancedVoxelPlugin;

impl Plugin for InstancedVoxelPlugin {
    fn build(&self, app: &mut App) {
        app.register_required_components::<InstancedVoxelLayer, SyncToRenderWorld>();
        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawVoxelInstanced>()
            .init_resource::<SpecializedMeshPipelines<VoxelInstancedPipeline>>()
            .add_systems(RenderStartup, init_voxel_instanced_pipeline)
            .add_systems(ExtractSchedule, extract_voxel_layers)
            .add_systems(
                Render,
                (
                    queue_voxel_instanced.in_set(RenderSystems::QueueMeshes),
                    prepare_instance_buffers.in_set(RenderSystems::PrepareResources),
                ),
            );
    }
}

/// Change-gated replacement for `ExtractComponentPlugin::<InstancedVoxelLayer>`.
/// The plugin would clone the whole `Vec<InstanceData>` from main to render
/// every frame; with the append-only local-map path the Vec grows
/// monotonically, so that clone cost scales with session length. Here we
/// only clone when the source layer was actually mutated (or first-seen).
fn extract_voxel_layers(
    mut commands: Commands,
    main: Extract<Query<(&RenderEntity, Ref<InstancedVoxelLayer>)>>,
) {
    for (render_entity, layer) in &main {
        if !layer.is_changed() {
            continue;
        }
        commands.entity(render_entity.id()).insert(InstancedVoxelLayer {
            data: layer.data.clone(),
            generation: layer.generation,
        });
    }
}
