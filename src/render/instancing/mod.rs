mod buffer;
mod components;
mod draw;
mod pipeline;
mod queue;

use bevy::core_pipeline::core_3d::Transparent3d;
use bevy::ecs::query::QueryItem;
use bevy::prelude::*;
use bevy::render::extract_component::{ExtractComponent, ExtractComponentPlugin};
use bevy::render::render_phase::AddRenderCommand;
use bevy::render::render_resource::SpecializedMeshPipelines;
use bevy::render::{Render, RenderApp, RenderStartup, RenderSystems};

pub use components::{InstanceData, InstancedVoxelLayer};

use buffer::prepare_instance_buffers;
use draw::DrawVoxelInstanced;
use pipeline::{init_voxel_instanced_pipeline, VoxelInstancedPipeline};
use queue::queue_voxel_instanced;

impl ExtractComponent for InstancedVoxelLayer {
    type QueryData = &'static InstancedVoxelLayer;
    type QueryFilter = ();
    type Out = Self;

    fn extract_component(item: QueryItem<'_, '_, Self::QueryData>) -> Option<Self> {
        Some(item.clone())
    }
}

pub struct InstancedVoxelPlugin;

impl Plugin for InstancedVoxelPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(ExtractComponentPlugin::<InstancedVoxelLayer>::default());
        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawVoxelInstanced>()
            .init_resource::<SpecializedMeshPipelines<VoxelInstancedPipeline>>()
            .add_systems(RenderStartup, init_voxel_instanced_pipeline)
            .add_systems(
                Render,
                (
                    queue_voxel_instanced.in_set(RenderSystems::QueueMeshes),
                    prepare_instance_buffers.in_set(RenderSystems::PrepareResources),
                ),
            );
    }
}
