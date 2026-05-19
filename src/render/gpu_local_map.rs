use bevy::camera::visibility::NoFrustumCulling;
use bevy::core_pipeline::core_3d::Transparent3d;
use bevy::ecs::query::QueryItem;
use bevy::pbr::{MeshPipelineKey, RenderMeshInstances};
use bevy::prelude::*;
use bevy::render::extract_component::{ExtractComponent, ExtractComponentPlugin};
use bevy::render::extract_resource::{ExtractResource, ExtractResourcePlugin};
use bevy::render::gpu_readback::{Readback, ReadbackComplete};
use bevy::render::mesh::RenderMesh;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_phase::{DrawFunctions, PhaseItemExtraIndex, ViewSortedRenderPhases};
use bevy::render::render_resource::{PipelineCache, SpecializedMeshPipelines};
use bevy::render::storage::GpuShaderStorageBuffer;
use bevy::render::sync_world::MainEntity;
use bevy::render::view::ExtractedView;
use bevy::render::{Render, RenderApp, RenderSystems};

use crate::lidar::gpu::{LocalInstanceCountBuffer, LocalInstanceVecBuffer};

use super::components::GpuLocalMapVoxel;
use super::instancing::{DrawVoxelInstanced, InstanceBuffer, VoxelInstancedPipeline};
use super::resources::CubeMesh;

/// Latest readback of the GPU-side instance counter. The observer fills
/// this in main world; `ExtractResourcePlugin` mirrors it to render world
/// where `prepare_gpu_local_instance_buffer` reads it.
#[derive(Resource, ExtractResource, Clone, Default)]
pub struct GpuLocalInstanceCount(pub u32);

/// Render-world mirror of `GpuLocalMapVoxel`. The main-world marker
/// can't ride along on the entity sync because Bevy refuses to extract
/// the same component twice; we extract a separate tag.
#[derive(Component, Clone, Default)]
pub struct GpuLocalMapVoxelTag;

impl ExtractComponent for GpuLocalMapVoxel {
    type QueryData = ();
    type QueryFilter = ();
    type Out = GpuLocalMapVoxelTag;

    fn extract_component(_item: QueryItem<'_, '_, Self::QueryData>) -> Option<Self::Out> {
        Some(GpuLocalMapVoxelTag)
    }
}

pub struct GpuLocalMapPlugin;

impl Plugin for GpuLocalMapPlugin {
    fn build(&self, app: &mut App) {
        // ExtractComponentPlugin handles `SyncToRenderWorld` registration
        // for us — registering it explicitly here panics with
        // `DuplicateRegistration`.
        app.init_resource::<GpuLocalInstanceCount>()
            .add_plugins(ExtractComponentPlugin::<GpuLocalMapVoxel>::default())
            .add_plugins(ExtractResourcePlugin::<GpuLocalInstanceCount>::default())
            .add_systems(
                Update,
                (spawn_gpu_local_map_voxel, spawn_count_readback).chain(),
            );

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app.add_systems(
            Render,
            (
                prepare_gpu_local_instance_buffer.in_set(RenderSystems::PrepareResources),
                queue_gpu_local_voxel.in_set(RenderSystems::QueueMeshes),
            ),
        );
    }
}

/// Spawn the single layer entity whose vertex buffer is the GPU-built
/// instance buffer. No `InstancedVoxelLayer` (so the CPU prepare/queue
/// systems skip it) and no `Material` (so Bevy's standard mesh pipeline
/// skips it).
fn spawn_gpu_local_map_voxel(
    mut commands: Commands,
    cube: Option<Res<CubeMesh>>,
    mut spawned: Local<bool>,
) {
    if *spawned {
        return;
    }
    let Some(cube) = cube else {
        return;
    };
    *spawned = true;
    commands.spawn((
        GpuLocalMapVoxel,
        Mesh3d(cube.0.clone()),
        NoFrustumCulling,
        Transform::IDENTITY,
        Visibility::default(),
    ));
}

fn spawn_count_readback(
    mut commands: Commands,
    count_handle: Option<Res<LocalInstanceCountBuffer>>,
    mut spawned: Local<bool>,
) {
    if *spawned {
        return;
    }
    let Some(count_handle) = count_handle else {
        return;
    };
    *spawned = true;
    commands
        .spawn(Readback::buffer(count_handle.0.clone()))
        .observe(
            |event: On<ReadbackComplete>, mut count: ResMut<GpuLocalInstanceCount>| {
                let data: Vec<u32> = event.to_shader_type();
                if let Some(&value) = data.first() {
                    count.0 = value;
                }
            },
        );
}

/// Each frame, point the GpuLocalMapVoxel entity at the GPU instance
/// buffer with the latest readback count. The existing `InstanceBuffer`
/// component matches the layout `DrawVoxelInstanced` already expects.
fn prepare_gpu_local_instance_buffer(
    mut commands: Commands,
    instances_handle: Res<LocalInstanceVecBuffer>,
    count: Res<GpuLocalInstanceCount>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
    layers: Query<Entity, With<GpuLocalMapVoxelTag>>,
) {
    let Some(buf) = buffers.get(&instances_handle.0) else {
        return;
    };
    let length = count.0 as usize;
    for entity in &layers {
        commands.entity(entity).insert(InstanceBuffer {
            buffer: buf.buffer.clone(),
            length,
            // capacity_bytes/last_generation aren't read on this path
            // since we never re-upload from the CPU side.
            capacity_bytes: buf.buffer.size(),
            last_generation: 0,
        });
    }
}

/// Mirrors `queue_voxel_instanced` but filters on `GpuLocalMapVoxelTag`
/// instead of `InstancedVoxelLayer`. Reuses the same pipeline and draw
/// command so the GPU and CPU layers share their vertex shader.
#[allow(clippy::too_many_arguments)]
fn queue_gpu_local_voxel(
    transparent_3d_draw_functions: Res<DrawFunctions<Transparent3d>>,
    custom_pipeline: Res<VoxelInstancedPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<VoxelInstancedPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    layer_meshes: Query<(Entity, &MainEntity), With<GpuLocalMapVoxelTag>>,
    mut transparent_render_phases: ResMut<ViewSortedRenderPhases<Transparent3d>>,
    views: Query<(&ExtractedView, &Msaa)>,
) {
    let draw_function = transparent_3d_draw_functions
        .read()
        .id::<DrawVoxelInstanced>();
    for (view, msaa) in &views {
        let Some(transparent_phase) =
            transparent_render_phases.get_mut(&view.retained_view_entity)
        else {
            continue;
        };
        let msaa_key = MeshPipelineKey::from_msaa_samples(msaa.samples());
        let view_key = msaa_key | MeshPipelineKey::from_hdr(view.hdr);
        let rangefinder = view.rangefinder3d();
        for (entity, main_entity) in &layer_meshes {
            let Some(mesh_instance) =
                render_mesh_instances.render_mesh_queue_data(*main_entity)
            else {
                continue;
            };
            let Some(mesh) = meshes.get(mesh_instance.mesh_asset_id) else {
                continue;
            };
            let key = view_key
                | MeshPipelineKey::from_primitive_topology(mesh.primitive_topology());
            let pipeline = pipelines
                .specialize(&pipeline_cache, &custom_pipeline, key, &mesh.layout)
                .unwrap();
            transparent_phase.add(Transparent3d {
                entity: (entity, *main_entity),
                pipeline,
                draw_function,
                distance: rangefinder.distance(&mesh_instance.center),
                batch_range: 0..1,
                extra_index: PhaseItemExtraIndex::None,
                indexed: true,
            });
        }
    }
}
