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

use crate::lidar::gpu::{GlobalInstanceCountBuffer, GlobalInstanceVecBuffer};

use super::components::GpuGlobalMapVoxel;
use super::instancing::{DrawVoxelInstanced, InstanceBuffer, VoxelInstancedPipeline};
use super::resources::CubeMesh;

#[derive(Resource, ExtractResource, Clone, Default)]
pub struct GpuGlobalInstanceCount(pub u32);

#[derive(Component, Clone, Default)]
pub struct GpuGlobalMapVoxelTag;

impl ExtractComponent for GpuGlobalMapVoxel {
    type QueryData = &'static GpuGlobalMapVoxel;
    type QueryFilter = ();
    type Out = GpuGlobalMapVoxelTag;

    fn extract_component(_item: QueryItem<'_, '_, Self::QueryData>) -> Option<Self::Out> {
        Some(GpuGlobalMapVoxelTag)
    }
}

pub struct GpuGlobalMapPlugin;

impl Plugin for GpuGlobalMapPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GpuGlobalInstanceCount>()
            .add_plugins(ExtractComponentPlugin::<GpuGlobalMapVoxel>::default())
            .add_plugins(ExtractResourcePlugin::<GpuGlobalInstanceCount>::default())
            .add_systems(
                Update,
                (spawn_gpu_global_map_voxel, spawn_global_count_readback).chain(),
            );

        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app.add_systems(
            Render,
            (
                prepare_gpu_global_instance_buffer.in_set(RenderSystems::PrepareResources),
                queue_gpu_global_voxel.in_set(RenderSystems::QueueMeshes),
            ),
        );
    }
}

#[derive(Component)]
pub struct GlobalCountReadbackTag;

fn spawn_gpu_global_map_voxel(
    mut commands: Commands,
    cube: Option<Res<CubeMesh>>,
    existing: Query<(), With<GpuGlobalMapVoxel>>,
) {
    if !existing.is_empty() {
        return;
    }
    let Some(cube) = cube else {
        return;
    };
    commands.spawn((
        GpuGlobalMapVoxel,
        Mesh3d(cube.0.clone()),
        NoFrustumCulling,
        Transform::IDENTITY,
        Visibility::default(),
    ));
}

fn spawn_global_count_readback(
    mut commands: Commands,
    count_handle: Option<Res<GlobalInstanceCountBuffer>>,
    existing: Query<(), With<GlobalCountReadbackTag>>,
) {
    if !existing.is_empty() {
        return;
    }
    let Some(count_handle) = count_handle else {
        return;
    };
    commands
        .spawn((
            Readback::buffer(count_handle.0.clone()),
            GlobalCountReadbackTag,
        ))
        .observe(
            |event: On<ReadbackComplete>, mut count: ResMut<GpuGlobalInstanceCount>| {
                let data: Vec<u32> = event.to_shader_type();
                if let Some(&value) = data.first() {
                    count.0 = value;
                }
            },
        );
}

fn prepare_gpu_global_instance_buffer(
    mut commands: Commands,
    instances_handle: Option<Res<GlobalInstanceVecBuffer>>,
    count: Res<GpuGlobalInstanceCount>,
    buffers: Res<RenderAssets<GpuShaderStorageBuffer>>,
    layers: Query<Entity, With<GpuGlobalMapVoxelTag>>,
) {
    let Some(instances_handle) = instances_handle else {
        return;
    };
    let Some(buf) = buffers.get(&instances_handle.0) else {
        return;
    };
    let length = count.0 as usize;
    for entity in &layers {
        commands.entity(entity).insert(InstanceBuffer {
            buffer: buf.buffer.clone(),
            length,
            capacity_bytes: buf.buffer.size(),
            last_generation: 0,
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn queue_gpu_global_voxel(
    transparent_3d_draw_functions: Res<DrawFunctions<Transparent3d>>,
    custom_pipeline: Res<VoxelInstancedPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<VoxelInstancedPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    layer_meshes: Query<(Entity, &MainEntity), With<GpuGlobalMapVoxelTag>>,
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
