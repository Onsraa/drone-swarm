use bevy::core_pipeline::core_3d::Transparent3d;
use bevy::pbr::{MeshPipelineKey, RenderMeshInstances};
use bevy::prelude::*;
use bevy::render::{
    mesh::RenderMesh,
    render_asset::RenderAssets,
    render_phase::{DrawFunctions, PhaseItemExtraIndex, ViewSortedRenderPhases},
    render_resource::{PipelineCache, SpecializedMeshPipelines},
    sync_world::MainEntity,
    view::ExtractedView,
};

use super::components::InstancedVoxelLayer;
use super::draw::DrawVoxelInstanced;
use super::pipeline::VoxelInstancedPipeline;

pub fn queue_voxel_instanced(
    transparent_3d_draw_functions: Res<DrawFunctions<Transparent3d>>,
    custom_pipeline: Res<VoxelInstancedPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<VoxelInstancedPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    layer_meshes: Query<(Entity, &MainEntity), With<InstancedVoxelLayer>>,
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
