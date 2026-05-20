//! Parallel of `instancing/` for the ground-truth voxel cubes.
//!
//! Same per-instance layout (`pos_scale` + `color`), but the mesh is
//! a real unit cube instead of a 2-tri quad, and the shader writes
//! `color.a < 1` so the Transparent3d phase blends the cube over
//! whatever the billboard layers drew underneath. Distinct marker
//! component (`InstancedCubeLayer`) keeps the buffer + queue paths
//! from clashing with the billboard layers.

use std::mem::size_of;

use bevy::asset::RenderAssetUsages;
use bevy::core_pipeline::core_3d::Transparent3d;
use bevy::ecs::system::{
    lifetimeless::{Read, SRes},
    SystemParamItem,
};
use bevy::mesh::{Indices, MeshVertexBufferLayoutRef, PrimitiveTopology, VertexBufferLayout};
use bevy::pbr::{
    MeshPipeline, MeshPipelineKey, RenderMeshInstances, SetMeshBindGroup, SetMeshViewBindGroup,
    SetMeshViewBindingArrayBindGroup,
};
use bevy::prelude::*;
use bevy::render::{
    mesh::{allocator::MeshAllocator, RenderMesh, RenderMeshBufferInfo},
    render_asset::RenderAssets,
    render_phase::{
        AddRenderCommand, DrawFunctions, PhaseItem, PhaseItemExtraIndex, RenderCommand,
        RenderCommandResult, SetItemPipeline, TrackedRenderPass, ViewSortedRenderPhases,
    },
    render_resource::{
        Buffer, BufferDescriptor, BufferUsages, PipelineCache, RenderPipelineDescriptor,
        SpecializedMeshPipeline, SpecializedMeshPipelineError, SpecializedMeshPipelines,
        VertexAttribute, VertexFormat, VertexStepMode,
    },
    renderer::{RenderDevice, RenderQueue},
    sync_world::{MainEntity, RenderEntity, SyncToRenderWorld},
    view::ExtractedView,
    Extract, ExtractSchedule, Render, RenderApp, RenderStartup, RenderSystems,
};

use super::instancing::InstanceData;

const SHADER_ASSET_PATH: &str = "shaders/instanced_cube.wgsl";

#[derive(Resource)]
pub struct CubeMeshHandle(pub Handle<Mesh>);

/// Per-entity GPU instance buffer for the cube path. Same shape as the
/// billboard `InstanceBuffer`, separate component so the two prepare
/// systems don't fight over the same slot.
#[derive(Component)]
pub struct CubeInstanceBuffer {
    pub buffer: Buffer,
    pub length: usize,
    pub capacity_bytes: u64,
    pub last_generation: u32,
}

/// Main-world component carrying the source instance data for the
/// ground-truth cube layer. `data` is uploaded to GPU by
/// `prepare_cube_instance_buffers`; `generation` bumps on full rewrite
/// so the uploader knows to re-send from offset 0.
#[derive(Component, Clone)]
pub struct InstancedCubeLayer {
    pub data: Vec<InstanceData>,
    pub generation: u32,
}

impl InstancedCubeLayer {
    pub fn new(data: Vec<InstanceData>) -> Self {
        Self { data, generation: 1 }
    }
}

#[derive(Resource)]
pub struct CubeInstancedPipeline {
    pub shader: Handle<Shader>,
    pub mesh_pipeline: MeshPipeline,
}

fn init_cube_instanced_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mesh_pipeline: Res<MeshPipeline>,
) {
    commands.insert_resource(CubeInstancedPipeline {
        shader: asset_server.load(SHADER_ASSET_PATH),
        mesh_pipeline: mesh_pipeline.clone(),
    });
}

impl SpecializedMeshPipeline for CubeInstancedPipeline {
    type Key = MeshPipelineKey;

    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayoutRef,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let mut descriptor = self.mesh_pipeline.specialize(key, layout)?;
        descriptor.vertex.shader = self.shader.clone();
        descriptor.vertex.buffers.push(VertexBufferLayout {
            array_stride: size_of::<InstanceData>() as u64,
            step_mode: VertexStepMode::Instance,
            attributes: vec![
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: 0,
                    shader_location: 3,
                },
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: VertexFormat::Float32x4.size(),
                    shader_location: 4,
                },
            ],
        });
        descriptor.fragment.as_mut().unwrap().shader = self.shader.clone();
        // Depth write off so transparent cubes don't occlude the
        // opaque billboard dots that local/central map layers spray
        // *inside* them — that's the "painting" effect.
        if let Some(depth_stencil) = descriptor.depth_stencil.as_mut() {
            depth_stencil.depth_write_enabled = false;
        }
        Ok(descriptor)
    }
}

fn prepare_cube_instance_buffers(
    mut commands: Commands,
    layers_q: Query<(Entity, &InstancedCubeLayer, Option<&CubeInstanceBuffer>)>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    let stride = size_of::<InstanceData>() as u64;
    for (entity, layer, existing) in &layers_q {
        if layer.data.is_empty() {
            commands.entity(entity).remove::<CubeInstanceBuffer>();
            continue;
        }
        let bytes: &[u8] = bytemuck::cast_slice::<InstanceData, u8>(layer.data.as_slice());
        let needed_capacity = bytes.len() as u64;
        let new_length = layer.data.len();
        let new_generation = layer.generation;
        let (buffer, capacity_bytes) = match existing {
            Some(buf) if buf.capacity_bytes >= needed_capacity => {
                if buf.last_generation == new_generation {
                    if new_length > buf.length {
                        let tail_offset = (buf.length as u64) * stride;
                        let tail_start = buf.length * size_of::<InstanceData>();
                        render_queue.write_buffer(&buf.buffer, tail_offset, &bytes[tail_start..]);
                    }
                } else {
                    render_queue.write_buffer(&buf.buffer, 0, bytes);
                }
                (buf.buffer.clone(), buf.capacity_bytes)
            }
            _ => {
                let new_capacity = needed_capacity.saturating_mul(3) / 2;
                let new_buffer = render_device.create_buffer(&BufferDescriptor {
                    label: Some("cube instance buffer"),
                    size: new_capacity,
                    usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                render_queue.write_buffer(&new_buffer, 0, bytes);
                (new_buffer, new_capacity)
            }
        };
        commands.entity(entity).insert(CubeInstanceBuffer {
            buffer,
            length: new_length,
            capacity_bytes,
            last_generation: new_generation,
        });
    }
}

pub type DrawCubeInstanced = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshViewBindingArrayBindGroup<1>,
    SetMeshBindGroup<2>,
    DrawCubeMeshInstanced,
);

pub struct DrawCubeMeshInstanced;

impl<P: PhaseItem> RenderCommand<P> for DrawCubeMeshInstanced {
    type Param = (
        SRes<RenderAssets<RenderMesh>>,
        SRes<RenderMeshInstances>,
        SRes<MeshAllocator>,
    );
    type ViewQuery = ();
    type ItemQuery = Read<CubeInstanceBuffer>;

    #[inline]
    fn render<'w>(
        item: &P,
        _view: (),
        instance_buffer: Option<&'w CubeInstanceBuffer>,
        (meshes, render_mesh_instances, mesh_allocator): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let mesh_allocator = mesh_allocator.into_inner();
        let Some(mesh_instance) =
            render_mesh_instances.render_mesh_queue_data(item.main_entity())
        else {
            return RenderCommandResult::Skip;
        };
        let Some(gpu_mesh) = meshes.into_inner().get(mesh_instance.mesh_asset_id) else {
            return RenderCommandResult::Skip;
        };
        let Some(instance_buffer) = instance_buffer else {
            return RenderCommandResult::Skip;
        };
        let Some(vertex_buffer_slice) =
            mesh_allocator.mesh_vertex_slice(&mesh_instance.mesh_asset_id)
        else {
            return RenderCommandResult::Skip;
        };
        pass.set_vertex_buffer(0, vertex_buffer_slice.buffer.slice(..));
        pass.set_vertex_buffer(1, instance_buffer.buffer.slice(..));
        match &gpu_mesh.buffer_info {
            RenderMeshBufferInfo::Indexed {
                index_format,
                count,
            } => {
                let Some(index_buffer_slice) =
                    mesh_allocator.mesh_index_slice(&mesh_instance.mesh_asset_id)
                else {
                    return RenderCommandResult::Skip;
                };
                pass.set_index_buffer(index_buffer_slice.buffer.slice(..), *index_format);
                pass.draw_indexed(
                    index_buffer_slice.range.start..(index_buffer_slice.range.start + count),
                    vertex_buffer_slice.range.start as i32,
                    0..instance_buffer.length as u32,
                );
            }
            RenderMeshBufferInfo::NonIndexed => {
                pass.draw(vertex_buffer_slice.range, 0..instance_buffer.length as u32);
            }
        }
        RenderCommandResult::Success
    }
}

fn queue_cube_instanced(
    transparent_3d_draw_functions: Res<DrawFunctions<Transparent3d>>,
    custom_pipeline: Res<CubeInstancedPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<CubeInstancedPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    layer_meshes: Query<(Entity, &MainEntity), With<InstancedCubeLayer>>,
    mut transparent_render_phases: ResMut<ViewSortedRenderPhases<Transparent3d>>,
    views: Query<(&ExtractedView, &Msaa)>,
) {
    let draw_function = transparent_3d_draw_functions
        .read()
        .id::<DrawCubeInstanced>();
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

fn extract_cube_layers(
    mut commands: Commands,
    main: Extract<Query<(&RenderEntity, Ref<InstancedCubeLayer>)>>,
) {
    for (render_entity, layer) in &main {
        if !layer.is_changed() {
            continue;
        }
        commands.entity(render_entity.id()).insert(InstancedCubeLayer {
            data: layer.data.clone(),
            generation: layer.generation,
        });
    }
}

fn init_cube_mesh(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    // Hand-rolled unit cube: 24 verts (per-face normals), 12 tris.
    // Vertex positions in [-0.5, +0.5] so the shader can multiply by
    // voxel_size and add the instance center.
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );

    // Six faces. Each face: 4 corners + 6 indices (two triangles).
    // Order: +x, -x, +y, -y, +z, -z.
    let positions: Vec<[f32; 3]> = vec![
        // +x
        [0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [0.5, 0.5, 0.5], [0.5, -0.5, 0.5],
        // -x
        [-0.5, -0.5, 0.5], [-0.5, 0.5, 0.5], [-0.5, 0.5, -0.5], [-0.5, -0.5, -0.5],
        // +y
        [-0.5, 0.5, -0.5], [-0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [0.5, 0.5, -0.5],
        // -y
        [-0.5, -0.5, 0.5], [-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, -0.5, 0.5],
        // +z
        [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5], [-0.5, -0.5, 0.5],
        // -z
        [-0.5, -0.5, -0.5], [-0.5, 0.5, -0.5], [0.5, 0.5, -0.5], [0.5, -0.5, -0.5],
    ];
    let normals: Vec<[f32; 3]> = vec![
        [1.0, 0.0, 0.0]; 4
    ].into_iter()
        .chain(vec![[-1.0, 0.0, 0.0]; 4])
        .chain(vec![[0.0, 1.0, 0.0]; 4])
        .chain(vec![[0.0, -1.0, 0.0]; 4])
        .chain(vec![[0.0, 0.0, 1.0]; 4])
        .chain(vec![[0.0, 0.0, -1.0]; 4])
        .collect();
    let uvs: Vec<[f32; 2]> = vec![[0.0, 0.0]; 24];
    let mut indices: Vec<u32> = Vec::with_capacity(36);
    for face in 0..6u32 {
        let base = face * 4;
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));

    commands.insert_resource(CubeMeshHandle(meshes.add(mesh)));
}

pub struct InstancedCubePlugin;

impl Plugin for InstancedCubePlugin {
    fn build(&self, app: &mut App) {
        app.register_required_components::<InstancedCubeLayer, SyncToRenderWorld>();
        app.add_systems(Startup, init_cube_mesh);
        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawCubeInstanced>()
            .init_resource::<SpecializedMeshPipelines<CubeInstancedPipeline>>()
            .add_systems(RenderStartup, init_cube_instanced_pipeline)
            .add_systems(ExtractSchedule, extract_cube_layers)
            .add_systems(
                Render,
                (
                    queue_cube_instanced.in_set(RenderSystems::QueueMeshes),
                    prepare_cube_instance_buffers.in_set(RenderSystems::PrepareResources),
                ),
            );
    }
}
