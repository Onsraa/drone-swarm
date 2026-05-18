use std::mem::size_of;

use bevy::mesh::{MeshVertexBufferLayoutRef, VertexBufferLayout};
use bevy::pbr::{MeshPipeline, MeshPipelineKey};
use bevy::prelude::*;
use bevy::render::render_resource::{
    RenderPipelineDescriptor, SpecializedMeshPipeline, SpecializedMeshPipelineError, VertexAttribute,
    VertexFormat, VertexStepMode,
};

use super::components::InstanceData;

const SHADER_ASSET_PATH: &str = "shaders/instanced_voxel.wgsl";

#[derive(Resource)]
pub struct VoxelInstancedPipeline {
    pub shader: Handle<Shader>,
    pub mesh_pipeline: MeshPipeline,
}

pub fn init_voxel_instanced_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mesh_pipeline: Res<MeshPipeline>,
) {
    commands.insert_resource(VoxelInstancedPipeline {
        shader: asset_server.load(SHADER_ASSET_PATH),
        mesh_pipeline: mesh_pipeline.clone(),
    });
}

impl SpecializedMeshPipeline for VoxelInstancedPipeline {
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
        Ok(descriptor)
    }
}
