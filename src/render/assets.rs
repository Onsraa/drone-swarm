use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use super::resources::CubeMesh;

/// Shared "point quad" mesh used by every instanced point-cloud layer.
/// Two triangles in the XY plane with vertex positions at `(±1, ±1, 0)`
/// so the billboard vertex shader can read `position.xy` as the corner
/// offset of the screen-space dot. The legacy `CubeMesh` resource name
/// is kept to avoid churning every layer's `Mesh3d` handle.
pub fn init_voxel_assets(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    let mut quad = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    quad.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
        ],
    );
    quad.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 0.0, 1.0]; 4]);
    quad.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0]; 4]);
    quad.insert_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3]));

    let handle = meshes.add(quad);
    commands.insert_resource(CubeMesh(handle));
}
