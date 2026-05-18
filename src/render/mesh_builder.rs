use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

/// Pre-baked face data for a unit cube centered at the origin: each face is
/// `(normal, 4 corner offsets in CCW order)`. Half-extents are inserted at
/// build time so the same table works for any `voxel_size`.
fn cube_faces(half: f32) -> [(Vec3, [Vec3; 4]); 6] {
    [
        (
            Vec3::X,
            [
                Vec3::new(half, -half, -half),
                Vec3::new(half, half, -half),
                Vec3::new(half, half, half),
                Vec3::new(half, -half, half),
            ],
        ),
        (
            Vec3::NEG_X,
            [
                Vec3::new(-half, -half, half),
                Vec3::new(-half, half, half),
                Vec3::new(-half, half, -half),
                Vec3::new(-half, -half, -half),
            ],
        ),
        (
            Vec3::Y,
            [
                Vec3::new(-half, half, -half),
                Vec3::new(-half, half, half),
                Vec3::new(half, half, half),
                Vec3::new(half, half, -half),
            ],
        ),
        (
            Vec3::NEG_Y,
            [
                Vec3::new(-half, -half, half),
                Vec3::new(-half, -half, -half),
                Vec3::new(half, -half, -half),
                Vec3::new(half, -half, half),
            ],
        ),
        (
            Vec3::Z,
            [
                Vec3::new(half, -half, half),
                Vec3::new(half, half, half),
                Vec3::new(-half, half, half),
                Vec3::new(-half, -half, half),
            ],
        ),
        (
            Vec3::NEG_Z,
            [
                Vec3::new(-half, -half, -half),
                Vec3::new(-half, half, -half),
                Vec3::new(half, half, -half),
                Vec3::new(half, -half, -half),
            ],
        ),
    ]
}

/// Build a single triangle-list mesh containing one axis-aligned cube per
/// occupied cell. Uses 24 vertices per cube (4 per face) so each face gets
/// its own normal for flat shading. One mesh = one draw call per layer.
pub fn build_voxel_chunk_mesh(
    cells: impl IntoIterator<Item = IVec3>,
    voxel_size: f32,
) -> Mesh {
    let half = voxel_size * 0.5;
    let faces = cube_faces(half);

    let cells_vec: Vec<IVec3> = cells.into_iter().collect();
    let face_count = cells_vec.len() * 6;
    let vertex_count = face_count * 4;
    let index_count = face_count * 6;

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(vertex_count);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(vertex_count);
    let mut indices: Vec<u32> = Vec::with_capacity(index_count);

    for cell in &cells_vec {
        let center = cell.as_vec3() * voxel_size + Vec3::splat(half);
        for (normal, corners) in &faces {
            let base = positions.len() as u32;
            for corner in corners {
                positions.push((center + *corner).to_array());
                normals.push(normal.to_array());
            }
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);
            indices.push(base);
            indices.push(base + 2);
            indices.push(base + 3);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

pub fn empty_voxel_mesh() -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, Vec::<[f32; 3]>::new());
    mesh.insert_indices(Indices::U32(Vec::new()));
    mesh
}
