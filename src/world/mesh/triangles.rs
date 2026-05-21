use bevy::math::{Mat4, Vec3A};
use bevy::mesh::{Indices, Mesh, VertexAttributeValues};
use obvhs::triangle::Triangle;

/// Flatten a single Bevy `Mesh` into a Vec of `obvhs::Triangle` in world
/// space. Returns an empty vec if the mesh lacks a Float32x3 position
/// attribute or doesn't carry indices. Caller composes per-mesh `Mat4`
/// from the parent SceneRoot's hierarchy.
pub fn extract_triangles_from_mesh(mesh: &Mesh, transform: Mat4) -> Vec<Triangle> {
    let Some(positions) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) else {
        return Vec::new();
    };
    let positions = match positions {
        VertexAttributeValues::Float32x3(v) => v,
        _ => return Vec::new(),
    };
    let world_positions: Vec<Vec3A> = positions
        .iter()
        .map(|p| transform.transform_point3a(Vec3A::from_array(*p)))
        .collect();

    let indices: Vec<u32> = match mesh.indices() {
        Some(Indices::U32(v)) => v.clone(),
        Some(Indices::U16(v)) => v.iter().map(|i| *i as u32).collect(),
        None => return Vec::new(),
    };

    indices
        .chunks_exact(3)
        .map(|tri| Triangle {
            v0: world_positions[tri[0] as usize],
            v1: world_positions[tri[1] as usize],
            v2: world_positions[tri[2] as usize],
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::asset::RenderAssetUsages;
    use bevy::mesh::PrimitiveTopology;

    fn unit_quad_xz() -> Mesh {
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
            ],
        );
        mesh.insert_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3]));
        mesh
    }

    #[test]
    fn extracts_two_triangles_from_unit_quad() {
        let mesh = unit_quad_xz();
        let tris = extract_triangles_from_mesh(&mesh, Mat4::IDENTITY);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn applies_translation_to_vertices() {
        let mesh = unit_quad_xz();
        let xform = Mat4::from_translation(bevy::math::Vec3::new(10.0, 0.0, 0.0));
        let tris = extract_triangles_from_mesh(&mesh, xform);
        assert_eq!(tris[0].v0, Vec3A::new(10.0, 0.0, 0.0));
        assert_eq!(tris[0].v1, Vec3A::new(11.0, 0.0, 0.0));
    }

    #[test]
    fn returns_empty_on_missing_indices() {
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0, 0.0, 0.0]]);
        let tris = extract_triangles_from_mesh(&mesh, Mat4::IDENTITY);
        assert!(tris.is_empty());
    }

    #[test]
    fn handles_u16_indices() {
        let mut mesh = unit_quad_xz();
        let _ = mesh.indices();
        // Swap to U16 to ensure widening path works.
        mesh.insert_indices(Indices::U16(vec![0, 1, 2]));
        let tris = extract_triangles_from_mesh(&mesh, Mat4::IDENTITY);
        assert_eq!(tris.len(), 1);
    }
}
