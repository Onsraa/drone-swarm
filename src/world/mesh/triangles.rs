use bevy::math::{Mat4, Vec3, Vec3A};
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

/// Compute an AABB of the central percentile range of triangles,
/// filtering by per-axis centroid. Discards outliers like sky-domes,
/// distant helper geometry, or stray vertices left over from .glb
/// authoring. `low` + `high` are fractions in [0.0, 1.0]; pass e.g.
/// `(0.05, 0.95)` to trim the lowest + highest 5% along each axis.
/// Falls back to the full AABB if filtering excludes every triangle.
pub fn percentile_trimmed_aabb(triangles: &[Triangle], low: f32, high: f32) -> (Vec3, Vec3) {
    if triangles.is_empty() {
        return (Vec3::ZERO, Vec3::ZERO);
    }
    let n = triangles.len();
    let centroid_axis = |axis: u8| -> Vec<f32> {
        let mut c: Vec<f32> = triangles
            .iter()
            .map(|t| {
                let s = t.v0 + t.v1 + t.v2;
                match axis {
                    0 => s.x / 3.0,
                    1 => s.y / 3.0,
                    _ => s.z / 3.0,
                }
            })
            .collect();
        c.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        c
    };
    let xs = centroid_axis(0);
    let ys = centroid_axis(1);
    let zs = centroid_axis(2);
    let lo_idx = ((n as f32) * low).clamp(0.0, (n - 1) as f32) as usize;
    let hi_idx = ((n as f32) * high).clamp(0.0, (n - 1) as f32) as usize;
    let bounds = Vec3::new(xs[lo_idx], ys[lo_idx], zs[lo_idx]);
    let bounds_hi = Vec3::new(xs[hi_idx], ys[hi_idx], zs[hi_idx]);

    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    let mut included = 0usize;
    for tri in triangles {
        let s = tri.v0 + tri.v1 + tri.v2;
        let cx = s.x / 3.0;
        let cy = s.y / 3.0;
        let cz = s.z / 3.0;
        if cx < bounds.x || cx > bounds_hi.x {
            continue;
        }
        if cy < bounds.y || cy > bounds_hi.y {
            continue;
        }
        if cz < bounds.z || cz > bounds_hi.z {
            continue;
        }
        included += 1;
        for v in [tri.v0, tri.v1, tri.v2] {
            min = min.min(Vec3::new(v.x, v.y, v.z));
            max = max.max(Vec3::new(v.x, v.y, v.z));
        }
    }

    if included == 0 {
        let mut full_min = Vec3::splat(f32::INFINITY);
        let mut full_max = Vec3::splat(f32::NEG_INFINITY);
        for tri in triangles {
            for v in [tri.v0, tri.v1, tri.v2] {
                full_min = full_min.min(Vec3::new(v.x, v.y, v.z));
                full_max = full_max.max(Vec3::new(v.x, v.y, v.z));
            }
        }
        return (full_min, full_max);
    }
    (min, max)
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

    fn tri_at(centre: Vec3A, half: f32) -> Triangle {
        Triangle {
            v0: centre + Vec3A::new(-half, -half, 0.0),
            v1: centre + Vec3A::new(half, -half, 0.0),
            v2: centre + Vec3A::new(0.0, half, 0.0),
        }
    }

    #[test]
    fn percentile_trim_excludes_y_outlier() {
        // 100 triangles in [-5, 5] cube + 1 outlier at y=10000.
        let mut tris: Vec<Triangle> = Vec::new();
        for i in 0..100 {
            let f = (i as f32) / 100.0;
            tris.push(tri_at(Vec3A::new(f * 10.0 - 5.0, f * 10.0 - 5.0, f * 10.0 - 5.0), 0.5));
        }
        tris.push(tri_at(Vec3A::new(0.0, 10_000.0, 0.0), 0.5));

        let (min, max) = percentile_trimmed_aabb(&tris, 0.05, 0.95);
        assert!(max.y < 100.0, "y-outlier should be trimmed, got max.y = {}", max.y);
        // bulk extends ~ -5..5
        assert!(max.y < 10.0, "trimmed max.y should be near the bulk, got {}", max.y);
    }

    #[test]
    fn percentile_trim_returns_full_for_uniform_distribution() {
        // 100 small triangles all roughly in a 10m cube around origin.
        let mut tris: Vec<Triangle> = Vec::new();
        for i in 0..100 {
            let f = (i as f32) / 100.0;
            tris.push(tri_at(Vec3A::new(f * 10.0 - 5.0, f * 10.0 - 5.0, f * 10.0 - 5.0), 0.1));
        }
        let (min, max) = percentile_trimmed_aabb(&tris, 0.05, 0.95);
        // Extent should be roughly the centroid range minus some endpoints.
        assert!(max.x - min.x < 11.0);
        assert!(max.x - min.x > 5.0);
    }

    #[test]
    fn percentile_trim_falls_back_for_empty() {
        let (min, max) = percentile_trimmed_aabb(&[], 0.05, 0.95);
        assert_eq!(min, Vec3::ZERO);
        assert_eq!(max, Vec3::ZERO);
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
