use bevy::math::{Vec3, Vec3A};
use bevy::prelude::*;
use obvhs::cwbvh::{builder::build_cwbvh_from_tris, CwBvh};
use obvhs::ray::{Ray, RayHit};
use obvhs::triangle::Triangle;
use obvhs::BvhBuildParams;
use std::time::Duration;

#[derive(Resource)]
#[allow(dead_code)]
pub struct WorldBvh {
    pub triangles: Vec<Triangle>,
    pub cwbvh: CwBvh,
    /// Material palette index per triangle (parallel to `triangles`,
    /// indexed by the unindexed-triangle index that `bvh.primitive_indices`
    /// resolves to).
    pub tri_materials: Vec<u32>,
    /// Per-material flat albedo. `(r, g, b, a)` in linear space, ready
    /// for direct GPU sampling. Index by the values in `tri_materials`.
    pub material_palette: Vec<Vec4>,
}

/// Build a CWBVH8 from a triangle list using obvhs' medium-quality
/// build preset. Builder is one-shot per scene load; expect ~100 ms for
/// 2 M tris on M4 Pro. Caller owns the triangle list lifetime. Tests +
/// pure helpers use this overload; the scene-walk path uses
/// `build_world_bvh_with_materials` to attach the real palette.
#[allow(dead_code)]
pub fn build_world_bvh(triangles: Vec<Triangle>) -> WorldBvh {
    let n = triangles.len();
    build_world_bvh_with_materials(triangles, vec![0u32; n], vec![Vec4::ONE])
}

/// Full builder: triangles + per-triangle material indices + palette.
/// `tri_materials.len()` must equal `triangles.len()` (one mat-id per
/// unindexed triangle).
pub fn build_world_bvh_with_materials(
    triangles: Vec<Triangle>,
    tri_materials: Vec<u32>,
    material_palette: Vec<Vec4>,
) -> WorldBvh {
    debug_assert_eq!(tri_materials.len(), triangles.len());
    let cwbvh = build_cwbvh_from_tris(
        &triangles,
        BvhBuildParams::medium_build(),
        &mut Duration::default(),
    );
    WorldBvh {
        triangles,
        cwbvh,
        tri_materials,
        material_palette,
    }
}

/// Cast a ray into the BVH. Returns the hit distance `t` along the ray
/// if any primitive was intersected. Direction is normalized internally.
/// Used in Phase 3 to port `raycast_dda` off the voxel grid.
pub fn cast_ray(bvh: &WorldBvh, origin: Vec3, direction: Vec3) -> Option<f32> {
    let ray = Ray::new_inf(
        Vec3A::new(origin.x, origin.y, origin.z),
        Vec3A::new(direction.x, direction.y, direction.z).normalize(),
    );
    let mut ray_hit = RayHit::none();
    let did_hit = bvh.cwbvh.ray_traverse(ray, &mut ray_hit, |ray, id| {
        bvh.triangles[bvh.cwbvh.primitive_indices[id] as usize].intersect(ray)
    });
    if did_hit {
        Some(ray_hit.t)
    } else {
        None
    }
}

/// Compute a recommended `(translation, scale)` that horizontally
/// fits the mesh AABB into `coverage` × world horizontal extent and
/// floor-aligns `min.y` to world `y = 0`. `aabb_min/max` are in
/// world meters at the *currently applied* transform; the helper
/// inverts that transform so the result is the new full transform
/// (not a delta).
pub fn recommended_transform(
    aabb_min: Vec3,
    aabb_max: Vec3,
    world_size: Vec3,
    current_translation: Vec3,
    current_scale: f32,
    coverage: f32,
) -> (Vec3, f32) {
    let extent = aabb_max - aabb_min;
    let horiz_extent = extent.x.max(extent.z).max(1.0e-6);
    let world_horiz = world_size.x.max(world_size.z);
    let target = world_horiz * coverage;
    let scale_mult = target / horiz_extent;
    let new_scale = current_scale * scale_mult;

    let aabb_center = (aabb_min + aabb_max) * 0.5;
    let local_center = aabb_center - current_translation;
    let local_min = aabb_min - current_translation;
    let world_center = world_size * 0.5;

    let new_translation = Vec3::new(
        world_center.x - local_center.x * scale_mult,
        -local_min.y * scale_mult,
        world_center.z - local_center.z * scale_mult,
    );

    (new_translation, new_scale)
}

/// World-Y of the first downward-hit on the BVH at world (x, z).
/// Casts from `sky_y` straight down; returns `Some(hit_y)` on hit,
/// `None` if no geometry is below the sky point.
pub fn ground_altitude(bvh: &WorldBvh, x: f32, z: f32, sky_y: f32) -> Option<f32> {
    let t = cast_ray(
        bvh,
        Vec3::new(x, sky_y, z),
        Vec3::new(0.0, -1.0, 0.0),
    )?;
    Some(sky_y - t)
}

/// Cast a ray with a finite maximum distance. Returns `(endpoint, hit)`
/// where `endpoint` is the world position of the first hit (when
/// `hit == true`) or `origin + unit_dir * max_dist` on miss. Direction
/// is normalized internally; `Vec3::ZERO` returns `(origin, false)`.
/// Same `(endpoint, hit)` contract as the legacy `raycast_dda`.
pub fn raycast_bvh(bvh: &WorldBvh, origin: Vec3, dir: Vec3, max_dist: f32) -> (Vec3, bool) {
    let unit_dir = dir.normalize_or_zero();
    if unit_dir == Vec3::ZERO {
        return (origin, false);
    }
    if let Some(t) = cast_ray(bvh, origin, unit_dir) {
        if t <= max_dist {
            return (origin + unit_dir * t, true);
        }
    }
    (origin + unit_dir * max_dist, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn single_xz_triangle() -> Triangle {
        Triangle {
            v0: Vec3A::new(0.0, 0.0, 0.0),
            v1: Vec3A::new(2.0, 0.0, 0.0),
            v2: Vec3A::new(0.0, 0.0, 2.0),
        }
    }

    #[test]
    fn ray_from_above_hits_triangle() {
        let bvh = build_world_bvh(vec![single_xz_triangle()]);
        // ray origin above (0.25, 0.25) on the XZ plane, pointing down
        let hit = cast_ray(
            &bvh,
            Vec3::new(0.25, 5.0, 0.25),
            Vec3::new(0.0, -1.0, 0.0),
        );
        assert!(hit.is_some(), "ray pointing down at triangle must hit");
        let t = hit.unwrap();
        assert!(
            (t - 5.0).abs() < 1e-3,
            "expected hit distance ~5.0, got {}",
            t
        );
    }

    fn assert_vec3_near(actual: Vec3, expected: Vec3, eps: f32) {
        assert!(
            (actual - expected).length() < eps,
            "expected ~{:?}, got {:?}",
            expected,
            actual
        );
    }

    const WORLD_640: Vec3 = Vec3::new(640.0, 24.0, 640.0);

    #[test]
    fn auto_fit_upscales_tiny_mesh() {
        // 10m cube centred at origin, current transform = identity.
        let (t, s) = recommended_transform(
            Vec3::new(-5.0, -5.0, -5.0),
            Vec3::new(5.0, 5.0, 5.0),
            WORLD_640,
            Vec3::ZERO,
            1.0,
            0.8,
        );
        // target = 640 * 0.8 = 512; horiz_extent = 10 -> scale = 51.2
        assert!((s - 51.2).abs() < 0.01, "scale = {}", s);
        // After scale, centre stays at origin, lowest y = -5*51.2 = -256
        // -> translation = (320, 256, 320)
        assert_vec3_near(t, Vec3::new(320.0, 256.0, 320.0), 0.1);
    }

    #[test]
    fn auto_fit_downscales_huge_mesh() {
        let (t, s) = recommended_transform(
            Vec3::new(-500.0, -500.0, -500.0),
            Vec3::new(500.0, 500.0, 500.0),
            WORLD_640,
            Vec3::ZERO,
            1.0,
            0.8,
        );
        // horiz_extent = 1000 -> scale = 512/1000 = 0.512
        assert!((s - 0.512).abs() < 0.01, "scale = {}", s);
        // translation = (320, 500*0.512=256, 320)
        assert_vec3_near(t, Vec3::new(320.0, 256.0, 320.0), 0.1);
    }

    #[test]
    fn auto_fit_centres_off_corner_mesh() {
        // 10m cube at (45..55) in x/z, (-5..5) in y, transform identity.
        let (t, s) = recommended_transform(
            Vec3::new(45.0, -5.0, 45.0),
            Vec3::new(55.0, 5.0, 55.0),
            WORLD_640,
            Vec3::ZERO,
            1.0,
            0.8,
        );
        // scale = 51.2. Local centre at (50, 0, 50). After scale_mult,
        // local_center * 51.2 = (2560, 0, 2560). Translation =
        // world_center - that = (320 - 2560, 5*51.2, 320 - 2560) =
        // (-2240, 256, -2240).
        assert!((s - 51.2).abs() < 0.01);
        assert_vec3_near(t, Vec3::new(-2240.0, 256.0, -2240.0), 1.0);
    }

    #[test]
    fn auto_fit_is_idempotent_after_first_fit() {
        // After fitting, AABB world = (64, 0, 64)..(576, 512, 576),
        // current_translation = (320, 256, 320), current_scale = 51.2.
        let aabb_min = Vec3::new(64.0, 0.0, 64.0);
        let aabb_max = Vec3::new(576.0, 512.0, 576.0);
        let cur_t = Vec3::new(320.0, 256.0, 320.0);
        let cur_s = 51.2;
        let (t, s) = recommended_transform(
            aabb_min, aabb_max, WORLD_640, cur_t, cur_s, 0.8,
        );
        // horiz_extent = 512 = target -> scale_mult = 1.0 -> new_scale = cur_s
        assert!((s - cur_s).abs() < 0.01, "scale should be stable: {} vs {}", s, cur_s);
        assert_vec3_near(t, cur_t, 0.5);
    }

    #[test]
    fn raycast_bvh_within_range_returns_hit() {
        let tri = Triangle {
            v0: Vec3A::new(0.0, 10.0, 0.0),
            v1: Vec3A::new(2.0, 10.0, 0.0),
            v2: Vec3A::new(0.0, 10.0, 2.0),
        };
        let bvh = build_world_bvh(vec![tri]);
        let (endpoint, hit) = raycast_bvh(
            &bvh,
            Vec3::new(0.25, 30.0, 0.25),
            Vec3::NEG_Y,
            100.0,
        );
        assert!(hit);
        assert!((endpoint.y - 10.0).abs() < 1e-3, "endpoint = {:?}", endpoint);
    }

    #[test]
    fn raycast_bvh_beyond_max_dist_returns_miss() {
        let tri = Triangle {
            v0: Vec3A::new(0.0, 10.0, 0.0),
            v1: Vec3A::new(2.0, 10.0, 0.0),
            v2: Vec3A::new(0.0, 10.0, 2.0),
        };
        let bvh = build_world_bvh(vec![tri]);
        let (endpoint, hit) = raycast_bvh(
            &bvh,
            Vec3::new(0.25, 30.0, 0.25),
            Vec3::NEG_Y,
            5.0,
        );
        assert!(!hit);
        assert!((endpoint.y - 25.0).abs() < 1e-3, "endpoint = {:?}", endpoint);
    }

    #[test]
    fn ground_altitude_returns_floor_height() {
        // A 2x2 floor triangle at y=10.
        let tri = Triangle {
            v0: Vec3A::new(0.0, 10.0, 0.0),
            v1: Vec3A::new(2.0, 10.0, 0.0),
            v2: Vec3A::new(0.0, 10.0, 2.0),
        };
        let bvh = build_world_bvh(vec![tri]);
        let h = ground_altitude(&bvh, 0.25, 0.25, 100.0);
        assert!(h.is_some());
        assert!(
            (h.unwrap() - 10.0).abs() < 1e-3,
            "expected ground y ~ 10, got {:?}",
            h
        );
    }

    #[test]
    fn ground_altitude_returns_none_off_geometry() {
        let tri = single_xz_triangle();
        let bvh = build_world_bvh(vec![tri]);
        let h = ground_altitude(&bvh, 100.0, 100.0, 50.0);
        assert!(h.is_none());
    }

    #[test]
    fn ray_missing_triangle_returns_none() {
        let bvh = build_world_bvh(vec![single_xz_triangle()]);
        // ray at (10, 5, 10) far from the triangle, pointing up
        let hit = cast_ray(
            &bvh,
            Vec3::new(10.0, 5.0, 10.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        assert!(hit.is_none(), "ray missing the triangle must not hit");
    }
}
