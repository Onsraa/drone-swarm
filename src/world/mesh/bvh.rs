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
}

/// Build a CWBVH8 from a triangle list using obvhs' medium-quality
/// build preset. Builder is one-shot per scene load; expect ~100 ms for
/// 2 M tris on M4 Pro. Caller owns the triangle list lifetime.
pub fn build_world_bvh(triangles: Vec<Triangle>) -> WorldBvh {
    let cwbvh = build_cwbvh_from_tris(
        &triangles,
        BvhBuildParams::medium_build(),
        &mut Duration::default(),
    );
    WorldBvh { triangles, cwbvh }
}

/// Cast a ray into the BVH. Returns the hit distance `t` along the ray
/// if any primitive was intersected. Direction is normalized internally.
/// Used in Phase 3 to port `raycast_dda` off the voxel grid.
#[allow(dead_code)]
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
