use bevy::prelude::*;

use crate::exploration::{Role, RoleParams};

#[derive(Clone, Copy, Debug)]
pub struct RoleConeRange {
    pub role: Role,
    pub offset: u32,
    pub count: u32,
}

/// Concatenate one fibonacci cone per role into a single ray buffer.
/// Each role's cone half-angle + ray count come from `RoleParams` so the
/// role table is the single source of truth for sensor shape.
pub fn build_role_ray_buffer() -> (Vec<Vec3>, [RoleConeRange; 3]) {
    let roles = [Role::Scout, Role::Mapper, Role::Anchor];
    let mut all: Vec<Vec3> = Vec::new();
    let mut ranges = [RoleConeRange {
        role: Role::Scout,
        offset: 0,
        count: 0,
    }; 3];
    let mut offset = 0u32;
    for (i, role) in roles.iter().enumerate() {
        let params = RoleParams::for_role(*role);
        let dirs = fibonacci_cone(
            params.rays_per_scan as usize,
            params.cone_half_angle_deg.to_radians(),
        );
        let count = dirs.len() as u32;
        ranges[i] = RoleConeRange {
            role: *role,
            offset,
            count,
        };
        all.extend(dirs);
        offset += count;
    }
    (all, ranges)
}

/// `n` approximately-uniformly-spaced unit vectors inside a spherical cap
/// (cone) around the drone's body-forward axis `-Z`, with the given
/// half-angle. Uses a fibonacci spiral on the cap so the angular spread
/// is even without random clumping. `t = i / (n-1)` walks from the cone's
/// axis (`-Z`) out to its rim; `phi` is the golden-angle azimuth.
pub fn fibonacci_cone(n: usize, half_angle_rad: f32) -> Vec<Vec3> {
    let cos_max = half_angle_rad.cos();
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0_f32.sqrt());
    let denom = n.saturating_sub(1).max(1) as f32;
    (0..n)
        .map(|i| {
            let t = i as f32 / denom;
            // cos(theta) sweeps [1, cos_max]; uniform-in-cos = uniform on cap area.
            let cos_theta = 1.0 - t * (1.0 - cos_max);
            let radius = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
            let phi = golden_angle * i as f32;
            Vec3::new(phi.cos() * radius, phi.sin() * radius, -cos_theta).normalize_or_zero()
        })
        .collect()
}
