use bevy::prelude::*;

use super::constants::{LIDAR_CONE_HALF_ANGLE_DEGREES, RAYS_PER_SCAN};

/// Precomputed ray directions for each lidar scan. Stored in the drone's
/// local frame, with the cone axis aligned to Bevy's body-forward (`-Z`);
/// `lidar_scan` rotates each direction by the drone's transform before
/// casting.
#[derive(Resource)]
pub struct LidarRayDirs(pub Vec<Vec3>);

impl LidarRayDirs {
    pub fn forward_cone(n: usize, half_angle_rad: f32) -> Self {
        Self(fibonacci_cone(n, half_angle_rad))
    }

    pub fn default_for_scan() -> Self {
        Self::forward_cone(
            RAYS_PER_SCAN,
            LIDAR_CONE_HALF_ANGLE_DEGREES.to_radians(),
        )
    }
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
