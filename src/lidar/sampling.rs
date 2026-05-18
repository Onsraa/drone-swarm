use bevy::prelude::*;

use super::constants::RAYS_PER_SCAN;

/// Precomputed ray directions for each lidar scan. The same `RAYS_PER_SCAN`
/// Fibonacci-sphere directions are reused every tick, every drone, so we
/// pay for the trig + Vec alloc once at plugin build instead of 5 Hz.
#[derive(Resource)]
pub struct LidarRayDirs(pub Vec<Vec3>);

impl LidarRayDirs {
    pub fn fibonacci(n: usize) -> Self {
        Self(fibonacci_sphere(n))
    }

    pub fn default_for_scan() -> Self {
        Self::fibonacci(RAYS_PER_SCAN)
    }
}

/// `n` approximately-uniformly-spaced unit vectors on the unit sphere via
/// the Fibonacci sphere construction.
pub fn fibonacci_sphere(n: usize) -> Vec<Vec3> {
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0_f32.sqrt());
    let denom = n.saturating_sub(1).max(1) as f32;
    (0..n)
        .map(|i| {
            let y = 1.0 - (i as f32 / denom) * 2.0;
            let radius = (1.0 - y * y).max(0.0).sqrt();
            let theta = golden_angle * i as f32;
            Vec3::new(theta.cos() * radius, y, theta.sin() * radius).normalize_or_zero()
        })
        .collect()
}
