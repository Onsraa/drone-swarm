use bevy::prelude::*;

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
