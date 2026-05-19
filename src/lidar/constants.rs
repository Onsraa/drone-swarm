/// Default number of rays in a single lidar scan. Tunable at runtime
/// via the side-panel Lidar sliders (bounded by `MIN_RAYS_PER_SCAN` ..
/// `MAX_RAYS_PER_SCAN`). The GPU buffer is allocated at the max so the
/// shader just reads `params.rays_per_scan` per dispatch.
pub const RAYS_PER_SCAN: usize = 64;

pub const MIN_RAYS_PER_SCAN: u32 = 4;
pub const MAX_RAYS_PER_SCAN: u32 = 256;

/// Default half-angle of the forward-facing lidar cone. 30 deg gives a
/// 60 deg FOV fan along the drone's heading. Tunable from
/// `MIN_CONE_HALF_ANGLE_DEGREES` (pencil beam) up to 180 (full sphere).
pub const LIDAR_CONE_HALF_ANGLE_DEGREES: f32 = 30.0;

pub const MIN_CONE_HALF_ANGLE_DEGREES: f32 = 1.0;
pub const MAX_CONE_HALF_ANGLE_DEGREES: f32 = 180.0;

/// Bounds for the UI slider on max DDA steps per ray (lidar range in
/// voxel cells). The compile-time default lives in
/// `lidar::gpu::resources::MAX_STEPS_PER_RAY`.
pub const MIN_STEPS_PER_RAY: u32 = 8;
pub const MAX_STEPS_PER_RAY_SLIDER: u32 = 256;

pub const MIN_SCAN_INTERVAL_FRAMES: u32 = 1;
pub const MAX_SCAN_INTERVAL_FRAMES: u32 = 30;
