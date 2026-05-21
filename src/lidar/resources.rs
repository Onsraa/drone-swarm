use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;

use super::constants::{
    LIDAR_CONE_HALF_ANGLE_DEGREES, MAX_CONE_HALF_ANGLE_DEGREES, MAX_RAYS_PER_SCAN,
    MAX_STEPS_PER_RAY_SLIDER, MAX_SCAN_INTERVAL_FRAMES, MIN_CONE_HALF_ANGLE_DEGREES,
    MIN_RAYS_PER_SCAN, MIN_STEPS_PER_RAY, MIN_SCAN_INTERVAL_FRAMES, RAYS_PER_SCAN,
};

/// Runtime-tunable lidar parameters wired to egui sliders. The
/// `upload_ray_dirs` system writes the fibonacci cone into
/// `RayDirsBuffer` whenever this resource changes; `upload_drone_state`
/// pushes `max_steps` + `rays_per_scan` into `LidarParams` every frame.
#[derive(Resource, ExtractResource, Clone, Copy, Debug)]
pub struct LidarSettings {
    pub rays_per_scan: u32,
    pub cone_half_angle_deg: f32,
    pub max_steps_per_ray: u32,
    pub scan_interval_frames: u32,
    /// When `true`, the lidar point counter never resets between
    /// scans. New hits append to the existing buffer until the soft
    /// cap (`MAX_LIDAR_POINTS`) is reached. Result: a SLAM-style
    /// accumulated point cloud instead of a pulsing spray. Toggling
    /// back to `false` naturally clears on the next dispatch.
    pub sticky_spray: bool,
    /// When `true`, the spray buffer's per-point colour is sampled
    /// from the mesh material's flat albedo at the hit triangle
    /// instead of the drone's own role tint.
    pub spray_use_albedo: bool,
}

impl Default for LidarSettings {
    fn default() -> Self {
        Self {
            rays_per_scan: RAYS_PER_SCAN as u32,
            cone_half_angle_deg: LIDAR_CONE_HALF_ANGLE_DEGREES,
            max_steps_per_ray: 96,
            scan_interval_frames: 1,
            sticky_spray: false,
            spray_use_albedo: false,
        }
    }
}

impl LidarSettings {
    pub fn rays_range() -> std::ops::RangeInclusive<u32> {
        MIN_RAYS_PER_SCAN..=MAX_RAYS_PER_SCAN
    }
    pub fn cone_range() -> std::ops::RangeInclusive<f32> {
        MIN_CONE_HALF_ANGLE_DEGREES..=MAX_CONE_HALF_ANGLE_DEGREES
    }
    pub fn steps_range() -> std::ops::RangeInclusive<u32> {
        MIN_STEPS_PER_RAY..=MAX_STEPS_PER_RAY_SLIDER
    }
    pub fn interval_range() -> std::ops::RangeInclusive<u32> {
        MIN_SCAN_INTERVAL_FRAMES..=MAX_SCAN_INTERVAL_FRAMES
    }
}

/// Main-world frame counter incremented once per Update tick. Extracted
/// into the render world so `ComputeLidarBvhNode` can skip dispatches based
/// on `LidarSettings.scan_interval_frames`.
#[derive(Resource, ExtractResource, Default, Clone, Copy, Debug)]
pub struct LidarFrameCounter(pub u32);
