/// Cool muted grey for the transparent ground-truth voxel cubes. Alpha
/// is low so the local + global map billboard dots underneath read
/// through as "paint" inside the cube volume.
pub const GROUND_TRUTH_CUBE_COLOR: [f32; 4] = [0.55, 0.58, 0.65, 0.18];

/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 0.7;
pub const LOCAL_MAP_ALPHA: f32 = 0.85;

/// Per-layer billboard radius in screen pixels. The shader treats
/// `pos_scale.w` as a pixel radius and synthesises a camera-facing
/// quad around the point. The central-map radius is mirrored as a
/// WGSL const in `build_global_instances.wgsl`; spray radius lives in
/// `lidar_compute.wgsl`. Ground truth no longer uses a billboard
/// radius (it's a real cube now).
pub const LOCAL_MAP_POINT_PX: f32 = 2.7;
