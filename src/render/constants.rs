pub const GROUND_TRUTH_INSTANCE_COLOR: [f32; 4] = [0.55, 0.55, 0.6, 0.5];

/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 0.7;
pub const LOCAL_MAP_ALPHA: f32 = 0.5;

/// Per-layer billboard radius in screen pixels. The shader treats
/// `pos_scale.w` as a pixel radius and synthesises a camera-facing
/// quad around the point. Ground-truth + central are mirrored as WGSL
/// consts in `build_global_instances.wgsl`; spray radius lives in
/// `lidar_compute.wgsl`.
pub const GROUND_TRUTH_POINT_PX: f32 = 1.5;
pub const LOCAL_MAP_POINT_PX: f32 = 2.7;
