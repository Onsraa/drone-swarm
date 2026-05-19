pub const GROUND_TRUTH_INSTANCE_COLOR: [f32; 4] = [0.55, 0.55, 0.6, 1.0];

/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 1.0;
pub const LOCAL_MAP_ALPHA: f32 = 0.85;

/// Per-layer billboard radius in screen pixels. The shader treats
/// `pos_scale.w` as a pixel radius and synthesises a camera-facing
/// quad around the point. Tuning these makes the visual hierarchy:
/// ground truth thin so it doesn't dominate, local maps a touch
/// bigger, spray brightest to read as "live lidar fire". Global-map
/// and spray sizes are mirrored as WGSL consts in
/// `build_global_instances.wgsl` and `lidar_compute.wgsl`.
pub const GROUND_TRUTH_POINT_PX: f32 = 2.0;
pub const LOCAL_MAP_POINT_PX: f32 = 4.0;
