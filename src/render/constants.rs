/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 0.7;
pub const LOCAL_MAP_ALPHA: f32 = 0.85;

/// Per-layer billboard radius in screen pixels. The shader treats
/// `pos_scale.w` as a pixel radius and synthesises a camera-facing
/// quad around the point. The central-map radius is mirrored as a
/// WGSL const in `build_global_instances.wgsl`; spray radius lives in
/// `lidar_bvh.wgsl`.
pub const LOCAL_MAP_POINT_PX: f32 = 2.7;
