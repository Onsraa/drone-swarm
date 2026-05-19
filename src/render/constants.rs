pub const GROUND_TRUTH_INSTANCE_COLOR: [f32; 4] = [0.55, 0.55, 0.6, 1.0];

/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 1.0;
pub const LOCAL_MAP_ALPHA: f32 = 0.85;

/// Per-layer scale multipliers on the cube size. Map layers (ground
/// truth, per-drone local, global) all render as point-cloud-sized
/// nubs (~15% of a voxel) so the result reads as a lidar-style
/// composition of points instead of a wall of solid cubes. At this
/// size z-fighting between overlapping layers stops mattering, so we
/// drop the historic +1% / +2% inflation.
pub const GROUND_TRUTH_SCALE_FACTOR: f32 = 0.15;
pub const LOCAL_MAP_SCALE_FACTOR: f32 = 0.15;
