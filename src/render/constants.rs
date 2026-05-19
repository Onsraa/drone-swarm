pub const GROUND_TRUTH_INSTANCE_COLOR: [f32; 4] = [0.55, 0.55, 0.6, 1.0];

/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 1.0;
pub const LOCAL_MAP_ALPHA: f32 = 0.85;

/// Per-layer scale multipliers on the cube size. When two or three layers
/// cover the same cell (e.g. ground truth + a drone's local map) the
/// surfaces would otherwise sit exactly co-planar and Z-fight under
/// transparency. Growing each successive layer by 1% pushes them slightly
/// apart so the depth test can pick a winner. Centers stay aligned.
pub const GROUND_TRUTH_SCALE_FACTOR: f32 = 1.0;
pub const LOCAL_MAP_SCALE_FACTOR: f32 = 1.02;
