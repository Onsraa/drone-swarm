pub const GROUND_TRUTH_INSTANCE_COLOR: [f32; 4] = [0.55, 0.55, 0.6, 1.0];
pub const GLOBAL_OCCUPIED_INSTANCE_COLOR: [f32; 4] = [0.1, 0.85, 1.0, 0.7];
/// Multiplier on the drone's own color for the local-map instance color
/// (lower = more saturated drone hue, higher = brighter glow).
pub const LOCAL_MAP_COLOR_FACTOR: f32 = 1.0;
pub const LOCAL_MAP_ALPHA: f32 = 0.85;
