use bevy::prelude::*;

pub const GROUND_TRUTH_BASE_COLOR: Color = Color::srgb(0.55, 0.55, 0.6);
pub const GROUND_TRUTH_ROUGHNESS: f32 = 0.9;

/// Emissive multiplier applied to a drone's own base color when building
/// its local-map material. Lower = drone color reads cleaner; higher =
/// more glow against the dark scene.
pub const LOCAL_MAP_EMISSIVE_FACTOR: f32 = 0.3;

pub const GLOBAL_OCCUPIED_BASE_COLOR: Color = Color::srgba(0.1, 0.85, 1.0, 0.7);
pub const GLOBAL_OCCUPIED_EMISSIVE: LinearRgba = LinearRgba {
    red: 0.0,
    green: 0.4,
    blue: 0.5,
    alpha: 1.0,
};
