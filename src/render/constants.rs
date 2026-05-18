use bevy::prelude::*;

pub const GROUND_TRUTH_BASE_COLOR: Color = Color::srgb(0.55, 0.55, 0.6);
pub const GROUND_TRUTH_ROUGHNESS: f32 = 0.9;

pub const LOCAL_OCCUPIED_BASE_COLOR: Color = Color::srgba(1.0, 0.55, 0.1, 0.85);
pub const LOCAL_OCCUPIED_EMISSIVE: LinearRgba = LinearRgba {
    red: 0.45,
    green: 0.18,
    blue: 0.0,
    alpha: 1.0,
};

pub const GLOBAL_OCCUPIED_BASE_COLOR: Color = Color::srgba(0.1, 0.85, 1.0, 0.7);
pub const GLOBAL_OCCUPIED_EMISSIVE: LinearRgba = LinearRgba {
    red: 0.0,
    green: 0.4,
    blue: 0.5,
    alpha: 1.0,
};
