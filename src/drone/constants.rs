use std::f32::consts::PI;

pub const DRONE_SCALE: f32 = 0.1;
pub const RANDOM_DIR_MIN_LENGTH: f32 = 0.1;
pub const DRONE_GLB_PATH: &str = "models/drone.glb";

/// Yaw offset applied to the SceneRoot child so the GLB's authored forward
/// axis aligns with Bevy's body forward (-Z). The current drone.glb is
/// modeled with +Z as forward; this rotates it 180 deg around Y.
pub const MODEL_YAW_OFFSET_RADIANS: f32 = PI;

pub const CRUISE_SPEED_MPS: f32 = 3.0;
/// Vertical-component bias for wander direction. < 1.0 makes the drone prefer
/// horizontal motion (planar wandering) while still exploring up and down.
pub const VERTICAL_SPEED_FACTOR: f32 = 0.4;
pub const WANDER_CHANGE_INTERVAL_SECS: f32 = 3.0;
/// Fraction-per-second the desired velocity lerps toward a freshly picked
/// wander target. Lower = laggier, more sustained heading.
pub const WANDER_LERP_RATE: f32 = 1.5;
