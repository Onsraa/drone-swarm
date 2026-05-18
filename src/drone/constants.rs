use std::f32::consts::PI;

pub const DRONE_SCALE: f32 = 0.1;
pub const RANDOM_DIR_MIN_LENGTH: f32 = 0.1;
pub const DRONE_GLB_PATH: &str = "models/drone.glb";

pub const DEFAULT_DRONE_COUNT: u32 = 3;
pub const MIN_DRONE_COUNT: u32 = 1;
pub const MAX_DRONE_COUNT: u32 = 50;
/// Horizontal radius (meters) of the spawn ring around the world center.
pub const DRONE_SPAWN_RADIUS_METERS: f32 = 5.0;
/// Golden-angle-in-degrees offset between consecutive drone hues so 50+
/// drones still get well-spaced, perceptually distinct colors.
pub const DRONE_HUE_STEP_DEGREES: f32 = 137.508;
pub const DRONE_COLOR_SATURATION: f32 = 0.85;
pub const DRONE_COLOR_LIGHTNESS: f32 = 0.55;
pub const DRONE_COLOR_ALPHA: f32 = 0.85;

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
