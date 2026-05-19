pub const DEFAULT_YAW: f32 = 0.6;
pub const DEFAULT_PITCH: f32 = 0.5;
pub const DEFAULT_DISTANCE: f32 = 900.0;
pub const PITCH_LIMIT: f32 = 1.4;
pub const ORBIT_SENSITIVITY: f32 = 0.005;
pub const ZOOM_FACTOR_PER_TICK: f32 = 0.1;
pub const MIN_DISTANCE: f32 = 4.0;
pub const MAX_DISTANCE: f32 = 4000.0;

/// Free-fly tuning. Move speed picked to suit the 640 m world; boost
/// gives a fast pan across the map.
pub const FREEFLY_MOVE_SPEED_MPS: f32 = 50.0;
pub const FREEFLY_BOOST_FACTOR: f32 = 4.0;
pub const FREEFLY_LOOK_SENSITIVITY: f32 = 0.003;
pub const FREEFLY_PITCH_LIMIT: f32 = 1.5;
