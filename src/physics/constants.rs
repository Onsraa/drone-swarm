use std::f32::consts::PI;

pub const DRONE_MASS_KG: f32 = 1.0;
pub const GRAVITY: f32 = 9.81;
pub const LINEAR_DRAG_COEF: f32 = 0.4;

/// Proportional gain mapping velocity error to commanded linear acceleration.
pub const VELOCITY_P_GAIN: f32 = 2.5;

/// Maximum thrust expressed as a multiple of hover thrust (mass * gravity).
pub const MAX_THRUST_MULTIPLE_OF_HOVER: f32 = 2.0;

/// Maximum allowed tilt of body +Y away from world +Y.
pub const MAX_TILT_RADIANS: f32 = PI * 35.0 / 180.0;

/// Slerp fraction-per-second toward the controller's target attitude.
pub const ATTITUDE_LERP_RATE: f32 = 6.0;

/// Soft repulsion zone width near each world boundary.
pub const BOUND_SOFT_MARGIN_METERS: f32 = 3.0;
/// Linear force per meter of intrusion into the soft margin.
pub const BOUND_REPULSION_K: f32 = 30.0;

/// Squared horizontal speed below which yaw stops tracking velocity.
pub const YAW_TRACK_MIN_SPEED_SQ: f32 = 0.25;
