use std::f32::consts::PI;

pub const DRONE_MASS_KG: f32 = 1.0;
pub const GRAVITY: f32 = 9.81;
pub const LINEAR_DRAG_COEF: f32 = 0.4;

/// Proportional gain mapping forward-speed error to commanded forward
/// acceleration (m/s^2 per m/s). Tuned with `MAX_PITCH_RADIANS` so the
/// drone reaches cruise speed in roughly half a second.
pub const FORWARD_P_GAIN: f32 = 2.0;

/// Proportional gain mapping vertical-velocity error to commanded vertical
/// acceleration (m/s^2 per m/s).
pub const VERTICAL_P_GAIN: f32 = 2.5;

/// Maximum forward-pitch angle of the body. Caps horizontal acceleration to
/// roughly `g * tan(MAX_PITCH_RADIANS)`.
pub const MAX_PITCH_RADIANS: f32 = PI * 35.0 / 180.0;

/// Maximum thrust expressed as a multiple of hover thrust (mass * gravity).
pub const MAX_THRUST_MULTIPLE_OF_HOVER: f32 = 2.0;

/// Slerp fraction-per-second toward the controller's target attitude.
/// Models the inner attitude-rate loop of a real cascaded PID controller.
pub const ATTITUDE_LERP_RATE: f32 = 6.0;

/// Below this desired horizontal speed the drone holds its current yaw
/// instead of snapping toward a near-zero target heading.
pub const HEADING_TRACK_MIN_SPEED: f32 = 0.3;

/// Soft repulsion zone width near each world boundary.
pub const BOUND_SOFT_MARGIN_METERS: f32 = 3.0;
/// Linear force per meter of intrusion into the soft margin.
pub const BOUND_REPULSION_K: f32 = 30.0;
