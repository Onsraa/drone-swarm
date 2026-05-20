//! Velocity-tracker physics constants. The cascade quadcopter
//! constants (gravity / mass / pitch / thrust / drag) are gone — the
//! current model is a point-mass that lerps its `LinearVelocity`
//! toward `DesiredVelocity` and integrates `Transform.translation +=
//! linvel * dt`. Visual orientation is cosmetic, driven separately.

/// Per-second rate at which `linvel` chases `desired`. At dt = 1/120 s
/// the lerp factor is ≈ 0.033 per frame; a stationary drone reaches
/// ~95% of `desired` in ~0.75 s. Higher = snappier, lower = floatier.
pub const VEL_TRACK_GAIN: f32 = 4.0;

/// Slerp rate of the cosmetic body rotation toward the orientation
/// derived from `linvel`. Higher than `VEL_TRACK_GAIN` so the visual
/// "leads" the motion slightly without lagging behind.
pub const COSMETIC_LERP_RATE: f32 = 8.0;

/// Radians of nose-down pitch per m/s of horizontal speed. Pure
/// cosmetic — sells the "leaning into the wind" look without
/// coupling to physics.
pub const COSMETIC_PITCH_FACTOR: f32 = 0.05;

/// Cap on cosmetic pitch in radians (≈ 15°). Keeps the drone from
/// flipping when `linvel` momentarily spikes.
pub const COSMETIC_PITCH_MAX: f32 = 0.26;

/// Radians of bank per m/s² of lateral acceleration. Banks the drone
/// into turns.
pub const COSMETIC_ROLL_FACTOR: f32 = 0.04;

/// Cap on cosmetic roll in radians (≈ 11°).
pub const COSMETIC_ROLL_MAX: f32 = 0.20;

/// Below this horizontal speed the cosmetic system holds the current
/// yaw instead of snapping to a near-zero direction.
pub const COSMETIC_MIN_SPEED: f32 = 0.3;
