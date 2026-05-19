/// Seconds between full rescans of the global occupancy bitset.
pub const FRONTIER_REFRESH_SECS: f32 = 1.0;

/// World-space distance below which a drone is considered to have
/// reached its current frontier target. Recomputed lazily on next assign.
pub const FRONTIER_REACHED_DIST: f32 = 6.0;

/// Soft cap on candidate cells gathered per scan, so the frontier
/// computation stays bounded as the explored region grows. Drone
/// assignment is O(drones * candidates), so this also caps that cost.
pub const MAX_FRONTIER_CANDIDATES: usize = 50_000;

/// Linear-interp rate (fraction-per-second) at which a drone's
/// DesiredVelocity lerps toward the unit vector pointing at its frontier
/// target, scaled by cruise speed.
pub const FRONTIER_LERP_RATE: f32 = 3.0;
