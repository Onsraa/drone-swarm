pub const DEFAULT_DRONE_COUNT: u32 = 3;
pub const MIN_DRONE_COUNT: u32 = 1;
pub const MAX_DRONE_COUNT: u32 = 50;
/// Horizontal radius (meters) of the spawn ring around the world center.
pub const DRONE_SPAWN_RADIUS_METERS: f32 = 120.0;

/// World-Y to start the ground-finding ray cast from. Anything below
/// this height will land on the mesh below (or fall through to the
/// voxel-grid fallback).
pub const SPAWN_SKY_CAST_Y: f32 = 2000.0;

/// Vertical clearance (meters) above the BVH ground hit at which a
/// drone spawns. Keeps drones from clipping into the mesh surface.
pub const SPAWN_GROUND_CLEARANCE_M: f32 = 4.0;
