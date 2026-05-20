use bevy::prelude::*;

/// World-space linear velocity in m/s. Tracked by the integrator;
/// `track_velocity` lerps it toward `DesiredVelocity` each frame.
#[derive(Component, Default)]
pub struct LinearVelocity(pub Vec3);

/// World-space velocity the steering layer wants the drone to attain.
/// Written by `steer_along_path`, `reactive_avoid`, etc.
#[derive(Component, Default)]
pub struct DesiredVelocity(pub Vec3);
