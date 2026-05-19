use bevy::prelude::*;

/// World-space target a drone is currently flying toward. `None` means
/// no frontier assigned yet (cold start or no candidates available); the
/// drone's random `wander` system supplies a fallback DesiredVelocity in
/// that case.
#[derive(Component, Default)]
pub struct FrontierTarget(pub Option<Vec3>);
