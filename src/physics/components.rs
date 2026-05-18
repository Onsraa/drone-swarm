use bevy::prelude::*;

#[derive(Component, Default)]
pub struct LinearVelocity(pub Vec3);

#[derive(Component, Default)]
pub struct DesiredVelocity(pub Vec3);

#[derive(Component, Default)]
pub struct ThrustState {
    pub magnitude: f32,
}

/// Target body orientation the controller wants the integrator to slerp toward.
#[derive(Component)]
pub struct DesiredAttitude {
    pub target_rotation: Quat,
}

impl Default for DesiredAttitude {
    fn default() -> Self {
        Self {
            target_rotation: Quat::IDENTITY,
        }
    }
}
