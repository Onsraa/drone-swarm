use bevy::prelude::*;

use super::constants::{DEFAULT_DISTANCE, DEFAULT_PITCH, DEFAULT_YAW};

#[derive(Component)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vec3::ZERO,
            yaw: DEFAULT_YAW,
            pitch: DEFAULT_PITCH,
            distance: DEFAULT_DISTANCE,
        }
    }
}

/// Free-fly camera state. `yaw` rotates around world Y, `pitch` around
/// local X. The combined `Transform.rotation` is rebuilt every frame
/// from these two values. Movement is WASD + Space/Shift for vertical,
/// scaled by `FREEFLY_MOVE_SPEED_MPS` and boosted by holding Ctrl.
#[derive(Component, Default)]
pub struct FreeFlyCamera {
    pub yaw: f32,
    pub pitch: f32,
}
