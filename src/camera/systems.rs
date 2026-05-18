use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

use crate::world::WorldConfig;

use super::components::OrbitCamera;
use super::constants::{
    MAX_DISTANCE, MIN_DISTANCE, ORBIT_SENSITIVITY, PITCH_LIMIT, ZOOM_FACTOR_PER_TICK,
};

pub fn spawn_camera(mut commands: Commands, world: Res<WorldConfig>) {
    commands.spawn((
        Camera3d::default(),
        OrbitCamera {
            target: world.center(),
            ..Default::default()
        },
        Transform::default(),
    ));
}

pub fn orbit_input(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut q: Query<&mut OrbitCamera>,
) {
    let Ok(mut cam) = q.single_mut() else {
        return;
    };

    if mouse_buttons.pressed(MouseButton::Left) && motion.delta != Vec2::ZERO {
        cam.yaw -= motion.delta.x * ORBIT_SENSITIVITY;
        cam.pitch =
            (cam.pitch + motion.delta.y * ORBIT_SENSITIVITY).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }

    if scroll.delta.y != 0.0 {
        let factor = 1.0 - scroll.delta.y * ZOOM_FACTOR_PER_TICK;
        cam.distance = (cam.distance * factor).clamp(MIN_DISTANCE, MAX_DISTANCE);
    }
}

pub fn sync_camera_transform(mut q: Query<(&OrbitCamera, &mut Transform)>) {
    for (cam, mut transform) in &mut q {
        let cp = cam.pitch.cos();
        let dir = Vec3::new(cam.yaw.sin() * cp, cam.pitch.sin(), cam.yaw.cos() * cp);
        let pos = cam.target + dir * cam.distance;
        *transform = Transform::from_translation(pos).looking_at(cam.target, Vec3::Y);
    }
}
