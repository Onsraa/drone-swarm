use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::render::view::NoIndirectDrawing;

use crate::world::WorldConfig;

use super::components::{FreeFlyCamera, OrbitCamera};
use super::constants::{
    FREEFLY_BOOST_FACTOR, FREEFLY_LOOK_SENSITIVITY, FREEFLY_MOVE_SPEED_MPS, FREEFLY_PITCH_LIMIT,
    MAX_DISTANCE, MIN_DISTANCE, ORBIT_SENSITIVITY, PITCH_LIMIT, ZOOM_FACTOR_PER_TICK,
};
use super::resources::CameraMode;

pub fn spawn_camera(mut commands: Commands, world: Res<WorldConfig>) {
    commands.spawn((
        Camera3d::default(),
        OrbitCamera {
            target: world.center(),
            ..Default::default()
        },
        FreeFlyCamera::default(),
        Transform::default(),
        // Required by our custom instanced render command which uses
        // direct `draw_indexed` / `draw` instead of the indirect variants.
        NoIndirectDrawing,
    ));
}

/// `F` toggles between orbit and free-fly modes. On each transition we
/// seed the destination controller's state from the current Transform
/// so the view doesn't jump.
pub fn toggle_camera_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut mode: ResMut<CameraMode>,
    mut q: Query<(&Transform, &mut OrbitCamera, &mut FreeFlyCamera)>,
) {
    if !keys.just_pressed(KeyCode::KeyF) {
        return;
    }
    let Ok((transform, mut orbit, mut freefly)) = q.single_mut() else {
        return;
    };
    match *mode {
        CameraMode::Orbit => {
            // Entering free-fly: derive yaw + pitch from the current
            // Transform's forward vector so the view stays put.
            let forward = transform.forward();
            freefly.yaw = forward.x.atan2(forward.z) + std::f32::consts::PI;
            freefly.pitch = (-forward.y).asin();
            *mode = CameraMode::FreeFly;
            info!("camera mode: free-fly (WASD move, Space/Shift up/down, RMB+drag look, Ctrl boost)");
        }
        CameraMode::FreeFly => {
            // Returning to orbit: place the orbit target ahead of the
            // current camera at the existing orbit.distance so the
            // user keeps roughly the same view.
            orbit.target = transform.translation + transform.forward() * orbit.distance;
            // Sync orbit yaw/pitch from the same forward.
            let forward = transform.forward();
            orbit.yaw = forward.x.atan2(forward.z) + std::f32::consts::PI;
            orbit.pitch = (-forward.y).asin().clamp(-PITCH_LIMIT, PITCH_LIMIT);
            *mode = CameraMode::Orbit;
            info!("camera mode: orbit (LMB drag, scroll zoom)");
        }
    }
}

pub fn orbit_input(
    mode: Res<CameraMode>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut q: Query<&mut OrbitCamera>,
) {
    if *mode != CameraMode::Orbit {
        return;
    }
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

pub fn sync_camera_transform(
    mode: Res<CameraMode>,
    mut q: Query<(&OrbitCamera, &mut Transform)>,
) {
    if *mode != CameraMode::Orbit {
        return;
    }
    for (cam, mut transform) in &mut q {
        let cp = cam.pitch.cos();
        let dir = Vec3::new(cam.yaw.sin() * cp, cam.pitch.sin(), cam.yaw.cos() * cp);
        let pos = cam.target + dir * cam.distance;
        *transform = Transform::from_translation(pos).looking_at(cam.target, Vec3::Y);
    }
}

/// WASD + Space/Shift + RMB-look. Speed boosts ×`FREEFLY_BOOST_FACTOR`
/// while Ctrl is held. World-Y axis is the vertical reference so
/// up/down feel intuitive regardless of pitch.
pub fn freefly_input(
    mode: Res<CameraMode>,
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    mut q: Query<(&mut FreeFlyCamera, &mut Transform)>,
) {
    if *mode != CameraMode::FreeFly {
        return;
    }
    let Ok((mut cam, mut transform)) = q.single_mut() else {
        return;
    };
    let dt = time.delta_secs();

    if mouse_buttons.pressed(MouseButton::Right) && motion.delta != Vec2::ZERO {
        cam.yaw -= motion.delta.x * FREEFLY_LOOK_SENSITIVITY;
        cam.pitch = (cam.pitch - motion.delta.y * FREEFLY_LOOK_SENSITIVITY)
            .clamp(-FREEFLY_PITCH_LIMIT, FREEFLY_PITCH_LIMIT);
    }

    let rotation = Quat::from_rotation_y(cam.yaw) * Quat::from_rotation_x(cam.pitch);
    transform.rotation = rotation;

    let forward = rotation * Vec3::NEG_Z;
    let right = rotation * Vec3::X;
    let up = Vec3::Y;

    let mut step = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        step += forward;
    }
    if keys.pressed(KeyCode::KeyS) {
        step -= forward;
    }
    if keys.pressed(KeyCode::KeyD) {
        step += right;
    }
    if keys.pressed(KeyCode::KeyA) {
        step -= right;
    }
    if keys.pressed(KeyCode::Space) {
        step += up;
    }
    if keys.pressed(KeyCode::ShiftLeft) {
        step -= up;
    }
    if step == Vec3::ZERO {
        return;
    }

    let boost = if keys.pressed(KeyCode::ControlLeft) {
        FREEFLY_BOOST_FACTOR
    } else {
        1.0
    };
    transform.translation += step.normalize() * FREEFLY_MOVE_SPEED_MPS * boost * dt;
}
