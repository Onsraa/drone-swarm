use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

use crate::world::WorldConfig;

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
            yaw: 0.6,
            pitch: 0.5,
            distance: 60.0,
        }
    }
}

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_camera)
            .add_systems(Update, (orbit_input, sync_camera_transform).chain());
    }
}

fn spawn_camera(mut commands: Commands, world: Res<WorldConfig>) {
    commands.spawn((
        Camera3d::default(),
        OrbitCamera {
            target: world.center(),
            ..Default::default()
        },
        Transform::default(),
    ));
}

fn orbit_input(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut q: Query<&mut OrbitCamera>,
) {
    let Ok(mut cam) = q.single_mut() else {
        return;
    };

    if mouse_buttons.pressed(MouseButton::Left) && motion.delta != Vec2::ZERO {
        cam.yaw -= motion.delta.x * 0.005;
        cam.pitch = (cam.pitch + motion.delta.y * 0.005).clamp(-1.4, 1.4);
    }

    if scroll.delta.y != 0.0 {
        let factor = 1.0 - scroll.delta.y * 0.1;
        cam.distance = (cam.distance * factor).clamp(4.0, 500.0);
    }
}

fn sync_camera_transform(mut q: Query<(&OrbitCamera, &mut Transform)>) {
    for (cam, mut t) in &mut q {
        let cp = cam.pitch.cos();
        let dir = Vec3::new(cam.yaw.sin() * cp, cam.pitch.sin(), cam.yaw.cos() * cp);
        let pos = cam.target + dir * cam.distance;
        *t = Transform::from_translation(pos).looking_at(cam.target, Vec3::Y);
    }
}
