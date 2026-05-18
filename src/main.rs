use bevy::prelude::*;
use bevy_egui::EguiPlugin;

mod camera;
mod drone;
mod lidar;
mod map;
mod ui;
mod voxel_render;
mod world;

use camera::OrbitCameraPlugin;
use drone::DronePlugin;
use lidar::LidarPlugin;
use ui::UiPlugin;
use voxel_render::VoxelRenderPlugin;
use world::{WorldConfig, WorldPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin::default())
        .add_plugins(WorldPlugin)
        .add_plugins(VoxelRenderPlugin)
        .add_plugins(DronePlugin)
        .add_plugins(LidarPlugin)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(UiPlugin)
        .add_systems(Startup, setup_lighting)
        .add_systems(Update, draw_world_bounds)
        .run();
}

fn setup_lighting(mut commands: Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            shadows_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(20.0, 50.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn draw_world_bounds(mut gizmos: Gizmos, world: Res<WorldConfig>) {
    let size = world.world_size();
    let center = world.center();
    gizmos.cube(
        Transform::from_translation(center).with_scale(size),
        Color::srgb(0.7, 0.7, 0.7),
    );
}
