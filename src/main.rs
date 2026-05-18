use bevy::prelude::*;
use bevy_egui::EguiPlugin;

mod camera;
mod debug;
mod drone;
mod lidar;
mod lighting;
mod map;
mod merge;
mod physics;
mod render;
mod ui;
mod world;

use camera::OrbitCameraPlugin;
use debug::DebugPlugin;
use drone::DronePlugin;
use lidar::LidarPlugin;
use lighting::LightingPlugin;
use merge::MergePlugin;
use physics::PhysicsPlugin;
use render::VoxelRenderPlugin;
use ui::UiPlugin;
use world::WorldPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin::default())
        .add_plugins(WorldPlugin)
        .add_plugins(LightingPlugin)
        .add_plugins(VoxelRenderPlugin)
        .add_plugins(PhysicsPlugin)
        .add_plugins(DronePlugin)
        .add_plugins(LidarPlugin)
        .add_plugins(MergePlugin)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(UiPlugin)
        .add_plugins(DebugPlugin)
        .run();
}
