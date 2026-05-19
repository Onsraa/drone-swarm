use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy_egui::EguiPlugin;

mod camera;
mod comms;
mod debug;
mod drone;
mod frontier;
mod groups;
mod lidar;
mod lighting;
mod maps;
mod physics;
mod render;
mod ui;
mod world;

use camera::OrbitCameraPlugin;
use comms::CommsPlugin;
use debug::DebugPlugin;
use drone::DronePlugin;
use frontier::FrontierPlugin;
use groups::DroneGroupPresetsPlugin;
use lidar::{GpuLidarPlugin, LidarPlugin};
use lighting::LightingPlugin;
use maps::MapsPlugin;
use physics::PhysicsPlugin;
use render::VoxelRenderPlugin;
use ui::UiPlugin;
use world::WorldPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(LogPlugin {
            // Drone GLB carries extra TEXCOORD_2..9 channels Bevy's glTF
            // loader doesn't know about. They're harmless but spam WARN on
            // every load. Bump that target above WARN to keep the boot log
            // readable.
            filter: "info,wgpu_core=warn,wgpu_hal=warn,naga=warn,bevy_gltf::loader=error".to_string(),
            level: bevy::log::Level::INFO,
            ..default()
        }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(EguiPlugin::default())
        .add_plugins(MapsPlugin)
        .add_plugins(WorldPlugin)
        .add_plugins(LightingPlugin)
        .add_plugins(VoxelRenderPlugin)
        .add_plugins(PhysicsPlugin)
        .add_plugins(DronePlugin)
        .add_plugins(LidarPlugin)
        .add_plugins(GpuLidarPlugin)
        .add_plugins(FrontierPlugin)
        .add_plugins(CommsPlugin)
        .add_plugins(DroneGroupPresetsPlugin)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(UiPlugin)
        .add_plugins(DebugPlugin)
        .run();
}
