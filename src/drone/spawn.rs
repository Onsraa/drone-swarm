use std::f32::consts::TAU;

use bevy::color::Hsla;
use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::lidar::LastScanRays;
use crate::map::{LocalMap, VoxelMap};
use crate::physics::{DesiredAttitude, DesiredVelocity, LinearVelocity, ThrustState};
use crate::world::WorldConfig;

use super::components::{Drone, DroneColor, DroneId, PendingCenter, WanderTarget, WanderTimer};
use super::constants::{
    DEFAULT_DRONE_COUNT, DRONE_COLOR_ALPHA, DRONE_COLOR_LIGHTNESS, DRONE_COLOR_SATURATION,
    DRONE_GLB_PATH, DRONE_HUE_STEP_DEGREES, DRONE_SCALE, DRONE_SPAWN_RADIUS_METERS,
    MODEL_YAW_OFFSET_RADIANS, RANDOM_DIR_MIN_LENGTH, WANDER_CHANGE_INTERVAL_SECS,
};

pub fn spawn_drones(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    config: Res<WorldConfig>,
) {
    let world_center = config.center();
    for id in 0..DEFAULT_DRONE_COUNT {
        let spawn_pos = ring_position(world_center, id, DEFAULT_DRONE_COUNT);
        let color = drone_color(id);
        spawn_one_drone(
            &mut commands,
            &asset_server,
            id,
            spawn_pos,
            color,
            config.size,
        );
    }
    info!("spawned {} drones", DEFAULT_DRONE_COUNT);
}

fn spawn_one_drone(
    commands: &mut Commands,
    asset_server: &AssetServer,
    id: u32,
    spawn_pos: Vec3,
    color: Color,
    map_dims: UVec3,
) {
    commands
        .spawn((
            Drone,
            DroneId(id),
            DroneColor(color),
            LinearVelocity::default(),
            DesiredVelocity::default(),
            ThrustState::default(),
            DesiredAttitude::default(),
            WanderTimer(Timer::from_seconds(
                WANDER_CHANGE_INTERVAL_SECS,
                TimerMode::Repeating,
            )),
            WanderTarget::default(),
            Transform::from_translation(spawn_pos).with_scale(Vec3::splat(DRONE_SCALE)),
            Visibility::default(),
            LocalMap(VoxelMap::new(map_dims)),
            LastScanRays::default(),
        ))
        .with_children(|parent| {
            parent.spawn((
                SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(DRONE_GLB_PATH))),
                Transform::from_rotation(Quat::from_rotation_y(MODEL_YAW_OFFSET_RADIANS)),
                PendingCenter,
            ));
        });
}

/// Stagger N drones around the world center on a horizontal ring of
/// `DRONE_SPAWN_RADIUS_METERS`. With N = 1 the drone lands at center.
fn ring_position(center: Vec3, id: u32, count: u32) -> Vec3 {
    if count <= 1 {
        return center;
    }
    let angle = (id as f32 / count as f32) * TAU;
    Vec3::new(
        center.x + angle.cos() * DRONE_SPAWN_RADIUS_METERS,
        center.y,
        center.z + angle.sin() * DRONE_SPAWN_RADIUS_METERS,
    )
}

/// Golden-ratio hue spacing keeps adjacent ids perceptually distinct even
/// for 50+ drones.
fn drone_color(id: u32) -> Color {
    let hue = (id as f32 * DRONE_HUE_STEP_DEGREES).rem_euclid(360.0);
    Hsla::new(
        hue,
        DRONE_COLOR_SATURATION,
        DRONE_COLOR_LIGHTNESS,
        DRONE_COLOR_ALPHA,
    )
    .into()
}

pub fn random_unit_dir(rng: &mut impl Rng) -> Vec3 {
    loop {
        let v = Vec3::new(
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
        );
        let len = v.length();
        if len > RANDOM_DIR_MIN_LENGTH {
            return v / len;
        }
    }
}
