use std::f32::consts::TAU;

use bevy::color::Hsla;
use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::exploration::{FrontierTarget, MovementHealth, Path};
use crate::physics::{DesiredAttitude, DesiredVelocity, LinearVelocity, ThrustState};
use crate::world::WorldConfig;

use super::components::{Drone, DroneColor, DroneId, PendingCenter, WanderTarget, WanderTimer};
use super::constants::{
    DRONE_COLOR_ALPHA, DRONE_COLOR_LIGHTNESS, DRONE_COLOR_SATURATION, DRONE_GLB_PATH,
    DRONE_HUE_STEP_DEGREES, DRONE_SCALE, DRONE_SPAWN_RADIUS_METERS, MODEL_YAW_OFFSET_RADIANS,
    RANDOM_DIR_MIN_LENGTH, WANDER_CHANGE_INTERVAL_SECS,
};
use super::resources::DroneSpawnConfig;

/// Each frame, if the drone count doesn't match `DroneSpawnConfig.target_count`,
/// despawn all current drones and respawn fresh ones. Cube cleanup of each
/// drone's local-map cubes is handled by the render module via removal
/// events, so this system only needs to manage drone entities themselves.
pub fn respawn_drones_if_needed(
    mut commands: Commands,
    spawn_config: Res<DroneSpawnConfig>,
    world: Res<WorldConfig>,
    asset_server: Res<AssetServer>,
    drones_q: Query<Entity, With<Drone>>,
) {
    let current_count = drones_q.iter().count() as u32;
    if current_count == spawn_config.target_count {
        return;
    }
    for entity in &drones_q {
        commands.entity(entity).despawn();
    }
    let world_center = world.center();
    let target = spawn_config.target_count;
    for id in 0..target {
        let spawn_pos = ring_position(world_center, id, target);
        let color = drone_color(id);
        spawn_one_drone(&mut commands, &asset_server, id, spawn_pos, color);
    }
    info!(
        "respawned drones: {} -> {}",
        current_count, spawn_config.target_count
    );
}

fn spawn_one_drone(
    commands: &mut Commands,
    asset_server: &AssetServer,
    id: u32,
    spawn_pos: Vec3,
    color: Color,
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
            FrontierTarget::default(),
            MovementHealth::default(),
            Path::default(),
            Transform::from_translation(spawn_pos).with_scale(Vec3::splat(DRONE_SCALE)),
            Visibility::default(),
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
