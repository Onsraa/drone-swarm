use std::f32::consts::TAU;

use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::exploration::{
    FrontierTarget, LastRoleChange, MovementHealth, Path, Role, RoleParams,
};
use crate::physics::{DesiredAttitude, DesiredVelocity, LinearVelocity, ThrustState};
use crate::world::WorldConfig;

use super::components::{Drone, DroneColor, DroneId, PendingCenter, WanderTarget, WanderTimer};
use super::constants::{
    DRONE_GLB_PATH, DRONE_SCALE, DRONE_SPAWN_RADIUS_METERS, MODEL_YAW_OFFSET_RADIANS,
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
        let role = role_for_index(id, target);
        let tint = RoleParams::for_role(role).tint;
        let color = Color::linear_rgba(tint[0], tint[1], tint[2], tint[3]);
        spawn_one_drone(&mut commands, &asset_server, id, spawn_pos, color, role);
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
    role: Role,
) {
    commands
        .spawn((
            Drone,
            DroneId(id),
            DroneColor(color),
            role,
            LinearVelocity::default(),
            DesiredVelocity::default(),
            ThrustState::default(),
            DesiredAttitude::default(),
            WanderTimer(Timer::from_seconds(
                WANDER_CHANGE_INTERVAL_SECS,
                TimerMode::Repeating,
            )),
            WanderTarget::default(),
        ))
        .insert((
            FrontierTarget::default(),
            MovementHealth::default(),
            Path::default(),
            LastRoleChange::default(),
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

/// Assign a default role based on position in the fleet.
/// Distribution: 60% Scout / 30% Mapper / 10% Anchor.
/// Guarantee: at least 1 Mapper when N >= 4; below that pure Scout/Mapper split.
fn role_for_index(id: u32, total: u32) -> Role {
    // Number of each role (rounded down, scouts get the remainder)
    let n_anchor = if total >= 10 { total / 10 } else { 0 };
    let n_mapper = if total >= 4 {
        (total * 3 / 10).max(1)
    } else {
        total / 3
    };
    // Anchors occupy the last slots, mappers the block before them, scouts the rest.
    if id >= total - n_anchor {
        Role::Anchor
    } else if id >= total - n_anchor - n_mapper {
        Role::Mapper
    } else {
        Role::Scout
    }
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
