use std::f32::consts::TAU;

use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::exploration::{
    FrontierTarget, LastRoleChange, MovementHealth, Path, Role, RoleParams, Trail,
};
use crate::physics::{DesiredVelocity, LinearVelocity, PrevLinvel};
use crate::world::WorldConfig;

use super::components::{Drone, DroneColor, DroneId, WanderTarget, WanderTimer};
use super::constants::{
    DRONE_SPAWN_RADIUS_METERS, RANDOM_DIR_MIN_LENGTH, WANDER_CHANGE_INTERVAL_SECS,
};
use super::resources::{DroneBodyAssets, DroneSpawnConfig};

/// Each frame, if the drone count doesn't match `DroneSpawnConfig.target_count`,
/// despawn all current drones and respawn fresh ones. Cube cleanup of each
/// drone's local-map cubes is handled by the render module via removal
/// events, so this system only needs to manage drone entities themselves.
/// One-shot startup: builds the shared drone body mesh (a flat
/// rectangular cuboid) + three per-role `StandardMaterial`s and parks
/// them in `DroneBodyAssets`. Roles tint the body via material;
/// `sync_color_to_role` swaps the handle when supervisor changes a
/// drone's role mid-sim.
pub fn init_drone_body_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Cuboid::new(1.2, 0.4, 1.2));
    let make_mat = |role: Role, materials: &mut Assets<StandardMaterial>| {
        let tint = RoleParams::for_role(role).tint;
        materials.add(StandardMaterial {
            base_color: Color::linear_rgba(tint[0], tint[1], tint[2], 1.0),
            metallic: 0.15,
            perceptual_roughness: 0.55,
            ..default()
        })
    };
    let scout_material = make_mat(Role::Scout, &mut materials);
    let mapper_material = make_mat(Role::Mapper, &mut materials);
    let anchor_material = make_mat(Role::Anchor, &mut materials);
    commands.insert_resource(DroneBodyAssets {
        mesh,
        scout_material,
        mapper_material,
        anchor_material,
    });
}

pub fn respawn_drones_if_needed(
    mut commands: Commands,
    spawn_config: Res<DroneSpawnConfig>,
    world: Res<WorldConfig>,
    map: Option<Res<crate::world::GroundTruthMap>>,
    body_assets: Option<Res<DroneBodyAssets>>,
    drones_q: Query<Entity, With<Drone>>,
) {
    let current_count = drones_q.iter().count() as u32;
    if current_count == spawn_config.target_count {
        return;
    }
    let Some(body_assets) = body_assets else {
        // Startup ordering: body assets may not be ready on frame 1.
        // The respawn check runs again next tick.
        return;
    };
    for entity in &drones_q {
        commands.entity(entity).despawn();
    }
    let world_center = world.center();
    let target = spawn_config.target_count;
    for id in 0..target {
        let spawn_pos = ring_position(
            world_center,
            id,
            target,
            map.as_deref(),
            world.voxel_size,
        );
        let role = role_for_index(id, target);
        let tint = RoleParams::for_role(role).tint;
        let color = Color::linear_rgba(tint[0], tint[1], tint[2], tint[3]);
        spawn_one_drone(&mut commands, &body_assets, id, spawn_pos, color, role);
    }
    info!(
        "respawned drones: {} -> {}",
        current_count, spawn_config.target_count
    );
}

fn spawn_one_drone(
    commands: &mut Commands,
    body: &DroneBodyAssets,
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
            PrevLinvel::default(),
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
            Trail::default(),
            LastRoleChange::default(),
            Transform::from_translation(spawn_pos),
            Visibility::default(),
            Mesh3d(body.mesh.clone()),
            MeshMaterial3d(body.material_for(role)),
        ));
}

/// When the supervisor reassigns a drone's role (e.g. promotes a Scout
/// to Anchor at a comms articulation point), re-tint the `DroneColor`
/// so the local + global maps + drone model all flip to the new role's
/// palette. Driven by `Changed<Role>` so it only fires on actual
/// transitions, not every frame.
pub fn sync_color_to_role(
    body_assets: Option<Res<DroneBodyAssets>>,
    mut q: Query<
        (&Role, &mut DroneColor, &mut MeshMaterial3d<StandardMaterial>),
        (With<Drone>, Changed<Role>),
    >,
) {
    let Some(body_assets) = body_assets else { return; };
    for (role, mut color, mut mat) in &mut q {
        let tint = RoleParams::for_role(*role).tint;
        color.0 = Color::linear_rgba(tint[0], tint[1], tint[2], tint[3]);
        mat.0 = body_assets.material_for(*role);
    }
}

/// Stagger N drones around the world center on a horizontal ring of
/// `DRONE_SPAWN_RADIUS_METERS`. With N = 1 the drone lands at center.
///
/// Altitude comes from `GroundTruthMap::safe_spawn_cell_y` when a map
/// is present: walks the column at the drone's (x, z) and picks the
/// lowest Free cell that has 4 cells of clearance above it. Falls back
/// to `center.y` if no map is loaded or the column is fully occupied.
/// A tiny per-id altitude jitter prevents perfect stacking on the
/// same Y plane.
fn ring_position(
    center: Vec3,
    id: u32,
    count: u32,
    map: Option<&crate::world::GroundTruthMap>,
    voxel_size: f32,
) -> Vec3 {
    let (x, z) = if count <= 1 {
        (center.x, center.z)
    } else {
        let angle = (id as f32 / count as f32) * TAU;
        (
            center.x + angle.cos() * DRONE_SPAWN_RADIUS_METERS,
            center.z + angle.sin() * DRONE_SPAWN_RADIUS_METERS,
        )
    };

    let cell_x = (x / voxel_size).floor() as i32;
    let cell_z = (z / voxel_size).floor() as i32;
    let y = map
        .and_then(|m| m.safe_spawn_cell_y(cell_x, cell_z, 4))
        .map(|cy| (cy as f32 + 0.5) * voxel_size + 3.0)
        .unwrap_or(center.y);

    // 0–1.5 m altitude jitter per id so consecutive drones don't
    // stack on identical Y at spawn (and immediately fight the
    // peer-bubble).
    let jitter = (id as f32 % 4.0) * 0.5;
    Vec3::new(x, y + jitter, z)
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
