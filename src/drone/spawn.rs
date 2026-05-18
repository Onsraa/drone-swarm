use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::lidar::LastScanRays;
use crate::map::{LocalMap, VoxelMap};
use crate::world::WorldConfig;

use super::components::{Drone, DroneId, PendingCenter, Velocity, WalkTimer};
use super::constants::{
    DRONE_GLB_PATH, DRONE_SCALE, DRONE_SPEED_METERS_PER_SEC, RANDOM_DIR_MIN_LENGTH,
    WALK_CHANGE_INTERVAL_SECS,
};

pub fn spawn_drone(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    config: Res<WorldConfig>,
) {
    let spawn_pos = config.center();
    let mut rng = rand::rng();
    let initial_dir = random_unit_dir(&mut rng);

    commands
        .spawn((
            Drone,
            DroneId(0),
            Velocity(initial_dir * DRONE_SPEED_METERS_PER_SEC),
            WalkTimer(Timer::from_seconds(
                WALK_CHANGE_INTERVAL_SECS,
                TimerMode::Repeating,
            )),
            Transform::from_translation(spawn_pos).with_scale(Vec3::splat(DRONE_SCALE)),
            Visibility::default(),
            LocalMap(VoxelMap::new(config.size)),
            LastScanRays::default(),
        ))
        .with_children(|parent| {
            parent.spawn((
                SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(DRONE_GLB_PATH))),
                Transform::default(),
                PendingCenter,
            ));
        });
    info!("spawned drone 0 at {:?}", spawn_pos);
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
