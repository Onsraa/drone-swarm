use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::world::WorldConfig;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
pub struct DroneId(pub u32);

#[derive(Component)]
pub struct Velocity(pub Vec3);

#[derive(Component)]
pub struct WalkTimer(Timer);

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_drone)
            .add_systems(Update, (random_walk, integrate_motion).chain());
    }
}

const DRONE_SPEED: f32 = 3.0;
const WALK_CHANGE_INTERVAL: f32 = 2.0;
const BOUND_MARGIN: f32 = 1.5;
const DRONE_SCALE: f32 = 0.1;
const ROTATION_LERP: f32 = 6.0;

fn spawn_drone(mut commands: Commands, asset_server: Res<AssetServer>, config: Res<WorldConfig>) {
    let world_size = config.world_size();
    let pos = Vec3::new(world_size.x * 0.5, world_size.y * 0.5, world_size.z * 0.5);

    let mut rng = rand::rng();
    let dir = random_unit_dir(&mut rng);

    commands.spawn((
        Drone,
        DroneId(0),
        Velocity(dir * DRONE_SPEED),
        WalkTimer(Timer::from_seconds(
            WALK_CHANGE_INTERVAL,
            TimerMode::Repeating,
        )),
        SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/drone.glb"))),
        Transform::from_translation(pos).with_scale(Vec3::splat(DRONE_SCALE)),
    ));
    info!("spawned drone 0 at {:?}", pos);
}

fn random_unit_dir(rng: &mut impl Rng) -> Vec3 {
    loop {
        let v = Vec3::new(
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
        );
        let len = v.length();
        if len > 0.1 {
            return v / len;
        }
    }
}

fn random_walk(time: Res<Time>, mut q: Query<(&mut Velocity, &mut WalkTimer), With<Drone>>) {
    let mut rng = rand::rng();
    for (mut vel, mut timer) in &mut q {
        timer.0.tick(time.delta());
        if timer.0.just_finished() {
            vel.0 = random_unit_dir(&mut rng) * DRONE_SPEED;
        }
    }
}

fn integrate_motion(
    time: Res<Time>,
    config: Res<WorldConfig>,
    mut q: Query<(&mut Transform, &mut Velocity), With<Drone>>,
) {
    let world_size = config.world_size();
    let lo = Vec3::splat(BOUND_MARGIN);
    let hi = world_size - Vec3::splat(BOUND_MARGIN);
    let dt = time.delta_secs();

    for (mut t, mut v) in &mut q {
        let p = t.translation + v.0 * dt;
        let (px, vx) = reflect_axis(p.x, lo.x, hi.x, v.0.x);
        let (py, vy) = reflect_axis(p.y, lo.y, hi.y, v.0.y);
        let (pz, vz) = reflect_axis(p.z, lo.z, hi.z, v.0.z);
        t.translation = Vec3::new(px, py, pz);
        v.0 = Vec3::new(vx, vy, vz);

        // Smoothly face direction of motion (Y is up). Slerp avoids snap on velocity flips.
        let dir = v.0.normalize_or_zero();
        if dir.length_squared() > 0.0 {
            let mut target = *t;
            target.look_to(dir, Vec3::Y);
            let alpha = (ROTATION_LERP * dt).min(1.0);
            t.rotation = t.rotation.slerp(target.rotation, alpha);
        }
    }
}

fn reflect_axis(p: f32, lo: f32, hi: f32, v: f32) -> (f32, f32) {
    if p < lo {
        (lo, v.abs())
    } else if p > hi {
        (hi, -v.abs())
    } else {
        (p, v)
    }
}
