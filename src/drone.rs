use bevy::camera::primitives::MeshAabb;
use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use rand::{Rng, RngExt};

use crate::lidar::LastScanRays;
use crate::map::{LocalMap, VoxelMap};
use crate::world::WorldConfig;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
pub struct DroneId(pub u32);

#[derive(Component)]
pub struct Velocity(pub Vec3);

#[derive(Component)]
pub struct WalkTimer(Timer);

/// Marker on the SceneRoot child while we wait for its mesh assets to load,
/// so we can shift the child by -mesh_center and pivot rotations on the model's geometric center.
#[derive(Component)]
struct PendingCenter;

pub struct DronePlugin;

impl Plugin for DronePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_drone)
            .add_systems(Update, (recenter_visuals, random_walk, integrate_motion).chain());
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

    commands
        .spawn((
            Drone,
            DroneId(0),
            Velocity(dir * DRONE_SPEED),
            WalkTimer(Timer::from_seconds(
                WALK_CHANGE_INTERVAL,
                TimerMode::Repeating,
            )),
            Transform::from_translation(pos).with_scale(Vec3::splat(DRONE_SCALE)),
            Visibility::default(),
            LocalMap(VoxelMap::new(config.size)),
            LastScanRays::default(),
        ))
        .with_children(|p| {
            p.spawn((
                SceneRoot(
                    asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/drone.glb")),
                ),
                Transform::default(),
                PendingCenter,
            ));
        });
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

/// Once GLB meshes finish loading, compute the union AABB of all descendant
/// meshes (in this entity's local frame) and offset translation by -center.
/// Combined with the parent's Transform, this makes rotations pivot on the
/// drone's geometric center instead of the model's authored origin (its head).
fn recenter_visuals(
    mut commands: Commands,
    mut pending_q: Query<(Entity, &mut Transform), With<PendingCenter>>,
    children_q: Query<&Children>,
    other_transform_q: Query<&Transform, Without<PendingCenter>>,
    mesh3d_q: Query<&Mesh3d>,
    meshes: Res<Assets<Mesh>>,
) {
    for (root, mut root_t) in &mut pending_q {
        let mut bounds: Option<(Vec3, Vec3)> = None;
        let mut stack: Vec<(Entity, Mat4)> = vec![(root, Mat4::IDENTITY)];

        while let Some((entity, to_root)) = stack.pop() {
            if let Ok(mesh3d) = mesh3d_q.get(entity)
                && let Some(mesh) = meshes.get(&mesh3d.0)
                && let Some(aabb) = mesh.compute_aabb()
            {
                let center = Vec3::from(aabb.center);
                let he = Vec3::from(aabb.half_extents);
                for i in 0..8u32 {
                    let sign = Vec3::new(
                        if i & 1 != 0 { 1.0 } else { -1.0 },
                        if i & 2 != 0 { 1.0 } else { -1.0 },
                        if i & 4 != 0 { 1.0 } else { -1.0 },
                    );
                    let p = to_root.transform_point3(center + sign * he);
                    bounds = Some(match bounds {
                        None => (p, p),
                        Some((lo, hi)) => (lo.min(p), hi.max(p)),
                    });
                }
            }

            if let Ok(children) = children_q.get(entity) {
                for &child in children {
                    let local = other_transform_q
                        .get(child)
                        .map(|t| t.to_matrix())
                        .unwrap_or(Mat4::IDENTITY);
                    stack.push((child, to_root * local));
                }
            }
        }

        if let Some((lo, hi)) = bounds {
            let model_center = (lo + hi) * 0.5;
            root_t.translation = -model_center;
            info!(
                "recentered drone GLB: mesh center {:?} in scene-root local space",
                model_center
            );
            commands.entity(root).remove::<PendingCenter>();
        }
    }
}
