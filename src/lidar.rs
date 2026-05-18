use bevy::prelude::*;

use crate::drone::Drone;
use crate::map::{CellState, LocalMap, VoxelMap};
use crate::world::{GroundTruthMap, WorldConfig};

const RAYS_PER_SCAN: usize = 64;
const MAX_RANGE: f32 = 20.0;

#[derive(Resource)]
pub struct ScanTimer(pub Timer);

/// Last frame's ray hits, kept for gizmo visualization.
#[derive(Component, Default)]
pub struct LastScanRays(pub Vec<(Vec3, Vec3)>);

pub struct LidarPlugin;

impl Plugin for LidarPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ScanTimer(Timer::from_seconds(0.2, TimerMode::Repeating)))
            .add_systems(Update, lidar_scan);
    }
}

pub fn fibonacci_sphere(n: usize) -> Vec<Vec3> {
    let golden = std::f32::consts::PI * (3.0 - (5.0_f32).sqrt());
    let denom = (n.saturating_sub(1).max(1)) as f32;
    (0..n)
        .map(|i| {
            let y = 1.0 - (i as f32 / denom) * 2.0;
            let r = (1.0 - y * y).max(0.0).sqrt();
            let theta = golden * i as f32;
            Vec3::new(theta.cos() * r, y, theta.sin() * r).normalize_or_zero()
        })
        .collect()
}

fn lidar_scan(
    time: Res<Time>,
    mut timer: ResMut<ScanTimer>,
    config: Res<WorldConfig>,
    ground: Res<GroundTruthMap>,
    mut drones_q: Query<(&Transform, &mut LocalMap, Option<&mut LastScanRays>), With<Drone>>,
) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let dirs = fibonacci_sphere(RAYS_PER_SCAN);
    let voxel_size = config.voxel_size;
    let max_steps = MAX_RANGE / voxel_size;

    for (t, mut local, rays_opt) in &mut drones_q {
        let origin_world = t.translation;
        let origin_grid = origin_world / voxel_size;
        let mut hits: Vec<(Vec3, Vec3)> = Vec::with_capacity(dirs.len());
        for &dir in &dirs {
            let end_cell = cast_ray(origin_grid, dir, max_steps, &ground, &mut local.0);
            let end_world = (end_cell.as_vec3() + Vec3::splat(0.5)) * voxel_size;
            hits.push((origin_world, end_world));
        }
        if let Some(mut r) = rays_opt {
            r.0 = hits;
        }
    }
}

fn cast_ray(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    ground: &GroundTruthMap,
    local: &mut VoxelMap,
) -> IVec3 {
    let mut last_cell = IVec3::new(
        origin.x.floor() as i32,
        origin.y.floor() as i32,
        origin.z.floor() as i32,
    );
    for (cell, _t) in voxel_traverse(origin, dir, max_dist) {
        last_cell = cell;
        if ground.get(cell) {
            local.upgrade(cell, CellState::Occupied);
            return cell;
        }
        local.upgrade(cell, CellState::Free);
    }
    last_cell
}

/// Amanatides-Woo voxel traversal. Walks the integer-cell grid along `dir`
/// from `origin` (in voxel units) up to `max_dist` voxel units. Yields each
/// cell along with the parametric distance `t` at which the ray crosses
/// into it.
pub fn voxel_traverse(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
) -> impl Iterator<Item = (IVec3, f32)> {
    let mut out: Vec<(IVec3, f32)> = Vec::new();
    let dir = dir.normalize_or_zero();
    if dir.length_squared() == 0.0 || max_dist <= 0.0 {
        return out.into_iter();
    }

    let mut cell = IVec3::new(
        origin.x.floor() as i32,
        origin.y.floor() as i32,
        origin.z.floor() as i32,
    );
    let step = IVec3::new(
        dir.x.signum() as i32,
        dir.y.signum() as i32,
        dir.z.signum() as i32,
    );
    let t_delta = Vec3::new(
        if dir.x != 0.0 { 1.0 / dir.x.abs() } else { f32::INFINITY },
        if dir.y != 0.0 { 1.0 / dir.y.abs() } else { f32::INFINITY },
        if dir.z != 0.0 { 1.0 / dir.z.abs() } else { f32::INFINITY },
    );

    let first_boundary = |coord: i32, step: i32, origin: f32| -> f32 {
        if step > 0 {
            (coord + 1) as f32 - origin
        } else if step < 0 {
            origin - coord as f32
        } else {
            f32::INFINITY
        }
    };

    let t_max_axis = |c: i32, s: i32, o: f32, d: f32| -> f32 {
        if s == 0 {
            f32::INFINITY
        } else {
            first_boundary(c, s, o) / d.abs()
        }
    };
    let mut t_max = Vec3::new(
        t_max_axis(cell.x, step.x, origin.x, dir.x),
        t_max_axis(cell.y, step.y, origin.y, dir.y),
        t_max_axis(cell.z, step.z, origin.z, dir.z),
    );

    out.push((cell, 0.0));
    let mut t: f32;
    loop {
        if t_max.x < t_max.y && t_max.x < t_max.z {
            cell.x += step.x;
            t = t_max.x;
            t_max.x += t_delta.x;
        } else if t_max.y < t_max.z {
            cell.y += step.y;
            t = t_max.y;
            t_max.y += t_delta.y;
        } else {
            cell.z += step.z;
            t = t_max.z;
            t_max.z += t_delta.z;
        }
        if t > max_dist {
            break;
        }
        out.push((cell, t));
    }
    out.into_iter()
}
