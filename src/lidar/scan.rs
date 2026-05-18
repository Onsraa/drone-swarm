use bevy::prelude::*;

use crate::drone::Drone;
use crate::map::{CellState, LocalMap, VoxelMap};
use crate::world::{GroundTruthMap, WorldConfig};

use super::components::LastScanRays;
use super::constants::{MAX_RANGE_METERS, RAYS_PER_SCAN};
use super::resources::ScanTimer;
use super::sampling::fibonacci_sphere;
use super::traversal::voxel_traverse;

pub fn lidar_scan(
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
    let max_grid_steps = MAX_RANGE_METERS / voxel_size;

    for (transform, mut local, rays_opt) in &mut drones_q {
        let origin_world = transform.translation;
        let origin_grid = origin_world / voxel_size;
        let mut hits: Vec<(Vec3, Vec3)> = Vec::with_capacity(dirs.len());
        for &dir in &dirs {
            let end_cell = cast_ray(origin_grid, dir, max_grid_steps, &ground, &mut local.0);
            let end_world = (end_cell.as_vec3() + Vec3::splat(0.5)) * voxel_size;
            hits.push((origin_world, end_world));
        }
        if let Some(mut rays) = rays_opt {
            rays.0 = hits;
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
