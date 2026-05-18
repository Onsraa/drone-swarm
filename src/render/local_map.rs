use bevy::prelude::*;

use crate::drone::{Drone, DroneColor};
use crate::map::{CellState, LocalMap};
use crate::world::WorldConfig;

use super::assets::VoxelAssets;
use super::components::{DroneMaterial, LocalMapRender, LocalMapVoxel};
use super::constants::LOCAL_MAP_EMISSIVE_FACTOR;

/// Lazily create per-drone material + render tracker. Lives in the render
/// module so the drone module never imports `StandardMaterial`.
pub fn ensure_local_render(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    drones_q: Query<
        (Entity, &DroneColor),
        (With<Drone>, With<LocalMap>, Without<LocalMapRender>),
    >,
) {
    for (entity, color) in &drones_q {
        let linear = color.0.to_linear();
        let material = materials.add(StandardMaterial {
            base_color: color.0,
            emissive: LinearRgba::rgb(
                linear.red * LOCAL_MAP_EMISSIVE_FACTOR,
                linear.green * LOCAL_MAP_EMISSIVE_FACTOR,
                linear.blue * LOCAL_MAP_EMISSIVE_FACTOR,
            ),
            alpha_mode: AlphaMode::Blend,
            ..Default::default()
        });
        commands
            .entity(entity)
            .insert((LocalMapRender::default(), DroneMaterial(material)));
    }
}

pub fn sync_local_maps(
    mut commands: Commands,
    assets: Option<Res<VoxelAssets>>,
    config: Res<WorldConfig>,
    mut drones_q: Query<(&LocalMap, &mut LocalMapRender, &DroneMaterial), With<Drone>>,
) {
    let Some(assets) = assets else {
        return;
    };
    let s = config.voxel_size;
    let half = Vec3::splat(s * 0.5);

    for (local_map, mut render, material) in &mut drones_q {
        for (cell, state) in local_map.0.iter_known() {
            if state == CellState::Occupied && !render.spawned.contains_key(&cell) {
                let position = cell.as_vec3() * s + half;
                let entity = commands
                    .spawn((
                        LocalMapVoxel,
                        Mesh3d(assets.cube.clone()),
                        MeshMaterial3d(material.0.clone()),
                        Transform::from_translation(position),
                    ))
                    .id();
                render.spawned.insert(cell, entity);
            }
        }
    }
}
