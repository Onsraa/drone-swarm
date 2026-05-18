use std::collections::HashMap;

use bevy::prelude::*;

use crate::drone::Drone;
use crate::map::{CellState, LocalMap};
use crate::world::{GroundTruthMap, WorldConfig};

pub struct VoxelRenderPlugin;

#[derive(Component)]
pub struct GroundTruthVoxel;

#[derive(Component)]
pub struct LocalMapVoxel;

#[derive(Component, Default)]
pub struct LocalMapRender {
    spawned: HashMap<IVec3, Entity>,
}

#[derive(Resource)]
pub struct VoxelAssets {
    cube: Handle<Mesh>,
    ground_mat: Handle<StandardMaterial>,
    local_occ_mat: Handle<StandardMaterial>,
}

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Startup,
            (
                init_voxel_assets,
                spawn_ground_truth_voxels.after(init_voxel_assets),
            ),
        )
        .add_systems(Update, (ensure_local_render, sync_local_maps).chain());
    }
}

fn init_voxel_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<WorldConfig>,
) {
    let s = config.voxel_size;
    let cube = meshes.add(Cuboid::new(s, s, s));
    let ground_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.55, 0.55, 0.6),
        perceptual_roughness: 0.9,
        ..Default::default()
    });
    let local_occ_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.55, 0.1, 0.85),
        emissive: LinearRgba::rgb(0.45, 0.18, 0.0),
        alpha_mode: AlphaMode::Blend,
        ..Default::default()
    });
    commands.insert_resource(VoxelAssets {
        cube,
        ground_mat,
        local_occ_mat,
    });
}

fn spawn_ground_truth_voxels(
    mut commands: Commands,
    assets: Res<VoxelAssets>,
    config: Res<WorldConfig>,
    map: Res<GroundTruthMap>,
) {
    let s = config.voxel_size;
    let half = Vec3::splat(s * 0.5);
    let mut count = 0;
    for cell in map.iter_occupied() {
        let pos = cell.as_vec3() * s + half;
        commands.spawn((
            GroundTruthVoxel,
            Mesh3d(assets.cube.clone()),
            MeshMaterial3d(assets.ground_mat.clone()),
            Transform::from_translation(pos),
        ));
        count += 1;
    }
    info!("spawned {} voxel cubes for ground truth", count);
}

fn ensure_local_render(
    mut commands: Commands,
    q: Query<Entity, (With<Drone>, With<LocalMap>, Without<LocalMapRender>)>,
) {
    for e in &q {
        commands.entity(e).insert(LocalMapRender::default());
    }
}

fn sync_local_maps(
    mut commands: Commands,
    assets: Option<Res<VoxelAssets>>,
    config: Res<WorldConfig>,
    mut drones_q: Query<(&LocalMap, &mut LocalMapRender), With<Drone>>,
) {
    let Some(assets) = assets else {
        return;
    };
    let s = config.voxel_size;
    let half = Vec3::splat(s * 0.5);

    for (lm, mut render) in &mut drones_q {
        for (cell, state) in lm.0.iter_known() {
            if state == CellState::Occupied && !render.spawned.contains_key(&cell) {
                let pos = cell.as_vec3() * s + half;
                let e = commands
                    .spawn((
                        LocalMapVoxel,
                        Mesh3d(assets.cube.clone()),
                        MeshMaterial3d(assets.local_occ_mat.clone()),
                        Transform::from_translation(pos),
                    ))
                    .id();
                render.spawned.insert(cell, e);
            }
        }
    }
}
