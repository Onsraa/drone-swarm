use bevy::prelude::*;

use crate::world::{GroundTruthMap, WorldConfig};

pub struct VoxelRenderPlugin;

impl Plugin for VoxelRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Startup,
            spawn_ground_truth_voxels.after(crate::world::build_test_scene),
        );
    }
}

fn spawn_ground_truth_voxels(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<WorldConfig>,
    map: Res<GroundTruthMap>,
) {
    let s = config.voxel_size;
    let cube = meshes.add(Cuboid::new(s, s, s));
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.55, 0.55, 0.6),
        perceptual_roughness: 0.9,
        ..Default::default()
    });
    let half = Vec3::splat(s * 0.5);

    let mut count = 0;
    for cell in map.iter_occupied() {
        let pos = cell.as_vec3() * s + half;
        commands.spawn((
            Mesh3d(cube.clone()),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(pos),
        ));
        count += 1;
    }
    info!("spawned {} voxel cubes for ground truth", count);
}
