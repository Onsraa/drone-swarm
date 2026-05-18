use bevy::prelude::*;

use super::resources::CubeMesh;

pub fn init_voxel_assets(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    let cube = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    commands.insert_resource(CubeMesh(cube));
}
