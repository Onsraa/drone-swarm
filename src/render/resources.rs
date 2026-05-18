use bevy::prelude::*;

/// Shared unit-cube mesh used by every instanced voxel layer.
#[derive(Resource)]
pub struct CubeMesh(pub Handle<Mesh>);
