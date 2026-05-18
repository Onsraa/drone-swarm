use bevy::prelude::*;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
#[allow(dead_code)]
pub struct DroneId(pub u32);

#[derive(Component)]
pub struct Velocity(pub Vec3);

#[derive(Component)]
pub struct WalkTimer(pub Timer);

/// Marker on the SceneRoot child while waiting for GLB meshes to load.
/// Once meshes resolve, the centering system shifts this entity by
/// -mesh_center so rotations pivot on the model's geometric center
/// instead of the authored origin.
#[derive(Component)]
pub struct PendingCenter;
