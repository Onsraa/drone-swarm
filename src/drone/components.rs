use bevy::prelude::*;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
#[allow(dead_code)]
pub struct DroneId(pub u32);

#[derive(Component)]
pub struct DroneColor(pub Color);

#[derive(Component)]
pub struct WanderTimer(pub Timer);

#[derive(Component, Default)]
pub struct WanderTarget(pub Vec3);

/// Marker on the SceneRoot child while waiting for GLB meshes to load.
/// Once meshes resolve, the centering system shifts this entity by
/// -mesh_center so rotations pivot on the model's geometric center
/// instead of the authored origin.
#[derive(Component)]
pub struct PendingCenter;
