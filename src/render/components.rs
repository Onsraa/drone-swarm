use bevy::prelude::*;

/// Marker on the single ground-truth chunk mesh entity.
#[derive(Component)]
pub struct GroundTruthVoxel;

/// Marker on a per-drone local-map chunk mesh entity.
#[derive(Component)]
pub struct LocalMapVoxel;

/// Marker on the single global-map chunk mesh entity.
#[derive(Component)]
pub struct GlobalMapVoxel;

#[derive(Component)]
pub struct DroneMaterial(pub Handle<StandardMaterial>);

/// Component on each drone pointing at the mesh handle that renders its
/// local map. Lets us rebuild only the meshes whose owning drone's
/// `LocalMap` changed this frame.
#[derive(Component)]
pub struct LocalMapMeshHandle {
    pub mesh: Handle<Mesh>,
}

/// Tags a per-drone mesh entity with its owning drone so cleanup can
/// despawn it when the drone goes away.
#[derive(Component)]
pub struct OwnedByDrone(pub Entity);
