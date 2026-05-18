use std::collections::HashMap;

use bevy::prelude::*;

#[derive(Component)]
pub struct GroundTruthVoxel;

#[derive(Component)]
pub struct LocalMapVoxel;

#[derive(Component)]
pub struct GlobalMapVoxel;

#[derive(Component, Default)]
pub struct LocalMapRender {
    pub spawned: HashMap<IVec3, Entity>,
}

#[derive(Component)]
pub struct DroneMaterial(pub Handle<StandardMaterial>);

/// Tags a `LocalMapVoxel` with the drone entity that owns it, so the cube
/// can be despawned when that drone goes away.
#[derive(Component)]
pub struct OwnedByDrone(pub Entity);
