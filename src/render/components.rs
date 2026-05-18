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
