use bevy::prelude::*;

#[derive(Component, Default)]
pub struct LastScanRays(pub Vec<(Vec3, Vec3)>);
