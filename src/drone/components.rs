use bevy::prelude::*;

#[derive(Component)]
pub struct Drone;

#[derive(Component)]
pub struct DroneId(pub u32);

#[derive(Component)]
pub struct DroneColor(pub Color);
