mod world_bounds;

use bevy::prelude::*;

use world_bounds::draw_world_bounds;

pub struct DebugPlugin;

impl Plugin for DebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, draw_world_bounds);
    }
}
