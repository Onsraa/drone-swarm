use bevy::prelude::*;

use crate::world::WorldConfig;

const BOUNDS_COLOR: Color = Color::srgb(0.7, 0.7, 0.7);

pub fn draw_world_bounds(mut gizmos: Gizmos, world: Res<WorldConfig>) {
    let size = world.world_size();
    let center = world.center();
    gizmos.cube(
        Transform::from_translation(center).with_scale(size),
        BOUNDS_COLOR,
    );
}
