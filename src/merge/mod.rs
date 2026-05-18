mod constants;
mod resources;
mod systems;

use bevy::prelude::*;

use constants::MERGE_INTERVAL_SECS;
use resources::MergeTimer;
use systems::{init_global_map, merge_local_into_global};

pub struct MergePlugin;

impl Plugin for MergePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MergeTimer(Timer::from_seconds(
            MERGE_INTERVAL_SECS,
            TimerMode::Repeating,
        )))
        .add_systems(Startup, init_global_map)
        .add_systems(Update, merge_local_into_global);
    }
}
