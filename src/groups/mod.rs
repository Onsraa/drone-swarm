mod constants;
mod io;
mod resources;
mod systems;

use bevy::prelude::*;

pub use resources::DroneGroupPresets;

use systems::{autosave_presets, load_presets_on_startup};

/// Saved drone-visibility masks. Loaded from `presets.txt` at startup,
/// auto-written on any change. The panel UI reads/mutates the
/// `DroneGroupPresets` resource directly.
pub struct DroneGroupPresetsPlugin;

impl Plugin for DroneGroupPresetsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DroneGroupPresets>()
            .add_systems(Startup, load_presets_on_startup)
            .add_systems(Update, autosave_presets);
    }
}
