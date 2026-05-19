use std::path::Path;

use bevy::prelude::*;

use super::constants::PRESETS_PATH;
use super::io::{load_from_disk, save_to_disk};
use super::resources::DroneGroupPresets;

pub fn load_presets_on_startup(mut presets: ResMut<DroneGroupPresets>) {
    match load_from_disk(Path::new(PRESETS_PATH)) {
        Some(entries) => {
            let count = entries.len();
            presets.entries = entries;
            info!("loaded {} drone presets from {}", count, PRESETS_PATH);
        }
        None => info!(
            "no presets file at {} — starting with empty list",
            PRESETS_PATH
        ),
    }
}

/// Persist whenever the presets resource changes (added / renamed /
/// removed). Skips the initial change-detect tick that fires right
/// after `load_presets_on_startup` writes the loaded list.
pub fn autosave_presets(
    presets: Res<DroneGroupPresets>,
    mut first: Local<bool>,
) {
    if !*first {
        *first = true;
        return;
    }
    if !presets.is_changed() {
        return;
    }
    if let Err(e) = save_to_disk(Path::new(PRESETS_PATH), &presets) {
        warn!("failed to write {}: {}", PRESETS_PATH, e);
    }
}
