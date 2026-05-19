use bevy::prelude::*;

use super::asset::MapAsset;

/// Published by the UI (map combo box) or any debug system that wants
/// to swap the current ground-truth map. The handler in `lidar/gpu`
/// reads the asset from `Assets<MapAsset>`, tears down + reallocates
/// the GPU lidar buffers at the new dims, and respawns drones.
#[derive(Message, Clone, Debug)]
pub struct MapSwapRequested {
    pub handle: Handle<MapAsset>,
    pub name: String,
}
