use std::path::Path;

use bevy::asset::Handle;
use bevy::prelude::*;

use super::asset::MapAsset;

/// One entry per `.dvm` file discovered under `assets/maps/`. `name` is
/// the bare filename (`clusters.dvm`); `handle` is the pre-loaded
/// asset handle the UI can drop into a `MapSwapRequested` event.
pub struct MapEntry {
    pub name: String,
    pub handle: Handle<MapAsset>,
}

#[derive(Resource, Default)]
pub struct AvailableMaps {
    pub entries: Vec<MapEntry>,
    pub selected: Option<usize>,
}

/// Scan `assets/maps/*.dvm`, load each one through the AssetServer, and
/// store the resulting handles in `AvailableMaps`. Runs at startup
/// after the maps directory has been seeded.
pub fn scan_maps_dir(asset_server: Res<AssetServer>, mut registry: ResMut<AvailableMaps>) {
    registry.entries.clear();
    let dir = Path::new("assets/maps");
    let mut names: Vec<String> = match std::fs::read_dir(dir) {
        Ok(read) => read
            .filter_map(|e| e.ok())
            .filter_map(|e| e.file_name().into_string().ok())
            .filter(|n| n.ends_with(".dvm") || n.ends_with(".vox"))
            .collect(),
        Err(e) => {
            warn!("failed to read {}: {}", dir.display(), e);
            Vec::new()
        }
    };
    names.sort();

    for name in &names {
        let asset_path = format!("maps/{}", name);
        let handle: Handle<MapAsset> = asset_server.load(&asset_path);
        registry.entries.push(MapEntry {
            name: name.clone(),
            handle,
        });
    }

    if !registry.entries.is_empty() {
        registry.selected = Some(0);
    }
    info!("maps scanned: {} entries", registry.entries.len());
}
