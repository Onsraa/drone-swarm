mod asset;
mod bootstrap;
mod events;
mod registry;
mod swap;
mod vox_loader;

use bevy::prelude::*;

pub use asset::MapAsset;
pub use events::MapSwapRequested;
pub use registry::AvailableMaps;

use asset::MapAssetLoader;
use bootstrap::bootstrap_default_maps;
use registry::scan_maps_dir;
use swap::{apply_pending_swap, bootstrap_initial_map, enqueue_map_swap, PendingMapSwap};
use vox_loader::VoxAssetLoader;

/// Owns the `.dvm` asset type + loader, the registry of maps available
/// on disk, the `MapSwapRequested` message channel that UI / debug
/// systems publish on, and the swap pipeline that tears the sim down
/// and rebuilds it on each request.
pub struct MapsPlugin;

impl Plugin for MapsPlugin {
    fn build(&self, app: &mut App) {
        app.init_asset::<MapAsset>()
            .init_asset_loader::<MapAssetLoader>()
            .init_asset_loader::<VoxAssetLoader>()
            .init_resource::<AvailableMaps>()
            .init_resource::<PendingMapSwap>()
            .add_message::<MapSwapRequested>()
            .add_systems(Startup, (bootstrap_default_maps, scan_maps_dir).chain())
            .add_systems(
                Update,
                (
                    bootstrap_initial_map,
                    enqueue_map_swap,
                    apply_pending_swap,
                )
                    .chain(),
            );
    }
}
