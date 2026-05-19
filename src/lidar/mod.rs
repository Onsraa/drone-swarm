mod components;
mod constants;
mod gpu;
mod sampling;

use bevy::prelude::*;

pub use components::LastScanRays;
pub use gpu::GpuLidarPlugin;

use sampling::LidarRayDirs;

/// Holds the cached fibonacci-cone ray directions used by the GPU lidar
/// shader. The actual traversal lives in `gpu::*`; this plugin only owns
/// the inputs the GPU side reads.
pub struct LidarPlugin;

impl Plugin for LidarPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(LidarRayDirs::default_for_scan());
    }
}
