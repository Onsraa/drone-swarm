mod components;
pub mod constants;
pub mod gpu;
mod resources;
mod sampling;

use bevy::prelude::*;

pub use gpu::GpuLidarPlugin;
pub use resources::{LidarFrameCounter, LidarSettings};

use sampling::LidarRayDirs;

/// Holds the cached fibonacci-cone ray directions used by the GPU lidar
/// shader plus the runtime-tunable `LidarSettings`. Per-frame upload
/// systems live in `gpu::*`.
pub struct LidarPlugin;

impl Plugin for LidarPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(LidarRayDirs::default_for_scan())
            .init_resource::<LidarSettings>()
            .init_resource::<LidarFrameCounter>()
            .add_systems(Update, tick_frame_counter);
    }
}

fn tick_frame_counter(mut counter: ResMut<LidarFrameCounter>) {
    counter.0 = counter.0.wrapping_add(1);
}
