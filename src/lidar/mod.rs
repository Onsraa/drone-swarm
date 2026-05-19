mod components;
pub mod constants;
pub mod gpu;
mod resources;
mod sampling;

use bevy::prelude::*;

pub use gpu::GpuLidarPlugin;
pub use resources::{LidarFrameCounter, LidarSettings};

/// Owns the runtime-tunable `LidarSettings` + frame counter. Ray-set
/// content lives in `sampling::build_role_ray_buffer`, called by
/// `setup_gpu_lidar_assets` to populate the GPU buffer at startup.
pub struct LidarPlugin;

impl Plugin for LidarPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LidarSettings>()
            .init_resource::<LidarFrameCounter>()
            .add_systems(Update, tick_frame_counter);
    }
}

fn tick_frame_counter(mut counter: ResMut<LidarFrameCounter>) {
    counter.0 = counter.0.wrapping_add(1);
}
