mod resources;

use bevy::prelude::*;
use bevy::render::{ExtractSchedule, RenderApp};

use resources::{ensure_ground_truth_gpu, GroundTruthGpuSlot};

/// Side-channel plugin that mirrors the ground-truth map onto the GPU as a
/// packed `u32` bitset. Foundation for Tier 3 #8 — once a compute lidar
/// pipeline lands it reads from this buffer instead of the CPU
/// `GroundTruthMap` resource.
pub struct GpuLidarPlugin;

impl Plugin for GpuLidarPlugin {
    fn build(&self, app: &mut App) {
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .init_resource::<GroundTruthGpuSlot>()
            .add_systems(ExtractSchedule, ensure_ground_truth_gpu);
    }
}
