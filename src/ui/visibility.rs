use bevy::prelude::*;

use crate::render::{GpuGlobalMapVoxel, GpuLocalMapVoxel, GroundTruthVoxel, LidarPointVoxel};

use super::resources::UiState;

/// Run every frame instead of only on `state.is_changed()`. Layer
/// entities are spawned lazily (and respawn after a map swap), so a
/// one-shot "apply when state changed" misses them and leaves them
/// with default `Visibility::Visible`.
pub fn apply_visibility(
    state: Res<UiState>,
    mut ground_truth_q: Query<
        &mut Visibility,
        (
            With<GroundTruthVoxel>,
            Without<GpuLocalMapVoxel>,
            Without<GpuGlobalMapVoxel>,
            Without<LidarPointVoxel>,
        ),
    >,
    mut local_map_q: Query<
        &mut Visibility,
        (
            With<GpuLocalMapVoxel>,
            Without<GroundTruthVoxel>,
            Without<GpuGlobalMapVoxel>,
            Without<LidarPointVoxel>,
        ),
    >,
    mut global_map_q: Query<
        &mut Visibility,
        (
            With<GpuGlobalMapVoxel>,
            Without<GroundTruthVoxel>,
            Without<GpuLocalMapVoxel>,
            Without<LidarPointVoxel>,
        ),
    >,
    mut lidar_points_q: Query<
        &mut Visibility,
        (
            With<LidarPointVoxel>,
            Without<GroundTruthVoxel>,
            Without<GpuLocalMapVoxel>,
            Without<GpuGlobalMapVoxel>,
        ),
    >,
) {
    set_layer(&mut ground_truth_q, to_visibility(state.show_ground_truth));
    set_layer(&mut local_map_q, to_visibility(state.show_local_maps));
    set_layer(&mut global_map_q, to_visibility(state.show_global_map));
    set_layer(&mut lidar_points_q, to_visibility(state.show_lidar_points));
}

fn set_layer<F: bevy::ecs::query::QueryFilter>(
    q: &mut Query<&mut Visibility, F>,
    target: Visibility,
) {
    for mut v in q.iter_mut() {
        if *v != target {
            *v = target;
        }
    }
}

fn to_visibility(shown: bool) -> Visibility {
    if shown {
        Visibility::Visible
    } else {
        Visibility::Hidden
    }
}
