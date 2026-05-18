use bevy::prelude::*;

use crate::render::{GlobalMapVoxel, GroundTruthVoxel, LocalMapVoxel};

use super::resources::UiState;

/// Run every frame instead of only on `state.is_changed()`. Layer entities
/// are spawned lazily by `sync_global_map` / `sync_local_maps` *after*
/// the user may have already set their preferred toggles, so a one-shot
/// "apply when state changed" missed the freshly-spawned entities and
/// left them with their default `Visibility::Visible`.
pub fn apply_visibility(
    state: Res<UiState>,
    mut ground_truth_q: Query<
        &mut Visibility,
        (
            With<GroundTruthVoxel>,
            Without<LocalMapVoxel>,
            Without<GlobalMapVoxel>,
        ),
    >,
    mut local_map_q: Query<
        &mut Visibility,
        (
            With<LocalMapVoxel>,
            Without<GroundTruthVoxel>,
            Without<GlobalMapVoxel>,
        ),
    >,
    mut global_map_q: Query<
        &mut Visibility,
        (
            With<GlobalMapVoxel>,
            Without<GroundTruthVoxel>,
            Without<LocalMapVoxel>,
        ),
    >,
) {
    let ground_truth_visibility = to_visibility(state.show_ground_truth);
    let local_map_visibility = to_visibility(state.show_local_maps);
    let global_map_visibility = to_visibility(state.show_global_map);

    set_layer(&mut ground_truth_q, ground_truth_visibility);
    set_layer(&mut local_map_q, local_map_visibility);
    set_layer(&mut global_map_q, global_map_visibility);
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
