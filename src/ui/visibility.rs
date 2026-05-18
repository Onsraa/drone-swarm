use bevy::prelude::*;

use crate::render::{GlobalMapVoxel, GroundTruthVoxel, LocalMapVoxel};

use super::resources::UiState;

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
    if !state.is_changed() {
        return;
    }
    let ground_truth_visibility = to_visibility(state.show_ground_truth);
    let local_map_visibility = to_visibility(state.show_local_maps);
    let global_map_visibility = to_visibility(state.show_global_map);

    for mut v in &mut ground_truth_q {
        *v = ground_truth_visibility;
    }
    for mut v in &mut local_map_q {
        *v = local_map_visibility;
    }
    for mut v in &mut global_map_q {
        *v = global_map_visibility;
    }
}

fn to_visibility(shown: bool) -> Visibility {
    if shown {
        Visibility::Visible
    } else {
        Visibility::Hidden
    }
}
