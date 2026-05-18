use bevy::prelude::*;

use crate::render::{GroundTruthVoxel, LocalMapVoxel};

use super::resources::UiState;

pub fn apply_visibility(
    state: Res<UiState>,
    mut ground_truth_q: Query<&mut Visibility, (With<GroundTruthVoxel>, Without<LocalMapVoxel>)>,
    mut local_map_q: Query<&mut Visibility, (With<LocalMapVoxel>, Without<GroundTruthVoxel>)>,
) {
    if !state.is_changed() {
        return;
    }
    let ground_truth_visibility = if state.show_ground_truth {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    let local_map_visibility = if state.show_local_maps {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    for mut v in &mut ground_truth_q {
        *v = ground_truth_visibility;
    }
    for mut v in &mut local_map_q {
        *v = local_map_visibility;
    }
}
