use bevy::prelude::*;

#[derive(Resource)]
pub struct UiState {
    pub show_ground_truth: bool,
    pub show_local_maps: bool,
    pub show_global_map: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_ground_truth: false,
            show_local_maps: true,
            show_global_map: false,
        }
    }
}
