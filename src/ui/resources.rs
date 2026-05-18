use bevy::prelude::*;

#[derive(Resource)]
pub struct UiState {
    pub show_ground_truth: bool,
    pub show_local_maps: bool,
    pub show_global_map: bool,
    pub show_rays: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_ground_truth: true,
            show_local_maps: false,
            show_global_map: true,
            show_rays: false,
        }
    }
}
