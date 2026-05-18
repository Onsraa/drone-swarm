mod constants;
mod gizmos;
mod panel;
mod resources;
mod visibility;

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

pub use resources::UiState;

use gizmos::draw_ray_gizmos;
use panel::draw_ui;
use visibility::apply_visibility;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<UiState>()
            .add_systems(EguiPrimaryContextPass, draw_ui)
            .add_systems(Update, (apply_visibility, draw_ray_gizmos));
    }
}
