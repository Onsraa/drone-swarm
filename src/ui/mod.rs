mod constants;
mod panel;
mod resources;
mod visibility;

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

pub use resources::{UiPointerCapture, UiState};

use panel::draw_ui;
use visibility::apply_visibility;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<UiState>()
            .init_resource::<UiPointerCapture>()
            .add_systems(EguiPrimaryContextPass, draw_ui)
            .add_systems(Update, apply_visibility);
    }
}
