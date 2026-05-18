use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::drone::Drone;
use crate::map::LocalMap;

use super::constants::SIDE_PANEL_DEFAULT_WIDTH;
use super::resources::UiState;

pub fn draw_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    drones_q: Query<&LocalMap, With<Drone>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("side_panel")
        .default_width(SIDE_PANEL_DEFAULT_WIDTH)
        .show(ctx, |ui| {
            ui.heading("Drones — Phase 3");
            ui.separator();
            ui.checkbox(&mut state.show_ground_truth, "Show ground truth");
            ui.checkbox(&mut state.show_local_maps, "Show drone local maps");
            ui.checkbox(&mut state.show_rays, "Show last-scan rays");
            ui.separator();
            ui.label("Drone scans:");
            for (i, local_map) in drones_q.iter().enumerate() {
                let (free, occupied) = local_map.0.count_known();
                let total = (local_map.0.dims.x * local_map.0.dims.y * local_map.0.dims.z) as usize;
                ui.label(format!(
                    "  drone {} — free {} | occ {} | / {}",
                    i, free, occupied, total
                ));
            }
            ui.separator();
            ui.label("Drag = orbit. Scroll = zoom.");
        });
    Ok(())
}
