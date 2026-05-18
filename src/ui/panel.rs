use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::drone::{Drone, DroneSpawnConfig, MAX_DRONE_COUNT, MIN_DRONE_COUNT};
use crate::map::{GlobalMap, LocalMap};

use super::constants::SIDE_PANEL_DEFAULT_WIDTH;
use super::resources::UiState;

pub fn draw_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut spawn_config: ResMut<DroneSpawnConfig>,
    drones_q: Query<&LocalMap, With<Drone>>,
    global_map: Option<Res<GlobalMap>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("side_panel")
        .default_width(SIDE_PANEL_DEFAULT_WIDTH)
        .show(ctx, |ui| {
            ui.heading("Drones — Phase 5");
            ui.separator();

            ui.label("Layers");
            ui.checkbox(&mut state.show_ground_truth, "Show ground truth");
            ui.checkbox(&mut state.show_local_maps, "Show drone local maps");
            ui.checkbox(&mut state.show_global_map, "Show central map");
            ui.checkbox(&mut state.show_rays, "Show last-scan rays");
            ui.separator();

            ui.label("Swarm size");
            ui.add(
                egui::Slider::new(
                    &mut spawn_config.target_count,
                    MIN_DRONE_COUNT..=MAX_DRONE_COUNT,
                )
                .text("drones"),
            );
            ui.separator();

            ui.label(format!("Drone scans ({} live):", drones_q.iter().count()));
            for (i, local_map) in drones_q.iter().enumerate().take(10) {
                let (free, occupied) = local_map.0.count_known();
                let total = (local_map.0.dims.x * local_map.0.dims.y * local_map.0.dims.z) as usize;
                ui.label(format!(
                    "  drone {} — free {} | occ {} | / {}",
                    i, free, occupied, total
                ));
            }
            if drones_q.iter().count() > 10 {
                ui.label(format!(
                    "  ... and {} more",
                    drones_q.iter().count() - 10
                ));
            }
            ui.separator();

            ui.label("Central map:");
            if let Some(global) = global_map.as_ref() {
                let (free, occupied) = global.0.count_known();
                let total = (global.0.dims.x * global.0.dims.y * global.0.dims.z) as usize;
                let coverage_pct = (free + occupied) as f32 / total as f32 * 100.0;
                ui.label(format!(
                    "  free {} | occ {} | / {} ({:.1}% known)",
                    free, occupied, total, coverage_pct
                ));
            } else {
                ui.label("  (not yet initialized)");
            }
            ui.separator();

            ui.label("Drag = orbit. Scroll = zoom.");
        });
    Ok(())
}
