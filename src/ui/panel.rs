use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::camera::CameraMode;
use crate::comms::{CommsSettings, CommsState, MAX_COMMS_RANGE_M, MIN_COMMS_RANGE_M};
use crate::drone::{Drone, DroneColor, DroneId, DroneSpawnConfig, MAX_DRONE_COUNT, MIN_DRONE_COUNT};
use crate::exploration::Role;
use crate::groups::DroneGroupPresets;
use crate::lidar::{gpu::GpuGlobalStats, LidarSettings};
use crate::maps::{AvailableMaps, MapSwapRequested};
use crate::world::WorldConfig;

use super::constants::SIDE_PANEL_DEFAULT_WIDTH;
use super::resources::UiState;

#[allow(clippy::too_many_arguments)]
pub fn draw_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut spawn_config: ResMut<DroneSpawnConfig>,
    mut available: ResMut<AvailableMaps>,
    mut swap_writer: MessageWriter<MapSwapRequested>,
    mut lidar_settings: ResMut<LidarSettings>,
    mut comms_settings: ResMut<CommsSettings>,
    mut presets: ResMut<DroneGroupPresets>,
    mut preset_name_buf: Local<String>,
    comms_state: Res<CommsState>,
    camera_mode: Res<CameraMode>,
    drones_q: Query<(&DroneId, &DroneColor), With<Drone>>,
    drones_role_q: Query<&Role, With<Drone>>,
    gpu_stats: Res<GpuGlobalStats>,
    world: Res<WorldConfig>,
    diagnostics: Res<DiagnosticsStore>,
) -> Result {
    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("side_panel")
        .default_width(SIDE_PANEL_DEFAULT_WIDTH)
        .show(ctx, |ui| {
            ui.heading("Drones — Phase 6");
            ui.label(format!("FPS: {:.0}", fps));
            ui.separator();

            draw_map_picker(ui, &mut available, &mut swap_writer);
            ui.separator();

            ui.label("Layers");
            ui.checkbox(&mut state.show_ground_truth, "Show ground truth (debug)");
            ui.checkbox(&mut state.show_local_maps, "Show drone local maps");
            ui.checkbox(&mut state.show_global_map, "Show central map");
            ui.checkbox(&mut state.show_lidar_points, "Show lidar spray (points)");
            ui.separator();

            ui.label("Swarm size");
            ui.add(
                egui::Slider::new(
                    &mut spawn_config.target_count,
                    MIN_DRONE_COUNT..=MAX_DRONE_COUNT,
                )
                .text("drones"),
            );
            let drone_count = drones_q.iter().count();
            ui.label(format!("Drones live: {}", drone_count));
            ui.separator();

            draw_lidar_sliders(ui, &mut lidar_settings);
            ui.separator();

            draw_comms_controls(ui, &mut comms_settings, &comms_state);
            ui.separator();

            draw_drone_visibility(ui, &mut state, &drones_q);
            draw_group_presets(ui, &mut state, &mut presets, &mut preset_name_buf);
            ui.separator();

            draw_roles(ui, &drones_role_q);
            ui.separator();

            ui.label("Central map (GPU readback):");
            let total = (world.size.x * world.size.y * world.size.z) as usize;
            let free = gpu_stats.free;
            let occupied = gpu_stats.occupied;
            let coverage_pct = if total > 0 {
                (free + occupied) as f32 / total as f32 * 100.0
            } else {
                0.0
            };
            ui.label(format!(
                "  free {} | occ {} | / {} ({:.1}% known)",
                free, occupied, total, coverage_pct
            ));
            ui.separator();

            match *camera_mode {
                CameraMode::Orbit => ui.label("Orbit cam: LMB drag, scroll zoom. F = free-fly."),
                CameraMode::FreeFly => {
                    ui.label("Free-fly: WASD move, Space/Shift up/down, RMB drag look, Ctrl boost. F = orbit.")
                }
            };
        });
    Ok(())
}

fn draw_map_picker(
    ui: &mut egui::Ui,
    available: &mut AvailableMaps,
    swap_writer: &mut MessageWriter<MapSwapRequested>,
) {
    ui.label("Map");
    let selected_label = available
        .selected
        .and_then(|i| available.entries.get(i))
        .map(|e| e.name.clone())
        .unwrap_or_else(|| "<none>".to_string());

    let mut chosen: Option<usize> = None;
    egui::ComboBox::from_id_salt("map_picker")
        .selected_text(selected_label)
        .show_ui(ui, |ui| {
            for (i, entry) in available.entries.iter().enumerate() {
                let is_selected = available.selected == Some(i);
                if ui.selectable_label(is_selected, &entry.name).clicked() {
                    chosen = Some(i);
                }
            }
        });

    if let Some(i) = chosen {
        if available.selected != Some(i) {
            available.selected = Some(i);
            let entry = &available.entries[i];
            swap_writer.write(MapSwapRequested {
                handle: entry.handle.clone(),
                name: entry.name.clone(),
            });
        }
    }
}

fn draw_group_presets(
    ui: &mut egui::Ui,
    state: &mut UiState,
    presets: &mut DroneGroupPresets,
    name_buf: &mut String,
) {
    ui.label("Presets");
    ui.horizontal(|ui| {
        ui.add(
            egui::TextEdit::singleline(name_buf)
                .hint_text("preset name")
                .desired_width(140.0),
        );
        let save_enabled = !name_buf.trim().is_empty();
        if ui
            .add_enabled(save_enabled, egui::Button::new("Save"))
            .clicked()
        {
            let name = name_buf.trim().to_string();
            presets.upsert(name, state.drone_mask);
            name_buf.clear();
        }
    });

    let mut to_apply: Option<[u32; 2]> = None;
    let mut to_delete: Option<usize> = None;
    for (i, entry) in presets.entries.iter().enumerate() {
        ui.horizontal(|ui| {
            if ui
                .button(&entry.name)
                .on_hover_text("click to apply this mask")
                .clicked()
            {
                to_apply = Some(entry.mask);
            }
            if ui.small_button("x").on_hover_text("delete").clicked() {
                to_delete = Some(i);
            }
        });
    }
    if let Some(mask) = to_apply {
        state.drone_mask = mask;
    }
    if let Some(idx) = to_delete {
        presets.remove(idx);
    }
}

fn draw_comms_controls(
    ui: &mut egui::Ui,
    settings: &mut CommsSettings,
    state: &CommsState,
) {
    ui.label("Comms");
    ui.checkbox(&mut settings.enabled, "Gate central map by radio range");
    ui.add_enabled(
        settings.enabled,
        egui::Slider::new(&mut settings.range_m, MIN_COMMS_RANGE_M..=MAX_COMMS_RANGE_M)
            .text("range (m)"),
    );
    ui.add_enabled(
        settings.enabled,
        egui::Checkbox::new(&mut settings.show_links, "Draw link gizmos"),
    );
    if settings.enabled {
        ui.label(format!(
            "Connected: {} / {}",
            state.connected_count, state.total_count
        ));
    }
}

fn draw_lidar_sliders(ui: &mut egui::Ui, settings: &mut LidarSettings) {
    ui.label("Lidar");
    let rays = LidarSettings::rays_range();
    let cone = LidarSettings::cone_range();
    let steps = LidarSettings::steps_range();
    let interval = LidarSettings::interval_range();

    ui.add(
        egui::Slider::new(&mut settings.rays_per_scan, rays)
            .text("rays / scan"),
    );
    ui.add(
        egui::Slider::new(&mut settings.cone_half_angle_deg, cone)
            .text("cone half-angle (deg)"),
    );
    ui.add(
        egui::Slider::new(&mut settings.max_steps_per_ray, steps)
            .text("max range (cells)"),
    );
    ui.add(
        egui::Slider::new(&mut settings.scan_interval_frames, interval)
            .text("scan every N frames"),
    );
    ui.checkbox(
        &mut settings.sticky_spray,
        "Sticky spray (accumulate hits)",
    );
}

fn draw_drone_visibility(
    ui: &mut egui::Ui,
    state: &mut UiState,
    drones_q: &Query<(&DroneId, &DroneColor), With<Drone>>,
) {
    ui.label("Drones (visibility)");
    ui.horizontal(|ui| {
        if ui.button("All").clicked() {
            state.drone_mask_all();
        }
        if ui.button("None").clicked() {
            state.drone_mask_none();
        }
        if ui.button("Invert").clicked() {
            state.drone_mask_invert();
        }
    });

    let mut ids: Vec<(u32, Color)> = drones_q.iter().map(|(id, c)| (id.0, c.0)).collect();
    ids.sort_by_key(|(id, _)| *id);

    egui::ScrollArea::vertical()
        .max_height(180.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            for (id, color) in ids {
                ui.horizontal(|ui| {
                    let linear = color.to_linear();
                    let swatch = egui::Color32::from_rgb(
                        (linear.red * 255.0) as u8,
                        (linear.green * 255.0) as u8,
                        (linear.blue * 255.0) as u8,
                    );
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
                    ui.painter().rect_filled(rect, 2.0, swatch);

                    let mut visible = state.is_drone_visible(id);
                    if ui.checkbox(&mut visible, format!("#{}", id)).changed() {
                        state.set_drone_visible(id, visible);
                    }
                });
            }
        });
}

fn draw_roles(
    ui: &mut egui::Ui,
    drones_q: &Query<&Role, With<Drone>>,
) {
    let mut scouts = 0u32;
    let mut mappers = 0u32;
    let mut anchors = 0u32;
    for role in drones_q.iter() {
        match role {
            Role::Scout => scouts += 1,
            Role::Mapper => mappers += 1,
            Role::Anchor => anchors += 1,
        }
    }
    let total = scouts + mappers + anchors;
    ui.label(format!("Roles ({} total)", total));
    ui.label(format!("  Scouts:  {}", scouts));
    ui.label(format!("  Mappers: {}", mappers));
    ui.label(format!("  Anchors: {}", anchors));
}
