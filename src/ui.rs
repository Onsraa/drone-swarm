use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};

use crate::drone::Drone;
use crate::lidar::LastScanRays;
use crate::map::LocalMap;
use crate::voxel_render::{GroundTruthVoxel, LocalMapVoxel};

#[derive(Resource)]
pub struct UiState {
    pub show_ground_truth: bool,
    pub show_local_maps: bool,
    pub show_rays: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_ground_truth: true,
            show_local_maps: true,
            show_rays: false,
        }
    }
}

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<UiState>()
            .add_systems(EguiPrimaryContextPass, draw_ui)
            .add_systems(Update, (apply_visibility, draw_ray_gizmos));
    }
}

fn draw_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    drones_q: Query<&LocalMap, With<Drone>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("side_panel")
        .default_width(280.0)
        .show(ctx, |ui| {
            ui.heading("Drones — Phase 3");
            ui.separator();
            ui.checkbox(&mut state.show_ground_truth, "Show ground truth");
            ui.checkbox(&mut state.show_local_maps, "Show drone local maps");
            ui.checkbox(&mut state.show_rays, "Show last-scan rays");
            ui.separator();
            ui.label("Drone scans:");
            for (i, lm) in drones_q.iter().enumerate() {
                let (free, occ) = lm.0.count_known();
                let total = (lm.0.dims.x * lm.0.dims.y * lm.0.dims.z) as usize;
                ui.label(format!(
                    "  drone {} — free {} | occ {} | / {}",
                    i, free, occ, total
                ));
            }
            ui.separator();
            ui.label("Drag = orbit. Scroll = zoom.");
        });
    Ok(())
}

fn apply_visibility(
    state: Res<UiState>,
    mut gt_q: Query<&mut Visibility, (With<GroundTruthVoxel>, Without<LocalMapVoxel>)>,
    mut lm_q: Query<&mut Visibility, (With<LocalMapVoxel>, Without<GroundTruthVoxel>)>,
) {
    if !state.is_changed() {
        return;
    }
    let gt_v = if state.show_ground_truth {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    let lm_v = if state.show_local_maps {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    for mut v in &mut gt_q {
        *v = gt_v;
    }
    for mut v in &mut lm_q {
        *v = lm_v;
    }
}

fn draw_ray_gizmos(
    state: Res<UiState>,
    mut gizmos: Gizmos,
    rays_q: Query<&LastScanRays, With<Drone>>,
) {
    if !state.show_rays {
        return;
    }
    let color = Color::srgba(0.0, 0.8, 1.0, 0.5);
    for rays in &rays_q {
        for (start, end) in &rays.0 {
            gizmos.line(*start, *end, color);
        }
    }
}
