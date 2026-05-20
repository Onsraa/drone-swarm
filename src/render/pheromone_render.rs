//! Pheromone-field heatmap render. Walks the `PheromoneField` scalar
//! grid each frame, drawing a small wireframe cube at every cell whose
//! intensity exceeds `VIZ_THRESHOLD`. Color interpolates between cool
//! cyan (low intensity) and warm yellow (high), alpha proportional to
//! intensity.
//!
//! Implementation uses gizmos rather than a custom instanced pipeline
//! because pheromone activity is sparse (typically <500 cells over
//! VIZ_THRESHOLD at swarm scale) and we want a one-system, easy-to-
//! tweak overlay.

use bevy::prelude::*;

use crate::pheromone::PheromoneField;
use crate::ui::UiState;

const VIZ_THRESHOLD: f32 = 5.0;
const VIZ_INTENSITY_FULL: f32 = 200.0;
const VIZ_CUBE_SIZE_RATIO: f32 = 0.4;

pub struct PheromoneRenderPlugin;

impl Plugin for PheromoneRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, draw_pheromone_field);
    }
}

fn draw_pheromone_field(
    ui_state: Res<UiState>,
    field: Res<PheromoneField>,
    mut gizmos: Gizmos,
) {
    if !ui_state.show_pheromone_field {
        return;
    }
    if field.cells.is_empty() {
        return;
    }
    let cell_size = field.cell_size();
    if cell_size <= 0.0 {
        return;
    }
    let viz_size = cell_size * VIZ_CUBE_SIZE_RATIO;
    let dx = field.dims.x;
    let dy = field.dims.y;
    let plane = dx * dy;

    for (i, &v) in field.cells.iter().enumerate() {
        if v < VIZ_THRESHOLD {
            continue;
        }
        let idx = i as u32;
        let z = idx / plane;
        let rem = idx % plane;
        let y = rem / dx;
        let x = rem % dx;
        let center = Vec3::new(
            (x as f32 + 0.5) * cell_size,
            (y as f32 + 0.5) * cell_size,
            (z as f32 + 0.5) * cell_size,
        );
        let t = (v / VIZ_INTENSITY_FULL).clamp(0.0, 1.0);
        // cyan (low) -> yellow (high)
        let r = 0.25 + 0.75 * t;
        let g = 0.85;
        let b = 0.95 - 0.85 * t;
        let alpha = 0.18 + 0.55 * t;
        let color = Color::linear_rgba(r, g, b, alpha);
        let iso = Isometry3d::from_translation(center);
        let scale = Vec3::splat(viz_size);
        let transform = Transform::from_isometry(iso).with_scale(scale);
        gizmos.cube(transform, color);
    }
}
