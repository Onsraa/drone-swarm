//! Pheromone-field heatmap render. CPU rebuilds an
//! `InstancedVoxelLayer` from the scalar field every
//! `PHEROMONE_REBUILD_INTERVAL_FRAMES` frames; the existing billboard
//! pipeline (`instanced_voxel.wgsl`) draws each cell as a screen-space
//! dot. Cheaper than the previous gizmo-cube path once activity is
//! dense (>500 cells).

use bevy::camera::visibility::NoFrustumCulling;
use bevy::prelude::*;

use crate::pheromone::PheromoneField;

use super::components::PheromoneVoxel;
use super::instancing::{InstanceData, InstancedVoxelLayer};
use super::resources::CubeMesh;

const VIZ_THRESHOLD: f32 = 5.0;
const VIZ_INTENSITY_FULL: f32 = 200.0;
const VIZ_PIXEL_RADIUS: f32 = 3.5;
const PHEROMONE_REBUILD_INTERVAL_FRAMES: u32 = 12;

pub struct PheromoneRenderPlugin;

impl Plugin for PheromoneRenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (spawn_pheromone_layer, rebuild_pheromone_instances).chain(),
        );
    }
}

fn spawn_pheromone_layer(
    mut commands: Commands,
    quad: Option<Res<CubeMesh>>,
    existing: Query<(), With<PheromoneVoxel>>,
) {
    if !existing.is_empty() {
        return;
    }
    let Some(quad) = quad else {
        return;
    };
    commands.spawn((
        PheromoneVoxel,
        Mesh3d(quad.0.clone()),
        InstancedVoxelLayer {
            data: Vec::new(),
            generation: 1,
        },
        NoFrustumCulling,
        Transform::IDENTITY,
        Visibility::default(),
    ));
}

fn rebuild_pheromone_instances(
    field: Res<PheromoneField>,
    mut layer_q: Query<&mut InstancedVoxelLayer, With<PheromoneVoxel>>,
    mut frame: Local<u32>,
) {
    *frame = frame.wrapping_add(1);
    if *frame % PHEROMONE_REBUILD_INTERVAL_FRAMES != 0 {
        return;
    }
    let Ok(mut layer) = layer_q.single_mut() else {
        return;
    };
    let cell_size = field.cell_size();
    if cell_size <= 0.0 || field.cells.is_empty() {
        if !layer.data.is_empty() {
            layer.data.clear();
            layer.generation = layer.generation.wrapping_add(1);
        }
        return;
    }
    let dx = field.dims.x;
    let dy = field.dims.y;
    let plane = dx * dy;
    layer.data.clear();

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
        // cyan (low) -> yellow (high). Alpha ramps with intensity so
        // weak cells stay subtle.
        let r = 0.25 + 0.75 * t;
        let g = 0.85;
        let b = 0.95 - 0.85 * t;
        let alpha = 0.20 + 0.55 * t;
        layer.data.push(InstanceData {
            pos_scale: [center.x, center.y, center.z, VIZ_PIXEL_RADIUS],
            color: [r, g, b, alpha],
        });
    }
    layer.generation = layer.generation.wrapping_add(1);
}
