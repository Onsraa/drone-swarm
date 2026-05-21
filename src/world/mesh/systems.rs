use bevy::prelude::*;
use bevy::scene::SceneRoot;

use std::collections::HashMap;

use super::bvh::{build_world_bvh_with_materials, recommended_transform, WorldBvh};
use super::components::GroundTruthMesh;
use super::constants::{AUTO_FIT_COVERAGE_RATIO, AUTO_FIT_TRIM_HIGH, AUTO_FIT_TRIM_LOW};
use super::resources::MeshGroundTruthConfig;
use super::triangles::{extract_triangles_from_mesh, percentile_trimmed_aabb};
use crate::world::WorldConfig;

fn material_albedo(
    handle: &Handle<StandardMaterial>,
    materials: &Assets<StandardMaterial>,
    images: Option<&Assets<Image>>,
) -> Vec4 {
    let Some(mat) = materials.get(handle) else {
        return Vec4::ONE;
    };
    let tint = {
        let lin = mat.base_color.to_linear();
        Vec4::new(lin.red, lin.green, lin.blue, lin.alpha)
    };
    if let (Some(tex_handle), Some(images)) = (&mat.base_color_texture, images) {
        if let Some(image) = images.get(tex_handle) {
            if let Some(mean) = image_mean_linear(image) {
                return mean * tint;
            }
        }
    }
    tint
}

fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// Downsampled mean of an RGBA image in linear space. Returns `None`
/// for unsupported pixel formats; the caller falls back to the
/// material's flat `base_color` tint. Decimates the pixel walk to
/// `~10k` samples so the per-scene material bake stays sub-millisecond
/// even for 4K textures.
fn image_mean_linear(image: &Image) -> Option<Vec4> {
    use bevy::render::render_resource::TextureFormat;
    let data = image.data.as_deref()?;
    let format = image.texture_descriptor.format;
    let (bytes_per_pixel, is_srgb) = match format {
        TextureFormat::Rgba8UnormSrgb => (4usize, true),
        TextureFormat::Rgba8Unorm => (4usize, false),
        _ => return None,
    };
    let pixel_count = data.len() / bytes_per_pixel;
    if pixel_count == 0 {
        return None;
    }
    let stride = (pixel_count / 10_000).max(1);
    let mut sum = Vec4::ZERO;
    let mut count = 0u32;
    let mut i = 0;
    while i < pixel_count {
        let base = i * bytes_per_pixel;
        let r = data[base] as f32 / 255.0;
        let g = data[base + 1] as f32 / 255.0;
        let b = data[base + 2] as f32 / 255.0;
        let a = data[base + 3] as f32 / 255.0;
        let (lr, lg, lb) = if is_srgb {
            (srgb_to_linear(r), srgb_to_linear(g), srgb_to_linear(b))
        } else {
            (r, g, b)
        };
        sum += Vec4::new(lr, lg, lb, a);
        count += 1;
        i += stride;
    }
    Some(sum / count as f32)
}

const APPLY_EPS: f32 = 1.0e-4;

fn transform_changed(current: (Vec3, f32), applied: (Vec3, f32)) -> bool {
    (current.0 - applied.0).length_squared() > APPLY_EPS
        || (current.1 - applied.1).abs() > APPLY_EPS
}

/// One-shot spawn of the ground-truth mesh entity. The scene asset is
/// loaded by path from `MeshGroundTruthConfig`; if the file is absent
/// the asset server logs a warning and the SceneRoot stays empty until
/// the file appears (asset hot reload). Translation + scale come from
/// the config so the scene lands centred on the voxel world.
pub fn spawn_mesh_ground_truth(
    mut commands: Commands,
    asset_server: Option<Res<AssetServer>>,
    mut config: ResMut<MeshGroundTruthConfig>,
    existing: Query<(), With<GroundTruthMesh>>,
) {
    if config.spawned || !existing.is_empty() {
        return;
    }
    let Some(asset_server) = asset_server else {
        return;
    };
    let handle: Handle<Scene> = asset_server.load(config.scene_asset_path.clone());
    let transform = Transform::from_translation(config.translation)
        .with_scale(Vec3::splat(config.scale));
    commands.spawn((
        GroundTruthMesh,
        SceneRoot(handle),
        transform,
        Visibility::default(),
    ));
    config.spawned = true;
    config.applied_transform = Some((config.translation, config.scale));
    info!(
        "spawned ground-truth mesh from {} at {:?} scale {}",
        config.scene_asset_path, config.translation, config.scale
    );
}

/// Tear down the current SceneRoot + clear `WorldBvh` so the next
/// frame respawns + rebuilds with the new transform. Fires when the
/// UI Apply button sets `apply_requested = true` AND the requested
/// transform differs from what was last applied.
pub fn invalidate_mesh_on_apply(
    mut commands: Commands,
    mut config: ResMut<MeshGroundTruthConfig>,
    existing: Query<Entity, With<GroundTruthMesh>>,
) {
    if !config.apply_requested {
        return;
    }
    let current = (config.translation, config.scale);
    let needs_rebuild = match config.applied_transform {
        Some(applied) => transform_changed(current, applied),
        None => true,
    };
    config.apply_requested = false;
    if !needs_rebuild {
        return;
    }
    for e in &existing {
        commands.entity(e).despawn();
    }
    commands.remove_resource::<WorldBvh>();
    config.spawned = false;
    config.applied_transform = None;
    info!(
        "mesh ground truth invalidated — respawn at {:?} scale {}",
        config.translation, config.scale
    );
}

pub fn apply_mesh_visibility(
    config: Res<MeshGroundTruthConfig>,
    mut q: Query<&mut Visibility, With<GroundTruthMesh>>,
) {
    let target = if config.visible {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    for mut v in q.iter_mut() {
        if *v != target {
            *v = target;
        }
    }
}

/// Walks the GroundTruthMesh entity's children, harvests `Mesh3d` +
/// `GlobalTransform` from each descendant, extracts world-space
/// triangles, and builds a CWBVH once. Scene spawning is async — first
/// few frames after spawn return zero triangles; once the SceneSpawner
/// has populated children, the build fires and `WorldBvh` is inserted.
/// Subsequent runs early-out via the `bvh_present` guard.
#[allow(clippy::too_many_arguments)]
pub fn build_bvh_when_scene_ready(
    mut commands: Commands,
    meshes: Option<Res<Assets<Mesh>>>,
    materials: Option<Res<Assets<StandardMaterial>>>,
    images: Option<Res<Assets<Image>>>,
    bvh_present: Option<Res<WorldBvh>>,
    mut config: ResMut<MeshGroundTruthConfig>,
    world_config: Option<Res<WorldConfig>>,
    root_query: Query<Entity, With<GroundTruthMesh>>,
    children_q: Query<&Children>,
    mesh_q: Query<(
        &Mesh3d,
        &GlobalTransform,
        Option<&MeshMaterial3d<StandardMaterial>>,
    )>,
) {
    if bvh_present.is_some() {
        return;
    }
    let Some(meshes) = meshes else {
        return;
    };
    let Ok(root) = root_query.single() else {
        return;
    };
    let materials_opt = materials.as_deref();
    let images_opt = images.as_deref();

    let mut triangles: Vec<obvhs::triangle::Triangle> = Vec::new();
    let mut tri_materials: Vec<u32> = Vec::new();
    let mut palette: Vec<Vec4> = Vec::new();
    let mut palette_lookup: HashMap<AssetId<StandardMaterial>, u32> = HashMap::new();

    let mut stack = vec![root];
    while let Some(entity) = stack.pop() {
        if let Ok((mesh3d, gx, mat)) = mesh_q.get(entity) {
            if let Some(mesh) = meshes.get(&mesh3d.0) {
                let new_tris = extract_triangles_from_mesh(mesh, gx.to_matrix());
                if !new_tris.is_empty() {
                    let mat_id = match (mat, materials_opt) {
                        (Some(mat3d), Some(materials)) => {
                            let id = mat3d.0.id();
                            *palette_lookup.entry(id).or_insert_with(|| {
                                palette.push(material_albedo(&mat3d.0, materials, images_opt));
                                (palette.len() - 1) as u32
                            })
                        }
                        _ => {
                            // Mesh without a StandardMaterial — slot in
                            // a default white albedo at palette[0].
                            if palette.is_empty() {
                                palette.push(Vec4::ONE);
                            }
                            0u32
                        }
                    };
                    let n = new_tris.len();
                    triangles.extend(new_tris);
                    tri_materials.extend(std::iter::repeat(mat_id).take(n));
                }
            }
        }
        if let Ok(children) = children_q.get(entity) {
            for c in children.iter() {
                stack.push(c);
            }
        }
    }

    if triangles.is_empty() {
        return;
    }

    // Auto-fit reads a percentile-trimmed AABB to ignore outlier
    // geometry (sky-domes, antennas, stray hierarchy nodes). Compute
    // it from the triangle list before consuming it into the BVH.
    let (trim_min, trim_max) =
        percentile_trimmed_aabb(&triangles, AUTO_FIT_TRIM_LOW, AUTO_FIT_TRIM_HIGH);

    let count = triangles.len();
    let mat_count = palette.len();
    let bvh = build_world_bvh_with_materials(triangles, tri_materials, palette);
    info!(
        "built ground-truth BVH from {} triangles, {} materials, full aabb min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1}), trimmed min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1})",
        count,
        mat_count,
        bvh.cwbvh.total_aabb.min.x,
        bvh.cwbvh.total_aabb.min.y,
        bvh.cwbvh.total_aabb.min.z,
        bvh.cwbvh.total_aabb.max.x,
        bvh.cwbvh.total_aabb.max.y,
        bvh.cwbvh.total_aabb.max.z,
        trim_min.x, trim_min.y, trim_min.z,
        trim_max.x, trim_max.y, trim_max.z,
    );

    // One-shot auto-fit: requires WorldConfig + a matching
    // applied_transform (so we only fit the first build, not the
    // post-fit rebuild). Uses the trimmed AABB so outlier geometry
    // doesn't bloat the scale calculation.
    let applied_matches = config
        .applied_transform
        .is_some_and(|a| a == (config.translation, config.scale));
    if config.auto_fit_on_first_build && applied_matches {
        if let Some(world) = world_config.as_ref() {
            let (new_t, new_s) = recommended_transform(
                trim_min,
                trim_max,
                world.world_size(),
                config.translation,
                config.scale,
                AUTO_FIT_COVERAGE_RATIO,
            );
            info!(
                "auto-fit suggested: translation={:?} scale={:.3} (was {:?} scale={})",
                new_t, new_s, config.translation, config.scale
            );
            config.translation = new_t;
            config.scale = new_s;
            config.apply_requested = true;
            config.auto_fit_on_first_build = false;
        }
    }

    commands.insert_resource(bvh);
}
