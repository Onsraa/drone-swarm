use bevy::prelude::*;
use bevy::scene::SceneRoot;

use std::collections::HashMap;

use super::bvh::{build_world_bvh_with_materials, recommended_transform, WorldBvh};
use super::components::GroundTruthMesh;
use super::constants::{
    ATLAS_TILE_PX, AUTO_FIT_COVERAGE_RATIO, AUTO_FIT_TRIM_HIGH, AUTO_FIT_TRIM_LOW,
};
use super::resources::MeshGroundTruthConfig;
use super::triangles::{extract_triangles_from_mesh, percentile_trimmed_aabb};
use crate::world::WorldConfig;

fn material_albedo(
    handle: &Handle<StandardMaterial>,
    materials: &Assets<StandardMaterial>,
    _images: Option<&Assets<Image>>,
) -> Vec4 {
    let Some(mat) = materials.get(handle) else {
        return Vec4::ONE;
    };
    // Return the raw linear `base_color` only. The atlas baker
    // multiplies tile pixels by this once at bake time; sampling the
    // texture mean here too would double-multiply when the atlas
    // resolves to a real pixel sample.
    let lin = mat.base_color.to_linear();
    Vec4::new(lin.red, lin.green, lin.blue, lin.alpha)
}

fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// Nearest-neighbour resample of an RGBA image into a `target × target`
/// tile of `u8` bytes. Returns `None` for unsupported pixel formats;
/// the caller falls back to a solid-tint fill. Used by the atlas baker
/// to give each material a uniform `ATLAS_TILE_PX` slot. Linearises
/// sRGB samples so the lidar shader can treat atlas pixels as linear
/// albedo without per-fetch decoding.
fn resample_image_linear_rgba(image: &Image, target: u32) -> Option<Vec<u8>> {
    use bevy::render::render_resource::TextureFormat;
    let data = image.data.as_deref()?;
    let format = image.texture_descriptor.format;
    let is_srgb = match format {
        TextureFormat::Rgba8UnormSrgb => true,
        TextureFormat::Rgba8Unorm => false,
        _ => return None,
    };
    let src_w = image.texture_descriptor.size.width.max(1);
    let src_h = image.texture_descriptor.size.height.max(1);
    let target = target.max(1);
    let mut out = vec![0u8; (target * target * 4) as usize];
    for y in 0..target {
        let sy = ((y as u64 * src_h as u64) / target as u64) as u32;
        for x in 0..target {
            let sx = ((x as u64 * src_w as u64) / target as u64) as u32;
            let src_off = ((sy * src_w + sx) * 4) as usize;
            let dst_off = ((y * target + x) * 4) as usize;
            let mut r = data[src_off] as f32 / 255.0;
            let mut g = data[src_off + 1] as f32 / 255.0;
            let mut b = data[src_off + 2] as f32 / 255.0;
            let a = data[src_off + 3];
            if is_srgb {
                r = srgb_to_linear(r);
                g = srgb_to_linear(g);
                b = srgb_to_linear(b);
            }
            out[dst_off] = (r.clamp(0.0, 1.0) * 255.0) as u8;
            out[dst_off + 1] = (g.clamp(0.0, 1.0) * 255.0) as u8;
            out[dst_off + 2] = (b.clamp(0.0, 1.0) * 255.0) as u8;
            out[dst_off + 3] = a;
        }
    }
    Some(out)
}

/// Bake all material textures into a square atlas + per-material UV
/// rect table. Returns `(atlas_pixels, atlas_size_px, rects)`. Pixels
/// are stored linear-RGBA8 packed into a single `u32` per pixel:
/// `(a << 24) | (b << 16) | (g << 8) | r`. The lidar shader unpacks +
/// multiplies by the per-material tint to get the final albedo.
fn bake_atlas(
    palette_handles: &[Option<Handle<StandardMaterial>>],
    palette: &[Vec4],
    materials: Option<&Assets<StandardMaterial>>,
    images: Option<&Assets<Image>>,
) -> (Vec<u32>, u32, Vec<Vec4>) {
    let n = palette.len().max(1);
    let grid = (n as f32).sqrt().ceil() as u32;
    let tile = ATLAS_TILE_PX;
    let atlas_size = grid * tile;
    let total_pixels = (atlas_size * atlas_size) as usize;
    let mut atlas: Vec<u32> = vec![pack_rgba(255, 255, 255, 255); total_pixels];
    let mut rects = Vec::with_capacity(n);

    for (i, handle_opt) in palette_handles.iter().enumerate() {
        let gx = (i as u32) % grid;
        let gy = (i as u32) / grid;
        let dx = gx * tile;
        let dy = gy * tile;

        let tile_pixels = handle_opt
            .as_ref()
            .and_then(|h| materials.and_then(|m| m.get(h)))
            .and_then(|m| {
                m.base_color_texture
                    .as_ref()
                    .and_then(|t| images.and_then(|imgs| imgs.get(t)))
            })
            .and_then(|img| resample_image_linear_rgba(img, tile));

        match tile_pixels {
            Some(bytes) => {
                // Multiply each pixel by the material tint then pack to u32.
                let tint = palette[i];
                for ty in 0..tile {
                    let row_src = (ty * tile * 4) as usize;
                    let row_dst = ((dy + ty) * atlas_size + dx) as usize;
                    for tx in 0..tile {
                        let s = row_src + (tx * 4) as usize;
                        let r = (bytes[s] as f32 / 255.0 * tint.x).clamp(0.0, 1.0);
                        let g = (bytes[s + 1] as f32 / 255.0 * tint.y).clamp(0.0, 1.0);
                        let b = (bytes[s + 2] as f32 / 255.0 * tint.z).clamp(0.0, 1.0);
                        let a = (bytes[s + 3] as f32 / 255.0 * tint.w).clamp(0.0, 1.0);
                        let packed = pack_rgba(
                            (r * 255.0) as u8,
                            (g * 255.0) as u8,
                            (b * 255.0) as u8,
                            (a * 255.0) as u8,
                        );
                        atlas[row_dst + tx as usize] = packed;
                    }
                }
            }
            None => {
                // Solid tint fill.
                let tint = palette[i];
                let packed = pack_rgba(
                    (tint.x.clamp(0.0, 1.0) * 255.0) as u8,
                    (tint.y.clamp(0.0, 1.0) * 255.0) as u8,
                    (tint.z.clamp(0.0, 1.0) * 255.0) as u8,
                    (tint.w.clamp(0.0, 1.0) * 255.0) as u8,
                );
                for ty in 0..tile {
                    let row_dst = ((dy + ty) * atlas_size + dx) as usize;
                    for tx in 0..tile {
                        atlas[row_dst + tx as usize] = packed;
                    }
                }
            }
        }

        let inv = 1.0 / atlas_size as f32;
        let rect = Vec4::new(
            dx as f32 * inv,
            dy as f32 * inv,
            tile as f32 * inv,
            tile as f32 * inv,
        );
        rects.push(rect);
    }

    (atlas, atlas_size, rects)
}

#[inline]
fn pack_rgba(r: u8, g: u8, b: u8, a: u8) -> u32 {
    (a as u32) << 24 | (b as u32) << 16 | (g as u32) << 8 | r as u32
}

#[allow(dead_code)]
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

    // Defer build until every material's base_color_texture is in
    // Assets<Image>. Atlas baking before textures load would emit
    // solid-tint tiles (typically all white for glTF cities where
    // base_color = WHITE + the actual colour lives in the texture).
    if let (Some(materials), Some(images)) = (materials_opt, images_opt) {
        let mut pending = 0usize;
        let mut stack = vec![root];
        while let Some(entity) = stack.pop() {
            if let Ok((_, _, Some(mat3d))) = mesh_q.get(entity) {
                if let Some(mat) = materials.get(&mat3d.0) {
                    if let Some(tex) = &mat.base_color_texture {
                        if images.get(tex).is_none() {
                            pending += 1;
                        }
                    }
                }
            }
            if let Ok(children) = children_q.get(entity) {
                for c in children.iter() {
                    stack.push(c);
                }
            }
        }
        if pending > 0 {
            debug!(
                "BVH build deferred: {} base_color_textures still loading",
                pending
            );
            return;
        }
    }

    let mut triangles: Vec<obvhs::triangle::Triangle> = Vec::new();
    let mut tri_uvs: Vec<Vec2> = Vec::new();
    let mut tri_materials: Vec<u32> = Vec::new();
    let mut palette: Vec<Vec4> = Vec::new();
    // Materials referenced by surviving triangles. Indexed by palette
    // slot. We hold the `Handle<StandardMaterial>` so the atlas baker
    // can re-fetch the source `base_color_texture` after the scene walk.
    let mut palette_handles: Vec<Option<Handle<StandardMaterial>>> = Vec::new();
    let mut palette_lookup: HashMap<AssetId<StandardMaterial>, u32> = HashMap::new();

    let mut stack = vec![root];
    while let Some(entity) = stack.pop() {
        if let Ok((mesh3d, gx, mat)) = mesh_q.get(entity) {
            if let Some(mesh) = meshes.get(&mesh3d.0) {
                let (new_tris, new_uvs) = extract_triangles_from_mesh(mesh, gx.to_matrix());
                if !new_tris.is_empty() {
                    let mat_id = match (mat, materials_opt) {
                        (Some(mat3d), Some(materials)) => {
                            let id = mat3d.0.id();
                            *palette_lookup.entry(id).or_insert_with(|| {
                                palette.push(material_albedo(&mat3d.0, materials, images_opt));
                                palette_handles.push(Some(mat3d.0.clone()));
                                (palette.len() - 1) as u32
                            })
                        }
                        _ => {
                            if palette.is_empty() {
                                palette.push(Vec4::ONE);
                                palette_handles.push(None);
                            }
                            0u32
                        }
                    };
                    let n = new_tris.len();
                    triangles.extend(new_tris);
                    for uv in &new_uvs {
                        tri_uvs.push(uv[0]);
                        tri_uvs.push(uv[1]);
                        tri_uvs.push(uv[2]);
                    }
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
    let (atlas_pixels, atlas_size, material_rects) =
        bake_atlas(&palette_handles, &palette, materials_opt, images_opt);
    let bvh = build_world_bvh_with_materials(
        triangles,
        tri_uvs,
        tri_materials,
        palette,
        atlas_pixels,
        atlas_size,
        material_rects,
    );
    info!(
        "built ground-truth BVH from {} triangles, {} materials, atlas {}×{} px, full aabb min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1}), trimmed min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1})",
        count,
        mat_count,
        atlas_size,
        atlas_size,
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
