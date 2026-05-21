use bevy::prelude::*;
use bevy::scene::SceneRoot;

use super::bvh::{build_world_bvh, recommended_transform, WorldBvh};
use super::components::GroundTruthMesh;
use super::constants::AUTO_FIT_COVERAGE_RATIO;
use super::resources::MeshGroundTruthConfig;
use super::triangles::extract_triangles_from_mesh;
use crate::world::WorldConfig;

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
pub fn build_bvh_when_scene_ready(
    mut commands: Commands,
    meshes: Option<Res<Assets<Mesh>>>,
    bvh_present: Option<Res<WorldBvh>>,
    mut config: ResMut<MeshGroundTruthConfig>,
    world_config: Option<Res<WorldConfig>>,
    root_query: Query<Entity, With<GroundTruthMesh>>,
    children_q: Query<&Children>,
    mesh_q: Query<(&Mesh3d, &GlobalTransform)>,
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

    let mut triangles = Vec::new();
    let mut stack = vec![root];
    while let Some(entity) = stack.pop() {
        if let Ok((mesh3d, gx)) = mesh_q.get(entity) {
            if let Some(mesh) = meshes.get(&mesh3d.0) {
                triangles.extend(extract_triangles_from_mesh(mesh, gx.to_matrix()));
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

    let count = triangles.len();
    let bvh = build_world_bvh(triangles);
    info!(
        "built ground-truth BVH from {} triangles, aabb min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1})",
        count,
        bvh.cwbvh.total_aabb.min.x,
        bvh.cwbvh.total_aabb.min.y,
        bvh.cwbvh.total_aabb.min.z,
        bvh.cwbvh.total_aabb.max.x,
        bvh.cwbvh.total_aabb.max.y,
        bvh.cwbvh.total_aabb.max.z,
    );

    // One-shot auto-fit: requires WorldConfig + a matching
    // applied_transform (so we only fit the first build, not the
    // post-fit rebuild).
    let applied_matches = config
        .applied_transform
        .is_some_and(|a| a == (config.translation, config.scale));
    if config.auto_fit_on_first_build && applied_matches {
        if let Some(world) = world_config.as_ref() {
            let aabb_min = Vec3::new(
                bvh.cwbvh.total_aabb.min.x,
                bvh.cwbvh.total_aabb.min.y,
                bvh.cwbvh.total_aabb.min.z,
            );
            let aabb_max = Vec3::new(
                bvh.cwbvh.total_aabb.max.x,
                bvh.cwbvh.total_aabb.max.y,
                bvh.cwbvh.total_aabb.max.z,
            );
            let (new_t, new_s) = recommended_transform(
                aabb_min,
                aabb_max,
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
