use bevy::prelude::*;
use bevy::scene::SceneRoot;

use super::bvh::{build_world_bvh, WorldBvh};
use super::components::GroundTruthMesh;
use super::resources::MeshGroundTruthConfig;
use super::triangles::extract_triangles_from_mesh;

/// One-shot spawn of the ground-truth mesh entity. The scene asset is
/// loaded by path from `MeshGroundTruthConfig`; if the file is absent
/// the asset server logs a warning and the SceneRoot stays empty until
/// the file appears (asset hot reload).
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
    commands.spawn((
        GroundTruthMesh,
        SceneRoot(handle),
        Transform::IDENTITY,
        Visibility::default(),
    ));
    config.spawned = true;
    info!("spawned ground-truth mesh from {}", config.scene_asset_path);
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
    info!("built ground-truth BVH from {} triangles", count);
    commands.insert_resource(bvh);
}
