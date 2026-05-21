use bevy::prelude::*;
use bevy::scene::SceneRoot;

use super::components::GroundTruthMesh;
use super::resources::MeshGroundTruthConfig;

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
