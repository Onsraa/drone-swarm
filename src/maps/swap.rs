use bevy::asset::Assets;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy::render::gpu_readback::Readback;

use crate::drone::Drone;
use crate::exploration::{FrontierClusters, FrontierTarget, MovementHealth, Path, PlannerGrid, ReplanTimer};

/// Bundles the four exploration-state params into one `SystemParam` so
/// `apply_pending_swap` stays within Bevy's 16-parameter system limit.
#[derive(SystemParam)]
pub(crate) struct ExplorationResetParams<'w, 's> {
    grid: ResMut<'w, PlannerGrid>,
    paths: Query<'w, 's, &'static mut Path>,
    health: Query<'w, 's, &'static mut MovementHealth>,
    replan_timers: Query<'w, 's, &'static mut ReplanTimer>,
}
use crate::lidar::gpu::{
    BuildLocalParamsBuffer, DroneColorsBuffer, DroneOrientationsBuffer, DronePositionsBuffer,
    GlobalInstanceCountBuffer, GlobalInstanceVecBuffer, GlobalOccupancyBuffer,
    GpuGlobalOccupancyMirror, GpuGlobalStats, GroundTruthBuffer, LidarParamsBuffer,
    LidarPointCountBuffer, LidarPointVecBuffer, LocalInstanceCountBuffer, LocalInstanceVecBuffer,
    LocalOccupancyBuffer, RayDirsBuffer,
};
use crate::render::{GpuGlobalMapVoxel, GpuLocalMapVoxel, GroundTruthVoxel, LidarPointVoxel};
use crate::world::{GroundTruthMap, WorldConfig};

use super::asset::MapAsset;
use super::events::MapSwapRequested;
use super::registry::AvailableMaps;

/// Latest swap request, queued until the referenced asset has finished
/// loading. `apply_pending_swap` polls each frame and tears the sim
/// down + rebuilds it once the asset is in `Assets<MapAsset>`.
#[derive(Resource, Default)]
pub struct PendingMapSwap {
    pub handle: Option<Handle<MapAsset>>,
    pub name: String,
}

/// Latest-wins coalesce: if the UI fires several swap requests in one
/// frame, keep only the most recent so we don't churn through them all.
pub fn enqueue_map_swap(
    mut reader: MessageReader<MapSwapRequested>,
    mut pending: ResMut<PendingMapSwap>,
) {
    if let Some(msg) = reader.read().last() {
        pending.handle = Some(msg.handle.clone());
        pending.name = msg.name.clone();
    }
}

/// First-frame bootstrap: as soon as the registry has scanned the maps
/// directory and there's no ground truth in the world yet, queue a swap
/// to the default (first) entry so the sim has something to display.
pub fn bootstrap_initial_map(
    available: Res<AvailableMaps>,
    ground_truth: Option<Res<GroundTruthMap>>,
    pending: ResMut<PendingMapSwap>,
    mut once: Local<bool>,
) {
    if *once || ground_truth.is_some() || available.entries.is_empty() {
        return;
    }
    if pending.handle.is_some() {
        return;
    }
    let entry = &available.entries[0];
    let pending = pending.into_inner();
    pending.handle = Some(entry.handle.clone());
    pending.name = entry.name.clone();
    *once = true;
    info!("bootstrapping initial map: {}", entry.name);
}

/// Execute a pending swap once the asset is loaded. Tears down all GPU
/// lidar buffer resources, despawns drones + render entities, resets
/// stats and frontier state, then drops a fresh `GroundTruthMap` +
/// `WorldConfig`. The startup-style `setup_gpu_lidar_assets` system in
/// `lidar/gpu` re-runs on the next frame because every buffer resource
/// has been removed, and the spawn-if-missing systems in
/// `render::{gpu_local_map, gpu_global_map, ground_truth}` respawn
/// their entities.
#[allow(clippy::too_many_arguments)]
pub fn apply_pending_swap(
    mut commands: Commands,
    mut pending: ResMut<PendingMapSwap>,
    assets: Res<Assets<MapAsset>>,
    drones: Query<Entity, With<Drone>>,
    ground_truth_entities: Query<Entity, With<GroundTruthVoxel>>,
    local_map_entities: Query<Entity, With<GpuLocalMapVoxel>>,
    global_map_entities: Query<Entity, With<GpuGlobalMapVoxel>>,
    point_entities: Query<Entity, With<LidarPointVoxel>>,
    readbacks: Query<Entity, With<Readback>>,
    mut stats: ResMut<GpuGlobalStats>,
    mut mirror: ResMut<GpuGlobalOccupancyMirror>,
    mut clusters: ResMut<FrontierClusters>,
    mut frontier_targets: Query<&mut FrontierTarget>,
    mut exploration: ExplorationResetParams,
) {
    let Some(handle) = pending.handle.clone() else {
        return;
    };
    let Some(asset) = assets.get(&handle) else {
        return;
    };

    info!("applying map swap: {}", pending.name);

    for e in &drones {
        commands.entity(e).despawn();
    }
    for e in &ground_truth_entities {
        commands.entity(e).despawn();
    }
    for e in &local_map_entities {
        commands.entity(e).despawn();
    }
    for e in &global_map_entities {
        commands.entity(e).despawn();
    }
    for e in &point_entities {
        commands.entity(e).despawn();
    }
    for e in &readbacks {
        commands.entity(e).despawn();
    }

    commands.remove_resource::<GroundTruthBuffer>();
    commands.remove_resource::<LidarParamsBuffer>();
    commands.remove_resource::<DronePositionsBuffer>();
    commands.remove_resource::<DroneOrientationsBuffer>();
    commands.remove_resource::<RayDirsBuffer>();
    commands.remove_resource::<LocalOccupancyBuffer>();
    commands.remove_resource::<GlobalOccupancyBuffer>();
    commands.remove_resource::<BuildLocalParamsBuffer>();
    commands.remove_resource::<DroneColorsBuffer>();
    commands.remove_resource::<LocalInstanceCountBuffer>();
    commands.remove_resource::<LocalInstanceVecBuffer>();
    commands.remove_resource::<GlobalInstanceCountBuffer>();
    commands.remove_resource::<GlobalInstanceVecBuffer>();
    commands.remove_resource::<LidarPointCountBuffer>();
    commands.remove_resource::<LidarPointVecBuffer>();

    *stats = GpuGlobalStats::default();
    mirror.data.clear();
    clusters.entries.clear();
    clusters.next_id = 0;
    for mut t in &mut frontier_targets {
        t.pos = None;
        t.cluster_id = None;
    }

    *exploration.grid = PlannerGrid::default();
    for mut p in &mut exploration.paths {
        p.waypoints.clear();
        p.cursor = 0;
    }
    for mut h in &mut exploration.health {
        *h = MovementHealth::default();
    }
    for mut rt in &mut exploration.replan_timers {
        *rt = ReplanTimer::default();
    }

    let map = GroundTruthMap::from_bitset(asset.dims, &asset.bitset);
    let occupied = map.count_occupied();
    commands.insert_resource(map);
    commands.insert_resource(WorldConfig {
        size: asset.dims,
        voxel_size: asset.voxel_size,
    });

    info!(
        "map swap applied: {} ({} occupied cells, dims {:?})",
        pending.name, occupied, asset.dims
    );
    pending.handle = None;
    pending.name.clear();
}
