//! Frontier extraction + clustering + per-drone assignment.
//!
//! Reads the CPU mirror of the global occupancy bitset
//! (`GpuGlobalOccupancyMirror`), downsamples to an 8 m coarse grid,
//! tags every coarse cell as `Unknown | Free | Occupied`, then scans
//! for frontier cells (`Free` with at least one `Unknown` 6-neighbour).
//! A union-find pass groups frontier cells into clusters with a
//! centroid + cell count; a greedy nearest-unclaimed pass assigns each
//! drone to one cluster. Consumers (`apply_role_steering`) read the
//! per-drone target out of `FrontierAssignments`.
//!
//! Cadence: refreshes every `FRONTIER_REFRESH_FRAMES` frames so the
//! per-cell scan amortises to <1 ms / frame on average.

use bevy::platform::collections::HashMap;
use bevy::prelude::*;

use crate::drone::{Drone, DroneId};
use crate::lidar::gpu::GpuGlobalOccupancyMirror;
use crate::world::WorldConfig;

/// One coarse-cell side in fine-voxel units (so a coarse cell is
/// `DOWNSAMPLE^3` fine voxels). 8 mirrors the pheromone field's
/// resolution so role steering and frontier targets share the same
/// spatial scale.
pub const FRONTIER_DOWNSAMPLE: u32 = 8;

/// Frame cadence for the frontier scan + cluster + assignment pipeline.
/// 30 frames ≈ 0.5 s at 60 Hz. Drones hold their target across the
/// gap, so the cadence is invisible at flight speeds < 30 m/s.
pub const FRONTIER_REFRESH_FRAMES: u32 = 30;

/// Drop clusters smaller than this — they're usually single noisy
/// coarse cells on the edge of detector reach. Keeps drones from
/// vibrating between micro-frontiers.
pub const MIN_CLUSTER_CELLS: u32 = 3;

/// Maximum number of fine voxels per coarse-cell axis. Keeps the
/// scan loop bounded when `WorldConfig.size` is not a clean multiple
/// of `FRONTIER_DOWNSAMPLE`.
#[inline]
fn coarse_range(start: u32, count: u32, fine_dim: u32) -> std::ops::Range<u32> {
    let end = (start + count).min(fine_dim);
    start..end
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CoarseState {
    Unknown = 0,
    Free = 1,
    Occupied = 2,
}

#[derive(Clone, Copy, Debug)]
pub struct FrontierCluster {
    pub centroid: Vec3,
    pub cell_count: u32,
}

/// Snapshot of the latest frontier scan. `coarse_state` + `cluster_id`
/// are row-major flat `(z * dy + y) * dx + x` over the coarse grid.
/// Refreshed every `FRONTIER_REFRESH_FRAMES` frames.
#[derive(Resource, Default, Debug)]
pub struct FrontierField {
    pub coarse_state: Vec<u8>,
    pub cluster_id: Vec<i32>,
    pub clusters: Vec<FrontierCluster>,
    pub coarse_dims: UVec3,
    pub coarse_cell_size: f32,
    pub last_refresh_frame: u32,
    frame: u32,
}

/// Per-drone frontier target. Empty when no cluster is assigned (e.g.
/// the world is fully Unknown — no `Free` exists yet — or every
/// cluster is too small to attract).
#[derive(Resource, Default, Debug)]
pub struct FrontierAssignments {
    pub targets: HashMap<u32, Vec3>,
}

pub struct FrontierPlugin;

impl Plugin for FrontierPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FrontierField>()
            .init_resource::<FrontierAssignments>()
            .add_systems(
                Update,
                (ensure_field_sized, extract_and_cluster, assign_frontiers).chain(),
            );
    }
}

fn ensure_field_sized(
    world: Option<Res<WorldConfig>>,
    mut field: ResMut<FrontierField>,
) {
    let Some(world) = world else { return; };
    let coarse_dims = UVec3::new(
        world.size.x.div_ceil(FRONTIER_DOWNSAMPLE),
        world.size.y.div_ceil(FRONTIER_DOWNSAMPLE),
        world.size.z.div_ceil(FRONTIER_DOWNSAMPLE),
    );
    let total = (coarse_dims.x * coarse_dims.y * coarse_dims.z) as usize;
    let want_size = world.voxel_size * FRONTIER_DOWNSAMPLE as f32;
    if field.coarse_dims == coarse_dims
        && (field.coarse_cell_size - want_size).abs() < f32::EPSILON
    {
        return;
    }
    field.coarse_state.clear();
    field.coarse_state.resize(total, CoarseState::Unknown as u8);
    field.cluster_id.clear();
    field.cluster_id.resize(total, -1);
    field.clusters.clear();
    field.coarse_dims = coarse_dims;
    field.coarse_cell_size = want_size;
    field.last_refresh_frame = 0;
}

/// Scan the global occupancy mirror into the coarse grid, mark frontier
/// cells, run a union-find clustering pass, store cluster centroids +
/// sizes. Runs every `FRONTIER_REFRESH_FRAMES` frames. No-op when the
/// mirror hasn't been populated yet.
fn extract_and_cluster(
    world: Option<Res<WorldConfig>>,
    mirror: Option<Res<GpuGlobalOccupancyMirror>>,
    mut field: ResMut<FrontierField>,
) {
    field.frame = field.frame.wrapping_add(1);
    if field.frame.wrapping_sub(field.last_refresh_frame) < FRONTIER_REFRESH_FRAMES
        && field.last_refresh_frame != 0
    {
        return;
    }
    let Some(world) = world else { return; };
    let Some(mirror) = mirror else { return; };
    if mirror.data.is_empty() {
        return;
    }
    let fine_dims = world.size;
    let coarse_dims = field.coarse_dims;
    let total = (coarse_dims.x * coarse_dims.y * coarse_dims.z) as usize;
    if field.coarse_state.len() != total {
        return;
    }

    // 1) Downsample: every coarse cell summarises its fine block as
    // (any Occupied) → Occupied else (any Free) → Free else Unknown.
    let words = mirror.data.as_slice();
    for cz in 0..coarse_dims.z {
        for cy in 0..coarse_dims.y {
            for cx in 0..coarse_dims.x {
                let mut has_occ = false;
                let mut has_free = false;
                'cell: for fz in coarse_range(
                    cz * FRONTIER_DOWNSAMPLE,
                    FRONTIER_DOWNSAMPLE,
                    fine_dims.z,
                ) {
                    for fy in coarse_range(
                        cy * FRONTIER_DOWNSAMPLE,
                        FRONTIER_DOWNSAMPLE,
                        fine_dims.y,
                    ) {
                        let row_base = (fz * fine_dims.y + fy) * fine_dims.x;
                        for fx in coarse_range(
                            cx * FRONTIER_DOWNSAMPLE,
                            FRONTIER_DOWNSAMPLE,
                            fine_dims.x,
                        ) {
                            let flat = row_base + fx;
                            let w_idx = (flat / 16) as usize;
                            if w_idx >= words.len() {
                                continue;
                            }
                            let bit_offset = ((flat % 16) * 2) as u32;
                            let state = (words[w_idx] >> bit_offset) & 0x3;
                            if state >= 2 {
                                has_occ = true;
                                break 'cell;
                            }
                            if state == 1 {
                                has_free = true;
                            }
                        }
                    }
                }
                let s = if has_occ {
                    CoarseState::Occupied
                } else if has_free {
                    CoarseState::Free
                } else {
                    CoarseState::Unknown
                };
                let i = coarse_idx(coarse_dims, UVec3::new(cx, cy, cz));
                field.coarse_state[i] = s as u8;
            }
        }
    }

    // 2) Frontier mask: Free with at least one Unknown 6-neighbour.
    // Reuse `cluster_id` as a workspace: -1 = not a frontier; otherwise
    // populated by the union-find pass below.
    let dx = coarse_dims.x as i32;
    let dy = coarse_dims.y as i32;
    let dz = coarse_dims.z as i32;
    let mut frontier_flat: Vec<usize> = Vec::new();
    for z in 0..dz {
        for y in 0..dy {
            for x in 0..dx {
                let i = coarse_idx(coarse_dims, UVec3::new(x as u32, y as u32, z as u32));
                if field.coarse_state[i] != CoarseState::Free as u8 {
                    field.cluster_id[i] = -1;
                    continue;
                }
                let mut is_frontier = false;
                for offset in [
                    IVec3::X, -IVec3::X, IVec3::Y, -IVec3::Y, IVec3::Z, -IVec3::Z,
                ] {
                    let nx = x + offset.x;
                    let ny = y + offset.y;
                    let nz = z + offset.z;
                    if nx < 0 || ny < 0 || nz < 0 || nx >= dx || ny >= dy || nz >= dz {
                        // World edge counts as Unknown — pushes drones
                        // toward unexplored borders.
                        is_frontier = true;
                        break;
                    }
                    let ni = coarse_idx(
                        coarse_dims,
                        UVec3::new(nx as u32, ny as u32, nz as u32),
                    );
                    if field.coarse_state[ni] == CoarseState::Unknown as u8 {
                        is_frontier = true;
                        break;
                    }
                }
                if is_frontier {
                    field.cluster_id[i] = 0;
                    frontier_flat.push(i);
                } else {
                    field.cluster_id[i] = -1;
                }
            }
        }
    }

    // 3) Union-find cluster labelling on the frontier cells.
    let total_cells = total;
    let mut parent: Vec<i32> = vec![-1; total_cells];
    for &i in &frontier_flat {
        parent[i] = i as i32;
    }
    for &i in &frontier_flat {
        let (x, y, z) = coarse_decode(coarse_dims, i);
        for offset in [IVec3::X, IVec3::Y, IVec3::Z] {
            let nx = x as i32 + offset.x;
            let ny = y as i32 + offset.y;
            let nz = z as i32 + offset.z;
            if nx < 0 || ny < 0 || nz < 0 || nx >= dx || ny >= dy || nz >= dz {
                continue;
            }
            let ni = coarse_idx(
                coarse_dims,
                UVec3::new(nx as u32, ny as u32, nz as u32),
            );
            if parent[ni] >= 0 {
                uf_union(&mut parent, i, ni);
            }
        }
    }

    // 4) Compact: group frontier cells by root, build clusters with
    // centroid + cell count. Reuse `cluster_id` to point at the index
    // in `field.clusters`.
    let mut root_to_idx: HashMap<i32, usize> = HashMap::new();
    field.clusters.clear();
    for &i in &frontier_flat {
        let root = uf_find(&mut parent, i as i32);
        let (cx, cy, cz) = coarse_decode(coarse_dims, i);
        let cell_world = Vec3::new(
            (cx as f32 + 0.5) * field.coarse_cell_size,
            (cy as f32 + 0.5) * field.coarse_cell_size,
            (cz as f32 + 0.5) * field.coarse_cell_size,
        );
        let idx = if let Some(&idx) = root_to_idx.get(&root) {
            idx
        } else {
            let idx = field.clusters.len();
            root_to_idx.insert(root, idx);
            field.clusters.push(FrontierCluster {
                centroid: Vec3::ZERO,
                cell_count: 0,
            });
            idx
        };
        let cluster = &mut field.clusters[idx];
        let n = cluster.cell_count as f32;
        cluster.centroid = (cluster.centroid * n + cell_world) / (n + 1.0);
        cluster.cell_count += 1;
        field.cluster_id[i] = idx as i32;
    }

    // 5) Drop clusters smaller than the noise floor. Compaction keeps
    // cluster indices contiguous; rewrite `cluster_id` entries that
    // pointed to dropped clusters back to -1.
    if !field.clusters.is_empty() {
        let mut remap: Vec<i32> = Vec::with_capacity(field.clusters.len());
        let mut kept: Vec<FrontierCluster> = Vec::new();
        for c in field.clusters.iter() {
            if c.cell_count >= MIN_CLUSTER_CELLS {
                remap.push(kept.len() as i32);
                kept.push(*c);
            } else {
                remap.push(-1);
            }
        }
        field.clusters = kept;
        for v in field.cluster_id.iter_mut() {
            if *v >= 0 {
                *v = remap[*v as usize];
            }
        }
    }

    field.last_refresh_frame = field.frame;
}

/// Greedy nearest-unclaimed assignment of frontier clusters to drones.
/// One target per cluster — when two drones could pick the same one,
/// the closer drone wins; the other moves to its next-nearest cluster.
fn assign_frontiers(
    field: Res<FrontierField>,
    mut assignments: ResMut<FrontierAssignments>,
    q: Query<(&DroneId, &Transform), With<Drone>>,
) {
    assignments.targets.clear();
    if field.clusters.is_empty() {
        return;
    }
    // Collect (drone_id, pos) once, then iteratively pick the
    // (drone, cluster) pair with the smallest distance.
    let mut drones: Vec<(u32, Vec3)> =
        q.iter().map(|(id, t)| (id.0, t.translation)).collect();
    let mut claimed: Vec<bool> = vec![false; field.clusters.len()];
    while !drones.is_empty() {
        let mut best: Option<(usize, usize, f32)> = None;
        for (di, (_id, pos)) in drones.iter().enumerate() {
            for (ci, cluster) in field.clusters.iter().enumerate() {
                if claimed[ci] {
                    continue;
                }
                let d = pos.distance_squared(cluster.centroid);
                match best {
                    Some((_, _, bd)) if d >= bd => {}
                    _ => best = Some((di, ci, d)),
                }
            }
        }
        let Some((di, ci, _)) = best else { break };
        let (id, _) = drones[di];
        assignments.targets.insert(id, field.clusters[ci].centroid);
        claimed[ci] = true;
        drones.swap_remove(di);
        // When more drones than clusters exist, the loop ends here —
        // unclaimed drones receive no target this round.
        if claimed.iter().all(|&c| c) {
            break;
        }
    }
}

#[inline]
fn coarse_idx(dims: UVec3, c: UVec3) -> usize {
    ((c.z * dims.y + c.y) * dims.x + c.x) as usize
}

#[inline]
fn coarse_decode(dims: UVec3, idx: usize) -> (u32, u32, u32) {
    let i = idx as u32;
    let plane = dims.x * dims.y;
    let z = i / plane;
    let rem = i % plane;
    let y = rem / dims.x;
    let x = rem % dims.x;
    (x, y, z)
}

fn uf_find(parent: &mut [i32], mut i: i32) -> i32 {
    while parent[i as usize] != i {
        let p = parent[i as usize];
        let pp = parent[p as usize];
        parent[i as usize] = pp;
        i = pp;
    }
    i
}

fn uf_union(parent: &mut [i32], a: usize, b: usize) {
    let ra = uf_find(parent, a as i32);
    let rb = uf_find(parent, b as i32);
    if ra != rb {
        parent[ra as usize] = rb;
    }
}
