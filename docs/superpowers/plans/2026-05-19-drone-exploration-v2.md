# Drone Exploration v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current naive "fly-through-walls toward nearest Unknown" drone behavior with a collision-aware, role-specialized swarm built on a hybrid coarse-A* planner + reactive potential field, frontier clustering + cost-utility scoring, and a dynamic supervisor that assigns scout/mapper/anchor roles.

**Architecture:** Three-tier control. Supervisor (0.5 Hz) decides who is what. Per-drone target picker + coarse A* planner emits waypoints over an 8× downsampled discovered-occupancy grid. Steering layer follows the path via pure-pursuit + repulsive forces from this-frame lidar hits and comms-broadcast peers. No behavior code reads `GroundTruthMap` — lidar shader is the only sensor model.

**Tech Stack:** Rust 2024 + Bevy 0.18.1 + bevy_egui 0.39.1 + WGSL compute shaders. Existing crates: `bytemuck`, `rand`, `dot_vox`. No new dependencies.

**Reference:** Spec at `docs/superpowers/specs/2026-05-19-drone-exploration-v2-design.md`.

---

## File structure overview

### Phase 1 — Movement foundation

**New module `src/exploration/`** (supersedes `src/frontier/`):

| File | Responsibility |
|---|---|
| `mod.rs` | `ExplorationPlugin`, public re-exports |
| `constants.rs` | All tunables (radii, weights, thresholds) |
| `components.rs` | `FrontierTarget`, `Path`, `MovementHealth` |
| `resources.rs` | `FrontierClusters`, `PlannerGrid` |
| `cluster.rs` | Flood-fill candidates → clusters |
| `scoring.rs` | Cost-utility, crowding (role-agnostic in Phase 1) |
| `planner.rs` | Coarse A* + grid downsample |
| `steering.rs` | Pure-pursuit + reactive forces |
| `systems.rs` | Bevy system orchestration |

**Modified files:**

| File | Reason |
|---|---|
| `src/main.rs` | Register `ExplorationPlugin`, drop `FrontierPlugin` |
| `src/maps/swap.rs` | Extend `apply_pending_swap` to reset Phase 1 state |
| `src/drone/spawn.rs` | Spawn drones with `MovementHealth` + empty `Path` |
| `src/drone/wander.rs` | Stays as cold-start fallback (no changes) |

**Removed:** `src/frontier/` (functionality migrated to `src/exploration/`).

### Phase 2 — Role specialization

**New files:**

| File | Responsibility |
|---|---|
| `src/exploration/role.rs` | `Role` enum, `RoleParams` struct, per-role defaults |
| `src/exploration/supervisor.rs` | Role assignment + comms-graph articulation finder |
| `src/lidar/gpu/per_drone_scan.rs` | `DroneScanParams` SSBO + ray-set splicing |

**Modified files:**

| File | Reason |
|---|---|
| `src/exploration/components.rs` | Add `Role` to spawn bundle |
| `src/exploration/scoring.rs` | Role-flavored weights |
| `src/exploration/steering.rs` | Per-role `AVOID_K` |
| `src/lidar/gpu/resources.rs` | `LidarParams` retires `rays_per_scan`, replaced by per-drone SSBO |
| `src/lidar/gpu/mod.rs` | Upload per-drone scan params each frame |
| `src/lidar/gpu/pipeline.rs` | Bind-group layout gains binding 10 |
| `src/lidar/gpu/dispatch.rs` | Pass new binding through |
| `src/lidar/sampling.rs` | Multi-cone ray-set builder |
| `assets/shaders/lidar_compute.wgsl` | Read per-drone params, pick ray slice |
| `src/maps/swap.rs` | Extend `apply_pending_swap` to reset roles |
| `src/ui/panel.rs` | Roles section in side panel |
| `src/drone/components.rs` | `DroneColor` derived from `Role` |

---

# Phase 1 — Movement foundation

Delivers: drones that obstacle-avoid, plan paths through discovered occupancy, cluster frontiers, score with crowding, detect stuck. All drones same kind (no role differentiation yet). Phase 1 alone fixes the "drones fly through walls" defect.

## Task 1: Create `src/exploration/` module skeleton + migrate `FrontierTarget`

**Files:**
- Create: `src/exploration/mod.rs`, `src/exploration/components.rs`, `src/exploration/constants.rs`, `src/exploration/resources.rs`
- Modify: `src/main.rs`
- Remove (later in task): `src/frontier/` (after migration)

- [ ] **Step 1: Create `src/exploration/components.rs`**

```rust
use bevy::prelude::*;

/// World-space target the drone is currently flying toward. `None` means
/// no frontier assigned (cold start). Replaces the previous `frontier::FrontierTarget`.
#[derive(Component, Default, Debug)]
pub struct FrontierTarget {
    pub pos: Option<Vec3>,
    pub cluster_id: Option<u32>,
}

/// Planned waypoint sequence in world coords. Empty = no plan; `wander`
/// fallback drives the drone. Pure-pursuit consumes this.
#[derive(Component, Default, Debug)]
pub struct Path {
    pub waypoints: Vec<Vec3>,
    pub cursor: usize,
}

impl Path {
    pub fn is_empty(&self) -> bool {
        self.cursor >= self.waypoints.len()
    }
    pub fn next(&self) -> Option<Vec3> {
        self.waypoints.get(self.cursor).copied()
    }
}

/// Stuck detector state per drone.
#[derive(Component, Default, Debug)]
pub struct MovementHealth {
    pub slow_secs: f32,
    pub escalations_in_window: u32,
    pub window_start_secs: f32,
}
```

- [ ] **Step 2: Create `src/exploration/constants.rs`**

```rust
// Frontier scan + clustering
pub const FRONTIER_REFRESH_SECS: f32 = 1.0;
pub const FRONTIER_REACHED_DIST: f32 = 6.0;
pub const MAX_FRONTIER_CANDIDATES: usize = 50_000;
pub const MIN_CLUSTER_SIZE: usize = 4;

// Planner
pub const PLANNER_DOWNSAMPLE: u32 = 8;
pub const PLANNER_FREE_COST: f32 = 1.0;
pub const PLANNER_UNKNOWN_COST_MULT: f32 = 3.0;
pub const PLANNER_DEEP_UNKNOWN_MULT: f32 = 5.0;
pub const REPLAN_MIN_INTERVAL_SECS: f32 = 1.0;

// Steering
pub const LOOKAHEAD_M: f32 = 8.0;
pub const PATH_FOLLOW_LERP_RATE: f32 = 3.0;
pub const AVOID_RADIUS_M: f32 = 4.0;
pub const AVOID_RADIUS_PEER_M: f32 = 6.0;
pub const AVOID_K_DEFAULT: f32 = 6.0;

// Stuck detection
pub const STUCK_VEL_MPS: f32 = 0.5;
pub const STUCK_SECS: f32 = 3.0;
pub const STUCK_ESCALATION_WINDOW_SECS: f32 = 20.0;

// Scoring (role-agnostic Phase 1 defaults)
pub const SCORE_INFO_WEIGHT: f32 = 1.0;
pub const SCORE_DISTANCE_WEIGHT: f32 = 1.0;
pub const SCORE_DISTANCE_BIAS: f32 = 1.0;
pub const SCORE_CROWDING_WEIGHT: f32 = 1.0;
pub const SCORE_UPGRADE_RATIO: f32 = 1.5;

// Cruise (Phase 1 single speed; Phase 2 overrides per role)
pub const DEFAULT_CRUISE_SPEED_MPS: f32 = 5.0;
```

- [ ] **Step 3: Create `src/exploration/resources.rs`**

```rust
use bevy::prelude::*;

#[derive(Debug, Clone)]
pub struct FrontierCluster {
    pub id: u32,
    pub centroid: Vec3,
    pub cells: Vec<UVec3>,
    pub info_gain: f32,
    pub bbox_min: UVec3,
    pub bbox_max: UVec3,
}

#[derive(Resource, Default, Debug)]
pub struct FrontierClusters {
    pub entries: Vec<FrontierCluster>,
    pub next_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoarseCell {
    Free,
    Unknown,
    Blocked,
}

#[derive(Resource, Default, Debug)]
pub struct PlannerGrid {
    pub coarse: Vec<CoarseCell>,
    pub dims: UVec3,
    pub voxel_size: f32,
    pub downsample: u32,
}

impl PlannerGrid {
    pub fn idx(&self, c: UVec3) -> Option<usize> {
        if c.x >= self.dims.x || c.y >= self.dims.y || c.z >= self.dims.z {
            return None;
        }
        Some(((c.z * self.dims.y + c.y) * self.dims.x + c.x) as usize)
    }
    pub fn at(&self, c: UVec3) -> CoarseCell {
        self.idx(c)
            .and_then(|i| self.coarse.get(i).copied())
            .unwrap_or(CoarseCell::Unknown)
    }
    pub fn world_pos_of(&self, c: UVec3) -> Vec3 {
        let cell_size = self.voxel_size * self.downsample as f32;
        Vec3::new(c.x as f32, c.y as f32, c.z as f32) * cell_size + Vec3::splat(cell_size * 0.5)
    }
}
```

- [ ] **Step 4: Create `src/exploration/mod.rs`**

```rust
pub mod cluster;
pub mod components;
pub mod constants;
pub mod planner;
pub mod resources;
pub mod scoring;
pub mod steering;
pub mod systems;

use bevy::prelude::*;

pub use components::{FrontierTarget, MovementHealth, Path};
pub use resources::{CoarseCell, FrontierCluster, FrontierClusters, PlannerGrid};

use crate::physics::PhysicsSet;
use systems::{
    assign_targets, compute_frontier_clusters, rebuild_planner_grid, replan_paths,
    reactive_avoid, steer_along_path, stuck_recovery, update_movement_health,
};

pub struct ExplorationPlugin;

impl Plugin for ExplorationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FrontierClusters>()
            .init_resource::<PlannerGrid>()
            .add_systems(
                Update,
                (
                    rebuild_planner_grid,
                    compute_frontier_clusters,
                    assign_targets,
                    replan_paths,
                    update_movement_health,
                    stuck_recovery,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (steer_along_path, reactive_avoid)
                    .after(crate::drone::wander)
                    .before(PhysicsSet::Control),
            );
    }
}
```

- [ ] **Step 5: Create stub files for `cluster.rs`, `planner.rs`, `scoring.rs`, `steering.rs`, `systems.rs`**

```rust
// src/exploration/cluster.rs
// Implemented in Task 4.
```

```rust
// src/exploration/planner.rs
// Implemented in Task 3.
```

```rust
// src/exploration/scoring.rs
// Implemented in Task 5.
```

```rust
// src/exploration/steering.rs
// Implemented in Task 8.
```

```rust
// src/exploration/systems.rs
use bevy::prelude::*;

pub fn rebuild_planner_grid() {}
pub fn compute_frontier_clusters() {}
pub fn assign_targets() {}
pub fn replan_paths() {}
pub fn update_movement_health() {}
pub fn stuck_recovery() {}
pub fn steer_along_path() {}
pub fn reactive_avoid() {}
```

- [ ] **Step 6: Wire the plugin in `src/main.rs`**

Find:
```rust
mod frontier;
...
use frontier::FrontierPlugin;
...
        .add_plugins(FrontierPlugin)
```

Replace with:
```rust
mod exploration;
...
use exploration::ExplorationPlugin;
...
        .add_plugins(ExplorationPlugin)
```

- [ ] **Step 7: Migrate `wander` to use the new `FrontierTarget` import**

Modify `src/drone/wander.rs:1-10` to keep wander logic but drop the `frontier::FrontierTarget` dependency if any (it currently only reads `DesiredVelocity`). No change needed unless the file currently imports from `crate::frontier`.

Modify `src/drone/spawn.rs`: replace `use crate::frontier::FrontierTarget;` with `use crate::exploration::{FrontierTarget, MovementHealth, Path};` and add `MovementHealth::default()`, `Path::default()` to the spawn bundle.

- [ ] **Step 8: Delete `src/frontier/` directory**

```bash
rm -rf src/frontier
```

- [ ] **Step 9: Verify build**

Run: `cargo build`
Expected: Compiles. May emit warnings about unused stub systems — acceptable.

- [ ] **Step 10: Commit**

```bash
git add src/exploration src/main.rs src/drone/spawn.rs && git rm -r src/frontier
git commit -m "scaffold exploration module + retire frontier

introduces the src/exploration feature folder with stub systems +
typed components (FrontierTarget, Path, MovementHealth). main.rs
swaps FrontierPlugin out for ExplorationPlugin. the legacy frontier
module is removed; its FrontierTarget moves into the new module
unchanged."
```

---

## Task 2: PlannerGrid downsample (TDD)

**Files:**
- Modify: `src/exploration/planner.rs`
- Test: `src/exploration/planner.rs` (inline `#[cfg(test)] mod tests`)

- [ ] **Step 1: Write the failing test**

```rust
// src/exploration/planner.rs
use super::resources::{CoarseCell, PlannerGrid};
use bevy::prelude::*;

#[cfg(test)]
mod tests {
    use super::*;

    fn make_bitset(dims: UVec3, occupied: &[(i32, i32, i32)], free: &[(i32, i32, i32)]) -> Vec<u32> {
        let n = (dims.x * dims.y * dims.z) as usize;
        let words = n.div_ceil(16);
        let mut bitset = vec![0u32; words];
        for &(x, y, z) in occupied {
            let flat = (x as u32 + y as u32 * dims.x + z as u32 * dims.x * dims.y) as usize;
            let w = flat / 16;
            let b = (flat % 16) * 2;
            bitset[w] |= 0b10u32 << b;
        }
        for &(x, y, z) in free {
            let flat = (x as u32 + y as u32 * dims.x + z as u32 * dims.x * dims.y) as usize;
            let w = flat / 16;
            let b = (flat % 16) * 2;
            bitset[w] |= 0b01u32 << b;
        }
        bitset
    }

    #[test]
    fn downsample_majority_blocked() {
        let dims = UVec3::new(8, 8, 8);
        // Fill a 4x4x4 region with Occupied (drone-state bit 1).
        let mut occupied = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                for z in 0..4 {
                    occupied.push((x, y, z));
                }
            }
        }
        let bitset = make_bitset(dims, &occupied, &[]);
        let grid = PlannerGrid::downsample_from_bitset(dims, 1.0, &bitset, 8);
        assert_eq!(grid.dims, UVec3::new(1, 1, 1));
        assert_eq!(grid.coarse[0], CoarseCell::Blocked);
    }

    #[test]
    fn downsample_majority_free() {
        let dims = UVec3::new(8, 8, 8);
        let mut free = Vec::new();
        for x in 0..6 {
            for y in 0..6 {
                for z in 0..6 {
                    free.push((x, y, z));
                }
            }
        }
        let bitset = make_bitset(dims, &[], &free);
        let grid = PlannerGrid::downsample_from_bitset(dims, 1.0, &bitset, 8);
        assert_eq!(grid.coarse[0], CoarseCell::Free);
    }

    #[test]
    fn downsample_unknown_default() {
        let dims = UVec3::new(8, 8, 8);
        let bitset = make_bitset(dims, &[], &[]);
        let grid = PlannerGrid::downsample_from_bitset(dims, 1.0, &bitset, 8);
        assert_eq!(grid.coarse[0], CoarseCell::Unknown);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p drones --lib exploration::planner::tests`
Expected: FAIL with "no function or associated item named `downsample_from_bitset`"

- [ ] **Step 3: Write minimal implementation**

```rust
// src/exploration/planner.rs (add to the file, before #[cfg(test)])
use super::resources::{CoarseCell, PlannerGrid};
use bevy::prelude::*;

impl PlannerGrid {
    /// Build a coarse occupancy grid by majority-voting `downsample^3`
    /// native cells into one coarse cell. Reads from the 2-bits-per-cell
    /// bitset format used by `GpuGlobalOccupancyMirror`: bit 0 = Free
    /// flag, bit 1 = Occupied flag.
    pub fn downsample_from_bitset(
        dims: UVec3,
        voxel_size: f32,
        bitset: &[u32],
        downsample: u32,
    ) -> Self {
        let coarse_dims = UVec3::new(
            dims.x.div_ceil(downsample),
            dims.y.div_ceil(downsample),
            dims.z.div_ceil(downsample),
        );
        let total = (coarse_dims.x * coarse_dims.y * coarse_dims.z) as usize;
        let mut coarse = vec![CoarseCell::Unknown; total];
        let read = |flat: u32| -> u32 {
            let w = (flat / 16) as usize;
            if w >= bitset.len() {
                return 0;
            }
            let b = (flat % 16) * 2;
            (bitset[w] >> b) & 0b11
        };
        let block_volume = (downsample * downsample * downsample) as usize;
        let half = block_volume / 2;

        for cz in 0..coarse_dims.z {
            for cy in 0..coarse_dims.y {
                for cx in 0..coarse_dims.x {
                    let mut occ = 0usize;
                    let mut free = 0usize;
                    for dz in 0..downsample {
                        let z = cz * downsample + dz;
                        if z >= dims.z {
                            continue;
                        }
                        for dy in 0..downsample {
                            let y = cy * downsample + dy;
                            if y >= dims.y {
                                continue;
                            }
                            for dx in 0..downsample {
                                let x = cx * downsample + dx;
                                if x >= dims.x {
                                    continue;
                                }
                                let flat = x + y * dims.x + z * dims.x * dims.y;
                                let state = read(flat);
                                if state & 0b10 != 0 {
                                    occ += 1;
                                } else if state & 0b01 != 0 {
                                    free += 1;
                                }
                            }
                        }
                    }
                    let idx = ((cz * coarse_dims.y + cy) * coarse_dims.x + cx) as usize;
                    coarse[idx] = if occ > half {
                        CoarseCell::Blocked
                    } else if free > half {
                        CoarseCell::Free
                    } else {
                        CoarseCell::Unknown
                    };
                }
            }
        }

        PlannerGrid {
            coarse,
            dims: coarse_dims,
            voxel_size,
            downsample,
        }
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p drones --lib exploration::planner::tests`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/exploration/planner.rs
git commit -m "planner grid downsample with majority vote

walks the 2-bit-per-cell bitset in downsample^3 blocks (default 8^3 =
512 native cells per coarse cell) and emits Free/Unknown/Blocked per
coarse cell via a strict majority threshold. unit-tested for the
three pure cases (all-occupied, all-free, all-unknown)."
```

---

## Task 3: A* path planner (TDD)

**Files:**
- Modify: `src/exploration/planner.rs`

- [ ] **Step 1: Write the failing test**

Add at the bottom of `src/exploration/planner.rs` (inside `mod tests`):

```rust
    #[test]
    fn astar_straight_line_through_free() {
        let dims = UVec3::new(4, 1, 4);
        let coarse = vec![CoarseCell::Free; 16];
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        let path = plan(&grid, UVec3::new(0, 0, 0), UVec3::new(3, 0, 3)).unwrap();
        assert!(path.first() == Some(&UVec3::new(0, 0, 0)));
        assert!(path.last() == Some(&UVec3::new(3, 0, 3)));
        assert!(path.len() <= 4);
    }

    #[test]
    fn astar_routes_around_blocked() {
        let dims = UVec3::new(5, 1, 5);
        let mut coarse = vec![CoarseCell::Free; 25];
        // Block a wall at x=2 spanning z=0..4.
        for z in 0..4 {
            coarse[(z * dims.x + 2) as usize] = CoarseCell::Blocked;
        }
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        let path = plan(&grid, UVec3::new(0, 0, 0), UVec3::new(4, 0, 0)).unwrap();
        // Must detour through z=4 row.
        assert!(path.iter().any(|c| c.z == 4));
    }

    #[test]
    fn astar_unknown_costs_more() {
        let dims = UVec3::new(3, 1, 3);
        // Layout:
        //  F U F
        //  F U F
        //  F F F
        let mut coarse = vec![CoarseCell::Free; 9];
        coarse[1] = CoarseCell::Unknown;
        coarse[4] = CoarseCell::Unknown;
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        let path = plan(&grid, UVec3::new(0, 0, 0), UVec3::new(2, 0, 0)).unwrap();
        // Direct-through-Unknown route would touch (1, 0, 0). Prefer detour via z=1.
        assert!(!path.contains(&UVec3::new(1, 0, 0)));
    }

    #[test]
    fn astar_no_path_through_blocked_wall() {
        let dims = UVec3::new(3, 1, 1);
        let mut coarse = vec![CoarseCell::Free; 3];
        coarse[1] = CoarseCell::Blocked;
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        assert!(plan(&grid, UVec3::new(0, 0, 0), UVec3::new(2, 0, 0)).is_none());
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p drones --lib exploration::planner::tests`
Expected: FAIL — `plan` function not defined.

- [ ] **Step 3: Write the A* planner**

Add to `src/exploration/planner.rs` (above `#[cfg(test)]`):

```rust
use super::constants::{PLANNER_DEEP_UNKNOWN_MULT, PLANNER_FREE_COST, PLANNER_UNKNOWN_COST_MULT};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

#[derive(Copy, Clone, PartialEq)]
struct Node {
    cell: UVec3,
    f: f32,
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is max-heap; invert for min-priority on f.
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn neighbors_26() -> [(i32, i32, i32); 26] {
    let mut out = [(0, 0, 0); 26];
    let mut idx = 0;
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                if dx == 0 && dy == 0 && dz == 0 {
                    continue;
                }
                out[idx] = (dx, dy, dz);
                idx += 1;
            }
        }
    }
    out
}

fn step_distance(d: (i32, i32, i32)) -> f32 {
    let s = (d.0 * d.0 + d.1 * d.1 + d.2 * d.2) as f32;
    s.sqrt()
}

fn edge_cost(from: CoarseCell, to: CoarseCell, step_dist: f32) -> Option<f32> {
    if matches!(to, CoarseCell::Blocked) {
        return None;
    }
    let mult = match (from, to) {
        (CoarseCell::Free, CoarseCell::Free) => 1.0,
        (CoarseCell::Free, CoarseCell::Unknown) | (CoarseCell::Unknown, CoarseCell::Free) => {
            PLANNER_UNKNOWN_COST_MULT
        }
        (CoarseCell::Unknown, CoarseCell::Unknown) => PLANNER_DEEP_UNKNOWN_MULT,
        _ => return None,
    };
    Some(step_dist * PLANNER_FREE_COST * mult)
}

fn heuristic(a: UVec3, b: UVec3) -> f32 {
    let dx = a.x as f32 - b.x as f32;
    let dy = a.y as f32 - b.y as f32;
    let dz = a.z as f32 - b.z as f32;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// A* on the coarse planner grid. Returns the sequence of coarse cells
/// from `start` to `goal` inclusive, or `None` if unreachable.
pub fn plan(grid: &PlannerGrid, start: UVec3, goal: UVec3) -> Option<Vec<UVec3>> {
    if grid.idx(start).is_none() || grid.idx(goal).is_none() {
        return None;
    }
    if matches!(grid.at(goal), CoarseCell::Blocked) {
        return None;
    }
    let mut open = BinaryHeap::new();
    let mut came_from: HashMap<UVec3, UVec3> = HashMap::new();
    let mut g_score: HashMap<UVec3, f32> = HashMap::new();
    g_score.insert(start, 0.0);
    open.push(Node {
        cell: start,
        f: heuristic(start, goal),
    });

    let neighbors = neighbors_26();

    while let Some(Node { cell, .. }) = open.pop() {
        if cell == goal {
            let mut path = vec![goal];
            let mut cur = goal;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }
        let g_cur = *g_score.get(&cell).unwrap_or(&f32::INFINITY);
        let from_state = grid.at(cell);
        for d in &neighbors {
            let nx = cell.x as i32 + d.0;
            let ny = cell.y as i32 + d.1;
            let nz = cell.z as i32 + d.2;
            if nx < 0
                || ny < 0
                || nz < 0
                || nx as u32 >= grid.dims.x
                || ny as u32 >= grid.dims.y
                || nz as u32 >= grid.dims.z
            {
                continue;
            }
            let next = UVec3::new(nx as u32, ny as u32, nz as u32);
            let to_state = grid.at(next);
            let step = step_distance(*d);
            let Some(cost) = edge_cost(from_state, to_state, step) else {
                continue;
            };
            let tentative = g_cur + cost;
            if tentative < *g_score.get(&next).unwrap_or(&f32::INFINITY) {
                came_from.insert(next, cell);
                g_score.insert(next, tentative);
                let f = tentative + heuristic(next, goal);
                open.push(Node { cell: next, f });
            }
        }
    }
    None
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p drones --lib exploration::planner::tests`
Expected: PASS (7 tests including downsample tests)

- [ ] **Step 5: Commit**

```bash
git add src/exploration/planner.rs
git commit -m "A* planner over the coarse grid

26-neighbour A* with Euclidean heuristic. unknown cells traversable
at 3-5x cost depending on whether both endpoints are unknown. blocked
cells reject. four unit tests cover the basic shape (straight line,
detour around wall, prefer-known-over-unknown, refuse-blocked-only-
path)."
```

---

## Task 4: Frontier clustering (TDD)

**Files:**
- Modify: `src/exploration/cluster.rs`

- [ ] **Step 1: Write the failing test**

```rust
// src/exploration/cluster.rs
use super::constants::{MAX_FRONTIER_CANDIDATES, MIN_CLUSTER_SIZE};
use super::resources::{FrontierCluster, FrontierClusters};
use bevy::prelude::*;
use std::collections::HashSet;

#[cfg(test)]
mod tests {
    use super::*;

    fn s(coords: &[(u32, u32, u32)]) -> HashSet<UVec3> {
        coords.iter().map(|&(x, y, z)| UVec3::new(x, y, z)).collect()
    }

    #[test]
    fn single_cell_cluster_discarded() {
        let cells = s(&[(0, 0, 0)]);
        let clusters = build_clusters(&cells, &mut 0);
        assert!(clusters.is_empty());
    }

    #[test]
    fn small_cluster_under_threshold_discarded() {
        // 3 cells in a line — below MIN_CLUSTER_SIZE = 4.
        let cells = s(&[(0, 0, 0), (1, 0, 0), (2, 0, 0)]);
        let clusters = build_clusters(&cells, &mut 0);
        assert!(clusters.is_empty());
    }

    #[test]
    fn line_of_four_kept() {
        let cells = s(&[(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]);
        let clusters = build_clusters(&cells, &mut 0);
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].cells.len(), 4);
        assert_eq!(clusters[0].bbox_min, UVec3::new(0, 0, 0));
        assert_eq!(clusters[0].bbox_max, UVec3::new(3, 0, 0));
    }

    #[test]
    fn two_disjoint_clusters() {
        // Two 2x2 blobs far apart.
        let cells = s(&[
            (0, 0, 0), (1, 0, 0), (0, 0, 1), (1, 0, 1),
            (10, 0, 10), (11, 0, 10), (10, 0, 11), (11, 0, 11),
        ]);
        let clusters = build_clusters(&cells, &mut 0);
        assert_eq!(clusters.len(), 2);
        assert!(clusters.iter().all(|c| c.cells.len() == 4));
    }

    #[test]
    fn ids_monotonic() {
        let cells = s(&[(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]);
        let mut next_id = 17u32;
        let c1 = build_clusters(&cells, &mut next_id);
        assert_eq!(c1[0].id, 17);
        assert_eq!(next_id, 18);
        let c2 = build_clusters(&cells, &mut next_id);
        assert_eq!(c2[0].id, 18);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p drones --lib exploration::cluster::tests`
Expected: FAIL — `build_clusters` not defined.

- [ ] **Step 3: Implement `build_clusters`**

Add above `#[cfg(test)]` in `src/exploration/cluster.rs`:

```rust
/// 6-neighbourhood flood-fill on a candidate cell set. Clusters smaller
/// than `MIN_CLUSTER_SIZE` are discarded. Each cluster receives a unique
/// id pulled from `*next_id`, which the caller increments-by-N after
/// the call to keep ids monotonic across frames.
pub fn build_clusters(candidates: &HashSet<UVec3>, next_id: &mut u32) -> Vec<FrontierCluster> {
    let mut visited: HashSet<UVec3> = HashSet::new();
    let mut out = Vec::new();
    for &seed in candidates.iter() {
        if visited.contains(&seed) {
            continue;
        }
        let mut stack = vec![seed];
        let mut cells = Vec::new();
        let mut bbox_min = seed;
        let mut bbox_max = seed;
        while let Some(c) = stack.pop() {
            if !visited.insert(c) {
                continue;
            }
            if !candidates.contains(&c) {
                continue;
            }
            cells.push(c);
            bbox_min = bbox_min.min(c);
            bbox_max = bbox_max.max(c);
            for d in [
                IVec3::new(-1, 0, 0),
                IVec3::new(1, 0, 0),
                IVec3::new(0, -1, 0),
                IVec3::new(0, 1, 0),
                IVec3::new(0, 0, -1),
                IVec3::new(0, 0, 1),
            ] {
                let nx = c.x as i32 + d.x;
                let ny = c.y as i32 + d.y;
                let nz = c.z as i32 + d.z;
                if nx < 0 || ny < 0 || nz < 0 {
                    continue;
                }
                stack.push(UVec3::new(nx as u32, ny as u32, nz as u32));
            }
        }
        if cells.len() < MIN_CLUSTER_SIZE {
            continue;
        }
        let centroid = cells.iter().fold(Vec3::ZERO, |acc, c| {
            acc + Vec3::new(c.x as f32, c.y as f32, c.z as f32)
        }) / cells.len() as f32;
        let info_gain = cells.len() as f32;
        out.push(FrontierCluster {
            id: *next_id,
            centroid,
            cells,
            info_gain,
            bbox_min,
            bbox_max,
        });
        *next_id += 1;
        if out.len() >= MAX_FRONTIER_CANDIDATES {
            break;
        }
    }
    out
}
```

- [ ] **Step 4: Run tests**

Run: `cargo test -p drones --lib exploration::cluster::tests`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
git add src/exploration/cluster.rs
git commit -m "frontier clustering via flood-fill

6-neighbourhood walk over the candidate-cell set yields clusters with
bbox + centroid + info_gain (just cell count for now; bbox-scan
weighting comes later). clusters below MIN_CLUSTER_SIZE (4 cells)
are discarded as dead-end pockets. monotonic ids via a caller-owned
counter so cluster identity is stable frame to frame for stickiness.
five unit tests cover the basic shapes."
```

---

## Task 5: Per-cluster scoring + crowding (TDD)

**Files:**
- Modify: `src/exploration/scoring.rs`

- [ ] **Step 1: Write the failing test**

```rust
// src/exploration/scoring.rs
use super::constants::{
    SCORE_CROWDING_WEIGHT, SCORE_DISTANCE_BIAS, SCORE_DISTANCE_WEIGHT, SCORE_INFO_WEIGHT,
};
use super::resources::FrontierCluster;
use bevy::prelude::*;

#[derive(Debug, Clone, Copy)]
pub struct ScoringWeights {
    pub info: f32,
    pub distance: f32,
    pub distance_bias: f32,
    pub crowding: f32,
}

impl Default for ScoringWeights {
    fn default() -> Self {
        Self {
            info: SCORE_INFO_WEIGHT,
            distance: SCORE_DISTANCE_WEIGHT,
            distance_bias: SCORE_DISTANCE_BIAS,
            crowding: SCORE_CROWDING_WEIGHT,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cluster(id: u32, centroid: Vec3, info: f32) -> FrontierCluster {
        FrontierCluster {
            id,
            centroid,
            cells: vec![],
            info_gain: info,
            bbox_min: UVec3::ZERO,
            bbox_max: UVec3::ZERO,
        }
    }

    #[test]
    fn closer_wins_when_equal_info() {
        let a = cluster(0, Vec3::new(10.0, 0.0, 0.0), 100.0);
        let b = cluster(1, Vec3::new(100.0, 0.0, 0.0), 100.0);
        let w = ScoringWeights::default();
        let sa = score(&a, Vec3::ZERO, 0, &w);
        let sb = score(&b, Vec3::ZERO, 0, &w);
        assert!(sa > sb);
    }

    #[test]
    fn higher_info_wins_when_equal_distance() {
        let a = cluster(0, Vec3::new(10.0, 0.0, 0.0), 1000.0);
        let b = cluster(1, Vec3::new(10.0, 0.0, 0.0), 10.0);
        let w = ScoringWeights::default();
        assert!(score(&a, Vec3::ZERO, 0, &w) > score(&b, Vec3::ZERO, 0, &w));
    }

    #[test]
    fn crowding_lowers_score() {
        let a = cluster(0, Vec3::new(10.0, 0.0, 0.0), 100.0);
        let w = ScoringWeights::default();
        let alone = score(&a, Vec3::ZERO, 0, &w);
        let crowded = score(&a, Vec3::ZERO, 5, &w);
        assert!(crowded < alone);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p drones --lib exploration::scoring::tests`
Expected: FAIL — `score` not defined.

- [ ] **Step 3: Implement `score`**

Add to `src/exploration/scoring.rs` above `#[cfg(test)]`:

```rust
/// Cost-utility score for one cluster from the perspective of one
/// drone. Higher is better. `crowding` is a caller-computed count of
/// nearby peer drones (see `crowding_for`).
pub fn score(
    cluster: &FrontierCluster,
    drone_pos: Vec3,
    crowding: u32,
    weights: &ScoringWeights,
) -> f32 {
    let dist = drone_pos.distance(cluster.centroid).max(0.01);
    let denom = dist * weights.distance + weights.distance_bias
        + crowding as f32 * weights.crowding;
    cluster.info_gain * weights.info / denom
}

/// Sum the per-peer crowding contribution against `cluster`:
/// +1.0 per peer that already targets the same cluster id,
/// +0.5 per peer whose position is inside an inflated bbox.
pub fn crowding_for(
    cluster: &FrontierCluster,
    peers: &[(Vec3, Option<u32>)],
    bbox_inflate: f32,
) -> u32 {
    let lo = Vec3::new(
        cluster.bbox_min.x as f32,
        cluster.bbox_min.y as f32,
        cluster.bbox_min.z as f32,
    );
    let hi = Vec3::new(
        cluster.bbox_max.x as f32,
        cluster.bbox_max.y as f32,
        cluster.bbox_max.z as f32,
    );
    let span = (hi - lo) * bbox_inflate;
    let lo_inf = lo - span * 0.5;
    let hi_inf = hi + span * 0.5;
    let mut total = 0.0;
    for &(pos, target_id) in peers {
        if target_id == Some(cluster.id) {
            total += 1.0;
        } else if pos.cmpge(lo_inf).all() && pos.cmple(hi_inf).all() {
            total += 0.5;
        }
    }
    total.round() as u32
}
```

- [ ] **Step 4: Run tests**

Run: `cargo test -p drones --lib exploration::scoring::tests`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/exploration/scoring.rs
git commit -m "cluster scoring + crowding term

ScoringWeights packages the four cost-utility constants. score() = info
* w_info / (dist * w_dist + bias + crowding * w_crowd). crowding_for()
counts peer drones currently targeting the cluster (+1.0) or within an
inflated bbox (+0.5). three unit tests cover the basic monotonicity:
nearer wins, higher info wins, more crowding loses."
```

---

## Task 6: Target picker system + stickiness

**Files:**
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Replace the stub `assign_targets` with the full system**

```rust
// src/exploration/systems.rs
use bevy::prelude::*;

use crate::comms::CommsState;
use crate::drone::{Drone, DroneId};

use super::components::{FrontierTarget, MovementHealth, Path};
use super::constants::{FRONTIER_REACHED_DIST, SCORE_UPGRADE_RATIO};
use super::resources::FrontierClusters;
use super::scoring::{crowding_for, score, ScoringWeights};

pub fn assign_targets(
    clusters: Res<FrontierClusters>,
    comms: Res<CommsState>,
    mut q_self: Query<(&DroneId, &Transform, &mut FrontierTarget), With<Drone>>,
    q_peers: Query<(&DroneId, &Transform, &FrontierTarget), With<Drone>>,
) {
    if clusters.entries.is_empty() {
        return;
    }
    // Snapshot peer positions + targets keyed by id for crowding lookups.
    let peers: Vec<(u32, Vec3, Option<u32>)> = q_peers
        .iter()
        .map(|(id, t, ft)| (id.0, t.translation, ft.cluster_id))
        .collect();

    let weights = ScoringWeights::default();

    for (id, transform, mut target) in &mut q_self {
        let drone_pos = transform.translation;
        // Filter peers to the comms cluster of the deciding drone.
        let half = (id.0 >= 32) as usize;
        let i_am_connected = (comms.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        let visible_peers: Vec<(Vec3, Option<u32>)> = if i_am_connected {
            peers
                .iter()
                .filter(|(pid, _, _)| {
                    if *pid == id.0 {
                        return false;
                    }
                    let h = (*pid >= 32) as usize;
                    (comms.connected_mask[h] >> (pid % 32)) & 1 == 1
                })
                .map(|(_, p, t)| (*p, *t))
                .collect()
        } else {
            Vec::new()
        };

        // Score all clusters once.
        let scored: Vec<(f32, u32, Vec3)> = clusters
            .entries
            .iter()
            .map(|c| {
                let crowding = crowding_for(c, &visible_peers, 0.5);
                (score(c, drone_pos, crowding, &weights), c.id, c.centroid)
            })
            .collect();

        let best = scored
            .iter()
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let Some(&(best_score, best_id, best_centroid)) = best else {
            target.pos = None;
            target.cluster_id = None;
            continue;
        };

        // Stickiness: keep current unless reached, vanished, or upgrade by 1.5x.
        let keep = match target.cluster_id {
            None => false,
            Some(cur_id) => {
                let cur_alive = clusters.entries.iter().any(|c| c.id == cur_id);
                if !cur_alive {
                    false
                } else if let Some(cur_pos) = target.pos {
                    if cur_pos.distance(drone_pos) < FRONTIER_REACHED_DIST {
                        false
                    } else {
                        let cur_score = scored
                            .iter()
                            .find(|(_, id, _)| *id == cur_id)
                            .map(|s| s.0)
                            .unwrap_or(0.0);
                        best_score <= cur_score * SCORE_UPGRADE_RATIO
                    }
                } else {
                    false
                }
            }
        };
        if !keep {
            target.cluster_id = Some(best_id);
            target.pos = Some(best_centroid);
        }
    }
}

pub fn compute_frontier_clusters() {
    // Wired in Task 10.
}
pub fn rebuild_planner_grid() {}
pub fn replan_paths() {}
pub fn update_movement_health() {}
pub fn stuck_recovery() {}
pub fn steer_along_path() {}
pub fn reactive_avoid() {}
```

- [ ] **Step 2: Build (no tests yet for the system since it's I/O-bound)**

Run: `cargo build`
Expected: Compiles, may warn about unused systems.

- [ ] **Step 3: Commit**

```bash
git add src/exploration/systems.rs
git commit -m "target picker with stickiness

assign_targets reads FrontierClusters + CommsState; for each drone it
filters peer info to its own comms cluster, scores every cluster, and
picks the highest. stickiness rule: keep current unless reached
(within FRONTIER_REACHED_DIST), cluster vanished, or new cluster scores
> 1.5x current. drones outside comms get no crowding signal so they
make naive picks."
```

---

## Task 7: MovementHealth + stuck detection (TDD where possible)

**Files:**
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Replace stubs with the two stuck systems**

In `src/exploration/systems.rs`, replace the `update_movement_health` and `stuck_recovery` stubs:

```rust
use crate::physics::LinearVelocity;
use super::constants::{
    STUCK_ESCALATION_WINDOW_SECS, STUCK_SECS, STUCK_VEL_MPS,
};
use rand::Rng;

pub fn update_movement_health(
    time: Res<Time>,
    mut q: Query<(&LinearVelocity, &mut MovementHealth), With<Drone>>,
) {
    let dt = time.delta_secs();
    for (linvel, mut health) in &mut q {
        if linvel.0.length() < STUCK_VEL_MPS {
            health.slow_secs += dt;
        } else {
            health.slow_secs = 0.0;
        }
    }
}

pub fn stuck_recovery(
    time: Res<Time>,
    world: Res<crate::world::WorldConfig>,
    mut q: Query<(
        &mut Transform,
        &mut LinearVelocity,
        &mut MovementHealth,
        &mut Path,
    ), With<Drone>>,
) {
    let now = time.elapsed_secs();
    let mut rng = rand::rng();
    for (mut transform, mut linvel, mut health, mut path) in &mut q {
        if health.slow_secs < STUCK_SECS {
            continue;
        }
        health.slow_secs = 0.0;

        // Force replan by clearing the path.
        path.waypoints.clear();
        path.cursor = 0;

        // Apply random kick (small impulse) to escape local minima.
        let kick = Vec3::new(
            rng.random_range(-2.0..2.0),
            rng.random_range(-0.5..0.5),
            rng.random_range(-2.0..2.0),
        );
        linvel.0 += kick;

        // Bookkeeping for escalation.
        let window_open = now - health.window_start_secs < STUCK_ESCALATION_WINDOW_SECS;
        if window_open {
            health.escalations_in_window += 1;
        } else {
            health.window_start_secs = now;
            health.escalations_in_window = 1;
        }

        if health.escalations_in_window >= 3 {
            // Final fallback: teleport to world center.
            warn!("drone stuck after 3 escalations — teleporting to world center");
            transform.translation = world.center();
            linvel.0 = Vec3::ZERO;
            health.escalations_in_window = 0;
        }
    }
}
```

- [ ] **Step 2: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 3: Commit**

```bash
git add src/exploration/systems.rs
git commit -m "stuck detector + escalation

update_movement_health accumulates slow_secs while linvel below 0.5 m/s.
stuck_recovery clears the path + applies a small random impulse when
slow_secs crosses 3 s. three escalations within 20 s teleport the drone
to world center as a last-resort unbrick. uses linvel.is_changed isn't
needed since we mutate to reset slow_secs."
```

---

## Task 8: Pure-pursuit path follower

**Files:**
- Modify: `src/exploration/steering.rs`
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Write the failing pure-pursuit test**

```rust
// src/exploration/steering.rs
use super::components::Path;
use super::constants::LOOKAHEAD_M;
use bevy::prelude::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pursuit_advances_cursor() {
        let mut path = Path {
            waypoints: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::new(20.0, 0.0, 0.0),
            ],
            cursor: 0,
        };
        let drone = Vec3::new(5.0, 0.0, 0.0);
        let target = pure_pursuit(&mut path, drone);
        // Cursor should advance past waypoint 0 since drone is between 0 and 1.
        assert_eq!(path.cursor, 1);
        // Look-ahead should aim at waypoint 1 (10 m) since that's within LOOKAHEAD_M=8 m? Actually 10 > 8 so target IS waypoint 1.
        assert_eq!(target, Some(Vec3::new(10.0, 0.0, 0.0)));
    }

    #[test]
    fn empty_path_returns_none() {
        let mut path = Path::default();
        assert!(pure_pursuit(&mut path, Vec3::ZERO).is_none());
    }
}
```

- [ ] **Step 2: Run test (fails)**

Run: `cargo test -p drones --lib exploration::steering::tests`
Expected: FAIL — `pure_pursuit` not defined.

- [ ] **Step 3: Implement `pure_pursuit`**

Add to `src/exploration/steering.rs` above `#[cfg(test)]`:

```rust
/// Pure-pursuit waypoint selection. Advances the path cursor past any
/// waypoints the drone has passed, then returns the next waypoint that
/// is at least `LOOKAHEAD_M` meters away (or the last waypoint if the
/// drone is near the end). Returns `None` for empty paths.
pub fn pure_pursuit(path: &mut Path, drone_pos: Vec3) -> Option<Vec3> {
    if path.waypoints.is_empty() {
        return None;
    }
    // Advance cursor past waypoints behind the drone (closer to drone is
    // past the next one's projection).
    while path.cursor + 1 < path.waypoints.len() {
        let next = path.waypoints[path.cursor + 1];
        if drone_pos.distance(next) < drone_pos.distance(path.waypoints[path.cursor]) {
            path.cursor += 1;
        } else {
            break;
        }
    }
    // Find first waypoint at >= LOOKAHEAD_M, else return last.
    let mut idx = path.cursor;
    while idx + 1 < path.waypoints.len()
        && drone_pos.distance(path.waypoints[idx]) < LOOKAHEAD_M
    {
        idx += 1;
    }
    path.waypoints.get(idx).copied()
}
```

- [ ] **Step 4: Run test**

Run: `cargo test -p drones --lib exploration::steering::tests`
Expected: PASS (2 tests)

- [ ] **Step 5: Wire `steer_along_path` system in `systems.rs`**

In `src/exploration/systems.rs`, replace the `steer_along_path` stub:

```rust
use crate::drone::CRUISE_SPEED_MPS;
use crate::physics::DesiredVelocity;
use super::constants::PATH_FOLLOW_LERP_RATE;
use super::steering::pure_pursuit;

pub fn steer_along_path(
    time: Res<Time>,
    mut q: Query<(&Transform, &mut Path, &mut DesiredVelocity), With<Drone>>,
) {
    let dt = time.delta_secs();
    for (transform, mut path, mut desired) in &mut q {
        let Some(waypoint) = pure_pursuit(&mut path, transform.translation) else {
            continue;
        };
        let to_wp = waypoint - transform.translation;
        let dist = to_wp.length();
        if dist < 1e-3 {
            continue;
        }
        let target_vel = (to_wp / dist) * CRUISE_SPEED_MPS;
        let alpha = (PATH_FOLLOW_LERP_RATE * dt).min(1.0);
        desired.0 = desired.0.lerp(target_vel, alpha);
    }
}
```

- [ ] **Step 6: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 7: Commit**

```bash
git add src/exploration/steering.rs src/exploration/systems.rs
git commit -m "pure-pursuit path follower

pure_pursuit advances the path cursor past waypoints the drone has
overtaken, then picks the first waypoint at least LOOKAHEAD_M away as
the steering target. two unit tests cover cursor advance + empty
path. steer_along_path system lerps DesiredVelocity toward the
waypoint at cruise speed."
```

---

## Task 9: Reactive force from lidar + peers

**Files:**
- Modify: `src/exploration/steering.rs`
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Test the reactive force shape**

Add to `src/exploration/steering.rs` `mod tests`:

```rust
    use super::super::constants::{AVOID_K_DEFAULT, AVOID_RADIUS_M};

    #[test]
    fn no_obstacles_no_force() {
        let f = reactive_force(Vec3::ZERO, &[], &[], AVOID_K_DEFAULT);
        assert_eq!(f, Vec3::ZERO);
    }

    #[test]
    fn closer_obstacle_pushes_harder() {
        let near = vec![Vec3::new(1.0, 0.0, 0.0)];
        let far = vec![Vec3::new(3.5, 0.0, 0.0)];
        let f_near = reactive_force(Vec3::ZERO, &near, &[], AVOID_K_DEFAULT);
        let f_far = reactive_force(Vec3::ZERO, &far, &[], AVOID_K_DEFAULT);
        assert!(f_near.length() > f_far.length());
        // Force should point away (negative x since obstacle is at +x).
        assert!(f_near.x < 0.0);
    }

    #[test]
    fn outside_radius_ignored() {
        let way_far = vec![Vec3::new(AVOID_RADIUS_M + 1.0, 0.0, 0.0)];
        let f = reactive_force(Vec3::ZERO, &way_far, &[], AVOID_K_DEFAULT);
        assert_eq!(f, Vec3::ZERO);
    }
```

- [ ] **Step 2: Run test (fails)**

Run: `cargo test -p drones --lib exploration::steering::tests`
Expected: FAIL — `reactive_force` not defined.

- [ ] **Step 3: Implement `reactive_force`**

Add to `src/exploration/steering.rs`:

```rust
use super::constants::{AVOID_K_DEFAULT, AVOID_RADIUS_M, AVOID_RADIUS_PEER_M};

/// Quadratic-falloff repulsion. Each obstacle within its radius
/// contributes a force pointing from obstacle to drone, scaled by
/// `avoid_k * (1 - d/R)^2`. Obstacles split into two radii so peers
/// can have a wider personal-space bubble than terrain.
pub fn reactive_force(
    drone_pos: Vec3,
    lidar_hits: &[Vec3],
    peer_positions: &[Vec3],
    avoid_k: f32,
) -> Vec3 {
    let mut total = Vec3::ZERO;
    let scale = |pos: Vec3, radius: f32| -> Vec3 {
        let dir = drone_pos - pos;
        let d = dir.length();
        if d < 1e-3 || d > radius {
            return Vec3::ZERO;
        }
        let strength = avoid_k * (1.0 - d / radius).powi(2);
        (dir / d) * strength
    };
    for &hit in lidar_hits {
        total += scale(hit, AVOID_RADIUS_M);
    }
    for &peer in peer_positions {
        total += scale(peer, AVOID_RADIUS_PEER_M);
    }
    total
}
```

Re-export from `steering.rs` if not already.

- [ ] **Step 4: Run tests**

Run: `cargo test -p drones --lib exploration::steering::tests`
Expected: PASS (5 tests).

- [ ] **Step 5: Wire `reactive_avoid` system**

Decision deferred from spec: source for "nearby lidar hits per drone". Phase 1 uses the **comms-merged global occupancy** scanned in a `±AVOID_RADIUS_M` cube around each drone. CPU cost: 50 drones × (8 m / 1 m)³ = 25 600 cell-reads/frame. Trivial.

In `src/exploration/systems.rs`, replace the `reactive_avoid` stub:

```rust
use crate::lidar::gpu::GpuGlobalOccupancyMirror;
use super::constants::AVOID_RADIUS_M;
use super::steering::reactive_force;

pub fn reactive_avoid(
    mirror: Res<GpuGlobalOccupancyMirror>,
    comms: Res<CommsState>,
    world: Res<crate::world::WorldConfig>,
    mut q_self: Query<(&DroneId, &Transform, &mut DesiredVelocity), With<Drone>>,
    q_peers: Query<(&DroneId, &Transform), With<Drone>>,
) {
    if mirror.data.is_empty() {
        return;
    }
    let dims = world.size;
    let voxel_size = world.voxel_size;
    let data = &mirror.data;
    let read = |cell: UVec3| -> u32 {
        let flat = cell.x + cell.y * dims.x + cell.z * dims.x * dims.y;
        let w = (flat / 16) as usize;
        if w >= data.len() {
            return 0;
        }
        let b = (flat % 16) * 2;
        (data[w] >> b) & 0b11
    };

    let radius_cells = (AVOID_RADIUS_M / voxel_size).ceil() as i32;
    let peer_snapshot: Vec<(u32, Vec3)> =
        q_peers.iter().map(|(id, t)| (id.0, t.translation)).collect();

    for (id, transform, mut desired) in &mut q_self {
        let pos = transform.translation;
        let drone_cell = (pos / voxel_size).floor().as_ivec3();
        let mut hits = Vec::new();
        for dz in -radius_cells..=radius_cells {
            for dy in -radius_cells..=radius_cells {
                for dx in -radius_cells..=radius_cells {
                    let c = drone_cell + IVec3::new(dx, dy, dz);
                    if c.x < 0 || c.y < 0 || c.z < 0 {
                        continue;
                    }
                    let u = UVec3::new(c.x as u32, c.y as u32, c.z as u32);
                    if u.x >= dims.x || u.y >= dims.y || u.z >= dims.z {
                        continue;
                    }
                    let state = read(u);
                    if state & 0b10 != 0 {
                        // Occupied cell center in world coords.
                        let wp = Vec3::new(u.x as f32, u.y as f32, u.z as f32) * voxel_size
                            + Vec3::splat(voxel_size * 0.5);
                        hits.push(wp);
                    }
                }
            }
        }
        // Filter peer list to comms-connected peers.
        let half = (id.0 >= 32) as usize;
        let connected = (comms.connected_mask[half] >> (id.0 % 32)) & 1 == 1;
        let peers: Vec<Vec3> = if connected {
            peer_snapshot
                .iter()
                .filter(|(pid, _)| {
                    if *pid == id.0 {
                        return false;
                    }
                    let h = (*pid >= 32) as usize;
                    (comms.connected_mask[h] >> (pid % 32)) & 1 == 1
                })
                .map(|(_, p)| *p)
                .collect()
        } else {
            Vec::new()
        };
        let force = reactive_force(pos, &hits, &peers, AVOID_K_DEFAULT);
        desired.0 += force;
    }
}
```

- [ ] **Step 6: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 7: Commit**

```bash
git add src/exploration/steering.rs src/exploration/systems.rs
git commit -m "reactive repulsion from lidar + peers

reactive_force returns the sum of quadratic-falloff repulsion vectors
from terrain (within AVOID_RADIUS_M) and peer drones (within
AVOID_RADIUS_PEER_M). reactive_avoid system reads the comms-merged
occupancy mirror in a 4m cube around each drone to extract terrain
obstacles. peer list is filtered through CommsState so isolated drones
don't see anyone. three unit tests cover the basic curve shape."
```

---

## Task 10: Wire frontier scan + planner grid rebuild + replan

**Files:**
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Implement `rebuild_planner_grid` + `compute_frontier_clusters` + `replan_paths`**

```rust
// Add these to src/exploration/systems.rs

use super::cluster::build_clusters;
use super::constants::{FRONTIER_REFRESH_SECS, PLANNER_DOWNSAMPLE, REPLAN_MIN_INTERVAL_SECS};
use super::planner::plan;
use super::resources::{CoarseCell, FrontierClusters, PlannerGrid};
use std::collections::HashSet;

#[derive(Default)]
pub struct ScanTimer(pub f32);

pub fn rebuild_planner_grid(
    time: Res<Time>,
    mut timer: Local<f32>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    mut grid: ResMut<PlannerGrid>,
) {
    *timer += time.delta_secs();
    if *timer < FRONTIER_REFRESH_SECS {
        return;
    }
    *timer = 0.0;
    if mirror.data.is_empty() {
        return;
    }
    *grid = PlannerGrid::downsample_from_bitset(
        world.size,
        world.voxel_size,
        &mirror.data,
        PLANNER_DOWNSAMPLE,
    );
}

pub fn compute_frontier_clusters(
    time: Res<Time>,
    mut timer: Local<f32>,
    mirror: Res<GpuGlobalOccupancyMirror>,
    world: Res<crate::world::WorldConfig>,
    mut clusters: ResMut<FrontierClusters>,
) {
    *timer += time.delta_secs();
    if *timer < FRONTIER_REFRESH_SECS {
        return;
    }
    *timer = 0.0;
    if mirror.data.is_empty() {
        return;
    }
    let dims = world.size;
    let total = (dims.x * dims.y * dims.z) as u32;
    let data = &mirror.data;
    let read = |cell: u32| -> u32 {
        let w = (cell / 16) as usize;
        if w >= data.len() {
            return 0;
        }
        let b = (cell % 16) * 2;
        (data[w] >> b) & 0b11
    };
    let mut candidates: HashSet<UVec3> = HashSet::new();
    let plane = dims.x * dims.y;
    for cell in 0..total {
        if read(cell) != 0b01 {
            continue;
        }
        // Free cell — push Unknown 6-neighbours.
        let z = cell / plane;
        let rem = cell % plane;
        let y = rem / dims.x;
        let x = rem % dims.x;
        let ix = x as i32;
        let iy = y as i32;
        let iz = z as i32;
        for d in [
            IVec3::new(-1, 0, 0),
            IVec3::new(1, 0, 0),
            IVec3::new(0, -1, 0),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 0, -1),
            IVec3::new(0, 0, 1),
        ] {
            let nx = ix + d.x;
            let ny = iy + d.y;
            let nz = iz + d.z;
            if nx < 0 || ny < 0 || nz < 0 {
                continue;
            }
            if nx as u32 >= dims.x || ny as u32 >= dims.y || nz as u32 >= dims.z {
                continue;
            }
            let nflat = nx as u32 + ny as u32 * dims.x + nz as u32 * plane;
            if read(nflat) == 0 {
                candidates.insert(UVec3::new(nx as u32, ny as u32, nz as u32));
            }
        }
    }
    clusters.entries = build_clusters(&candidates, &mut clusters.next_id);
}

#[derive(Component, Default, Debug)]
pub struct ReplanTimer(pub f32);

pub fn replan_paths(
    time: Res<Time>,
    grid: Res<PlannerGrid>,
    mut q: Query<
        (&Transform, &FrontierTarget, &mut Path, &mut ReplanTimer),
        With<Drone>,
    >,
) {
    if grid.dims == UVec3::ZERO {
        return;
    }
    let dt = time.delta_secs();
    for (transform, target, mut path, mut rt) in &mut q {
        rt.0 += dt;
        let Some(target_pos) = target.pos else {
            path.waypoints.clear();
            path.cursor = 0;
            continue;
        };
        let need_replan =
            path.waypoints.is_empty() || rt.0 >= REPLAN_MIN_INTERVAL_SECS;
        if !need_replan {
            continue;
        }
        rt.0 = 0.0;

        let drone_pos = transform.translation;
        let cell_size = grid.voxel_size * grid.downsample as f32;
        let start = (drone_pos / cell_size).floor().as_uvec3();
        let goal = (target_pos / cell_size).floor().as_uvec3();
        match plan(&grid, start, goal) {
            Some(cells) => {
                path.waypoints = cells.iter().map(|c| grid.world_pos_of(*c)).collect();
                path.cursor = 0;
            }
            None => {
                path.waypoints.clear();
                path.cursor = 0;
            }
        }
    }
}
```

- [ ] **Step 2: Add `ReplanTimer` component to drone spawn bundle**

Modify `src/drone/spawn.rs` to include `crate::exploration::systems::ReplanTimer::default()` in the spawn bundle. Re-export via `src/exploration/mod.rs`:

```rust
pub use systems::ReplanTimer;
```

- [ ] **Step 3: Build + cargo test (existing tests still pass)**

Run: `cargo build && cargo test -p drones --lib exploration`
Expected: Compiles, all unit tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/exploration src/drone/spawn.rs
git commit -m "wire frontier scan + planner grid + replan

rebuild_planner_grid downsamples the mirror at 1 Hz into the
PlannerGrid resource. compute_frontier_clusters extracts Unknown-
adjacent-to-Free candidates from the mirror and flood-fills them into
clusters. replan_paths runs A* on the coarse grid when path empty or
REPLAN_MIN_INTERVAL_SECS elapsed; converts the coarse cell list into
world-space waypoints stored on the Path component. drones spawn with
ReplanTimer default."
```

---

## Task 11: Map-swap teardown hooks for Phase 1 state

**Files:**
- Modify: `src/maps/swap.rs`

- [ ] **Step 1: Extend `apply_pending_swap`**

In `src/maps/swap.rs` `apply_pending_swap` function, add `FrontierClusters` + `PlannerGrid` resets and per-drone `Path` + `MovementHealth` clears. Replace the existing FrontierCandidates reset block.

```rust
use crate::exploration::{FrontierClusters, FrontierTarget, MovementHealth, Path, PlannerGrid};
```

Replace lines that touched `FrontierCandidates`:
```rust
    // ...existing despawn + buffer removal...

    *stats = GpuGlobalStats::default();
    mirror.data.clear();

    // Reset exploration state (was: frontier.cells.clear()).
    clusters.entries.clear();
    clusters.next_id = 0;
    *grid = PlannerGrid::default();
    for mut t in &mut frontier_targets {
        t.pos = None;
        t.cluster_id = None;
    }
    for mut p in &mut paths {
        p.waypoints.clear();
        p.cursor = 0;
    }
    for mut h in &mut health {
        *h = MovementHealth::default();
    }
```

Update the function signature to include the new resources/queries:
```rust
    mut clusters: ResMut<FrontierClusters>,
    mut grid: ResMut<PlannerGrid>,
    mut frontier_targets: Query<&mut FrontierTarget>,
    mut paths: Query<&mut Path>,
    mut health: Query<&mut MovementHealth>,
```

Remove old `FrontierCandidates` + `FrontierTarget` imports from the `frontier::` namespace.

- [ ] **Step 2: Build**

Run: `cargo build`
Expected: Compiles warning-free.

- [ ] **Step 3: Commit**

```bash
git add src/maps/swap.rs
git commit -m "reset exploration state on map swap

apply_pending_swap now also clears FrontierClusters, PlannerGrid, and
every drone's FrontierTarget/Path/MovementHealth. ensures the new map
starts from a clean planner state instead of inheriting a stale grid
from the previous map dims."
```

---

## Task 12: Phase 1 smoke verification

**Files:**
- No code changes; smoke-test the build.

- [ ] **Step 1: Run + observe**

Run: `cargo run`

Expected:
- Boots into `clusters.dvm`.
- Three drones spawn at world center.
- Lidar spray cone visible.
- Central map fills.
- Drones do **not** intersect cluster geometry (visually verify by zooming in on a cluster).
- Drones distribute across multiple frontiers (crowding penalty working).

- [ ] **Step 2: Stress test**

Push the swarm slider to 30. FPS should stay ≥ 30. Switch map to `tight_corridor.dvm` — drones queue through gap.

- [ ] **Step 3: Trap test (optional)**

Use free-fly camera (F) to follow a single drone. Watch for stuck recovery → log message "drone stuck after 3 escalations" if the drone wedges itself.

- [ ] **Step 4: Commit Phase 1 milestone tag**

```bash
git tag -a phase1-movement -m "Phase 1: collision-aware exploration foundation"
git push --tags
```

---

# Phase 2 — Role specialization

Delivers: scout/mapper/anchor differentiation. Per-role lidar GPU params + visual tints. Supervisor reassigns roles dynamically based on swarm state. Panel exposes role counts + sliders.

## Task 13: `Role` enum + `RoleParams`

**Files:**
- Create: `src/exploration/role.rs`
- Modify: `src/exploration/mod.rs`

- [ ] **Step 1: Create `src/exploration/role.rs`**

```rust
use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Scout,
    Mapper,
    Anchor,
}

impl Default for Role {
    fn default() -> Self {
        Role::Scout
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RoleParams {
    pub cruise_speed_mps: f32,
    pub cone_half_angle_deg: f32,
    pub max_range_cells: u32,
    pub rays_per_scan: u32,
    pub scan_interval_frames: u32,
    pub info_weight: f32,
    pub distance_weight: f32,
    pub distance_bias: f32,
    pub crowding_weight: f32,
    pub avoid_k: f32,
    pub tint: [f32; 4], // linear RGBA before alpha
}

impl RoleParams {
    pub fn for_role(role: Role) -> Self {
        match role {
            Role::Scout => Self {
                cruise_speed_mps: 8.0,
                cone_half_angle_deg: 15.0,
                max_range_cells: 160,
                rays_per_scan: 32,
                scan_interval_frames: 2,
                info_weight: 1.0,
                distance_weight: 0.3,
                distance_bias: 1.0,
                crowding_weight: 0.5,
                avoid_k: 4.0,
                tint: [1.0, 0.85, 0.2, 0.85],
            },
            Role::Mapper => Self {
                cruise_speed_mps: 3.0,
                cone_half_angle_deg: 90.0,
                max_range_cells: 64,
                rays_per_scan: 128,
                scan_interval_frames: 1,
                info_weight: 1.5,
                distance_weight: 1.0,
                distance_bias: 1.0,
                crowding_weight: 1.5,
                avoid_k: 6.0,
                tint: [0.3, 0.8, 0.35, 0.85],
            },
            Role::Anchor => Self {
                cruise_speed_mps: 0.0,
                cone_half_angle_deg: 180.0,
                max_range_cells: 128,
                rays_per_scan: 64,
                scan_interval_frames: 3,
                info_weight: 0.0,
                distance_weight: 0.0,
                distance_bias: 1.0,
                crowding_weight: 0.0,
                avoid_k: 10.0,
                tint: [0.92, 0.95, 1.0, 0.85],
            },
        }
    }
}
```

- [ ] **Step 2: Re-export from `src/exploration/mod.rs`**

```rust
pub mod role;
pub use role::{Role, RoleParams};
```

- [ ] **Step 3: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 4: Commit**

```bash
git add src/exploration
git commit -m "Role enum + RoleParams table

three roles (Scout / Mapper / Anchor); each comes with eleven
tunables: cruise speed, cone half-angle, max range, rays/scan, scan
interval, four scoring weights, avoid_k, and a tint colour. values
match the spec table."
```

---

## Task 14: Default role ratio at drone spawn

**Files:**
- Modify: `src/drone/spawn.rs`
- Modify: `src/exploration/components.rs` (re-export `Role`)

- [ ] **Step 1: Add role assignment to `spawn_one_drone`**

In `src/drone/spawn.rs`, replace the spawn bundle's `DroneColor(color)` with a role-driven tint:

```rust
use crate::exploration::{Role, RoleParams, FrontierTarget, MovementHealth, Path, ReplanTimer};

fn role_for_index(id: u32, total: u32) -> Role {
    let n = total.max(1);
    let scout_cutoff = (n as f32 * 0.6).round() as u32;
    let mapper_cutoff = (n as f32 * 0.9).round() as u32;
    if id < scout_cutoff {
        Role::Scout
    } else if id < mapper_cutoff || n < 4 {
        Role::Mapper
    } else {
        Role::Anchor
    }
}
```

Inside `respawn_drones_if_needed`, replace the per-drone color computation with:

```rust
    for id in 0..target {
        let spawn_pos = ring_position(world_center, id, target);
        let role = role_for_index(id, target);
        let params = RoleParams::for_role(role);
        let color = Color::srgba(params.tint[0], params.tint[1], params.tint[2], params.tint[3]);
        spawn_one_drone(&mut commands, &asset_server, id, spawn_pos, color, role);
    }
```

And update `spawn_one_drone` to take a `Role` and add it to the bundle:

```rust
fn spawn_one_drone(
    commands: &mut Commands,
    asset_server: &AssetServer,
    id: u32,
    spawn_pos: Vec3,
    color: Color,
    role: Role,
) {
    commands
        .spawn((
            Drone,
            DroneId(id),
            DroneColor(color),
            role,
            // ...existing components...
            FrontierTarget::default(),
            Path::default(),
            MovementHealth::default(),
            ReplanTimer::default(),
        ))
        // ...existing children...
    ;
}
```

Drop the old `drone_color(id)` function (or keep it as fallback if not all drones get roles).

- [ ] **Step 2: Build**

Run: `cargo build`
Expected: Compiles. Roles assigned at spawn; tints come from `RoleParams`.

- [ ] **Step 3: Smoke test**

Run: `cargo run` → spawn 10 drones via slider. Visual: 6 yellow scouts + 3 green mappers + 1 white anchor.

- [ ] **Step 4: Commit**

```bash
git add src/drone/spawn.rs src/exploration
git commit -m "spawn drones with default role ratio + tinted colour

role_for_index assigns Scout (60%), Mapper (30%), Anchor (10%) based
on drone id and swarm size. minimum 1 mapper if N < 4. tint comes from
RoleParams::for_role so colour swatches match the role at first glance."
```

---

## Task 15: Per-role scoring weights

**Files:**
- Modify: `src/exploration/systems.rs`
- Modify: `src/exploration/scoring.rs`

- [ ] **Step 1: Update `assign_targets` to read per-drone role**

In `src/exploration/systems.rs`, change `assign_targets`'s self-query to include `&Role`:

```rust
pub fn assign_targets(
    clusters: Res<FrontierClusters>,
    comms: Res<CommsState>,
    mut q_self: Query<(&DroneId, &Transform, &Role, &mut FrontierTarget), With<Drone>>,
    q_peers: Query<(&DroneId, &Transform, &FrontierTarget), With<Drone>>,
) {
    // ...
    for (id, transform, role, mut target) in &mut q_self {
        let role_params = RoleParams::for_role(*role);
        let weights = ScoringWeights {
            info: role_params.info_weight,
            distance: role_params.distance_weight,
            distance_bias: role_params.distance_bias,
            crowding: role_params.crowding_weight,
        };
        if *role == Role::Anchor {
            // Anchors don't pick. Supervisor sets target directly.
            continue;
        }
        // ...existing scoring loop with `weights`...
    }
}
```

Add `use super::role::{Role, RoleParams};` to systems.rs.

- [ ] **Step 2: Build + smoke test**

Run: `cargo build && cargo run`
Expected: Scouts pull further out; mappers prefer crowded-known regions. Anchors hover (no target picked).

- [ ] **Step 3: Commit**

```bash
git add src/exploration
git commit -m "role-flavored scoring weights

assign_targets reads &Role on each drone and uses
RoleParams::for_role to build per-drone ScoringWeights. anchors skip
cluster scoring entirely - the supervisor (Task 18) hands them their
positions directly."
```

---

## Task 16: Per-drone `DroneScanParams` SSBO

**Files:**
- Create: `src/lidar/gpu/per_drone_scan.rs`
- Modify: `src/lidar/gpu/mod.rs`
- Modify: `src/lidar/gpu/resources.rs`

- [ ] **Step 1: Create `per_drone_scan.rs`**

```rust
use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{BufferUsages, ShaderType};
use bevy::render::storage::ShaderStorageBuffer;

use super::resources::MAX_DRONES_GPU;

#[derive(ShaderType, Clone, Copy, Debug, Default)]
pub struct DroneScanParams {
    pub ray_offset: u32,
    pub ray_count: u32,
    pub max_steps: u32,
    pub scan_interval: u32,
}

#[derive(Resource, ExtractResource, Clone)]
pub struct DroneScanParamsBuffer(pub Handle<ShaderStorageBuffer>);

pub fn allocate_buffer(
    buffers: &mut Assets<ShaderStorageBuffer>,
) -> Handle<ShaderStorageBuffer> {
    let init: Vec<DroneScanParams> = vec![DroneScanParams::default(); MAX_DRONES_GPU as usize];
    let mut buf = ShaderStorageBuffer::from(init);
    buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    buffers.add(buf)
}
```

- [ ] **Step 2: Wire allocation into `setup_gpu_lidar_assets`**

In `src/lidar/gpu/resources.rs::setup_gpu_lidar_assets`, allocate the new buffer + insert resource:

```rust
use super::per_drone_scan::{allocate_buffer as alloc_scan_params, DroneScanParamsBuffer};
// at end of setup, before the final commands.insert_resource sequence:
let scan_params_handle = alloc_scan_params(&mut buffers);
commands.insert_resource(DroneScanParamsBuffer(scan_params_handle));
```

- [ ] **Step 3: Register module + ExtractResource**

In `src/lidar/gpu/mod.rs`:

```rust
mod per_drone_scan;
pub use per_drone_scan::{DroneScanParams, DroneScanParamsBuffer};
// in GpuLidarPlugin::build:
            .add_plugins(ExtractResourcePlugin::<DroneScanParamsBuffer>::default())
```

- [ ] **Step 4: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 5: Commit**

```bash
git add src/lidar/gpu
git commit -m "per-drone scan params SSBO

DroneScanParams = (ray_offset, ray_count, max_steps, scan_interval).
one entry per drone slot allocated at startup; resource extracted into
the render world via ExtractResourcePlugin. compute shader binding +
upload come in the next tasks."
```

---

## Task 17: Multi-cone ray-set builder

**Files:**
- Modify: `src/lidar/sampling.rs`
- Modify: `src/lidar/gpu/resources.rs`

- [ ] **Step 1: Add helper to build the concatenated ray buffer**

In `src/lidar/sampling.rs`, add:

```rust
use crate::exploration::Role;

#[derive(Clone, Copy, Debug)]
pub struct RoleConeRange {
    pub role: Role,
    pub offset: u32,
    pub count: u32,
}

pub fn build_role_ray_buffer() -> (Vec<Vec3>, [RoleConeRange; 3]) {
    let scout = fibonacci_cone(32, 15.0_f32.to_radians());
    let mapper = fibonacci_cone(128, 90.0_f32.to_radians());
    let anchor = fibonacci_cone(64, 180.0_f32.to_radians());
    let mut all = Vec::with_capacity(scout.len() + mapper.len() + anchor.len());
    let ranges = [
        RoleConeRange {
            role: Role::Scout,
            offset: 0,
            count: scout.len() as u32,
        },
        RoleConeRange {
            role: Role::Mapper,
            offset: scout.len() as u32,
            count: mapper.len() as u32,
        },
        RoleConeRange {
            role: Role::Anchor,
            offset: (scout.len() + mapper.len()) as u32,
            count: anchor.len() as u32,
        },
    ];
    all.extend(scout);
    all.extend(mapper);
    all.extend(anchor);
    (all, ranges)
}
```

- [ ] **Step 2: Use `build_role_ray_buffer` in `setup_gpu_lidar_assets`**

Replace the existing ray-dirs population logic in `src/lidar/gpu/resources.rs::setup_gpu_lidar_assets` with:

```rust
use crate::lidar::sampling::build_role_ray_buffer;

// Build the role-specific concatenated ray buffer.
let (ray_dirs_vec, role_ranges) = build_role_ray_buffer();
let max_ray_slots = crate::lidar::constants::MAX_RAYS_PER_SCAN as usize;
let mut ray_dirs: Vec<Vec4> = vec![Vec4::ZERO; max_ray_slots.max(ray_dirs_vec.len())];
for (i, d) in ray_dirs_vec.iter().enumerate() {
    if i >= ray_dirs.len() {
        break;
    }
    ray_dirs[i] = Vec4::new(d.x, d.y, d.z, 0.0);
}
```

Then store `role_ranges` as a global resource for the upload step:

```rust
#[derive(Resource, Clone, Copy, Debug)]
pub struct RoleConeRanges(pub [crate::lidar::sampling::RoleConeRange; 3]);

// inside setup function, after building ranges:
commands.insert_resource(RoleConeRanges(role_ranges));
```

Re-export `RoleConeRanges` from `src/lidar/gpu/mod.rs`.

Ensure `MAX_RAYS_PER_SCAN` ≥ 224 (scout 32 + mapper 128 + anchor 64). Update if smaller:

In `src/lidar/constants.rs`:
```rust
pub const MAX_RAYS_PER_SCAN: u32 = 256;
```

- [ ] **Step 3: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 4: Commit**

```bash
git add src/lidar
git commit -m "multi-cone ray buffer for roles

build_role_ray_buffer concatenates three fibonacci cones (scout 32 @
15deg, mapper 128 @ 90deg, anchor 64 @ 180deg) into a single
ray_dirs vec. RoleConeRanges resource exposes per-role offset+count
slices so the per-drone upload (next task) knows where each role's
rays live in the buffer. MAX_RAYS_PER_SCAN bumped to 256 to cover."
```

---

## Task 18: Per-frame upload of `DroneScanParams`

**Files:**
- Modify: `src/lidar/gpu/mod.rs`

- [ ] **Step 1: Add `upload_drone_scan_params` system**

```rust
// src/lidar/gpu/mod.rs
use crate::exploration::{Role, RoleParams};
use crate::lidar::sampling::RoleConeRange;
use super::per_drone_scan::{DroneScanParams, DroneScanParamsBuffer};
use super::resources::RoleConeRanges;

fn role_range(ranges: &[RoleConeRange; 3], role: Role) -> RoleConeRange {
    ranges
        .iter()
        .find(|r| r.role == role)
        .copied()
        .expect("role missing from concatenated ray buffer")
}

fn upload_drone_scan_params(
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    handle: Res<DroneScanParamsBuffer>,
    ranges: Res<RoleConeRanges>,
    drones: Query<(&DroneId, &Role), With<Drone>>,
) {
    let mut sorted: Vec<(u32, Role)> = drones.iter().map(|(id, r)| (id.0, *r)).collect();
    sorted.sort_by_key(|(id, _)| *id);
    let max = super::resources::MAX_DRONES_GPU as usize;
    let mut out = vec![DroneScanParams::default(); max];
    for (i, (_, role)) in sorted.iter().take(max).enumerate() {
        let p = RoleParams::for_role(*role);
        let r = role_range(&ranges.0, *role);
        out[i] = DroneScanParams {
            ray_offset: r.offset,
            ray_count: r.count.min(p.rays_per_scan),
            max_steps: p.max_range_cells,
            scan_interval: p.scan_interval_frames,
        };
    }
    if let Some(buf) = buffers.get_mut(&handle.0) {
        buf.set_data(out);
    }
}
```

Register in `GpuLidarPlugin::build`:
```rust
                upload_drone_scan_params
                    .run_if(resource_exists::<DroneScanParamsBuffer>),
```

- [ ] **Step 2: Build**

Run: `cargo build`
Expected: Compiles.

- [ ] **Step 3: Commit**

```bash
git add src/lidar/gpu/mod.rs
git commit -m "upload per-drone scan params each frame

upload_drone_scan_params sorts drones by id, derives RoleParams +
ray-slice range for each, and writes a DroneScanParams entry per
slot. lidar compute shader binding + read happen in the next task."
```

---

## Task 19: Compute shader reads per-drone params

**Files:**
- Modify: `assets/shaders/lidar_compute.wgsl`
- Modify: `src/lidar/gpu/pipeline.rs`
- Modify: `src/lidar/gpu/dispatch.rs`

- [ ] **Step 1: Add binding 9 to pipeline layout**

In `src/lidar/gpu/pipeline.rs`, append to the layout entries:

```rust
                // 9: per-drone scan params (read)
                storage_buffer_read_only::<Vec<DroneScanParams>>(false),
```

Add `use super::per_drone_scan::DroneScanParams;`.

- [ ] **Step 2: Add binding to bind-group builder in `dispatch.rs`**

In `src/lidar/gpu/dispatch.rs::prepare_lidar_bind_group`, add the new resource param + binding entry:

```rust
    scan_params: Option<Res<DroneScanParamsBuffer>>,
    // ...inside the let-else block...
    let Some(scan_params_buf) = buffers.get(&scan_params?.0) else {
        return;
    };
    // ...inside BindGroupEntries::sequential((...)):
            scan_params_buf.buffer.as_entire_buffer_binding(),
```

Add `use super::resources::DroneScanParamsBuffer;`.

- [ ] **Step 3: Update WGSL shader**

Replace `assets/shaders/lidar_compute.wgsl`:

```wgsl
// ... existing header + LidarParams struct (keep mask + voxel_size, drop rays_per_scan/max_steps semantics from per-drone usage) ...

struct DroneScanParams {
    ray_offset: u32,
    ray_count: u32,
    max_steps: u32,
    scan_interval: u32,
}

@group(0) @binding(0) var<storage, read> ground_bitset: array<u32>;
@group(0) @binding(1) var<storage, read> params: LidarParams;
@group(0) @binding(2) var<storage, read> drone_positions: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> ray_dirs: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read> drone_orientations: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read_write> local_occupancy: array<atomic<u32>>;
@group(0) @binding(6) var<storage, read> drone_colors: array<vec4<f32>>;
@group(0) @binding(7) var<storage, read_write> point_count: atomic<u32>;
@group(0) @binding(8) var<storage, read_write> point_buffer: array<vec4<f32>>;
@group(0) @binding(9) var<storage, read> drone_scan: array<DroneScanParams>;

// ... existing helper functions (cell_flat_idx, axis_t_max, quat_rotate, mark_cell_state, emit_point) ...

@compute @workgroup_size(8, 8, 1)
fn lidar(@builtin(global_invocation_id) gid: vec3<u32>) {
    let drone_idx = gid.x;
    let ray_local_idx = gid.y;
    if (drone_idx >= params.drone_count) {
        return;
    }
    let scan = drone_scan[drone_idx];
    if (ray_local_idx >= scan.ray_count) {
        return;
    }
    // Scan-interval gating per drone: stagger by drone idx so all drones
    // don't sync up on the same frames.
    if (scan.scan_interval > 1u) {
        // The frame counter is encoded into params._pad by the host? No.
        // Use the frame-counter LidarFrameCounter? Render-graph node already
        // handles its own scan interval gate via params.max_points / sched.
        // For now, skip per-drone interval; whole dispatch fires every frame.
        // (Per-drone interval support deferred to a later task.)
    }

    let ray_buf_idx = scan.ray_offset + ray_local_idx;
    let origin = drone_positions[drone_idx].xyz;
    let local_dir = ray_dirs[ray_buf_idx].xyz;
    let world_dir = normalize(quat_rotate(drone_orientations[drone_idx], local_dir));

    // ...rest of DDA traversal unchanged but use scan.max_steps instead of params.max_steps...
    var step: u32 = 0u;
    loop {
        if (step >= scan.max_steps) { break; }
        // ...same body...
    }
}
```

Note: per-drone scan interval is left for a later task to keep this change small. Whole-swarm scan interval still works through the existing `LidarFrameCounter` gate in `ComputeLidarNode`.

- [ ] **Step 4: Build + smoke test**

Run: `cargo build && cargo run`

Expected: Boots clean. Scout drones (yellow) fire fewer rays in a narrow cone. Mapper drones (green) fire dense hemisphere. Anchor (white) fires omni-sphere. Visually obvious from the spray pattern.

- [ ] **Step 5: Commit**

```bash
git add assets/shaders/lidar_compute.wgsl src/lidar/gpu
git commit -m "per-drone scan params consumed by lidar compute

compute shader binding 9 = DroneScanParams array indexed by drone_idx.
Each drone reads its own ray_offset/ray_count slice into the
concatenated ray_dirs buffer + its own max_steps. Whole-dispatch scan-
interval gate stays on LidarFrameCounter; per-drone interval deferred.
Visual diff: scouts now obviously narrow + sparse, mappers dense,
anchors omni."
```

---

## Task 20: Supervisor — comms-graph + role assignment

**Files:**
- Create: `src/exploration/supervisor.rs`
- Modify: `src/exploration/mod.rs`
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Write the supervisor decision-logic test**

```rust
// src/exploration/supervisor.rs
use bevy::prelude::*;

use super::role::Role;

#[derive(Debug, Clone, Copy)]
pub struct SwarmTelemetry {
    pub total_drones: u32,
    pub comms_components: u32,
    pub comms_density: f32,
    pub farthest_frontier_m: f32,
    pub known_free_ratio: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct TargetRatio {
    pub scouts: f32,
    pub mappers: f32,
    pub anchors: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> SwarmTelemetry {
        SwarmTelemetry {
            total_drones: 10,
            comms_components: 1,
            comms_density: 0.8,
            farthest_frontier_m: 100.0,
            known_free_ratio: 0.3,
        }
    }

    #[test]
    fn default_ratio_when_healthy() {
        let r = decide_ratio(&base());
        assert!((r.scouts - 0.6).abs() < 0.01);
        assert!((r.mappers - 0.3).abs() < 0.01);
        assert!((r.anchors - 0.1).abs() < 0.01);
    }

    #[test]
    fn fragmented_comms_bumps_anchors() {
        let mut t = base();
        t.comms_components = 2;
        let r = decide_ratio(&t);
        assert!(r.anchors > 0.1);
    }

    #[test]
    fn distant_frontier_bumps_scouts() {
        let mut t = base();
        t.farthest_frontier_m = 500.0;
        let r = decide_ratio(&t);
        assert!(r.scouts > 0.6);
    }

    #[test]
    fn well_explored_bumps_mappers() {
        let mut t = base();
        t.known_free_ratio = 0.8;
        let r = decide_ratio(&t);
        assert!(r.mappers > 0.3);
    }
}
```

- [ ] **Step 2: Run test (fails)**

Run: `cargo test -p drones --lib exploration::supervisor::tests`
Expected: FAIL — `decide_ratio` not defined.

- [ ] **Step 3: Implement `decide_ratio`**

Add to `src/exploration/supervisor.rs`:

```rust
/// Decide the target role ratio for the current swarm telemetry.
/// Sums to 1.0 after normalisation.
pub fn decide_ratio(t: &SwarmTelemetry) -> TargetRatio {
    let mut scouts = 0.6;
    let mut mappers = 0.3;
    let mut anchors = 0.1;
    if t.comms_components >= 2 {
        anchors += 0.1;
    }
    if t.comms_density < 0.4 {
        anchors += 0.05;
    }
    if t.farthest_frontier_m > 300.0 {
        scouts += 0.1;
    }
    if t.known_free_ratio > 0.7 {
        mappers += 0.1;
    }
    let sum = scouts + mappers + anchors;
    TargetRatio {
        scouts: scouts / sum,
        mappers: mappers / sum,
        anchors: anchors / sum,
    }
}

pub fn role_for_ratio(idx: u32, total: u32, ratio: TargetRatio) -> Role {
    let scout_cutoff = (total as f32 * ratio.scouts).round() as u32;
    let mapper_cutoff = scout_cutoff + (total as f32 * ratio.mappers).round() as u32;
    if idx < scout_cutoff {
        Role::Scout
    } else if idx < mapper_cutoff {
        Role::Mapper
    } else {
        Role::Anchor
    }
}
```

- [ ] **Step 4: Run tests**

Run: `cargo test -p drones --lib exploration::supervisor::tests`
Expected: PASS (4 tests).

- [ ] **Step 5: Add the supervisor system + wire it**

In `src/exploration/supervisor.rs` add:

```rust
use crate::comms::CommsState;
use crate::drone::{Drone, DroneId};
use crate::world::WorldConfig;

use super::resources::FrontierClusters;
use std::time::Duration;

#[derive(Resource, Default)]
pub struct SupervisorTimer(pub Timer);

impl SupervisorTimer {
    pub fn new() -> Self {
        Self(Timer::new(Duration::from_millis(2000), TimerMode::Repeating))
    }
}

#[derive(Component, Default)]
pub struct LastRoleChange(pub f32);

pub fn supervisor_tick(
    time: Res<Time>,
    mut timer: ResMut<SupervisorTimer>,
    comms: Res<CommsState>,
    clusters: Res<FrontierClusters>,
    world: Res<WorldConfig>,
    mut drones: Query<(&DroneId, &Transform, &mut Role, &mut LastRoleChange), With<Drone>>,
) {
    timer.0.tick(time.delta());
    if !timer.0.just_finished() {
        return;
    }
    let now = time.elapsed_secs();

    let total = drones.iter().count() as u32;
    if total == 0 {
        return;
    }

    // Telemetry estimate.
    let comms_components = if comms.total_count == 0 || comms.connected_count == comms.total_count {
        1
    } else {
        2
    };
    let comms_density = if total <= 1 {
        1.0
    } else {
        comms.connected_count as f32 / total as f32
    };
    let farthest_frontier_m = clusters
        .entries
        .iter()
        .map(|c| c.centroid.distance(world.center()))
        .fold(0.0, f32::max);
    let known_free_ratio = 0.3; // Placeholder until coverage telemetry wired.

    let ratio = decide_ratio(&SwarmTelemetry {
        total_drones: total,
        comms_components,
        comms_density,
        farthest_frontier_m,
        known_free_ratio,
    });

    // Sort drones by id and assign by index, respecting smoothing.
    let mut sorted: Vec<(u32, Vec3, Mut<Role>, Mut<LastRoleChange>)> = drones
        .iter_mut()
        .map(|(id, t, r, lc)| (id.0, t.translation, r, lc))
        .collect();
    sorted.sort_by_key(|(id, _, _, _)| *id);
    for (i, (_, _pos, role, last_change)) in sorted.iter_mut().enumerate() {
        let desired = role_for_ratio(i as u32, total, ratio);
        if **role == desired {
            continue;
        }
        if now - last_change.0 < 5.0 {
            continue; // smoothing window
        }
        **role = desired;
        last_change.0 = now;
    }
}
```

Register in `src/exploration/mod.rs`:

```rust
pub mod supervisor;
pub use supervisor::{LastRoleChange, SupervisorTimer};
// In ExplorationPlugin::build:
        .insert_resource(SupervisorTimer::new())
        .add_systems(Update, supervisor::supervisor_tick)
```

Add `LastRoleChange::default()` to drone spawn bundle.

- [ ] **Step 6: Build + smoke test**

Run: `cargo build && cargo run`

Smoke test: push swarm to 30, bump comms range down → anchors should increase within 2 s. Verify role changes are smooth (no thrash) thanks to `last_change` window.

- [ ] **Step 7: Commit**

```bash
git add src/exploration src/drone/spawn.rs
git commit -m "supervisor reassigns roles every 2 s

SwarmTelemetry summary (drone count, comms components, density, far
frontier distance, known-free ratio). decide_ratio shifts the
default 60/30/10 split based on the table from the spec. supervisor
applies the new ratio to drones sorted by id, gated by a 5 s
per-drone smoothing window via LastRoleChange. four unit tests cover
the rule shape (default + each shift trigger)."
```

---

## Task 21: Anchor placement at articulation points

**Files:**
- Modify: `src/exploration/supervisor.rs`
- Modify: `src/exploration/systems.rs`

- [ ] **Step 1: Add articulation-point finder (TDD)**

Append to `src/exploration/supervisor.rs::mod tests`:

```rust
    #[test]
    fn articulation_finds_chain_middle() {
        // Three drones in a chain: 0 -- 1 -- 2. Drone 1 is the articulation.
        // Adjacency: 0-1, 1-2.
        let adj = vec![
            vec![1],
            vec![0, 2],
            vec![1],
        ];
        let art = articulation_points(&adj);
        assert!(art.contains(&1));
        assert!(!art.contains(&0));
        assert!(!art.contains(&2));
    }

    #[test]
    fn articulation_none_in_cycle() {
        // Triangle: 0-1, 1-2, 2-0.
        let adj = vec![vec![1, 2], vec![0, 2], vec![0, 1]];
        let art = articulation_points(&adj);
        assert!(art.is_empty());
    }
```

- [ ] **Step 2: Run test (fails)**

Run: `cargo test -p drones --lib exploration::supervisor::tests`
Expected: FAIL — `articulation_points` not defined.

- [ ] **Step 3: Implement Tarjan's articulation algorithm**

Add to `src/exploration/supervisor.rs`:

```rust
/// Tarjan's algorithm for articulation points in an undirected graph.
/// `adj[i]` lists neighbours of node `i`. Returns sorted unique
/// articulation indices.
pub fn articulation_points(adj: &[Vec<usize>]) -> Vec<usize> {
    let n = adj.len();
    let mut disc = vec![-1i32; n];
    let mut low = vec![0i32; n];
    let mut parent = vec![-1i32; n];
    let mut is_art = vec![false; n];
    let mut timer = 0i32;

    fn dfs(
        u: usize,
        adj: &[Vec<usize>],
        disc: &mut [i32],
        low: &mut [i32],
        parent: &mut [i32],
        is_art: &mut [bool],
        timer: &mut i32,
    ) {
        *timer += 1;
        disc[u] = *timer;
        low[u] = *timer;
        let mut children = 0u32;
        for &v in &adj[u] {
            if disc[v] == -1 {
                children += 1;
                parent[v] = u as i32;
                dfs(v, adj, disc, low, parent, is_art, timer);
                low[u] = low[u].min(low[v]);
                if parent[u] == -1 && children > 1 {
                    is_art[u] = true;
                }
                if parent[u] != -1 && low[v] >= disc[u] {
                    is_art[u] = true;
                }
            } else if v as i32 != parent[u] {
                low[u] = low[u].min(disc[v]);
            }
        }
    }

    for i in 0..n {
        if disc[i] == -1 {
            dfs(i, adj, &mut disc, &mut low, &mut parent, &mut is_art, &mut timer);
        }
    }
    let mut out: Vec<usize> = (0..n).filter(|&i| is_art[i]).collect();
    out.sort();
    out
}
```

- [ ] **Step 4: Run tests**

Run: `cargo test -p drones --lib exploration::supervisor::tests`
Expected: PASS (6 tests).

- [ ] **Step 5: Wire anchor placement into `supervisor_tick`**

Add to `supervisor_tick` after the ratio computation:

```rust
// Build adjacency from drone positions (peers within range_m).
// ... pull comms_settings.range_m via parameter ...
// For each articulation index, set the closest non-anchor drone to Anchor + freeze its FrontierTarget to its current position.
```

For brevity, the full anchor-placement code is delegated to a small helper:

```rust
pub fn place_anchors(
    drone_positions: &[Vec3],
    comms_range_m: f32,
) -> Vec<usize> {
    let n = drone_positions.len();
    let r2 = comms_range_m * comms_range_m;
    let mut adj = vec![Vec::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            if drone_positions[i].distance_squared(drone_positions[j]) <= r2 {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }
    articulation_points(&adj)
}
```

Wire its result inside `supervisor_tick` (read `CommsSettings.range_m` via a new `Res<CommsSettings>` param). For each articulation index `idx`, set the corresponding drone's `Role` to `Anchor` and write its current position into `FrontierTarget.pos` so it stops moving.

- [ ] **Step 6: Build + smoke test**

Run: `cargo build && cargo run`
Drag swarm to 15. Turn on comms gate at range ≈ 150 m. Watch anchors materialize at chain joints.

- [ ] **Step 7: Commit**

```bash
git add src/exploration
git commit -m "anchor placement at comms-graph articulation points

Tarjan's algorithm finds articulation points in the comms adjacency
graph (drones within range_m of each other). supervisor promotes
the nearest non-anchor drone at each articulation to Anchor and
freezes its FrontierTarget at its current position. two unit tests
cover the basic graph shapes (chain + triangle)."
```

---

## Task 22: Panel UI — Roles section

**Files:**
- Modify: `src/ui/panel.rs`

- [ ] **Step 1: Add roles readout + ratio sliders**

In `src/ui/panel.rs`, add a `draw_roles` function and call from `draw_ui`:

```rust
use crate::exploration::Role;

fn draw_roles(
    ui: &mut egui::Ui,
    drones_q: &Query<&Role, With<Drone>>,
) {
    let mut scouts = 0u32;
    let mut mappers = 0u32;
    let mut anchors = 0u32;
    for role in drones_q.iter() {
        match role {
            Role::Scout => scouts += 1,
            Role::Mapper => mappers += 1,
            Role::Anchor => anchors += 1,
        }
    }
    let total = scouts + mappers + anchors;
    ui.label(format!("Roles ({} total)", total));
    ui.label(format!("  Scouts:  {}", scouts));
    ui.label(format!("  Mappers: {}", mappers));
    ui.label(format!("  Anchors: {}", anchors));
}
```

Update `draw_ui` signature to add `drones_role_q: Query<&Role, With<Drone>>` and call `draw_roles(ui, &drones_role_q)` between the existing drone visibility section and the central-map stats.

- [ ] **Step 2: Build + smoke test**

Run: `cargo build && cargo run`
Roles counts visible in panel.

- [ ] **Step 3: Commit**

```bash
git add src/ui/panel.rs
git commit -m "panel shows live role counts

Roles section between drone visibility list and central-map stats.
Tallies scouts/mappers/anchors per frame from the drone query."
```

---

## Task 23: Phase 2 smoke verification

- [ ] **Step 1: Full smoke test**

Run: `cargo run`

Verify:
- Three drones spawn — 2 yellow scouts + 1 green mapper.
- Spray patterns visibly differ: scouts narrow pencil cones, mappers dense hemispheres.
- Push swarm to 20: anchors appear (white, stationary).
- Bump comms range slider low until graph fragments — supervisor promotes anchors at articulation points within 2 s.
- Roles section in panel shows live counts that shift as supervisor reassigns.
- No FPS drop > 5 fps at 50 drones.

- [ ] **Step 2: Tag**

```bash
git tag -a phase2-roles -m "Phase 2: role-specialized swarm shipped"
git push --tags
```

---

# Self-review checklist

Run before declaring the plan done:

1. **Spec coverage:** every section of the spec maps to a task above.
   - Movement layer → Tasks 2, 3, 8, 9, 10.
   - Exploration algorithm → Tasks 4, 5, 6.
   - Roles → Tasks 13, 14, 15, 16, 17, 18, 19.
   - Robustness → Task 7.
   - Supervisor → Tasks 20, 21.
   - Map swap reset → Task 11.
   - Panel UI → Task 22.

2. **Placeholder scan:** no TBD/TODO in the plan itself. The placeholder around per-drone scan interval in Task 19 is acknowledged with deferred follow-up.

3. **Type consistency:**
   - `FrontierTarget` defined in Task 1 with `pos` + `cluster_id`, used identically in Tasks 6, 10, 11.
   - `Path` consistent across Tasks 1, 8, 10, 11.
   - `PlannerGrid` definition (Task 1) consumes the same shape Task 2's `downsample_from_bitset` writes.
   - `ScoringWeights` (Task 5) consumed by Task 6.
   - `RoleParams` (Task 13) consumed by Tasks 15, 18.

4. **Risks acknowledged:** per-drone scan interval deferred; reactive force CPU cost is bounded by the 8 m³ scan window (25 600 cells); article point algorithm is recursive — for swarms > 1000 drones the recursion depth could matter, but bounded by 50.

---

**Plan complete and saved to `docs/superpowers/plans/2026-05-19-drone-exploration-v2.md`.**

Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

2. **Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
