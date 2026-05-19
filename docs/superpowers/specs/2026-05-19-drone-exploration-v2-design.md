# Drone Exploration v2 — collision-aware, role-specialised swarm

## Context

Today the sim has drones that fly through walls. They pick the nearest Unknown-adjacent-to-Free cell as a target, lerp toward it in a straight line, ignore obstacles and each other, and have no failure recovery. Frontier exploration works in the toy sense — coverage grows — but the behaviour is visually broken (drones intersect geometry) and algorithmically naive (all drones converge on the same cell, no path planning, no role differentiation).

This redesign turns the swarm into a credible blind-deployment exploration team. The drones know nothing of the map at deploy time; they perceive only via lidar + comms-shared peer maps. Every behaviour decision — what to scan, where to go, how to get there, who to coordinate with — reads from the drone's own discovered occupancy, never from ground truth. The result should look and behave like a real autonomous swarm: a few fast scouts pushing the outline, slower mappers densifying behind, stationary anchors holding the comms graph open, all routing around obstacles via a hybrid planner-plus-reactive control stack.

The end goal is portfolio-grade: visible role differentiation, real robotics primitives (A*, potential fields, frontier clustering, cost-utility scoring, swarm supervisor), and a credible "this could be real fieldwork" feel.

## Core invariant

**No omniscient access.** No behaviour code reads `GroundTruthMap`. All planning and decisions go through:
- The drone's own per-drone occupancy slice (when out of comms).
- The comms-merged `GpuGlobalOccupancyMirror` (when in comms cluster).
- This-frame lidar hits + comms-broadcast peer positions (reactive layer).

Lidar shader is the only path that touches ground truth — it's the sensor model.

## Three-tier control architecture

```
[Supervisor]            0.5 Hz, swarm-wide
   role assignment + anchor placement
            │
            ▼
[Target picker + Planner]   per drone, on-demand
   cost-utility on frontier clusters → coarse A* path
            │
            ▼
[Steering]              60 Hz, per drone
   pure-pursuit + reactive forces → DesiredVelocity
```

Each tier owns one job, runs at its own cadence, can be ablated for testing.

## Roles

Three role types. Each role tunes the same drone hardware along seven axes: speed, cone half-angle, max range, rays/scan, scan interval, target-picker bias, colour tint.

### Scout — push the frontier outward

| Axis | Value | Why |
|---|---|---|
| Cruise speed | 8 m/s (+60%) | Cover ground fast |
| Cone half-angle | 15° | Narrow forward pencil; deep penetration |
| Max range | 160 cells | Sees twice as far |
| Rays per scan | 32 | Fewer rays in narrow cone |
| Scan interval | every 2 frames | Avoid scanning identical positions |
| Target-picker bias | high info, low distance penalty | Drawn outward |
| Colour | yellow | Distinct |

**Tradeoff.** Tunnel vision; side obstacles unseen. Sparse scan = leaves dotted outlines for mappers to fill in.

### Mapper — densify partially-mapped regions

| Axis | Value | Why |
|---|---|---|
| Cruise speed | 3 m/s (-40%) | Slow + thorough |
| Cone half-angle | 90° | Hemisphere coverage |
| Max range | 64 cells | Short + dense |
| Rays per scan | 128 | Dense angular sampling |
| Scan interval | every frame | Maximise samples-per-metre |
| Target-picker bias | high info weight, distance damped | Cleanup behaviour |
| Colour | green | Methodical |

**Tradeoff.** Slow, short range — can't open new territory alone. Heaviest GPU load per drone.

### Anchor — stationary comms relay

| Axis | Value | Why |
|---|---|---|
| Cruise speed | 0 m/s | Hover |
| Cone half-angle | 180° | Full omni sphere |
| Max range | 128 cells | Far horizon since stationary |
| Rays per scan | 64 | Even sphere coverage |
| Scan interval | every 3 frames | No movement = no refresh urgency |
| Target | assigned by supervisor | Doesn't pick |
| Colour | white | Beacon |

**Tradeoff.** Zero frontier push. Pure infrastructure.

### Supervisor (0.5 Hz)

Default ratio at spawn for swarm of N: `0.6N` scouts, `0.3N` mappers, `0.1N` anchors (minimum 1 anchor if `N ≥ 4`).

Dynamic triggers override the ratio:
| Condition | Action |
|---|---|
| Comms graph has ≥2 components | Promote nearest non-anchor drone in the disconnected component to anchor |
| Comms density < 0.4 of max edges | +5% anchor share |
| Farthest frontier > 300 m | +10% scout share |
| > 70% of comms-known cells are Free | +10% mapper share |
| Scout same target > 10 s | Demote to mapper for 30 s cooldown |

**Anchor placement.** Articulation points of the comms graph (BFS the peer-distance graph; nodes whose removal disconnects a component). Drone nearest each articulation point gets re-roled + ordered to hover there.

**Smoothing.** Role can't change more than once per 5 s per drone.

## Movement layer

### Coarse A* planner

- **Grid.** 8× downsample of `WorldConfig.size`. 640×24×640 → 80×3×80 = 19 200 nodes. Each coarse cell summarises 512 native cells via majority vote (`>50% Occupied = Blocked`).
- **Source.** Comms-merged `GpuGlobalOccupancyMirror` when in comms cluster; per-drone occupancy slice (new readback) when isolated. Fallback: frozen last-known snapshot.
- **Update cadence.** Rebuild at 1 Hz on the existing frontier scan tick.
- **Edge weights.** `Free→Free = 1.0d`, `→Unknown = 3.0d`, `Unknown→Unknown = 5.0d`, `→Blocked` rejected. 26-neighbour connectivity (diagonals at `sqrt(2)` / `sqrt(3)`).
- **Output.** `Vec<Vec3>` waypoints in world coords. Stored as `Path` component.

**Replan triggers (per drone):**
1. New target assigned.
2. Next waypoint coarse cell turned `Blocked` (sensor surprise).
3. Stuck > 3 s (linvel below 0.5 m/s).
4. Comms cluster membership changed.
5. Path empty + target exists.

**Cost.** A* on 19 200 nodes → ~10K node-visits worst case → ~100K ops. 50 drones × ~0.1 Hz average = 500 K ops/s. Trivial.

### Steering (every frame)

```
desired = wander_random                          // cold-start fallback
if FrontierTarget present:
    desired = role_target_picker_output
if Path present:
    waypoint = pure_pursuit(path, position, lookahead=8 m)
    desired = lerp(desired, normalize(waypoint - pos) * role.cruise_speed, alpha)
desired += reactive_force(lidar_hits, peers, role)
```

**Pure pursuit.** Find closest point on path, look ahead 8 m for steering target, pop waypoints behind the drone.

**Reactive force.** Quadratic falloff repulsion:
```
strength = AVOID_K * (1.0 - d / AVOID_RADIUS)^2
```
- Obstacles from lidar: ray hits within `AVOID_RADIUS = 4.0 m`.
- Obstacles from peers: comms-connected drones within `AVOID_RADIUS_PEER = 6.0 m`.
- Per-role `AVOID_K`: scouts low (commit to forward motion), anchors high (don't drift).

**Failure mode.** Trapped in a U-shape → sum-of-forces averages to zero → drone halts → stuck detector fires in 3 s → replan against the now-updated occupancy.

## Exploration algorithm

### Step 1: frontier clustering (1 Hz)

Extract Unknown-adjacent-to-Free candidates (existing code at `src/frontier/systems.rs:18`). Flood-fill via 6-neighbourhood within the candidate set. Discard clusters smaller than `MIN_CLUSTER_SIZE = 4`. Output `FrontierClusters { entries: Vec<FrontierCluster> }`.

```rust
struct FrontierCluster {
    id: u32,
    centroid: Vec3,
    cells: Vec<UVec3>,
    info_gain: f32,
    bbox_min: UVec3,
    bbox_max: UVec3,
}
```

### Step 2: per-cluster scoring

```
info_gain = cells.len() as f32
          + 0.25 * count_unknown_in_bbox(centroid, radius=8 cells)
```

### Step 3: per-drone picking

```
score(drone, cluster) =
    info_gain * role.info_weight
  / (distance * role.distance_weight + role.distance_bias
     + crowding(cluster, peers) * role.crowding_weight)
```

| Role | info_weight | distance_weight | crowding_weight |
|---|---|---|---|
| Scout | 1.0 | 0.3 | 0.5 |
| Mapper | 1.5 | 1.0 | 1.5 |
| Anchor | n/a (supervisor-assigned) | n/a | n/a |

**Crowding.** Sum over each comms-peer p: `+1` if `p.target_cluster == cluster.id`, `+0.5` if `p.position` within `cluster.bbox * 1.5`. Drones outside comms see 0 → naive picking → realistic isolation.

**Stickiness.** Keep current cluster unless: reached (within 6 m of any cell), cluster vanished, or new cluster scores > 1.5× current.

## Robustness

### Stuck detection

`MovementHealth { slow_secs, last_position, samples_under_threshold }` component per drone. Accumulate `slow_secs` while `linvel < 0.5 m/s`. At 3 s: force replan + small random `DesiredVelocity` perturbation. After 3 escalations in 20 s: demote scout → mapper; if still stuck, teleport to world center.

### Comms-loss behaviour

- Planner input swaps to per-drone occupancy slice.
- Crowding term goes to 0.
- If the drone was an anchor, supervisor immediately re-roles it to scout and promotes another drone to anchor at a new position.
- Per-drone occupancy CPU mirror: readback the full local SSBO at 0.25 Hz (10 MB/s); only used by isolated drones.
- Fallback if readback cost too high: frozen-snapshot of last comms-merged occupancy at the moment of disconnection.

### Planner failure

A* exhausts the open set. Two cases:
1. Goal cell `Blocked`: discard cluster, blacklist 30 s, pick next.
2. Goal `Unknown` but unreachable through known Free: pick closest reachable Unknown-bordering cell, accept partial progress.

### Map swap

`apply_pending_swap` extends to:
- Clear `FrontierClusters.entries`.
- Reset all `Path`, `MovementHealth`, `Role`.
- `PlannerGrid` reallocated on first post-swap frame.

### Cold start

Existing random `wander` fallback when `FrontierClusters.entries.is_empty()` or planner failed with no fallback.

## File layout

```
src/exploration/                  ← new feature folder, supersedes src/frontier/
  mod.rs                          ← ExplorationPlugin
  constants.rs                    ← tunables (radii, weights, thresholds)
  components.rs                   ← Role, Path, FrontierTarget, MovementHealth
  resources.rs                    ← FrontierClusters, PlannerGrid
  cluster.rs                      ← flood-fill candidates → clusters
  scoring.rs                      ← cost-utility, crowding
  planner.rs                      ← coarse A* over PlannerGrid
  steering.rs                     ← pure-pursuit + reactive force
  supervisor.rs                   ← role assignment + anchor placement
  systems.rs                      ← Bevy system orchestration

src/lidar/gpu/
  per_drone_scan.rs (new)         ← DroneScanParams SSBO + ray-set splicing
  (existing files extended)       ← bind group adds per-drone params

assets/shaders/
  lidar_compute.wgsl              ← per-drone params; pick ray slice

src/ui/panel.rs                   ← Roles section
```

## Lidar GPU extension

- New SSBO `DroneScanParamsBuffer: Vec<DroneScanParams>` (50 × 16 B = 800 B). Per-frame upload from each drone's `Role` + `RoleParams`.
- `RayDirsBuffer` rebuilt to hold concatenated cone sets: scout `32 rays @ 15°` + mapper `128 rays @ 90°` + anchor `64 rays @ 180°` = 224 total. Each role's slice indexed by `ray_offset` + `ray_count`.
- `LidarParams.rays_per_scan` retired — per-drone now.
- Compute shader: dispatch grid stays `(MAX_DRONES, MAX_RAYS_PER_SCAN, 1)`; threads bail when `ray_idx >= drone.ray_count`.

## System graph (Bevy schedule)

```
Update set, ordered:

[1 Hz, gated by Time tick]
  readback observer                       (existing) writes GpuGlobalOccupancyMirror
  rebuild_planner_grid                    (new) writes PlannerGrid
  compute_frontier_clusters               (new) writes FrontierClusters

[0.5 Hz, internal timer]
  supervisor_tick                         (new) reads CommsState + clusters → mutates Role + anchor pos

[every frame]
  assign_targets                          (new) per drone reads clusters + role + peers → FrontierTarget
  replan_paths                            (new) on triggers → A* → Path
  update_movement_health                  (new) tracks slow_secs
  stuck_recovery                          (new) escalation logic

[before PhysicsSet::Control]
  wander                                  (existing) cold-start fallback
  steer_along_path                        (new) pure-pursuit → DesiredVelocity
  reactive_avoid                          (new) lidar + peers → adds force
```

## Reused utilities (do not rewrite)

- `GpuGlobalOccupancyMirror` (in `src/lidar/gpu/mod.rs`) — already populated by the global-stats readback observer; planner + clusters consume it.
- `CommsState.connected_mask` (in `src/comms/resources.rs`) — gates which peers a drone perceives.
- `LidarSettings` (in `src/lidar/resources.rs`) — global lidar defaults; per-role overrides layer on top via the new `DroneScanParams` array.
- `LidarFrameCounter` — frame counter for scan-interval gating.
- `MapSwapRequested` event + `apply_pending_swap` — extended to reset new state.
- Existing `FrontierTarget` component semantics retained; `seek_frontier` system retired, replaced by `steer_along_path`.

## Performance budget (M4 Pro)

| Subsystem | Frequency | Cost |
|---|---|---|
| Planner grid rebuild | 1 Hz | ~10 ms |
| Frontier clustering | 1 Hz | < 1 ms |
| Cluster scoring | 5 Hz | < 0.1 ms |
| Supervisor + role swap | 0.5 Hz | negligible |
| A* per drone | on-demand | < 1 ms each |
| Reactive forces | 60 Hz | < 0.2 ms |
| Per-role lidar GPU | 60 Hz | net zero |

Expected FPS impact: < 5 fps drop at 50 drones.

## Verification

End-to-end smoke test:

1. `cargo build` warning-free.
2. `cargo run` on `clusters.dvm`. Drones spawn coloured by role (yellow scouts, green mappers, possibly 1 white anchor at base).
3. Push swarm slider to 10. Supervisor adds anchors at comms-graph articulation points within 2 s.
4. Bump comms range down until graph fragments. Supervisor reassigns: a scout becomes an anchor near the gap. Console logs the role change.
5. Spawn 30 drones in `tight_corridor.dvm`. Watch drones queue through the gap, not overlap.
6. In `tower.dvm`: drones visibly route *around* towers, not through them.
7. Toggle ground truth on briefly: planned paths visibly avoid `Blocked` coarse cells.
8. Trap a drone in a sealed test pocket: stuck recovery kicks in within 6 s, drone teleports to world center.
9. Map swap mid-exploration: roles reset, drones respawn with default ratio, planner grid recomputed within 1 s.

Visual checks:
- Scouts = yellow streaks toward farthest unknown corner, thin pencil-cones of spray.
- Mappers = green orbs orbiting partial map, dense hemisphere spray.
- Anchors = white stationary nodes, spherical pulses, comms gizmos cluster on them.
- Role transitions = smooth colour shift over 1 frame.

Panel additions:

```
Roles
  Scouts:   X / Y
  Mappers:  X / Y
  Anchors:  X / Y
  [slider] Scout ratio       default 0.6
  [slider] Mapper ratio      default 0.3
  [slider] Anchor ratio      default 0.1
  [ ] Dynamic role assignment (default on)
```

## Out of scope (explicit punt)

- Battery / drone death.
- Pheromone trails (deferred polish).
- Information heatmap overlay (deferred polish).
- Replay / time scrub.
- Rigid-body drone-vs-drone collision (use reactive repulsion only).
- Multi-chunk .vox stitching (separate TODO item).
- Probabilistic log-odds occupancy (separate TODO item).

## Risks + open questions

- **Per-drone occupancy readback cost.** 50 × 2.4 MB = 120 MB readback at full rate exceeds budget. Decision deferred to implementation: either (a) readback at 0.25 Hz, (b) frozen-snapshot fallback for disconnected drones, or (c) GPU-side compaction of "near obstacles per drone" into a small SSBO. Track as the highest-risk piece of the plan.
- **Reactive force source.** Cheapest source for "nearby obstacles per drone" is open: scan the comms-merged occupancy CPU-side around each drone (cheap if cached), or have the lidar shader emit a per-drone near-hit list. Decide during planner implementation.
- **Role thrash under degenerate comms.** If comms range slider is set very low, supervisor may flip roles rapidly. The 5 s smoothing window may not be enough — observe and tune.
- **A* through Unknown.** If a goal sits deep in Unknown, the planner produces optimistic paths that lidar-surprises may invalidate often. Replan rate may spike. Mitigation: cap replan rate per drone at 1 Hz.
