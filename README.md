# drones

GPU-accelerated drone-swarm exploration sim built on Bevy 0.18. Up to 50 quadcopters do blind frontier exploration of a voxel world they've never seen, lidar in hand. Everything visible on screen — per-drone lidar, role assignment, comms graph, central map — runs on real data through the same render path you'd ship to a robot fleet, just without the hardware.

## Run

```
cargo run --release
```

You boot into `clusters.dvm`, a 640×24×640 voxel city. Three drones drift, scan, and paint the central map cyan as they explore. Push the swarm slider in the side panel up to 50 and the sim holds 120 FPS on an Apple M4 Pro.

## What you're looking at

Three rendered layers, all camera-facing billboard quads sized in screen pixels:

- **Ground truth** — what the map actually contains. Off by default; toggle in the side panel.
- **Per-drone local maps** — each drone owns its own occupancy grid, tinted to its color. The merge into the global map is gated by the comms graph.
- **Central global map** — what the connected swarm has agreed on. Cyan.

Plus a fourth channel for live lidar: each ray hit emits one sub-voxel point in the cone in front of the drone. Cleared every frame so it pulses with the scan rate.

Drones get one of three roles, reassigned on a 2 s tick based on swarm telemetry:

- **Scout** (yellow) — wide range, narrow cone, fast cruise. Pushes the frontier outward.
- **Mapper** (green) — short range, wide cone, slow cruise. Fills in walls.
- **Anchor** (white) — articulation-point hover. Sits where the comms graph would fragment if it left.

## Controls

- Orbit camera: drag + scroll. Press `F` to toggle free-fly (WASD + mouse look).
- Side panel: map combo box, swarm slider, layer toggles, per-drone visibility, role + coverage stats.

## How it actually works

Lidar runs entirely on the GPU. One compute pass per frame casts every (drone, ray) pair through the ground-truth bitset via Amanatides-Woo DDA, atomicOr's the result into a per-drone occupancy SSBO, and (if comms-connected) into the shared global SSBO. The same pass appends the cell to a small "active cells" list the moment it flips Unknown→Occupied. Two more compute passes walk only those active lists to emit the billboard instance buffers — no full-world sweeps.

Frontier exploration runs on the CPU side of a readback of the global occupancy. A time-sliced 1 Hz scan finds Free cells next to Unknown ones, clusters them by flood-fill, scores each cluster per drone using the role's weights, and runs A* on a downsampled coarse grid to plot the path. Replans are budgeted and event-driven so they don't tank the frame.

## Layout

```
assets/maps/      .dvm dense-bitset + .vox MagicaVoxel input
assets/shaders/   lidar_compute, build_local, build_global, instanced_voxel
src/main.rs       plugin wiring
src/lidar/        GPU lidar plugin, SSBOs, compute + build passes
src/exploration/  roles, planner, A*, frontier clustering, supervisor
src/comms/        comms graph + range gate
src/drone/        spawn, wander, GLB centering
src/physics/      quadcopter dynamics
src/render/       billboard pipeline + per-layer entities
src/maps/         hot-swap, registry, .dvm + .vox loaders
src/ui/           egui side panel, visibility mask, presets
src/camera/       orbit + free-fly
src/world/        GroundTruthMap, WorldConfig
```

## Dependencies

Bevy 0.18.1, bevy_egui 0.39, dot_vox 5.2 for MagicaVoxel import, bytemuck for zero-alloc readback decode, rand for wander noise. That's it.

## Status

Phases 1–6 of the original roadmap shipped, plus the v2 exploration redesign (roles + A* + frontier clustering + articulation anchors) and a full perf pass that hit the 120 FPS native target at 50 drones. Open work and known pitfalls live in `TODO.md` (local only).
