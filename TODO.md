# Drones Sim — TODO

## Where we left off

- Bevy 0.18.1, 64×24×64 voxel world, 1–50 drone slider, quadcopter dynamics
- True GPU instancing live for all three voxel layers (ground, global, per-drone-local) — one draw call each via a custom RenderPipeline + `step_mode=Instance` vertex buffer
- FPS overlay in the egui side panel
- Tier 1 perf done: cached fibonacci sphere, allocation-free voxel traversal, persistent GPU instance buffer with `write_buffer`, visibility toggle fixed for lazily-spawned layer entities
- Tier 2 partial: lidar runs `par_iter_mut` across drones, `VoxelMap.known` is a `HashSet<u32>` of flat indices on Bevy's FxHash-backed set
- Tier 2 #6 done: `VoxelMap` keeps a `dirty_occupied` queue of flat indices. `sync_local_maps` drains per drone and appends per-color instances each frame instead of rebuilding the aggregated `Vec`. `InstancedVoxelLayer` carries a generation counter; `prepare_instance_buffers` streams the new tail to GPU when the gen matches and re-uploads from offset 0 on a drone-count change
- Tier 2 #7 done: hand-rolled extract system in `ExtractSchedule` only clones `InstancedVoxelLayer` Main→Render when `Ref::is_changed()`. `SyncToRenderWorld` is wired in via `register_required_components`, so spawn sites stay unchanged

`cargo build` is warning-free. `cargo run` smoke-tested clean. Working tree clean.

## Push status

The remote `origin` is `https://github.com/Onsraa/drone-swarm.git`. Local git config is signed in as `onsralorant` which can't push to that repo (403). Several recent commits sit locally:

```
a2a955e parallel lidar + linearised voxel keys
5093d9b fix lazy-spawned layers ignoring visibility toggles + tier 1 perf
dfd2328 show smoothed FPS in the side panel
85102f0 fix periodic hitch on the bigger map
f3f5a52 world: scale up to 64 x 24 x 64 with six obstacle clusters
a8f36c5 ui: default to local-maps-only view
d2c03ea fix: guard instanced-voxel pipeline against empty layers
3d0f4b3 Phase 6b: true GPU instancing for all voxel layers
```

Fix auth (`gh auth login` or correct credential helper), then `git push` from the repo root.

## Next perf wins, in order

### Future Tier 2 polish — Tail-only extract
Even with `Ref::is_changed()` gating, on append frames the extract still clones the *full* `Vec<InstanceData>`. For pure-append frames we could extract just the new tail + generation and have the render-side append to its mirror. Bigger refactor (need a render-world mirror component plus reset on gen bump); defer until clone cost shows up in profiles.

Files: `src/render/instancing/mod.rs`.

### Tier 3 #8 — GPU compute lidar
Move `GroundTruthMap` into a GPU storage buffer. Compute pass keyed on `(drone_index, ray_index)` does Amanatides-Woo on the GPU and writes hits to a per-drone SSBO that the existing instance buffer can consume.

Bigger project. Sketch:
- New module `src/lidar/gpu/` with a WGSL compute shader.
- Resource that holds the ground-truth GPU buffer (initialized once at startup).
- Per-drone scan output SSBO; bind to the render-side instance buffer for the local-map layer.
- CPU only dispatches and reads back stats at slow cadence.

### Tier 3 #9 — Voxel state on GPU
Combine #8 with per-drone `LocalMap` as SSBO. Updates from compute, reads from render. CPU drops out of the map-update hot path entirely. Pairs with `GlobalMap` as another SSBO updated via a reduce-style compute step.

### Tier 3 #10 — Pack `CellState` to 2 bits
Modest gain on its own (12 KB vs 98 KB). Worth doing once #9 puts per-drone maps on GPU and bandwidth starts to matter.

## Phase 6+ features (after perf is solid)

- Frontier-based exploration replacing random wander
- Probabilistic log-odds cells (replaces tri-state)
- Comms range constraints on merge (drones can only contribute when near base or a peer)
- Free-fly camera toggle (WASD + mouse) alongside the orbit camera
- Configurable lidar params via egui sliders (rays/scan, max range, scan rate)

## Known paper cuts

- `bevy_gltf` warns about `Unknown vertex attribute TEXCOORD_2..9` from `drone.glb`. Harmless; trim the asset in Blender or filter the log.
- If the user toggles ground-truth on while local maps are also on, the two layers occupy the same cells. The cyan blend reads OK but Z-fighting can appear. Small per-layer scale offset (1.0 / 1.01 / 1.02) would fix.
- `VoxelMap::get` currently carries `#[allow(dead_code)]`; remove the allow once a caller materializes.

## Critical files

```
src/render/instancing/buffer.rs    # persistent GPU buffer + write_buffer (done)
src/render/instancing/mod.rs       # ExtractComponent clone every frame (target)
src/render/local_map.rs            # per-drone color aggregation
src/render/global_map.rs           # central map chunk rebuild
src/lidar/scan.rs                  # par_iter_mut lidar
src/lidar/sampling.rs              # LidarRayDirs cached resource
src/lidar/traversal.rs             # alloc-free VoxelTraversal iterator
src/map/voxel_map.rs               # u32-keyed FxHash set
src/merge/systems.rs               # 1 Hz merge fold
src/ui/visibility.rs               # toggles applied every frame
src/ui/panel.rs                    # FPS + sliders + stats
assets/shaders/instanced_voxel.wgsl  # instanced cube vertex/fragment
```

## Verification flow

1. `cargo build` — must be warning-free.
2. `cargo run` — orbit camera works (drag + scroll), three drones drift, side panel shows FPS + coverage.
3. Push the drone slider to 30–50, watch FPS in the panel.
4. Toggle each layer in the panel; the layer should hide/show within a frame.

## Plan + memory pointers

Long-form perf audit + roadmap: `~/.claude/plans/let-s-work-on-my-wise-popcorn.md` (one plan file is reused as project history).

Memory entries the next session should respect (loaded automatically):
- Bevy is Y-up; ground is XZ, +Y is altitude.
- Add `bevy_egui` only when a phase actually needs interactive controls.
- Commit + push after each notable step. Commit messages must be humanized — no `Phase X:` prefixes, no AI bullet dumps, no "Effect:" / "Result:" sections.
- Feature-folder code structure (`<feature>/mod.rs` + `components.rs` + `resources.rs` + `systems.rs` + `constants.rs`).
- Keep WebGPU + compute-shader paths on the table for scaling past ~50 drones; voxel rendering is the bottleneck, not motion.
- Local-only excludes (`.claude/`, `CLAUDE.md`) live in `.git/info/exclude`, never in the tracked `.gitignore`.
