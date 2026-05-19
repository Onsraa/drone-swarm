# Voxel Viewer — Design Spec

**Date:** 2026-05-19
**Status:** Approved design, ready for implementation plan
**Repo:** New standalone repo (separate from drones sim)

---

## Purpose

Standalone Bevy desktop app that converts a DICOM medical scan or a GeoTIFF
elevation file into a MagicaVoxel `.vox` file. The user drops a file, sees a
live 3D voxel preview, adjusts thresholds + crop + voxel density with sliders,
then exports a `.vox` (single or multi-model) consumable by the drones sim
and MagicaVoxel.

One app handles both formats. Format is auto-detected from file extension.

## Goals

- **Live preview** — user sees the voxelization update as they drag sliders
- **Two formats, one tool** — DICOM (`.dcm` / folder) and GeoTIFF (`.tif`)
- **No fixed resolution cap** — multi-model `.vox` output when volume exceeds 256³
- **Format-aware UI** — control panel swaps based on loaded format
- **Responsive UI** — heavy parsing runs off the main thread

## Non-Goals (Out of Scope)

- Drone sim importer multi-model support — separate work on sim repo
- GDAL reprojection for projected-CRS GeoTIFF — initial release: EPSG:4326 only
- MRI percentile auto-threshold — initial release: manual sliders only
- Compressed DICOM transfer syntaxes (JPEG2000, JPEG-LS) — deferred
- Custom user palette files — built-in presets only
- Web/WASM target — desktop only
- DICOM anonymization or PHI handling — assume sample data only

---

## Architecture

### Platform

Bevy 0.14 native desktop app. Separate repo from the drones sim. Reuses no
code from the sim; output `.vox` files are the integration point.

### App States

```
Idle → Loading → Previewing → Exporting → Idle
```

| State        | Behavior                                                              |
|--------------|-----------------------------------------------------------------------|
| `Idle`       | Drag-and-drop target, file picker button                              |
| `Loading`    | Parsing runs on `AsyncComputeTaskPool`, progress bar shown            |
| `Previewing` | 3D viewport + format-aware control panel, sliders update preview live |
| `Exporting`  | Background thread writes `.vox` chunks, status indicator              |

### Core Resources (Bevy ECS)

| Resource          | Purpose                                                       |
|-------------------|---------------------------------------------------------------|
| `RawVolume`       | Original parsed data (HU grid or elevation grid). Source of truth |
| `VoxelGrid`       | Current voxelized version after density downsample             |
| `VisibilityMask`  | `BitVec`, one bit per `VoxelGrid` cell — which voxels render   |
| `PreviewSettings` | All slider values: threshold min/max, crop bounds, density     |
| `SourceKind`      | `Dicom` or `GeoTiff` enum — drives panel dispatch              |
| `LoadProgress`    | `f32` 0.0–1.0 + status string                                  |
| `MemoryBudget`    | Triggers warning if estimated `RawVolume` > 1 GB               |

### VolumeSource Trait — Format Abstraction

```rust
trait VolumeSource: Send + Sync {
    fn parse(path: &Path, progress: Arc<AtomicProgress>) -> Result<RawVolume>;
    fn default_thresholds(&self) -> ThresholdConfig;
    fn palette_preset(&self) -> Palette;
    fn control_panel(&self, ui: &mut egui::Ui, settings: &mut PreviewSettings) -> bool;
}
```

Implementations: `DicomSource`, `GeoTiffSource`. Adding a new format (e.g.,
NetCDF later) means one new `impl` and one new panel module — no changes to
render/export pipeline.

### Slider Systems — Split by Cost

The hot-path optimization. Different sliders trigger different work:

| Slider input     | Cost  | Work performed                                |
|------------------|-------|-----------------------------------------------|
| Threshold change | ~1 ms | Recompute `VisibilityMask` only               |
| Crop change      | ~1 ms | Adjust render bounds (frustum-cull voxels)    |
| Density change   | ~50–300 ms | Re-voxelize `RawVolume` → `VoxelGrid`, rebuild mesh |

Threshold and crop drags stay at 60 fps. Density change is debounced 300 ms.

### Renderer — Surface-Only Mesh

The naive approach (instanced cube per voxel) chokes on dense terrain. The
renderer instead emits only **exterior faces**: for each solid voxel, for each
of 6 neighbors, emit a quad if the neighbor is empty. Interior voxels
contribute zero triangles.

Implementation: custom Bevy `Mesh` with positions/normals/colors. Optional
future enhancement: greedy face merging for further triangle reduction.

### Chunked `.vox` Output

Output targets MagicaVoxel 0.99.7+ extended `.vox` format via the `dot_vox` v5
crate (supports scene graph nodes `nTRN`, `nGRP`, `nSHP`).

`.vox` format hard limit: 256³ per model. When the output `VoxelGrid` exceeds
that on any axis, the export pipeline splits it into 256³ chunks. Each chunk
becomes one `Model` in `dot_vox::DotVoxData`, placed in the world via a
`nTRN` scene node at chunk-aligned coordinates `(cx * 256, cy * 256, cz * 256)`.

**Drone sim compatibility note:** The existing sim importer (commit `b0fee49`)
reads a single model. Multi-model output requires a sim-side update — tracked
as future work on the sim repo, not this project.

---

## Data Flow

### Load Phase

1. User drops file (or picks via dialog)
2. `SourceKind` detected from extension: `.dcm`/folder → `Dicom`; `.tif`/`.tiff` → `GeoTiff`
3. State transitions to `Loading`
4. `AsyncComputeTaskPool` spawns the parser
5. Parser writes `LoadProgress` as slices/rows arrive
6. Memory estimate before allocating `RawVolume`. If > 1 GB, show warning dialog
7. `RawVolume` populated
8. `default_thresholds()` + default density applied
9. State transitions to `Previewing`

### Preview Phase — Hot Path

- **Threshold slider** → recompute `VisibilityMask` → mark mesh dirty (per-face visibility update)
- **Crop slider** → adjust render bounds → cull voxels outside bounds
- **Density slider** (debounced 300 ms) → re-voxelize `RawVolume` → rebuild `VisibilityMask` → rebuild surface mesh

### Re-Voxelize Pipeline (Density Change)

```
RawVolume (512×512×1000 i16)
    │ density = 4 mm/voxel
    │ (GeoTIFF only: multiply elevation by vertical-exaggeration factor)
    ▼
Downsample by ratio → VoxelGrid (128×128×250 u8 palette index)
    │ apply current thresholds
    ▼
Classify each voxel → palette index or empty
    │ for each solid voxel, check 6 neighbors
    ▼
SurfaceMesh (Vec<Quad>) — exterior faces only
    │ upload to Bevy Mesh
    ▼
Rendered preview
```

### Export Phase

1. User clicks Export, picks output path (native file dialog)
2. State transitions to `Exporting`
3. Compute chunk layout:
   `chunks = ceil(grid_dim / 256)` on each axis
4. For each chunk:
   - Slice `VoxelGrid` bounds `[cx*256..(cx+1)*256, …]`
   - Apply `VisibilityMask` (skip filtered voxels)
   - Build a `dot_vox::Model`
   - Add `nTRN` scene node at chunk world position
5. Assemble `DotVoxData` with palette
6. Write to disk on background thread
7. State transitions back to `Idle`

---

## UI Layout

### Window

- **Left**: 3D viewport with `bevy_panorbit_camera` (orbit, pan, zoom). HUD overlay shows grid dims, visible-voxel count, triangle count, predicted chunk count on export.
- **Right (~280 px)**: format-aware control panel (egui), scrollable.

### Format-Aware Panels

**DICOM panel**

- `DENSITY` slider: mm per voxel (0.5–10.0)
- `HU THRESHOLD`: min + max numeric inputs
- Tissue presets: `Bone`, `Soft`, `Full` (sets HU min/max)
- `CROP`: X/Y/Z ranges in voxel coords
- `PALETTE`: tissue-based (bone, soft tissue, lung, fat)

**GeoTIFF panel**

- `DENSITY` slider: meters per voxel (1.0–100.0)
- `ELEVATION`: min + max numeric inputs
- Biome mode: `elevation` / `slope` / `flat`
- `CROP`: lat/lon bbox or voxel coords
- `SEA LEVEL`: fill water below this elevation
- `VERTICAL EXAGG`: 1.0×–5.0×

### Loading State

Centered progress bar with status text (e.g., `slice 620 / 1000 (62%)`).

### Memory Warning Dialog

Shown when estimated `RawVolume` > 1 GB. Options: Continue / Cancel.

---

## Module Structure

```
voxel-viewer/
├── Cargo.toml
└── src/
    ├── main.rs                 # Bevy app + plugin wiring
    ├── state.rs                # AppState enum + transitions
    │
    ├── source/                 # VolumeSource trait + impls
    │   ├── mod.rs              # trait definition
    │   ├── dicom.rs            # DicomSource (dicom-rs parsing)
    │   └── geotiff.rs          # GeoTiffSource (tiff parsing)
    │
    ├── volume.rs               # RawVolume + VoxelGrid + downsample
    ├── classify.rs             # threshold + palette index lookup
    ├── visibility.rs           # VisibilityMask + threshold/crop systems
    │
    ├── render/                 # preview rendering
    │   ├── mod.rs              # camera + light + mesh entity setup
    │   ├── surface_mesh.rs     # exterior-faces-only mesh builder
    │   └── palette.rs          # palette → Bevy material colors
    │
    ├── ui/                     # egui panels
    │   ├── mod.rs              # panel dispatcher (SourceKind-aware)
    │   ├── dicom_panel.rs      # HU sliders + tissue presets
    │   ├── geotiff_panel.rs    # elevation + biome + sea level
    │   ├── load_screen.rs      # drag-drop + file picker
    │   ├── progress.rs         # Loading state progress bar
    │   └── memory_warn.rs      # budget warning dialog
    │
    ├── export/                 # .vox writing
    │   ├── mod.rs              # orchestration + path picker
    │   ├── chunk.rs            # split VoxelGrid into 256³ chunks
    │   └── writer.rs           # dot_vox assembly + scene graph
    │
    └── tasks.rs                # AsyncComputeTaskPool helpers
```

## Cargo Dependencies

```toml
[dependencies]
bevy                 = "0.14"
bevy_egui            = "0.28"        # control panel
bevy_panorbit_camera = "0.19"        # orbit/pan/zoom
bevy_file_dialog     = "0.6"         # native file picker

dicom                = "0.7"         # DICOM read
dicom-pixeldata      = "0.7"         # pixel decode
tiff                 = "0.9"         # GeoTIFF read

dot_vox              = "5"           # .vox write

ndarray              = "0.15"        # 3D array ops
image                = "0.25"        # Lanczos resample (GeoTIFF)
bitvec               = "1"           # VisibilityMask
anyhow               = "1"           # error handling

[profile.release]
opt-level = 3
lto       = "thin"
```

## Plugin Wiring (main.rs sketch)

```rust
App::new()
    .add_plugins(DefaultPlugins)
    .add_plugins(EguiPlugin)
    .add_plugins(PanOrbitCameraPlugin)
    .init_state::<AppState>()
    .init_resource::<PreviewSettings>()
    .add_plugins((SourcePlugin, RenderPlugin, UiPlugin, ExportPlugin))
    .run();
```

---

## Testing Strategy

- **Unit tests**: classify (HU → palette index), downsample (factor math), chunk layout (size + position), surface mesh (face culling correctness on hand-crafted grids).
- **Integration tests**: parse a known small DICOM fixture + a small SRTM tile, run through to `.vox`, verify output bytes against a snapshot or assert chunk count + non-zero voxel count.
- **Manual UI smoke**: drag-drop, slider responsiveness, export round-trip into MagicaVoxel and visual check.

## Error Handling

- File parse failure: return to `Idle`, show error in load screen.
- Memory budget exceeded: warning dialog before allocation.
- Export write failure: status indicator with error message, state returns to `Previewing` so user can retry without re-loading.

## Performance Targets

- Threshold/crop drag → 60 fps sustained on a 128³ grid.
- Density change → preview rebuild < 500 ms for a 256³ grid on a mid-range laptop.
- Load: 1000-slice DICOM series in < 10 s.

---

## Open Questions

None at design approval — all resolved during brainstorming.

## Future Work (post-v1)

- Sim-side importer multi-model support
- NetCDF (atmospheric / weather) as a third `VolumeSource`
- GDAL bindings (optional feature) for projected-CRS GeoTIFF
- MRI auto-threshold via percentile normalization
- Custom palette `.csv` import
- WASM build target with file-size limit warnings
