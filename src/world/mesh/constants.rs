use bevy::math::Vec3;

pub const DEFAULT_SCENE_PATH: &str = "scenes/test.glb#Scene0";

/// World-space translation for the scene root on first spawn. Centred
/// on the default 640×24×640 voxel world so the mesh overlaps the
/// drone spawn region.
pub const DEFAULT_SCENE_POS: Vec3 = Vec3::new(320.0, 0.0, 320.0);

/// Uniform scale for the scene root on first spawn. 1.0 = native glTF
/// scale. Bump via the side-panel slider to fit small hand-authored
/// scenes into the larger voxel world.
pub const DEFAULT_SCENE_SCALE: f32 = 1.0;

/// Fraction of the world's horizontal extent the mesh AABB should
/// cover after auto-fit. 0.8 leaves ~10% padding on each side for
/// drones to fly around the geometry without spawning inside it.
pub const AUTO_FIT_COVERAGE_RATIO: f32 = 0.8;

/// Centroid-percentile range used to trim outlier geometry when
/// computing the AABB the auto-fit reads. `(0.05, 0.95)` ignores the
/// lowest + highest 5% along each axis — handles sky-domes, distant
/// helpers, stray vertices that would otherwise bloat the AABB.
pub const AUTO_FIT_TRIM_LOW: f32 = 0.05;
pub const AUTO_FIT_TRIM_HIGH: f32 = 0.95;

/// Per-material tile size (pixels) in the lidar-sample atlas. Each
/// material gets a `ATLAS_TILE_PX × ATLAS_TILE_PX` slot in a square
/// grid. Source textures are nearest-neighbour resampled to that size.
/// 256 keeps the atlas <= 4 MB at the 9-material city scene (3×3 grid
/// of 256² = 768² × 4 B = 2.3 MB).
pub const ATLAS_TILE_PX: u32 = 256;
