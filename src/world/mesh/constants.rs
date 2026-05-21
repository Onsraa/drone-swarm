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
