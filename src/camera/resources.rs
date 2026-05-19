use bevy::prelude::*;

/// Active camera control scheme. Toggled with `F`. Both `OrbitCamera`
/// and `FreeFlyCamera` components live on the same entity; the input
/// systems gate on this resource.
#[derive(Resource, Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum CameraMode {
    #[default]
    Orbit,
    FreeFly,
}
