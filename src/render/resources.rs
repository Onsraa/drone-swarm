use bevy::prelude::*;

/// Handle to the single global-map mesh asset. Lazily populated by
/// `sync_global_map` once the GlobalMap resource exists.
#[derive(Resource, Default)]
pub struct GlobalMapRender {
    pub handle: Option<Handle<Mesh>>,
}
