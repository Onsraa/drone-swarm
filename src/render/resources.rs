use std::collections::HashMap;

use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct GlobalMapRender {
    pub spawned: HashMap<IVec3, Entity>,
}
