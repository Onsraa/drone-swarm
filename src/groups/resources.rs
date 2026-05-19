use bevy::prelude::*;

#[derive(Clone, Debug)]
pub struct Preset {
    pub name: String,
    pub mask: [u32; 2],
}

/// In-memory list of saved drone-visibility masks. Loaded from disk
/// on startup; auto-saved whenever the resource changes.
#[derive(Resource, Default, Debug)]
pub struct DroneGroupPresets {
    pub entries: Vec<Preset>,
}

impl DroneGroupPresets {
    pub fn upsert(&mut self, name: String, mask: [u32; 2]) {
        if let Some(existing) = self.entries.iter_mut().find(|p| p.name == name) {
            existing.mask = mask;
        } else {
            self.entries.push(Preset { name, mask });
        }
    }

    pub fn remove(&mut self, idx: usize) {
        if idx < self.entries.len() {
            self.entries.remove(idx);
        }
    }
}
