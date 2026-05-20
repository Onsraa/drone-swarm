use bevy::prelude::*;

use crate::exploration::Role;

use super::constants::DEFAULT_DRONE_COUNT;

/// Desired number of drones in the world. The respawn system observes
/// changes here and rebuilds the swarm accordingly.
#[derive(Resource)]
pub struct DroneSpawnConfig {
    pub target_count: u32,
}

impl Default for DroneSpawnConfig {
    fn default() -> Self {
        Self {
            target_count: DEFAULT_DRONE_COUNT,
        }
    }
}

/// Shared mesh + per-role materials for the drone body. Initialized
/// once at startup so every spawned drone reuses the same handles.
/// Mesh is a flat rectangular cuboid (1.2 × 0.4 × 1.2) that reads as a
/// quadcopter at a glance without the GLB's visual noise.
#[derive(Resource, Clone)]
pub struct DroneBodyAssets {
    pub mesh: Handle<Mesh>,
    pub scout_material: Handle<StandardMaterial>,
    pub mapper_material: Handle<StandardMaterial>,
    pub anchor_material: Handle<StandardMaterial>,
}

impl DroneBodyAssets {
    pub fn material_for(&self, role: Role) -> Handle<StandardMaterial> {
        match role {
            Role::Scout => self.scout_material.clone(),
            Role::Mapper => self.mapper_material.clone(),
            Role::Anchor => self.anchor_material.clone(),
        }
    }
}
