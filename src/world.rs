use bevy::prelude::*;

#[derive(Resource, Clone)]
pub struct WorldConfig {
    pub size: UVec3,
    pub voxel_size: f32,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            // Bevy Y-up: X = width, Y = height, Z = depth.
            // Ground footprint 32 x 32 (X by Z), vertical height 16 (Y).
            size: UVec3::new(32, 16, 32),
            voxel_size: 1.0,
        }
    }
}

impl WorldConfig {
    pub fn world_size(&self) -> Vec3 {
        self.size.as_vec3() * self.voxel_size
    }

    pub fn center(&self) -> Vec3 {
        self.world_size() * 0.5
    }
}
