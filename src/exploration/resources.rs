use bevy::prelude::*;

#[derive(Debug, Clone)]
pub struct FrontierCluster {
    pub id: u32,
    pub centroid: Vec3,
    pub cells: Vec<UVec3>,
    pub info_gain: f32,
    pub bbox_min: UVec3,
    pub bbox_max: UVec3,
}

#[derive(Resource, Default, Debug)]
pub struct FrontierClusters {
    pub entries: Vec<FrontierCluster>,
    pub next_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoarseCell {
    Free,
    Unknown,
    Blocked,
}

#[derive(Resource, Default, Debug)]
pub struct PlannerGrid {
    pub coarse: Vec<CoarseCell>,
    pub dims: UVec3,
    pub voxel_size: f32,
    pub downsample: u32,
}

impl PlannerGrid {
    pub fn idx(&self, c: UVec3) -> Option<usize> {
        if c.x >= self.dims.x || c.y >= self.dims.y || c.z >= self.dims.z {
            return None;
        }
        Some(((c.z * self.dims.y + c.y) * self.dims.x + c.x) as usize)
    }
    pub fn at(&self, c: UVec3) -> CoarseCell {
        self.idx(c)
            .and_then(|i| self.coarse.get(i).copied())
            .unwrap_or(CoarseCell::Unknown)
    }
    pub fn world_pos_of(&self, c: UVec3) -> Vec3 {
        let cell_size = self.voxel_size * self.downsample as f32;
        Vec3::new(c.x as f32, c.y as f32, c.z as f32) * cell_size + Vec3::splat(cell_size * 0.5)
    }
}
