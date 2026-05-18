use bevy::prelude::*;

#[derive(Resource)]
pub struct GroundTruthMap {
    pub dims: UVec3,
    cells: Vec<bool>,
}

impl GroundTruthMap {
    pub fn new(dims: UVec3) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        Self {
            dims,
            cells: vec![false; n],
        }
    }

    pub fn idx(&self, p: IVec3) -> Option<usize> {
        if p.x < 0 || p.y < 0 || p.z < 0 {
            return None;
        }
        let (x, y, z) = (p.x as u32, p.y as u32, p.z as u32);
        if x >= self.dims.x || y >= self.dims.y || z >= self.dims.z {
            return None;
        }
        Some((x + y * self.dims.x + z * self.dims.x * self.dims.y) as usize)
    }

    pub fn get(&self, p: IVec3) -> bool {
        self.idx(p).map(|i| self.cells[i]).unwrap_or(false)
    }

    pub fn set(&mut self, p: IVec3, v: bool) {
        if let Some(i) = self.idx(p) {
            self.cells[i] = v;
        }
    }

    pub fn iter_occupied(&self) -> impl Iterator<Item = IVec3> + '_ {
        let dx = self.dims.x as i32;
        let dy = self.dims.y as i32;
        let dz = self.dims.z as i32;
        (0..dz)
            .flat_map(move |z| (0..dy).flat_map(move |y| (0..dx).map(move |x| IVec3::new(x, y, z))))
            .filter(move |p| self.get(*p))
    }

    pub fn count_occupied(&self) -> usize {
        self.cells.iter().filter(|c| **c).count()
    }
}
