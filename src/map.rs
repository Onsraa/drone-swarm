use bevy::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CellState {
    Unknown,
    Free,
    Occupied,
}

#[derive(Clone)]
pub struct VoxelMap {
    pub dims: UVec3,
    cells: Vec<CellState>,
}

impl VoxelMap {
    pub fn new(dims: UVec3) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        Self {
            dims,
            cells: vec![CellState::Unknown; n],
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

    pub fn get(&self, p: IVec3) -> CellState {
        self.idx(p)
            .map(|i| self.cells[i])
            .unwrap_or(CellState::Unknown)
    }

    /// Upgrade rule: `Occupied` is sticky; `Free` overrides `Unknown` only.
    /// Prevents transient ray-misses from erasing a previously-detected wall.
    pub fn upgrade(&mut self, p: IVec3, observed: CellState) {
        let Some(i) = self.idx(p) else {
            return;
        };
        let cur = self.cells[i];
        self.cells[i] = match (cur, observed) {
            (CellState::Occupied, _) | (_, CellState::Occupied) => CellState::Occupied,
            (CellState::Free, _) | (_, CellState::Free) => CellState::Free,
            _ => CellState::Unknown,
        };
    }

    pub fn count_known(&self) -> (usize, usize) {
        let mut free = 0;
        let mut occ = 0;
        for c in &self.cells {
            match c {
                CellState::Free => free += 1,
                CellState::Occupied => occ += 1,
                _ => {}
            }
        }
        (free, occ)
    }

    pub fn iter_known(&self) -> impl Iterator<Item = (IVec3, CellState)> + '_ {
        let dx = self.dims.x as i32;
        let dy = self.dims.y as i32;
        let dz = self.dims.z as i32;
        (0..dz)
            .flat_map(move |z| (0..dy).flat_map(move |y| (0..dx).map(move |x| IVec3::new(x, y, z))))
            .filter_map(move |p| {
                let s = self.get(p);
                (s != CellState::Unknown).then_some((p, s))
            })
    }
}

#[derive(Component)]
pub struct LocalMap(pub VoxelMap);
