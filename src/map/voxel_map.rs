use std::collections::HashSet;

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
    /// All cells whose current state is non-`Unknown`. Lets `iter_known` and
    /// `count_known` run in O(known) instead of O(dims.x * dims.y * dims.z),
    /// which matters once the world grows past a few tens of thousands of
    /// cells (per-frame instance rebuilds were the cause of the periodic
    /// hitch otherwise).
    known: HashSet<IVec3>,
    free_count: usize,
    occupied_count: usize,
}

impl VoxelMap {
    pub fn new(dims: UVec3) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        Self {
            dims,
            cells: vec![CellState::Unknown; n],
            known: HashSet::new(),
            free_count: 0,
            occupied_count: 0,
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

    /// `Occupied` is sticky; `Free` overrides `Unknown` only. Prevents transient
    /// ray-misses from erasing a previously-detected wall.
    pub fn upgrade(&mut self, p: IVec3, observed: CellState) {
        let Some(i) = self.idx(p) else {
            return;
        };
        let cur = self.cells[i];
        let new_state = match (cur, observed) {
            (CellState::Occupied, _) | (_, CellState::Occupied) => CellState::Occupied,
            (CellState::Free, _) | (_, CellState::Free) => CellState::Free,
            _ => CellState::Unknown,
        };
        if new_state == cur {
            return;
        }
        self.adjust_counts(cur, new_state);
        if cur == CellState::Unknown && new_state != CellState::Unknown {
            self.known.insert(p);
        } else if new_state == CellState::Unknown && cur != CellState::Unknown {
            self.known.remove(&p);
        }
        self.cells[i] = new_state;
    }

    fn adjust_counts(&mut self, old: CellState, new: CellState) {
        match old {
            CellState::Free => self.free_count -= 1,
            CellState::Occupied => self.occupied_count -= 1,
            _ => {}
        }
        match new {
            CellState::Free => self.free_count += 1,
            CellState::Occupied => self.occupied_count += 1,
            _ => {}
        }
    }

    pub fn count_known(&self) -> (usize, usize) {
        (self.free_count, self.occupied_count)
    }

    pub fn iter_known(&self) -> impl Iterator<Item = (IVec3, CellState)> + '_ {
        self.known.iter().map(move |&p| (p, self.get(p)))
    }
}
