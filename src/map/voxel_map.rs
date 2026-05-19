use bevy::platform::collections::HashSet;
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
    /// Flat-index set of all non-`Unknown` cells. Linearised to `u32` so the
    /// FxHash-backed `bevy::platform` HashSet hashes a single integer per
    /// insert/lookup instead of the three i32 fields of `IVec3`.
    known: HashSet<u32>,
    /// Flat indices of cells that transitioned into `Occupied` since the
    /// last drain. Lets renderers append-only their instance buffers
    /// instead of rebuilding from `iter_known()` each frame.
    dirty_occupied: Vec<u32>,
    free_count: usize,
    occupied_count: usize,
}

impl VoxelMap {
    pub fn new(dims: UVec3) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        Self {
            dims,
            cells: vec![CellState::Unknown; n],
            known: HashSet::default(),
            dirty_occupied: Vec::new(),
            free_count: 0,
            occupied_count: 0,
        }
    }

    pub fn idx(&self, p: IVec3) -> Option<u32> {
        if p.x < 0 || p.y < 0 || p.z < 0 {
            return None;
        }
        let (x, y, z) = (p.x as u32, p.y as u32, p.z as u32);
        if x >= self.dims.x || y >= self.dims.y || z >= self.dims.z {
            return None;
        }
        Some(x + y * self.dims.x + z * self.dims.x * self.dims.y)
    }

    #[allow(dead_code)]
    pub fn get(&self, p: IVec3) -> CellState {
        self.idx(p)
            .map(|i| self.cells[i as usize])
            .unwrap_or(CellState::Unknown)
    }

    /// `Occupied` is sticky; `Free` overrides `Unknown` only. Prevents transient
    /// ray-misses from erasing a previously-detected wall.
    pub fn upgrade(&mut self, p: IVec3, observed: CellState) {
        let Some(i) = self.idx(p) else {
            return;
        };
        let idx = i as usize;
        let cur = self.cells[idx];
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
            self.known.insert(i);
        } else if new_state == CellState::Unknown && cur != CellState::Unknown {
            self.known.remove(&i);
        }
        if new_state == CellState::Occupied && cur != CellState::Occupied {
            self.dirty_occupied.push(i);
        }
        self.cells[idx] = new_state;
    }

    /// Hands the renderer the flat indices that flipped to `Occupied` since
    /// the previous call. `Occupied` is sticky so the returned set is purely
    /// additive — callers can `extend` an instance buffer without removal.
    pub fn drain_dirty_occupied(&mut self) -> std::vec::Drain<'_, u32> {
        self.dirty_occupied.drain(..)
    }

    pub fn has_dirty_occupied(&self) -> bool {
        !self.dirty_occupied.is_empty()
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
        let dims = self.dims;
        self.known.iter().map(move |&i| {
            let cell = unflatten(i, dims);
            (cell, self.cells[i as usize])
        })
    }
}

pub fn unflatten(flat: u32, dims: UVec3) -> IVec3 {
    let plane = dims.x * dims.y;
    let z = flat / plane;
    let rem = flat % plane;
    let y = rem / dims.x;
    let x = rem % dims.x;
    IVec3::new(x as i32, y as i32, z as i32)
}
