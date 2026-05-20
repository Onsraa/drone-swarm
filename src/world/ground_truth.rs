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

    /// Walk the column at `(x, z)` from y = 0 upward looking for the
    /// lowest Free cell that has `clearance_cells` consecutive Free
    /// cells above it. Returns the cell-Y of that lowest cell, or
    /// `None` if no such pocket exists in the column (i.e. the entire
    /// vertical span is too cluttered for a safe spawn). Caller uses
    /// the result + voxel_size to turn it into a world Y, plus any
    /// jitter / altitude offset desired.
    pub fn safe_spawn_cell_y(&self, x: i32, z: i32, clearance_cells: u32) -> Option<u32> {
        if x < 0 || z < 0 {
            return None;
        }
        let need = clearance_cells.max(1) as i32;
        let dy = self.dims.y as i32;
        let mut run_above = 0i32; // Free cells above this cell, walking down.
        // Walk top-down so the run-count accumulates as we descend.
        // For each cell, after seeing `need` Free cells above it,
        // record it as a candidate and keep the LOWEST.
        let mut lowest: Option<u32> = None;
        for y in (0..dy).rev() {
            let occupied = self.get(IVec3::new(x, y, z));
            if occupied {
                run_above = 0;
            } else if run_above >= need {
                // This Free cell has at least `need` Free cells above
                // it — it's a valid spawn cell. Keep updating to the
                // lowest one encountered.
                lowest = Some(y as u32);
                run_above += 1;
            } else {
                run_above += 1;
            }
        }
        lowest
    }

    /// Packs the boolean grid into a `Vec<u32>` of `ceil(N/32)` words, with
    /// flat-index `i` mapped to bit `i % 32` of word `i / 32`. Matches the
    /// shape the GPU compute lidar expects in its storage buffer.
    pub fn pack_bitset(&self) -> Vec<u32> {
        let n = self.cells.len();
        let words = n.div_ceil(32);
        let mut out = vec![0u32; words];
        for (i, &occupied) in self.cells.iter().enumerate() {
            if occupied {
                out[i / 32] |= 1u32 << (i % 32);
            }
        }
        out
    }

    /// Inverse of `pack_bitset`: build a fresh map of `dims` by reading
    /// occupancy from a packed `u32` bitset. Used by the map-swap path
    /// when loading a `.dvm` asset.
    pub fn from_bitset(dims: UVec3, bitset: &[u32]) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        let mut cells = vec![false; n];
        for (i, slot) in cells.iter_mut().enumerate() {
            let w = i / 32;
            if w < bitset.len() && (bitset[w] >> (i % 32)) & 1 == 1 {
                *slot = true;
            }
        }
        Self { dims, cells }
    }
}
