use super::resources::{CoarseCell, PlannerGrid};
use bevy::prelude::*;

impl PlannerGrid {
    /// Build a coarse occupancy grid by downsampling `downsample^3`
    /// native cells into one coarse cell. Reads from the 2-bits-per-cell
    /// bitset format used by `GpuGlobalOccupancyMirror`: bit 0 = Free
    /// flag, bit 1 = Occupied flag.
    ///
    /// For each coarse cell:
    /// - Count the observed cells (those with Free or Occupied flag set).
    /// - If Occupied > Free, the coarse cell is Blocked.
    /// - If Free > Occupied, the coarse cell is Free.
    /// - If Occupied == Free or no cells are observed, the coarse cell is Unknown.
    /// Unknown cells do not count toward either side.
    pub fn downsample_from_bitset(
        dims: UVec3,
        voxel_size: f32,
        bitset: &[u32],
        downsample: u32,
    ) -> Self {
        let coarse_dims = UVec3::new(
            dims.x.div_ceil(downsample),
            dims.y.div_ceil(downsample),
            dims.z.div_ceil(downsample),
        );
        let total = (coarse_dims.x * coarse_dims.y * coarse_dims.z) as usize;
        let mut coarse = vec![CoarseCell::Unknown; total];
        let read = |flat: u32| -> u32 {
            let w = (flat / 16) as usize;
            if w >= bitset.len() {
                return 0;
            }
            let b = (flat % 16) * 2;
            (bitset[w] >> b) & 0b11
        };
        for cz in 0..coarse_dims.z {
            for cy in 0..coarse_dims.y {
                for cx in 0..coarse_dims.x {
                    let mut occ = 0usize;
                    let mut free = 0usize;
                    for dz in 0..downsample {
                        let z = cz * downsample + dz;
                        if z >= dims.z {
                            continue;
                        }
                        for dy in 0..downsample {
                            let y = cy * downsample + dy;
                            if y >= dims.y {
                                continue;
                            }
                            for dx in 0..downsample {
                                let x = cx * downsample + dx;
                                if x >= dims.x {
                                    continue;
                                }
                                let flat = x + y * dims.x + z * dims.x * dims.y;
                                let state = read(flat);
                                if state & 0b10 != 0 {
                                    occ += 1;
                                } else if state & 0b01 != 0 {
                                    free += 1;
                                }
                            }
                        }
                    }
                    let idx = ((cz * coarse_dims.y + cy) * coarse_dims.x + cx) as usize;
                    coarse[idx] = if occ > free {
                        CoarseCell::Blocked
                    } else if free > occ {
                        CoarseCell::Free
                    } else {
                        CoarseCell::Unknown
                    };
                }
            }
        }

        PlannerGrid {
            coarse,
            dims: coarse_dims,
            voxel_size,
            downsample,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_bitset(dims: UVec3, occupied: &[(i32, i32, i32)], free: &[(i32, i32, i32)]) -> Vec<u32> {
        let n = (dims.x * dims.y * dims.z) as usize;
        let words = n.div_ceil(16);
        let mut bitset = vec![0u32; words];
        for &(x, y, z) in occupied {
            let flat = (x as u32 + y as u32 * dims.x + z as u32 * dims.x * dims.y) as usize;
            let w = flat / 16;
            let b = (flat % 16) * 2;
            bitset[w] |= 0b10u32 << b;
        }
        for &(x, y, z) in free {
            let flat = (x as u32 + y as u32 * dims.x + z as u32 * dims.x * dims.y) as usize;
            let w = flat / 16;
            let b = (flat % 16) * 2;
            bitset[w] |= 0b01u32 << b;
        }
        bitset
    }

    #[test]
    fn downsample_majority_blocked() {
        let dims = UVec3::new(8, 8, 8);
        let mut occupied = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                for z in 0..4 {
                    occupied.push((x, y, z));
                }
            }
        }
        let bitset = make_bitset(dims, &occupied, &[]);
        let grid = PlannerGrid::downsample_from_bitset(dims, 1.0, &bitset, 8);
        assert_eq!(grid.dims, UVec3::new(1, 1, 1));
        assert_eq!(grid.coarse[0], CoarseCell::Blocked);
    }

    #[test]
    fn downsample_majority_free() {
        let dims = UVec3::new(8, 8, 8);
        let mut free = Vec::new();
        for x in 0..6 {
            for y in 0..6 {
                for z in 0..6 {
                    free.push((x, y, z));
                }
            }
        }
        let bitset = make_bitset(dims, &[], &free);
        let grid = PlannerGrid::downsample_from_bitset(dims, 1.0, &bitset, 8);
        assert_eq!(grid.coarse[0], CoarseCell::Free);
    }

    #[test]
    fn downsample_unknown_default() {
        let dims = UVec3::new(8, 8, 8);
        let bitset = make_bitset(dims, &[], &[]);
        let grid = PlannerGrid::downsample_from_bitset(dims, 1.0, &bitset, 8);
        assert_eq!(grid.coarse[0], CoarseCell::Unknown);
    }
}
