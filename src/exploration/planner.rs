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
        let mut occ = vec![0u32; total];
        let mut free = vec![0u32; total];

        // Walk the bitset by word, skipping all-zero words (vast majority
        // at cold start). Each non-zero word decodes 16 cells; for each
        // observed cell we route the count into the coarse-cell bucket.
        let plane = dims.x * dims.y;
        let total_cells = dims.x * dims.y * dims.z;
        for w_idx in 0..bitset.len() {
            let word = bitset[w_idx];
            if word == 0 {
                continue;
            }
            let base_cell = (w_idx as u32) * 16;
            for slot in 0..16u32 {
                let cell = base_cell + slot;
                if cell >= total_cells {
                    break;
                }
                let state = (word >> (slot * 2)) & 0b11;
                if state == 0 {
                    continue;
                }
                let z = cell / plane;
                let rem = cell % plane;
                let y = rem / dims.x;
                let x = rem % dims.x;
                let cx = x / downsample;
                let cy = y / downsample;
                let cz = z / downsample;
                let idx = ((cz * coarse_dims.y + cy) * coarse_dims.x + cx) as usize;
                if state & 0b10 != 0 {
                    occ[idx] += 1;
                } else if state & 0b01 != 0 {
                    free[idx] += 1;
                }
            }
        }

        let coarse: Vec<CoarseCell> = (0..total)
            .map(|i| {
                if occ[i] > free[i] {
                    CoarseCell::Blocked
                } else if free[i] > occ[i] {
                    CoarseCell::Free
                } else {
                    CoarseCell::Unknown
                }
            })
            .collect();

        PlannerGrid {
            coarse,
            dims: coarse_dims,
            voxel_size,
            downsample,
        }
    }
}

use super::constants::{PLANNER_DEEP_UNKNOWN_MULT, PLANNER_FREE_COST, PLANNER_UNKNOWN_COST_MULT};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Sentinel meaning "no predecessor" in the flat `came_from` Vec.
const NONE_IDX: u32 = u32::MAX;

#[derive(Copy, Clone, PartialEq)]
struct Node {
    idx: u32,
    f: f32,
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is max-heap; invert for min-priority on f.
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn neighbors_26() -> [(i32, i32, i32); 26] {
    let mut out = [(0, 0, 0); 26];
    let mut idx = 0;
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                if dx == 0 && dy == 0 && dz == 0 {
                    continue;
                }
                out[idx] = (dx, dy, dz);
                idx += 1;
            }
        }
    }
    out
}

fn step_distance(d: (i32, i32, i32)) -> f32 {
    let s = (d.0 * d.0 + d.1 * d.1 + d.2 * d.2) as f32;
    s.sqrt()
}

fn edge_cost(from: CoarseCell, to: CoarseCell, step_dist: f32) -> Option<f32> {
    if matches!(to, CoarseCell::Blocked) {
        return None;
    }
    let mult = match (from, to) {
        (CoarseCell::Free, CoarseCell::Free) => 1.0,
        (CoarseCell::Free, CoarseCell::Unknown) | (CoarseCell::Unknown, CoarseCell::Free) => {
            PLANNER_UNKNOWN_COST_MULT
        }
        (CoarseCell::Unknown, CoarseCell::Unknown) => PLANNER_DEEP_UNKNOWN_MULT,
        _ => return None,
    };
    Some(step_dist * PLANNER_FREE_COST * mult)
}

fn heuristic(a: UVec3, b: UVec3) -> f32 {
    let dx = a.x as f32 - b.x as f32;
    let dy = a.y as f32 - b.y as f32;
    let dz = a.z as f32 - b.z as f32;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// A* on the coarse planner grid. Returns the sequence of coarse cells
/// from `start` to `goal` inclusive, or `None` if unreachable.
///
/// Uses flat `Vec<f32>` + `Vec<u32>` indexed by `grid.idx(cell)` instead
/// of hash maps. PlannerGrid is dense (~19 K nodes at 640³/8³); flat
/// arrays are 50–200× faster than HashMap probes for this size.
pub fn plan(grid: &PlannerGrid, start: UVec3, goal: UVec3) -> Option<Vec<UVec3>> {
    let Some(start_idx) = grid.idx(start) else { return None; };
    let Some(goal_idx) = grid.idx(goal) else { return None; };
    if matches!(grid.at(goal), CoarseCell::Blocked) {
        return None;
    }
    let start_idx = start_idx as u32;
    let goal_idx = goal_idx as u32;
    let dims = grid.dims;
    let plane = dims.x * dims.y;
    let n = grid.coarse.len();

    // Inverse-lookup helper: cell from flat index.
    let cell_of = |idx: u32| -> UVec3 {
        let z = idx / plane;
        let rem = idx % plane;
        let y = rem / dims.x;
        let x = rem % dims.x;
        UVec3::new(x, y, z)
    };

    let mut g_score: Vec<f32> = vec![f32::INFINITY; n];
    let mut came_from: Vec<u32> = vec![NONE_IDX; n];
    let mut open = BinaryHeap::new();
    g_score[start_idx as usize] = 0.0;
    open.push(Node {
        idx: start_idx,
        f: heuristic(start, goal),
    });

    let neighbors = neighbors_26();

    while let Some(Node { idx, .. }) = open.pop() {
        if idx == goal_idx {
            // Reconstruct via came_from.
            let mut path = Vec::with_capacity(16);
            path.push(goal);
            let mut cur = goal_idx;
            while came_from[cur as usize] != NONE_IDX {
                cur = came_from[cur as usize];
                path.push(cell_of(cur));
            }
            path.reverse();
            return Some(path);
        }
        let g_cur = g_score[idx as usize];
        let cell = cell_of(idx);
        let from_state = grid.coarse[idx as usize];
        for d in &neighbors {
            let nx = cell.x as i32 + d.0;
            let ny = cell.y as i32 + d.1;
            let nz = cell.z as i32 + d.2;
            if nx < 0
                || ny < 0
                || nz < 0
                || nx as u32 >= dims.x
                || ny as u32 >= dims.y
                || nz as u32 >= dims.z
            {
                continue;
            }
            let next = UVec3::new(nx as u32, ny as u32, nz as u32);
            let next_idx = (next.x + next.y * dims.x + next.z * plane) as u32;
            let to_state = grid.coarse[next_idx as usize];
            let step = step_distance(*d);
            let Some(cost) = edge_cost(from_state, to_state, step) else {
                continue;
            };
            let tentative = g_cur + cost;
            if tentative < g_score[next_idx as usize] {
                came_from[next_idx as usize] = idx;
                g_score[next_idx as usize] = tentative;
                let f = tentative + heuristic(next, goal);
                open.push(Node { idx: next_idx, f });
            }
        }
    }
    None
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

    #[test]
    fn astar_straight_line_through_free() {
        let dims = UVec3::new(4, 1, 4);
        let coarse = vec![CoarseCell::Free; 16];
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        let path = plan(&grid, UVec3::new(0, 0, 0), UVec3::new(3, 0, 3)).unwrap();
        assert!(path.first() == Some(&UVec3::new(0, 0, 0)));
        assert!(path.last() == Some(&UVec3::new(3, 0, 3)));
        assert!(path.len() <= 4);
    }

    #[test]
    fn astar_routes_around_blocked() {
        let dims = UVec3::new(5, 1, 5);
        let mut coarse = vec![CoarseCell::Free; 25];
        // Block a wall at x=2 spanning z=0..4.
        for z in 0..4 {
            coarse[(z * dims.x + 2) as usize] = CoarseCell::Blocked;
        }
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        let path = plan(&grid, UVec3::new(0, 0, 0), UVec3::new(4, 0, 0)).unwrap();
        // Must detour through z=4 row.
        assert!(path.iter().any(|c| c.z == 4));
    }

    #[test]
    fn astar_unknown_costs_more() {
        let dims = UVec3::new(3, 1, 3);
        // Layout:
        //  F U F
        //  F U F
        //  F F F
        let mut coarse = vec![CoarseCell::Free; 9];
        coarse[1] = CoarseCell::Unknown;
        coarse[4] = CoarseCell::Unknown;
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        let path = plan(&grid, UVec3::new(0, 0, 0), UVec3::new(2, 0, 0)).unwrap();
        // Direct-through-Unknown route would touch (1, 0, 0). Prefer detour via z=1.
        assert!(!path.contains(&UVec3::new(1, 0, 0)));
    }

    #[test]
    fn astar_no_path_through_blocked_wall() {
        let dims = UVec3::new(3, 1, 1);
        let mut coarse = vec![CoarseCell::Free; 3];
        coarse[1] = CoarseCell::Blocked;
        let grid = PlannerGrid {
            coarse,
            dims,
            voxel_size: 1.0,
            downsample: 1,
        };
        assert!(plan(&grid, UVec3::new(0, 0, 0), UVec3::new(2, 0, 0)).is_none());
    }
}
