use super::constants::{MAX_FRONTIER_CANDIDATES, MIN_CLUSTER_SIZE};
use super::resources::FrontierCluster;
use bevy::prelude::*;
use std::collections::HashSet;

/// 6-neighbourhood flood-fill on a candidate cell set. Clusters smaller
/// than `MIN_CLUSTER_SIZE` are discarded. Each cluster receives a unique
/// id pulled from `*next_id`, which the caller increments-by-N after
/// the call to keep ids monotonic across frames.
pub fn build_clusters(candidates: &HashSet<UVec3>, next_id: &mut u32) -> Vec<FrontierCluster> {
    let mut visited: HashSet<UVec3> = HashSet::new();
    let mut out = Vec::new();
    for &seed in candidates.iter() {
        if visited.contains(&seed) {
            continue;
        }
        let mut stack = vec![seed];
        let mut cells = Vec::new();
        let mut bbox_min = seed;
        let mut bbox_max = seed;
        while let Some(c) = stack.pop() {
            if !visited.insert(c) {
                continue;
            }
            if !candidates.contains(&c) {
                continue;
            }
            cells.push(c);
            bbox_min = bbox_min.min(c);
            bbox_max = bbox_max.max(c);
            for d in [
                IVec3::new(-1, 0, 0),
                IVec3::new(1, 0, 0),
                IVec3::new(0, -1, 0),
                IVec3::new(0, 1, 0),
                IVec3::new(0, 0, -1),
                IVec3::new(0, 0, 1),
            ] {
                let nx = c.x as i32 + d.x;
                let ny = c.y as i32 + d.y;
                let nz = c.z as i32 + d.z;
                if nx < 0 || ny < 0 || nz < 0 {
                    continue;
                }
                stack.push(UVec3::new(nx as u32, ny as u32, nz as u32));
            }
        }
        if cells.len() < MIN_CLUSTER_SIZE {
            continue;
        }
        let centroid = cells.iter().fold(Vec3::ZERO, |acc, c| {
            acc + Vec3::new(c.x as f32, c.y as f32, c.z as f32)
        }) / cells.len() as f32;
        let info_gain = cells.len() as f32;
        out.push(FrontierCluster {
            id: *next_id,
            centroid,
            cells,
            info_gain,
            bbox_min,
            bbox_max,
        });
        *next_id += 1;
        if out.len() >= MAX_FRONTIER_CANDIDATES {
            break;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(coords: &[(u32, u32, u32)]) -> HashSet<UVec3> {
        coords.iter().map(|&(x, y, z)| UVec3::new(x, y, z)).collect()
    }

    #[test]
    fn single_cell_cluster_discarded() {
        let cells = s(&[(0, 0, 0)]);
        let clusters = build_clusters(&cells, &mut 0);
        assert!(clusters.is_empty());
    }

    #[test]
    fn small_cluster_under_threshold_discarded() {
        let cells = s(&[(0, 0, 0), (1, 0, 0), (2, 0, 0)]);
        let clusters = build_clusters(&cells, &mut 0);
        assert!(clusters.is_empty());
    }

    #[test]
    fn line_of_four_kept() {
        let cells = s(&[(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]);
        let clusters = build_clusters(&cells, &mut 0);
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].cells.len(), 4);
        assert_eq!(clusters[0].bbox_min, UVec3::new(0, 0, 0));
        assert_eq!(clusters[0].bbox_max, UVec3::new(3, 0, 0));
    }

    #[test]
    fn two_disjoint_clusters() {
        let cells = s(&[
            (0, 0, 0), (1, 0, 0), (0, 0, 1), (1, 0, 1),
            (10, 0, 10), (11, 0, 10), (10, 0, 11), (11, 0, 11),
        ]);
        let clusters = build_clusters(&cells, &mut 0);
        assert_eq!(clusters.len(), 2);
        assert!(clusters.iter().all(|c| c.cells.len() == 4));
    }

    #[test]
    fn ids_monotonic() {
        let cells = s(&[(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]);
        let mut next_id = 17u32;
        let c1 = build_clusters(&cells, &mut next_id);
        assert_eq!(c1[0].id, 17);
        assert_eq!(next_id, 18);
        let c2 = build_clusters(&cells, &mut next_id);
        assert_eq!(c2[0].id, 18);
    }
}
