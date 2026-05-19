use super::constants::{
    SCORE_CROWDING_WEIGHT, SCORE_DISTANCE_BIAS, SCORE_DISTANCE_WEIGHT, SCORE_INFO_WEIGHT,
};
use super::resources::FrontierCluster;
use bevy::prelude::*;

#[derive(Debug, Clone, Copy)]
pub struct ScoringWeights {
    pub info: f32,
    pub distance: f32,
    pub distance_bias: f32,
    pub crowding: f32,
}

impl Default for ScoringWeights {
    fn default() -> Self {
        Self {
            info: SCORE_INFO_WEIGHT,
            distance: SCORE_DISTANCE_WEIGHT,
            distance_bias: SCORE_DISTANCE_BIAS,
            crowding: SCORE_CROWDING_WEIGHT,
        }
    }
}

/// Cost-utility score for one cluster from the perspective of one
/// drone. Higher is better. `crowding` is a caller-computed count of
/// nearby peer drones (see `crowding_for`).
pub fn score(
    cluster: &FrontierCluster,
    drone_pos: Vec3,
    crowding: u32,
    weights: &ScoringWeights,
) -> f32 {
    let dist = drone_pos.distance(cluster.centroid).max(0.01);
    let denom = dist * weights.distance + weights.distance_bias
        + crowding as f32 * weights.crowding;
    cluster.info_gain * weights.info / denom
}

/// Sum the per-peer crowding contribution against `cluster`:
/// +1.0 per peer that already targets the same cluster id,
/// +0.5 per peer whose position is inside an inflated bbox.
pub fn crowding_for(
    cluster: &FrontierCluster,
    peers: &[(Vec3, Option<u32>)],
    bbox_inflate: f32,
) -> u32 {
    let lo = Vec3::new(
        cluster.bbox_min.x as f32,
        cluster.bbox_min.y as f32,
        cluster.bbox_min.z as f32,
    );
    let hi = Vec3::new(
        cluster.bbox_max.x as f32,
        cluster.bbox_max.y as f32,
        cluster.bbox_max.z as f32,
    );
    let span = (hi - lo) * bbox_inflate;
    let lo_inf = lo - span * 0.5;
    let hi_inf = hi + span * 0.5;
    let mut total: f32 = 0.0;
    for &(pos, target_id) in peers {
        if target_id == Some(cluster.id) {
            total += 1.0;
        } else if pos.cmpge(lo_inf).all() && pos.cmple(hi_inf).all() {
            total += 0.5;
        }
    }
    total.round() as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cluster(id: u32, centroid: Vec3, info: f32) -> FrontierCluster {
        FrontierCluster {
            id,
            centroid,
            cells: vec![],
            info_gain: info,
            bbox_min: UVec3::ZERO,
            bbox_max: UVec3::ZERO,
        }
    }

    #[test]
    fn closer_wins_when_equal_info() {
        let a = cluster(0, Vec3::new(10.0, 0.0, 0.0), 100.0);
        let b = cluster(1, Vec3::new(100.0, 0.0, 0.0), 100.0);
        let w = ScoringWeights::default();
        let sa = score(&a, Vec3::ZERO, 0, &w);
        let sb = score(&b, Vec3::ZERO, 0, &w);
        assert!(sa > sb);
    }

    #[test]
    fn higher_info_wins_when_equal_distance() {
        let a = cluster(0, Vec3::new(10.0, 0.0, 0.0), 1000.0);
        let b = cluster(1, Vec3::new(10.0, 0.0, 0.0), 10.0);
        let w = ScoringWeights::default();
        assert!(score(&a, Vec3::ZERO, 0, &w) > score(&b, Vec3::ZERO, 0, &w));
    }

    #[test]
    fn crowding_lowers_score() {
        let a = cluster(0, Vec3::new(10.0, 0.0, 0.0), 100.0);
        let w = ScoringWeights::default();
        let alone = score(&a, Vec3::ZERO, 0, &w);
        let crowded = score(&a, Vec3::ZERO, 5, &w);
        assert!(crowded < alone);
    }
}
