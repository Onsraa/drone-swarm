use super::constants::{AVOID_RADIUS_M, AVOID_RADIUS_PEER_M, PEER_BUBBLE_RADIUS_M};
use bevy::prelude::*;

/// Quadratic-falloff repulsion for terrain hits. Each obstacle within
/// `AVOID_RADIUS_M` contributes a force pointing from obstacle to
/// drone, scaled by `avoid_k * (1 - d/R)^2`. Peers are handled by the
/// separate `reactive_force_peers` so each (self_role, peer_role) pair
/// can carry its own multiplier.
pub fn reactive_force(
    drone_pos: Vec3,
    lidar_hits: &[Vec3],
    peer_positions: &[Vec3],
    avoid_k: f32,
) -> Vec3 {
    let mut total = Vec3::ZERO;
    let terrain_scale = |pos: Vec3, radius: f32| -> Vec3 {
        let dir = drone_pos - pos;
        let d = dir.length();
        if d < 1e-3 || d > radius {
            return Vec3::ZERO;
        }
        let strength = avoid_k * (1.0 - d / radius).powi(2);
        (dir / d) * strength
    };
    // Peers get a quadratic falloff outside `PEER_BUBBLE_RADIUS_M`
    // (same as terrain) plus a near-singular `avoid_k * (R/d - 1)`
    // term inside it. The singular term shoots toward infinity as
    // d → 0, which is what prevents two drones from inter-penetrating.
    let peer_scale = |pos: Vec3| -> Vec3 {
        let dir = drone_pos - pos;
        let d = dir.length();
        if d < 1e-3 || d > AVOID_RADIUS_PEER_M {
            return Vec3::ZERO;
        }
        let outer = avoid_k * (1.0 - d / AVOID_RADIUS_PEER_M).powi(2);
        let inner = if d < PEER_BUBBLE_RADIUS_M {
            avoid_k * (PEER_BUBBLE_RADIUS_M / d - 1.0)
        } else {
            0.0
        };
        (dir / d) * (outer + inner)
    };
    for &hit in lidar_hits {
        total += terrain_scale(hit, AVOID_RADIUS_M);
    }
    for &peer in peer_positions {
        total += peer_scale(peer);
    }
    total
}

/// Peer repulsion with per-pair stiffness. Each `peers` entry carries
/// its position + a precomputed `k` value selected by the caller from
/// the `peer_repulsion_for(self_role, peer_role)` table. This is what
/// makes Scouts plow through Mapper bubbles (low k from Scout's side)
/// while Mappers actively yield (high k from Mapper's side). Same
/// falloff curve as the terrain path — quadratic outside the bubble,
/// near-singular `(R/d - 1)` term inside `PEER_BUBBLE_RADIUS_M` so
/// drones cannot inter-penetrate even when k is low.
pub fn reactive_force_peers(drone_pos: Vec3, peers: &[(Vec3, f32)]) -> Vec3 {
    let mut total = Vec3::ZERO;
    for &(pos, k) in peers {
        let dir = drone_pos - pos;
        let d = dir.length();
        if d < 1e-3 || d > AVOID_RADIUS_PEER_M {
            continue;
        }
        let outer = k * (1.0 - d / AVOID_RADIUS_PEER_M).powi(2);
        let inner = if d < PEER_BUBBLE_RADIUS_M {
            k * (PEER_BUBBLE_RADIUS_M / d - 1.0)
        } else {
            0.0
        };
        total += (dir / d) * (outer + inner);
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::constants::AVOID_RADIUS_M;
    const TEST_AVOID_K: f32 = 6.0;

    #[test]
    fn no_obstacles_no_force() {
        let f = reactive_force(Vec3::ZERO, &[], &[], TEST_AVOID_K);
        assert_eq!(f, Vec3::ZERO);
    }

    #[test]
    fn closer_obstacle_pushes_harder() {
        let near = vec![Vec3::new(1.0, 0.0, 0.0)];
        let far = vec![Vec3::new(3.5, 0.0, 0.0)];
        let f_near = reactive_force(Vec3::ZERO, &near, &[], TEST_AVOID_K);
        let f_far = reactive_force(Vec3::ZERO, &far, &[], TEST_AVOID_K);
        assert!(f_near.length() > f_far.length());
        // Force should point away (negative x since obstacle is at +x).
        assert!(f_near.x < 0.0);
    }

    #[test]
    fn outside_radius_ignored() {
        let way_far = vec![Vec3::new(AVOID_RADIUS_M + 1.0, 0.0, 0.0)];
        let f = reactive_force(Vec3::ZERO, &way_far, &[], TEST_AVOID_K);
        assert_eq!(f, Vec3::ZERO);
    }

}
