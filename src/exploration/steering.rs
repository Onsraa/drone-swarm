use super::components::Path;
use super::constants::{AVOID_RADIUS_M, AVOID_RADIUS_PEER_M, PEER_BUBBLE_RADIUS_M};
use bevy::prelude::*;

/// Distance under which a waypoint is considered "reached" and the
/// cursor advances unconditionally. Without this, `pure_pursuit` will
/// sit at a waypoint that's slightly closer than the next one, even
/// when the drone is essentially AT the waypoint — particularly
/// `waypoints[0]`, which equals the drone's position at the moment the
/// path was planned. Drone wobbles in place around the start waypoint,
/// never advancing.
const WAYPOINT_REACHED_M: f32 = 2.0;

/// Pure-pursuit waypoint selection. Advances the path cursor past any
/// waypoints the drone has reached or passed, then returns the current
/// cursor waypoint as the steering target. Returns `None` for empty
/// paths.
pub fn pure_pursuit(path: &mut Path, drone_pos: Vec3) -> Option<Vec3> {
    if path.waypoints.is_empty() {
        return None;
    }
    while path.cursor + 1 < path.waypoints.len() {
        let cursor_dist = drone_pos.distance(path.waypoints[path.cursor]);
        // 1. Reached the current waypoint → skip to the next one.
        if cursor_dist < WAYPOINT_REACHED_M {
            path.cursor += 1;
            continue;
        }
        // 2. Next waypoint is at least as close → drone has passed
        //    the current one along the path direction.
        let next = path.waypoints[path.cursor + 1];
        if drone_pos.distance(next) <= cursor_dist {
            path.cursor += 1;
        } else {
            break;
        }
    }
    path.waypoints.get(path.cursor).copied()
}

/// Quadratic-falloff repulsion for terrain hits. Each obstacle within
/// its radius contributes a force pointing from obstacle to drone,
/// scaled by `avoid_k * (1 - d/R)^2`.
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

    #[test]
    fn pursuit_advances_cursor() {
        let mut path = Path {
            waypoints: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::new(20.0, 0.0, 0.0),
            ],
            cursor: 0,
        };
        let drone = Vec3::new(5.0, 0.0, 0.0);
        let target = pure_pursuit(&mut path, drone);
        // Cursor should advance past waypoint 0 since drone is between 0 and 1.
        assert_eq!(path.cursor, 1);
        // Look-ahead should aim at waypoint 1 (10 m) since that's within LOOKAHEAD_M=8 m? Actually 10 > 8 so target IS waypoint 1.
        assert_eq!(target, Some(Vec3::new(10.0, 0.0, 0.0)));
    }

    #[test]
    fn empty_path_returns_none() {
        let mut path = Path::default();
        assert!(pure_pursuit(&mut path, Vec3::ZERO).is_none());
    }
}
