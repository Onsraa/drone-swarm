use super::components::Path;
use super::constants::{AVOID_RADIUS_M, AVOID_RADIUS_PEER_M};
use bevy::prelude::*;

/// Pure-pursuit waypoint selection. Advances the path cursor past any
/// waypoints the drone has passed, then returns the current cursor waypoint
/// as the steering target. Returns `None` for empty paths.
pub fn pure_pursuit(path: &mut Path, drone_pos: Vec3) -> Option<Vec3> {
    if path.waypoints.is_empty() {
        return None;
    }
    // Advance cursor past waypoints that are now behind the drone.
    // A waypoint is "behind" when the next one is at least as close.
    while path.cursor + 1 < path.waypoints.len() {
        let next = path.waypoints[path.cursor + 1];
        if drone_pos.distance(next) <= drone_pos.distance(path.waypoints[path.cursor]) {
            path.cursor += 1;
        } else {
            break;
        }
    }
    path.waypoints.get(path.cursor).copied()
}

/// Quadratic-falloff repulsion. Each obstacle within its radius
/// contributes a force pointing from obstacle to drone, scaled by
/// `avoid_k * (1 - d/R)^2`. Obstacles split into two radii so peers
/// can have a wider personal-space bubble than terrain.
pub fn reactive_force(
    drone_pos: Vec3,
    lidar_hits: &[Vec3],
    peer_positions: &[Vec3],
    avoid_k: f32,
) -> Vec3 {
    let mut total = Vec3::ZERO;
    let scale = |pos: Vec3, radius: f32| -> Vec3 {
        let dir = drone_pos - pos;
        let d = dir.length();
        if d < 1e-3 || d > radius {
            return Vec3::ZERO;
        }
        let strength = avoid_k * (1.0 - d / radius).powi(2);
        (dir / d) * strength
    };
    for &hit in lidar_hits {
        total += scale(hit, AVOID_RADIUS_M);
    }
    for &peer in peer_positions {
        total += scale(peer, AVOID_RADIUS_PEER_M);
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
