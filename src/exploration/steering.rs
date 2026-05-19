use super::components::Path;
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

#[cfg(test)]
mod tests {
    use super::*;

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
