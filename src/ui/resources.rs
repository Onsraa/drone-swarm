use bevy::prelude::*;

#[derive(Resource)]
pub struct UiState {
    pub show_local_maps: bool,
    pub show_global_map: bool,
    pub show_lidar_points: bool,
    /// Gizmo trail line through each drone's recent positions, tinted
    /// in the drone's color. Past = solid, fading with age.
    pub show_trails: bool,
    /// Gizmo line from each drone to its current frontier target,
    /// plus the A* waypoint polyline ahead.
    pub show_paths: bool,
    /// Per-drone raycast gizmo lines showing the role's sensor shape
    /// (legacy name — kept for migration). New code uses the two
    /// fields below.
    #[allow(dead_code)]
    pub show_raycast_lines: bool,
    /// Short-range grey collision-probe rays, per role's detector set.
    pub show_detector_rays: bool,
    /// Role-tinted mapping lidar cone (longer range). Anchors draw
    /// nothing because their `rays_per_scan = 0`.
    pub show_lidar_rays: bool,
    /// Subsampled pheromone-field heatmap as billboard squares.
    pub show_pheromone_field: bool,
    /// Frontier cluster centroids (orange spheres sized by cell count)
    /// + a faint line from each drone to its assigned frontier target.
    pub show_frontiers: bool,
    /// Comms BFS tree edges (green→red by stretch ratio) + a cyan line
    /// from each anchor to the stretched-edge midpoint the planner
    /// assigned it.
    pub show_anchor_targets: bool,
    /// 64-bit visibility mask, bit `i` = drone id `i` rendered in the
    /// local-map layer. `[lo, hi]` halves match the WGSL pair on the
    /// build-shader side. Default all-ones (every drone visible).
    pub drone_mask: [u32; 2],
}

impl UiState {
    pub fn is_drone_visible(&self, id: u32) -> bool {
        let half = if id >= 32 { self.drone_mask[1] } else { self.drone_mask[0] };
        (half >> (id % 32)) & 1 == 1
    }

    pub fn set_drone_visible(&mut self, id: u32, visible: bool) {
        let idx = (id >= 32) as usize;
        let bit = 1u32 << (id % 32);
        if visible {
            self.drone_mask[idx] |= bit;
        } else {
            self.drone_mask[idx] &= !bit;
        }
    }

    pub fn drone_mask_all(&mut self) {
        self.drone_mask = [u32::MAX, u32::MAX];
    }

    pub fn drone_mask_none(&mut self) {
        self.drone_mask = [0, 0];
    }

    pub fn drone_mask_invert(&mut self) {
        self.drone_mask = [!self.drone_mask[0], !self.drone_mask[1]];
    }
}

/// Set by the side-panel draw system each frame: `true` when the
/// pointer is over the egui panel (or egui wants pointer input).
/// Camera input systems early-return when set so dragging on the panel
/// doesn't rotate or zoom the scene.
#[derive(Resource, Default, Clone, Copy, Debug)]
pub struct UiPointerCapture(pub bool);

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_local_maps: true,
            show_global_map: true,
            show_lidar_points: false,
            show_trails: true,
            show_paths: false,
            show_raycast_lines: false,
            show_detector_rays: true,
            show_lidar_rays: true,
            show_pheromone_field: true,
            show_frontiers: false,
            show_anchor_targets: false,
            drone_mask: [u32::MAX, u32::MAX],
        }
    }
}
