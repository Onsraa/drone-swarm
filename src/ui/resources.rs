use bevy::prelude::*;

#[derive(Resource)]
pub struct UiState {
    pub show_ground_truth: bool,
    pub show_local_maps: bool,
    pub show_global_map: bool,
    pub show_lidar_points: bool,
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

impl Default for UiState {
    fn default() -> Self {
        Self {
            show_ground_truth: false,
            show_local_maps: true,
            show_global_map: true,
            show_lidar_points: true,
            drone_mask: [u32::MAX, u32::MAX],
        }
    }
}
