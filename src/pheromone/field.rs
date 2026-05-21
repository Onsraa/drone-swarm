use bevy::prelude::*;

use super::constants::DOWNSAMPLE;

/// Which pheromone channel a deposit / gradient query refers to.
/// Scouts and Mappers deposit independently so a mapper can mark
/// "I've detail-mapped here" without interfering with a scout's
/// novelty-seeking anti-gradient signal.
#[derive(Clone, Copy, Debug)]
pub enum Channel {
    Scout = 0,
    Mapper = 1,
}

pub const CHANNEL_COUNT: usize = 2;

/// CPU-side scalar field representing the swarm's pheromone trails.
/// Two channels (Scout / Mapper) share the same coarse grid layout
/// (`DOWNSAMPLE^3` native voxels per cell). Each channel decays + diffuses
/// independently; consumers query a single channel's gradient or the
/// sum gradient depending on role logic.
#[derive(Resource, Default, Debug)]
pub struct PheromoneField {
    pub channels: [Vec<f32>; CHANNEL_COUNT],
    pub dims: UVec3,
    pub voxel_size: f32,
    pub downsample: u32,
}

impl PheromoneField {
    pub fn cell_size(&self) -> f32 {
        self.voxel_size * self.downsample as f32
    }

    pub fn world_to_index(&self, pos: Vec3) -> Option<usize> {
        let cs = self.cell_size();
        if cs <= 0.0 || self.dims == UVec3::ZERO {
            return None;
        }
        let c = (pos / cs).floor();
        if c.x < 0.0 || c.y < 0.0 || c.z < 0.0 {
            return None;
        }
        let cu = UVec3::new(c.x as u32, c.y as u32, c.z as u32);
        if cu.x >= self.dims.x || cu.y >= self.dims.y || cu.z >= self.dims.z {
            return None;
        }
        Some(self.idx(cu))
    }

    pub fn world_to_cell(&self, pos: Vec3) -> Option<IVec3> {
        let cs = self.cell_size();
        if cs <= 0.0 {
            return None;
        }
        let c = (pos / cs).floor();
        Some(IVec3::new(c.x as i32, c.y as i32, c.z as i32))
    }

    fn channel_cells(&self, channel: Channel) -> &[f32] {
        &self.channels[channel as usize]
    }

    fn at_signed_channel(&self, c: IVec3, channel: Channel) -> f32 {
        if c.x < 0 || c.y < 0 || c.z < 0 {
            return 0.0;
        }
        let cu = UVec3::new(c.x as u32, c.y as u32, c.z as u32);
        if cu.x >= self.dims.x || cu.y >= self.dims.y || cu.z >= self.dims.z {
            return 0.0;
        }
        let cells = self.channel_cells(channel);
        cells[self.idx(cu)]
    }

    fn at_signed_sum(&self, c: IVec3) -> f32 {
        if c.x < 0 || c.y < 0 || c.z < 0 {
            return 0.0;
        }
        let cu = UVec3::new(c.x as u32, c.y as u32, c.z as u32);
        if cu.x >= self.dims.x || cu.y >= self.dims.y || cu.z >= self.dims.z {
            return 0.0;
        }
        let i = self.idx(cu);
        self.channels[0][i] + self.channels[1][i]
    }

    /// Gradient on a single channel.
    pub fn gradient_at_channel(&self, pos: Vec3, channel: Channel) -> Vec3 {
        let Some(cell) = self.world_to_cell(pos) else {
            return Vec3::ZERO;
        };
        let dx = self.at_signed_channel(cell + IVec3::X, channel)
            - self.at_signed_channel(cell - IVec3::X, channel);
        let dy = self.at_signed_channel(cell + IVec3::Y, channel)
            - self.at_signed_channel(cell - IVec3::Y, channel);
        let dz = self.at_signed_channel(cell + IVec3::Z, channel)
            - self.at_signed_channel(cell - IVec3::Z, channel);
        Vec3::new(dx, dy, dz) * 0.5
    }

    /// Gradient of (Scout + Mapper). Used by Scouts whose anti-gradient
    /// should repel them from anywhere any drone has recently visited.
    pub fn gradient_at_sum(&self, pos: Vec3) -> Vec3 {
        let Some(cell) = self.world_to_cell(pos) else {
            return Vec3::ZERO;
        };
        let dx = self.at_signed_sum(cell + IVec3::X) - self.at_signed_sum(cell - IVec3::X);
        let dy = self.at_signed_sum(cell + IVec3::Y) - self.at_signed_sum(cell - IVec3::Y);
        let dz = self.at_signed_sum(cell + IVec3::Z) - self.at_signed_sum(cell - IVec3::Z);
        Vec3::new(dx, dy, dz) * 0.5
    }

    /// Heatmap render reads channel sum per cell.
    pub fn value_sum_at_index(&self, idx: usize) -> f32 {
        self.channels[0][idx] + self.channels[1][idx]
    }

    /// Drop a pheromone deposit into the cell containing `pos` on a
    /// single channel. A `neighbor_fraction` slice spills into each of
    /// the six face-neighbours so the gradient is smooth across cell
    /// boundaries.
    pub fn deposit_at_channel(
        &mut self,
        pos: Vec3,
        channel: Channel,
        amount: f32,
        neighbor_fraction: f32,
    ) {
        let Some(idx) = self.world_to_index(pos) else { return; };
        let ch = channel as usize;
        self.channels[ch][idx] += amount;
        let Some(cell) = self.world_to_cell(pos) else { return; };
        for offset in [
            IVec3::X, -IVec3::X, IVec3::Y, -IVec3::Y, IVec3::Z, -IVec3::Z,
        ] {
            let neighbor = cell + offset;
            if neighbor.x < 0 || neighbor.y < 0 || neighbor.z < 0 {
                continue;
            }
            let nu = UVec3::new(neighbor.x as u32, neighbor.y as u32, neighbor.z as u32);
            if nu.x >= self.dims.x || nu.y >= self.dims.y || nu.z >= self.dims.z {
                continue;
            }
            let ni = self.idx(nu);
            self.channels[ch][ni] += amount * neighbor_fraction;
        }
    }

    /// One explicit Laplacian diffusion step on every channel:
    /// `new[c] = old[c] + rate * (mean_of_6_neighbours - old[c])`.
    /// Uses `scratch` as the read-buffer (caller owns it to avoid a
    /// per-frame Vec allocation). Out-of-bounds neighbours count as
    /// "self" so a wall doesn't drain the field into zeros.
    pub fn diffuse(&mut self, rate: f32, scratch: &mut Vec<f32>) {
        if rate <= 0.0 || self.dims == UVec3::ZERO {
            return;
        }
        let total = (self.dims.x * self.dims.y * self.dims.z) as usize;
        for ch in 0..CHANNEL_COUNT {
            let cells = &mut self.channels[ch];
            if cells.len() != total {
                continue;
            }
            scratch.clear();
            scratch.extend_from_slice(cells);
            let dx = self.dims.x as i32;
            let dy = self.dims.y as i32;
            let dz = self.dims.z as i32;
            let stride_y = self.dims.x as usize;
            let stride_z = (self.dims.x * self.dims.y) as usize;
            for z in 0..dz {
                for y in 0..dy {
                    for x in 0..dx {
                        let i = (z as usize) * stride_z
                            + (y as usize) * stride_y
                            + (x as usize);
                        let here = scratch[i];
                        let n_xp = if x + 1 < dx { scratch[i + 1] } else { here };
                        let n_xm = if x > 0 { scratch[i - 1] } else { here };
                        let n_yp = if y + 1 < dy { scratch[i + stride_y] } else { here };
                        let n_ym = if y > 0 { scratch[i - stride_y] } else { here };
                        let n_zp = if z + 1 < dz { scratch[i + stride_z] } else { here };
                        let n_zm = if z > 0 { scratch[i - stride_z] } else { here };
                        let mean = (n_xp + n_xm + n_yp + n_ym + n_zp + n_zm) * (1.0 / 6.0);
                        cells[i] = here + rate * (mean - here);
                    }
                }
            }
        }
    }

    fn idx(&self, c: UVec3) -> usize {
        ((c.z * self.dims.y + c.y) * self.dims.x + c.x) as usize
    }
}

/// Allocate / resize the pheromone field to match the current
/// `WorldConfig`. Runs every frame but is a no-op once the field is
/// correctly sized for the current map. Clears every channel whenever
/// the map dims change (handles map-swap reset for free).
pub fn ensure_pheromone_sized(
    world: Option<Res<crate::world::WorldConfig>>,
    mut field: ResMut<PheromoneField>,
) {
    let Some(world) = world else { return; };
    let coarse_dims = UVec3::new(
        world.size.x.div_ceil(DOWNSAMPLE),
        world.size.y.div_ceil(DOWNSAMPLE),
        world.size.z.div_ceil(DOWNSAMPLE),
    );
    if field.dims == coarse_dims
        && (field.voxel_size - world.voxel_size).abs() < f32::EPSILON
    {
        return;
    }
    let total = (coarse_dims.x * coarse_dims.y * coarse_dims.z) as usize;
    for ch in 0..CHANNEL_COUNT {
        field.channels[ch].clear();
        field.channels[ch].resize(total, 0.0);
    }
    field.dims = coarse_dims;
    field.voxel_size = world.voxel_size;
    field.downsample = DOWNSAMPLE;
}
