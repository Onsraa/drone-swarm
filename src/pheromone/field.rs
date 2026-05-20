use bevy::prelude::*;

use super::constants::DOWNSAMPLE;

/// CPU-side scalar field representing the swarm's pheromone trail.
/// Stores one `f32` per coarse cell (`DOWNSAMPLE^3` native voxels). The
/// field is the swarm's shared memory: drones deposit into their
/// current cell each frame, the field decays globally, and per-role
/// steering reads the local gradient to decide where to fly.
#[derive(Resource, Default, Debug)]
pub struct PheromoneField {
    pub cells: Vec<f32>,
    pub dims: UVec3,
    pub voxel_size: f32,
    pub downsample: u32,
}

impl PheromoneField {
    /// Size in meters of one coarse cell along any axis.
    pub fn cell_size(&self) -> f32 {
        self.voxel_size * self.downsample as f32
    }

    /// World position → flat cell index. Returns `None` if the
    /// position is outside the field bounds.
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

    /// Lookup the value at an integer-cell coord (i32 to allow
    /// negative offsets in neighbour walks). Out-of-bounds returns 0.
    pub fn at_signed(&self, c: IVec3) -> f32 {
        if c.x < 0 || c.y < 0 || c.z < 0 {
            return 0.0;
        }
        let cu = UVec3::new(c.x as u32, c.y as u32, c.z as u32);
        if cu.x >= self.dims.x || cu.y >= self.dims.y || cu.z >= self.dims.z {
            return 0.0;
        }
        self.cells[self.idx(cu)]
    }

    /// World position → integer cell, or `None` if outside the field.
    pub fn world_to_cell(&self, pos: Vec3) -> Option<IVec3> {
        let cs = self.cell_size();
        if cs <= 0.0 {
            return None;
        }
        let c = (pos / cs).floor();
        Some(IVec3::new(c.x as i32, c.y as i32, c.z as i32))
    }

    /// Estimate the local gradient at `pos` using a 6-neighbour stencil
    /// (∂φ/∂x, ∂φ/∂y, ∂φ/∂z). Result is in pheromone-units-per-cell-
    /// width; the role-steering caller scales by its own `K`. Returns
    /// `Vec3::ZERO` if the position is outside the field.
    pub fn gradient_at(&self, pos: Vec3) -> Vec3 {
        let Some(cell) = self.world_to_cell(pos) else {
            return Vec3::ZERO;
        };
        let dx = self.at_signed(cell + IVec3::X) - self.at_signed(cell - IVec3::X);
        let dy = self.at_signed(cell + IVec3::Y) - self.at_signed(cell - IVec3::Y);
        let dz = self.at_signed(cell + IVec3::Z) - self.at_signed(cell - IVec3::Z);
        Vec3::new(dx, dy, dz) * 0.5
    }

    /// Drop a pheromone deposit into the cell containing `pos`. A
    /// `DEPOSIT_NEIGHBOR_FRACTION` slice goes to each of the six
    /// face-neighbours so the gradient is smooth across cell boundaries.
    pub fn deposit_at(&mut self, pos: Vec3, amount: f32, neighbor_fraction: f32) {
        let Some(idx) = self.world_to_index(pos) else { return; };
        self.cells[idx] += amount;
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
            self.cells[ni] += amount * neighbor_fraction;
        }
    }

    fn idx(&self, c: UVec3) -> usize {
        ((c.z * self.dims.y + c.y) * self.dims.x + c.x) as usize
    }
}

/// Allocate / resize the pheromone field to match the current
/// `WorldConfig`. Runs every frame but is a no-op once the field is
/// correctly sized for the current map. Clears the field whenever the
/// map dims change (handles map-swap reset for free).
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
    field.cells.clear();
    field.cells.resize(total, 0.0);
    field.dims = coarse_dims;
    field.voxel_size = world.voxel_size;
    field.downsample = DOWNSAMPLE;
}
