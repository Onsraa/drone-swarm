use bevy::prelude::*;

/// Amanatides-Woo voxel traversal as a non-allocating iterator. Walks the
/// integer-cell grid along `dir` from `origin` (in voxel units) up to
/// `max_dist` voxel units. Yields each cell along with the parametric
/// distance `t` at which the ray crosses into it.
pub struct VoxelTraversal {
    cell: IVec3,
    step: IVec3,
    t_max: Vec3,
    t_delta: Vec3,
    max_dist: f32,
    yielded_origin: bool,
    finished: bool,
}

impl VoxelTraversal {
    pub fn new(origin: Vec3, dir: Vec3, max_dist: f32) -> Self {
        let dir = dir.normalize_or_zero();
        if dir.length_squared() == 0.0 || max_dist <= 0.0 {
            return Self::done();
        }

        let cell = IVec3::new(
            origin.x.floor() as i32,
            origin.y.floor() as i32,
            origin.z.floor() as i32,
        );
        let step = IVec3::new(
            dir.x.signum() as i32,
            dir.y.signum() as i32,
            dir.z.signum() as i32,
        );
        let t_delta = Vec3::new(
            inv_or_inf(dir.x),
            inv_or_inf(dir.y),
            inv_or_inf(dir.z),
        );
        let t_max = Vec3::new(
            t_max_axis(cell.x, step.x, origin.x, dir.x),
            t_max_axis(cell.y, step.y, origin.y, dir.y),
            t_max_axis(cell.z, step.z, origin.z, dir.z),
        );

        Self {
            cell,
            step,
            t_max,
            t_delta,
            max_dist,
            yielded_origin: false,
            finished: false,
        }
    }

    fn done() -> Self {
        Self {
            cell: IVec3::ZERO,
            step: IVec3::ZERO,
            t_max: Vec3::splat(f32::INFINITY),
            t_delta: Vec3::splat(f32::INFINITY),
            max_dist: 0.0,
            yielded_origin: true,
            finished: true,
        }
    }
}

impl Iterator for VoxelTraversal {
    type Item = (IVec3, f32);

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        if !self.yielded_origin {
            self.yielded_origin = true;
            return Some((self.cell, 0.0));
        }
        let t = if self.t_max.x < self.t_max.y && self.t_max.x < self.t_max.z {
            self.cell.x += self.step.x;
            let t = self.t_max.x;
            self.t_max.x += self.t_delta.x;
            t
        } else if self.t_max.y < self.t_max.z {
            self.cell.y += self.step.y;
            let t = self.t_max.y;
            self.t_max.y += self.t_delta.y;
            t
        } else {
            self.cell.z += self.step.z;
            let t = self.t_max.z;
            self.t_max.z += self.t_delta.z;
            t
        };
        if t > self.max_dist {
            self.finished = true;
            return None;
        }
        Some((self.cell, t))
    }
}

pub fn voxel_traverse(origin: Vec3, dir: Vec3, max_dist: f32) -> VoxelTraversal {
    VoxelTraversal::new(origin, dir, max_dist)
}

fn inv_or_inf(v: f32) -> f32 {
    if v != 0.0 {
        1.0 / v.abs()
    } else {
        f32::INFINITY
    }
}

fn t_max_axis(cell: i32, step: i32, origin: f32, dir: f32) -> f32 {
    if step == 0 {
        return f32::INFINITY;
    }
    let boundary = if step > 0 {
        (cell + 1) as f32 - origin
    } else {
        origin - cell as f32
    };
    boundary / dir.abs()
}
