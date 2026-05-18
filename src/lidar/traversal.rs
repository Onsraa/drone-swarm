use bevy::prelude::*;

/// Amanatides-Woo voxel traversal. Walks the integer-cell grid along `dir`
/// from `origin` (in voxel units) up to `max_dist` voxel units. Yields each
/// cell along with the parametric distance `t` at which the ray crosses
/// into it.
pub fn voxel_traverse(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
) -> impl Iterator<Item = (IVec3, f32)> {
    let mut out: Vec<(IVec3, f32)> = Vec::new();
    let dir = dir.normalize_or_zero();
    if dir.length_squared() == 0.0 || max_dist <= 0.0 {
        return out.into_iter();
    }

    let mut cell = IVec3::new(
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
        if dir.x != 0.0 {
            1.0 / dir.x.abs()
        } else {
            f32::INFINITY
        },
        if dir.y != 0.0 {
            1.0 / dir.y.abs()
        } else {
            f32::INFINITY
        },
        if dir.z != 0.0 {
            1.0 / dir.z.abs()
        } else {
            f32::INFINITY
        },
    );

    let first_boundary = |coord: i32, step: i32, origin: f32| -> f32 {
        if step > 0 {
            (coord + 1) as f32 - origin
        } else if step < 0 {
            origin - coord as f32
        } else {
            f32::INFINITY
        }
    };
    let t_max_axis = |c: i32, s: i32, o: f32, d: f32| -> f32 {
        if s == 0 {
            f32::INFINITY
        } else {
            first_boundary(c, s, o) / d.abs()
        }
    };
    let mut t_max = Vec3::new(
        t_max_axis(cell.x, step.x, origin.x, dir.x),
        t_max_axis(cell.y, step.y, origin.y, dir.y),
        t_max_axis(cell.z, step.z, origin.z, dir.z),
    );

    out.push((cell, 0.0));
    let mut t: f32;
    loop {
        if t_max.x < t_max.y && t_max.x < t_max.z {
            cell.x += step.x;
            t = t_max.x;
            t_max.x += t_delta.x;
        } else if t_max.y < t_max.z {
            cell.y += step.y;
            t = t_max.y;
            t_max.y += t_delta.y;
        } else {
            cell.z += step.z;
            t = t_max.z;
            t_max.z += t_delta.z;
        }
        if t > max_dist {
            break;
        }
        out.push((cell, t));
    }
    out.into_iter()
}
