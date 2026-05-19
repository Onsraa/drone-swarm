// Stage 3 GPU lidar. One thread per (drone, ray). Walks Amanatides-Woo
// through the ground-truth bitset and writes the traversal trail to the
// hits buffer. Per-step entries pack (state << 30) | (flat_cell_idx).
// State encoding:
//   0u = sentinel / unwritten
//   1u = Free  (cell visited, ground bit unset)
//   2u = Occupied (cell visited, ground bit set; trail ends here)
//
// Layout: hits[(drone * rays + ray) * max_steps + step].

struct LidarParams {
    dims: vec4<u32>,         // (x, y, z, _pad)
    max_steps: u32,
    rays_per_scan: u32,
    drone_count: u32,
    _pad: u32,
}

@group(0) @binding(0) var<storage, read> ground_bitset: array<u32>;
@group(0) @binding(1) var<storage, read> params: LidarParams;
@group(0) @binding(2) var<storage, read> drone_positions: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> ray_dirs: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read_write> hits: array<u32>;

fn cell_flat_idx(cell: vec3<i32>) -> u32 {
    return u32(cell.x)
        + u32(cell.y) * params.dims.x
        + u32(cell.z) * params.dims.x * params.dims.y;
}

fn cell_in_bounds(cell: vec3<i32>) -> bool {
    if (cell.x < 0 || cell.y < 0 || cell.z < 0) {
        return false;
    }
    return u32(cell.x) < params.dims.x
        && u32(cell.y) < params.dims.y
        && u32(cell.z) < params.dims.z;
}

fn ground_is_occupied(cell: vec3<i32>) -> bool {
    if (!cell_in_bounds(cell)) {
        return false;
    }
    let flat = cell_flat_idx(cell);
    let word = flat / 32u;
    let bit = flat % 32u;
    return (ground_bitset[word] & (1u << bit)) != 0u;
}

fn pack_entry(state: u32, flat: u32) -> u32 {
    return (state << 30u) | (flat & 0x3FFFFFFFu);
}

fn axis_t_max(step_sign: i32, origin: f32, cell: i32, dir: f32) -> f32 {
    if (step_sign == 0) {
        return 1e30;
    }
    var boundary: f32;
    if (step_sign > 0) {
        boundary = f32(cell + 1) - origin;
    } else {
        boundary = origin - f32(cell);
    }
    return boundary / abs(dir);
}

@compute @workgroup_size(8, 8, 1)
fn lidar(@builtin(global_invocation_id) gid: vec3<u32>) {
    let drone_idx = gid.x;
    let ray_idx = gid.y;
    if (drone_idx >= params.drone_count || ray_idx >= params.rays_per_scan) {
        return;
    }

    let origin = drone_positions[drone_idx].xyz;
    let dir = normalize(ray_dirs[ray_idx].xyz);
    let out_base = (drone_idx * params.rays_per_scan + ray_idx) * params.max_steps;

    var cell = vec3<i32>(floor(origin));
    let step_sign = vec3<i32>(
        i32(sign(dir.x)),
        i32(sign(dir.y)),
        i32(sign(dir.z)),
    );
    var t_max = vec3<f32>(
        axis_t_max(step_sign.x, origin.x, cell.x, dir.x),
        axis_t_max(step_sign.y, origin.y, cell.y, dir.y),
        axis_t_max(step_sign.z, origin.z, cell.z, dir.z),
    );
    let t_delta = vec3<f32>(
        select(1e30, 1.0 / abs(dir.x), dir.x != 0.0),
        select(1e30, 1.0 / abs(dir.y), dir.y != 0.0),
        select(1e30, 1.0 / abs(dir.z), dir.z != 0.0),
    );

    var step: u32 = 0u;
    loop {
        if (step >= params.max_steps) { break; }

        let in_bounds = cell_in_bounds(cell);
        let occupied = in_bounds && ground_is_occupied(cell);
        if (in_bounds) {
            let state: u32 = select(1u, 2u, occupied);
            hits[out_base + step] = pack_entry(state, cell_flat_idx(cell));
        } else {
            hits[out_base + step] = 0u;
        }

        if (occupied || !in_bounds) {
            // Sentinel-fill the remainder so CPU readers can scan to first zero.
            var s = step + 1u;
            loop {
                if (s >= params.max_steps) { break; }
                hits[out_base + s] = 0u;
                s = s + 1u;
            }
            return;
        }

        if (t_max.x < t_max.y && t_max.x < t_max.z) {
            cell.x = cell.x + step_sign.x;
            t_max.x = t_max.x + t_delta.x;
        } else if (t_max.y < t_max.z) {
            cell.y = cell.y + step_sign.y;
            t_max.y = t_max.y + t_delta.y;
        } else {
            cell.z = cell.z + step_sign.z;
            t_max.z = t_max.z + t_delta.z;
        }

        step = step + 1u;
    }
}
