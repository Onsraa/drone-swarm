// Stage 4 GPU lidar. One thread per (drone, ray). The ray direction is
// stored in the drone's local frame (forward = -Z), rotated by the
// drone's world-quaternion before traversal so each drone sweeps its
// own forward cone. Trail entries pack (state << 30) | flat_cell_idx.
// State encoding: 0 = sentinel, 1 = Free, 2 = Occupied.
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
@group(0) @binding(5) var<storage, read> drone_orientations: array<vec4<f32>>;
@group(0) @binding(6) var<storage, read_write> local_occupancy: array<atomic<u32>>;

// Per-drone occupancy is 2 bits per cell, 16 cells per u32.
//   bit 0 = Free flag, bit 1 = Occupied flag.
// Both flags are sticky under atomicOr; Unknown = 0b00.
fn mark_cell_state(drone_idx: u32, flat: u32, state_bits: u32) {
    let cells_per_drone = params.dims.x * params.dims.y * params.dims.z;
    let words_per_drone = (cells_per_drone + 15u) / 16u;
    let word_idx = drone_idx * words_per_drone + flat / 16u;
    let bit_offset = (flat % 16u) * 2u;
    let mask = state_bits << bit_offset;
    atomicOr(&local_occupancy[word_idx], mask);
}

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

// Rotate `v` by quaternion `q` stored as (x, y, z, w).
fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let qv = vec3<f32>(q.x, q.y, q.z);
    let t = 2.0 * cross(qv, v);
    return v + q.w * t + cross(qv, t);
}

@compute @workgroup_size(8, 8, 1)
fn lidar(@builtin(global_invocation_id) gid: vec3<u32>) {
    let drone_idx = gid.x;
    let ray_idx = gid.y;
    if (drone_idx >= params.drone_count || ray_idx >= params.rays_per_scan) {
        return;
    }

    let origin = drone_positions[drone_idx].xyz;
    let local_dir = ray_dirs[ray_idx].xyz;
    let world_dir = normalize(quat_rotate(drone_orientations[drone_idx], local_dir));
    let out_base = (drone_idx * params.rays_per_scan + ray_idx) * params.max_steps;

    var cell = vec3<i32>(floor(origin));
    let step_sign = vec3<i32>(
        i32(sign(world_dir.x)),
        i32(sign(world_dir.y)),
        i32(sign(world_dir.z)),
    );
    var t_max = vec3<f32>(
        axis_t_max(step_sign.x, origin.x, cell.x, world_dir.x),
        axis_t_max(step_sign.y, origin.y, cell.y, world_dir.y),
        axis_t_max(step_sign.z, origin.z, cell.z, world_dir.z),
    );
    let t_delta = vec3<f32>(
        select(1e30, 1.0 / abs(world_dir.x), world_dir.x != 0.0),
        select(1e30, 1.0 / abs(world_dir.y), world_dir.y != 0.0),
        select(1e30, 1.0 / abs(world_dir.z), world_dir.z != 0.0),
    );

    var step: u32 = 0u;
    loop {
        if (step >= params.max_steps) { break; }

        let in_bounds = cell_in_bounds(cell);
        let occupied = in_bounds && ground_is_occupied(cell);
        if (in_bounds) {
            let state: u32 = select(1u, 2u, occupied);
            let flat = cell_flat_idx(cell);
            hits[out_base + step] = pack_entry(state, flat);
            // Mirror the state into the per-drone occupancy SSBO. State
            // bits land at `flat`'s 2-bit slot via atomicOr; Stage 9B's
            // render pass reads this buffer instead of the CPU LocalMap.
            mark_cell_state(drone_idx, flat, state);
        } else {
            hits[out_base + step] = 0u;
        }

        if (occupied || !in_bounds) {
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
