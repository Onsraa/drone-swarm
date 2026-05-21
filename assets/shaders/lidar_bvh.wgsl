// GPU lidar — BVH path. One thread per (drone, ray). Replaces
// lidar_compute.wgsl's DDA traversal with a CWBVH8 stack-based walk
// against the mesh ground truth.
//
// Ray construction matches the DDA shader: drone_positions[] is in
// voxel-grid space, ray_dirs[] is in the drone's local frame
// (forward = -Z), drone_orientations[] is a unit quaternion. The ray
// is converted to world meters before BVH traversal (the BVH was
// built from world-space mesh vertices).
//
// On hit, the world-space hit point is converted back to a cell
// index and atomicOr'd into the per-drone occupancy SSBO; if the
// drone is comms-connected, the same bit is OR'd into the global
// occupancy SSBO. A point is emitted into the spray buffer for
// visualization. Free-cell marking along the ray is intentionally
// omitted for Phase 2c — the BVH returns only the hit, not the
// cells traversed; Phase 4 may revisit if coverage parity matters.
//
// CWBVH8 reference: Ylitie 2017 HPG paper "Efficient Incoherent Ray
// Traversal on GPUs Through Compressed Wide BVHs"; algorithm ported
// from obvhs::cwbvh (MIT/Apache 2.0).

struct LidarParams {
    dims: vec4<u32>,
    max_steps: u32,
    rays_per_scan: u32,
    drone_count: u32,
    voxel_size: f32,
    drone_mask_lo: u32,
    drone_mask_hi: u32,
    max_points: u32,
    connected_mask_lo: u32,
    connected_mask_hi: u32,
    _pad0: u32,
    _pad1: u32,
};

struct DroneScanParams {
    ray_offset: u32,
    ray_count: u32,
    max_steps: u32,
    scan_interval: u32,
};

@group(0) @binding(0) var<storage, read> ground_bitset: array<u32>;
@group(0) @binding(1) var<storage, read> params: LidarParams;
@group(0) @binding(2) var<storage, read> drone_positions: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> ray_dirs: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read> drone_orientations: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read_write> local_occupancy: array<atomic<u32>>;
@group(0) @binding(6) var<storage, read> drone_colors: array<vec4<f32>>;
@group(0) @binding(7) var<storage, read_write> point_count: atomic<u32>;
@group(0) @binding(8) var<storage, read_write> point_buffer: array<vec4<f32>>;
@group(0) @binding(9) var<storage, read> drone_scan: array<DroneScanParams>;
@group(0) @binding(10) var<storage, read_write> global_occupancy: array<atomic<u32>>;
@group(0) @binding(11) var<storage, read_write> local_active_cells: array<u32>;
@group(0) @binding(12) var<storage, read_write> local_active_count: array<atomic<u32>>;
@group(0) @binding(13) var<storage, read_write> global_active_cells: array<u32>;
@group(0) @binding(14) var<storage, read_write> global_active_count: atomic<u32>;
@group(0) @binding(15) var<storage, read> bvh_nodes: array<u32>;
@group(0) @binding(16) var<storage, read> bvh_primitive_indices: array<u32>;
@group(0) @binding(17) var<storage, read> bvh_triangle_vertices: array<vec4<f32>>;

const MAX_LOCAL_ACTIVE_PER_DRONE: u32 = 200000u;
const MAX_GLOBAL_ACTIVE: u32 = 500000u;
const STACK_SIZE: u32 = 32u;
const TRAVERSAL_LOOP_CAP: u32 = 1024u;
const EPSILON: f32 = 0.0001;
const TMAX_MISS: f32 = 1.0e30;

fn extract_byte(x: u32, b: u32) -> u32 {
    return (x >> (b * 8u)) & 0xffu;
}

fn get_child_byte(lo: u32, hi: u32, ch: u32) -> u32 {
    if (ch < 4u) {
        return extract_byte(lo, ch);
    }
    return extract_byte(hi, ch - 4u);
}

fn safe_inv(x: f32) -> f32 {
    if (abs(x) <= 1.0e-20) {
        return select(-1.0e20, 1.0e20, x >= 0.0);
    }
    return 1.0 / x;
}

fn ray_octant_inv4(dir: vec3<f32>) -> u32 {
    var oct: u32 = 0u;
    if (dir.x >= 0.0) { oct = oct | 0x04040404u; }
    if (dir.y >= 0.0) { oct = oct | 0x02020202u; }
    if (dir.z >= 0.0) { oct = oct | 0x01010101u; }
    return oct;
}

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let qv = vec3<f32>(q.x, q.y, q.z);
    let t = 2.0 * cross(qv, v);
    return v + q.w * t + cross(qv, t);
}

// CWBVH8 node packed into 20 u32. See CwBvhNode in obvhs::cwbvh::node.
struct CwNode {
    p: vec3<f32>,
    e: vec3<u32>,
    imask: u32,
    child_base_idx: u32,
    primitive_base_idx: u32,
    meta_lo: u32,
    meta_hi: u32,
    min_x_lo: u32, min_x_hi: u32,
    max_x_lo: u32, max_x_hi: u32,
    min_y_lo: u32, min_y_hi: u32,
    max_y_lo: u32, max_y_hi: u32,
    min_z_lo: u32, min_z_hi: u32,
    max_z_lo: u32, max_z_hi: u32,
};

fn load_node(node_idx: u32) -> CwNode {
    let base = node_idx * 20u;
    var n: CwNode;
    n.p = vec3<f32>(
        bitcast<f32>(bvh_nodes[base + 0u]),
        bitcast<f32>(bvh_nodes[base + 1u]),
        bitcast<f32>(bvh_nodes[base + 2u]),
    );
    let w3 = bvh_nodes[base + 3u];
    n.e = vec3<u32>(extract_byte(w3, 0u), extract_byte(w3, 1u), extract_byte(w3, 2u));
    n.imask = extract_byte(w3, 3u);
    n.child_base_idx = bvh_nodes[base + 4u];
    n.primitive_base_idx = bvh_nodes[base + 5u];
    n.meta_lo = bvh_nodes[base + 6u];
    n.meta_hi = bvh_nodes[base + 7u];
    n.min_x_lo = bvh_nodes[base + 8u]; n.min_x_hi = bvh_nodes[base + 9u];
    n.max_x_lo = bvh_nodes[base + 10u]; n.max_x_hi = bvh_nodes[base + 11u];
    n.min_y_lo = bvh_nodes[base + 12u]; n.min_y_hi = bvh_nodes[base + 13u];
    n.max_y_lo = bvh_nodes[base + 14u]; n.max_y_hi = bvh_nodes[base + 15u];
    n.min_z_lo = bvh_nodes[base + 16u]; n.min_z_hi = bvh_nodes[base + 17u];
    n.max_z_lo = bvh_nodes[base + 18u]; n.max_z_hi = bvh_nodes[base + 19u];
    return n;
}

fn node_extent(e: vec3<u32>) -> vec3<f32> {
    return vec3<f32>(
        bitcast<f32>(e.x << 23u),
        bitcast<f32>(e.y << 23u),
        bitcast<f32>(e.z << 23u),
    );
}

// Per-child AABB hit test. Returns the hit_mask bit pattern for this node:
// high 8 bits = inner child hits, low 24 bits = leaf primitive bits.
fn intersect_node(
    node: CwNode,
    ro: vec3<f32>,
    rd: vec3<f32>,
    rd_inv: vec3<f32>,
    tmax: f32,
    oct_inv4: u32,
) -> u32 {
    let extent = node_extent(node.e);
    let adj_dir_inv = extent * rd_inv;
    let adj_origin = (node.p - ro) * rd_inv;

    let rdx = rd.x < 0.0;
    let rdy = rd.y < 0.0;
    let rdz = rd.z < 0.0;

    var hit_mask: u32 = 0u;

    for (var ch: u32 = 0u; ch < 8u; ch = ch + 1u) {
        let q_lo_x = get_child_byte(node.min_x_lo, node.min_x_hi, ch);
        let q_lo_y = get_child_byte(node.min_y_lo, node.min_y_hi, ch);
        let q_lo_z = get_child_byte(node.min_z_lo, node.min_z_hi, ch);
        let q_hi_x = get_child_byte(node.max_x_lo, node.max_x_hi, ch);
        let q_hi_y = get_child_byte(node.max_y_lo, node.max_y_hi, ch);
        let q_hi_z = get_child_byte(node.max_z_lo, node.max_z_hi, ch);

        let x_min = select(q_lo_x, q_hi_x, rdx);
        let x_max = select(q_hi_x, q_lo_x, rdx);
        let y_min = select(q_lo_y, q_hi_y, rdy);
        let y_max = select(q_hi_y, q_lo_y, rdy);
        let z_min = select(q_lo_z, q_hi_z, rdz);
        let z_max = select(q_hi_z, q_lo_z, rdz);

        var tmin3 = vec3<f32>(f32(x_min), f32(y_min), f32(z_min));
        var tmax3 = vec3<f32>(f32(x_max), f32(y_max), f32(z_max));
        tmin3 = tmin3 * adj_dir_inv + adj_origin;
        tmax3 = tmax3 * adj_dir_inv + adj_origin;

        let entry = max(max(max(tmin3.x, tmin3.y), tmin3.z), EPSILON);
        let exit = min(min(min(tmax3.x, tmax3.y), tmax3.z), tmax);

        if (entry <= exit) {
            let meta_byte = get_child_byte(node.meta_lo, node.meta_hi, ch);
            let inner_test = meta_byte & (meta_byte << 1u) & 0x10u;
            let inner_mask_byte = select(0u, 0xffu, inner_test != 0u);
            let oct_inv_byte = extract_byte(oct_inv4, ch & 3u);
            let bit_index = (meta_byte ^ (oct_inv_byte & inner_mask_byte)) & 0x1fu;
            let child_bits = (meta_byte >> 5u) & 0x7u;
            hit_mask = hit_mask | (child_bits << bit_index);
        }
    }
    return hit_mask;
}

// Möller-Trumbore ray-triangle intersection. Returns t on hit (>= 0),
// TMAX_MISS on miss. Matches obvhs::triangle::Triangle::intersect.
fn triangle_intersect(
    v0: vec3<f32>,
    v1: vec3<f32>,
    v2: vec3<f32>,
    ro: vec3<f32>,
    rd: vec3<f32>,
    tmax: f32,
) -> f32 {
    let e1 = v0 - v1;
    let e2 = v2 - v0;
    let n = cross(e1, e2);
    let c = v0 - ro;
    let r = cross(rd, c);
    let n_dot_d = dot(n, rd);
    if (n_dot_d == 0.0) {
        return TMAX_MISS;
    }
    let inv_det = 1.0 / n_dot_d;
    let u = dot(r, e2) * inv_det;
    let v = dot(r, e1) * inv_det;
    let w = 1.0 - u - v;
    if (u < 0.0 || v < 0.0 || w < 0.0) {
        return TMAX_MISS;
    }
    let t = dot(n, c) * inv_det;
    if (t >= 0.0 && t <= tmax) {
        return t;
    }
    return TMAX_MISS;
}

// Stack-based CWBVH8 closest-hit traversal. Returns (t, primitive_id)
// — primitive_id = 0xffffffff on miss (encoded via bitcast<f32>).
struct TraversalResult {
    t: f32,
    primitive_id: u32,
};

fn traverse_ray(ro: vec3<f32>, rd: vec3<f32>, tmax_init: f32) -> TraversalResult {
    let rd_inv = vec3<f32>(safe_inv(rd.x), safe_inv(rd.y), safe_inv(rd.z));
    let oct_inv4 = ray_octant_inv4(rd);

    var stack: array<vec2<u32>, 32>;
    var stack_size: u32 = 0u;
    var current_group: vec2<u32> = vec2<u32>(0u, 0x80000000u);
    var primitive_group: vec2<u32> = vec2<u32>(0u, 0u);
    var closest_t: f32 = tmax_init;
    var hit_prim: u32 = 0xffffffffu;

    var loop_count: u32 = 0u;
    loop {
        loop_count = loop_count + 1u;
        if (loop_count > TRAVERSAL_LOOP_CAP) { break; }

        // Drain primitive group.
        while (primitive_group.y != 0u) {
            let local_idx = u32(firstLeadingBit(primitive_group.y));
            primitive_group.y = primitive_group.y & ~(1u << local_idx);
            let global_idx = primitive_group.x + local_idx;
            let tri_idx = bvh_primitive_indices[global_idx];
            let v0 = bvh_triangle_vertices[tri_idx * 3u + 0u].xyz;
            let v1 = bvh_triangle_vertices[tri_idx * 3u + 1u].xyz;
            let v2 = bvh_triangle_vertices[tri_idx * 3u + 2u].xyz;
            let t = triangle_intersect(v0, v1, v2, ro, rd, closest_t);
            if (t < closest_t) {
                closest_t = t;
                hit_prim = global_idx;
            }
        }

        if ((current_group.y & 0xff000000u) != 0u) {
            let hits_imask = current_group.y;
            let child_index_offset = u32(firstLeadingBit(hits_imask));
            let child_index_base = current_group.x;
            current_group.y = current_group.y & ~(1u << child_index_offset);

            if ((current_group.y & 0xff000000u) != 0u) {
                if (stack_size < STACK_SIZE) {
                    stack[stack_size] = current_group;
                    stack_size = stack_size + 1u;
                }
            }

            let slot_index = (child_index_offset - 24u) ^ (oct_inv4 & 0xffu);
            let lo_mask = ~(0xffffffffu << slot_index);
            let relative_index = countOneBits(hits_imask & lo_mask);
            let child_node_index = child_index_base + relative_index;

            let node = load_node(child_node_index);
            let hitmask = intersect_node(node, ro, rd, rd_inv, closest_t, oct_inv4);

            current_group.x = node.child_base_idx;
            primitive_group.x = node.primitive_base_idx;
            current_group.y = (hitmask & 0xff000000u) | node.imask;
            primitive_group.y = hitmask & 0x00ffffffu;
        } else {
            current_group = vec2<u32>(0u, 0u);
        }

        if (primitive_group.y == 0u && (current_group.y & 0xff000000u) == 0u) {
            if (stack_size == 0u) { break; }
            stack_size = stack_size - 1u;
            current_group = stack[stack_size];
        }
    }

    var out: TraversalResult;
    out.t = closest_t;
    out.primitive_id = hit_prim;
    return out;
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

fn mark_cell_occupied(drone_idx: u32, flat: u32) {
    let cells_per_drone = params.dims.x * params.dims.y * params.dims.z;
    let words_per_drone = (cells_per_drone + 15u) / 16u;
    let word_idx = drone_idx * words_per_drone + flat / 16u;
    let bit_offset = (flat % 16u) * 2u;
    let mask = 2u << bit_offset;
    let prev = atomicOr(&local_occupancy[word_idx], mask);
    let was_occupied = ((prev >> bit_offset) & 0x2u) != 0u;

    if (!was_occupied) {
        let slot = atomicAdd(&local_active_count[drone_idx], 1u);
        if (slot < MAX_LOCAL_ACTIVE_PER_DRONE) {
            local_active_cells[drone_idx * MAX_LOCAL_ACTIVE_PER_DRONE + slot] = flat;
        }
    }

    let comms = select(params.connected_mask_lo, params.connected_mask_hi, drone_idx >= 32u);
    if (((comms >> (drone_idx % 32u)) & 1u) != 0u) {
        let global_word = flat / 16u;
        let prev_global = atomicOr(&global_occupancy[global_word], mask);
        let was_global = ((prev_global >> bit_offset) & 0x2u) != 0u;
        if (!was_global) {
            let g_slot = atomicAdd(&global_active_count, 1u);
            if (g_slot < MAX_GLOBAL_ACTIVE) {
                global_active_cells[g_slot] = flat;
            }
        }
    }
}

fn emit_point(drone_idx: u32, hit_world: vec3<f32>) {
    let mask = select(params.drone_mask_lo, params.drone_mask_hi, drone_idx >= 32u);
    if (((mask >> (drone_idx % 32u)) & 1u) == 0u) {
        return;
    }
    let slot = atomicAdd(&point_count, 1u);
    if (slot >= params.max_points) { return; }
    let base = slot * 2u;
    let spray_px: f32 = 4.0;
    point_buffer[base] = vec4<f32>(hit_world, spray_px);
    point_buffer[base + 1u] = drone_colors[drone_idx];
}

@compute @workgroup_size(8, 8, 1)
fn lidar_bvh(@builtin(global_invocation_id) gid: vec3<u32>) {
    let drone_idx = gid.x;
    if (drone_idx >= params.drone_count) { return; }
    let scan = drone_scan[drone_idx];
    let ray_local_idx = gid.y;
    if (ray_local_idx >= scan.ray_count) { return; }

    let ray_buf_idx = scan.ray_offset + ray_local_idx;
    let local_dir = ray_dirs[ray_buf_idx].xyz;

    // drone_positions is voxel-grid space; BVH is world meters.
    let origin_grid = drone_positions[drone_idx].xyz;
    let origin_world = origin_grid * params.voxel_size;
    let world_dir = normalize(quat_rotate(drone_orientations[drone_idx], local_dir));

    // Max ray length in world meters, matching DDA scan range.
    let max_t_world = f32(scan.max_steps) * params.voxel_size;

    let result = traverse_ray(origin_world, world_dir, max_t_world);
    if (result.primitive_id == 0xffffffffu) { return; }

    let hit_world = origin_world + world_dir * result.t;
    let hit_cell = vec3<i32>(floor(hit_world / params.voxel_size));
    if (!cell_in_bounds(hit_cell)) { return; }

    let flat = cell_flat_idx(hit_cell);
    mark_cell_occupied(drone_idx, flat);
    emit_point(drone_idx, hit_world);
}

// Reference unused bindings so naga doesn't strip them.
// `ground_bitset` is shared with the DDA layout but the BVH path
// doesn't read it. Touching it once with a guarded no-op keeps the
// layout in sync.
fn _layout_anchor() {
    if (params.drone_count == 0xFFFFFFFFu) {
        let unused = ground_bitset[0];
        if (unused != 0u) {
            atomicStore(&point_count, unused);
        }
    }
}
