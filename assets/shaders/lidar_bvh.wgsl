// Phase 2b scaffold: full bind-group declaration mirroring
// lidar_compute.wgsl (15 shared bindings) plus 3 BVH SSBOs (nodes,
// primitive indices, triangle vertices). Entry point is an empty
// no-op — the CWBVH8 stack-based traversal + Möller-Trumbore land in
// Phase 2c, at which point this shader replaces lidar_compute.wgsl on
// the per-drone occupancy write path.
//
// The bind group must be declared in full so naga compiles the
// pipeline cleanly; once the dispatch is queued in Phase 2c, the body
// will read drone positions / ray dirs / orientations and traverse the
// BVH against the triangle vertex buffer, writing hits via atomicOr
// into the same per-drone occupancy buffer as the DDA shader.

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
@group(0) @binding(7) var<storage, read_write> point_count: array<atomic<u32>>;
@group(0) @binding(8) var<storage, read_write> point_buffer: array<vec4<f32>>;
@group(0) @binding(9) var<storage, read> drone_scan: array<DroneScanParams>;
@group(0) @binding(10) var<storage, read_write> global_occupancy: array<atomic<u32>>;
@group(0) @binding(11) var<storage, read_write> local_active_cells: array<u32>;
@group(0) @binding(12) var<storage, read_write> local_active_count: array<atomic<u32>>;
@group(0) @binding(13) var<storage, read_write> global_active_cells: array<u32>;
@group(0) @binding(14) var<storage, read_write> global_active_count: array<atomic<u32>>;

// CWBVH8 node: 20 × u32 (80 bytes) packed from obvhs::cwbvh::node::CwBvhNode.
// Phase 2c will unpack via extractBits / shifts for the byte fields
// (`e`, `imask`, `child_meta`, `child_min_*`, `child_max_*`).
@group(0) @binding(15) var<storage, read> bvh_nodes: array<u32>;
@group(0) @binding(16) var<storage, read> bvh_primitive_indices: array<u32>;
@group(0) @binding(17) var<storage, read> bvh_triangle_vertices: array<vec4<f32>>;

@compute @workgroup_size(8, 8, 1)
fn lidar_bvh(@builtin(global_invocation_id) gid: vec3<u32>) {
    // Phase 2b: intentionally empty so the pipeline compiles. The
    // CWBVH8 traversal + Möller-Trumbore land in Phase 2c.
    //
    // Reference uses so naga keeps every binding live. WGSL strips
    // unreferenced bindings from the layout, which would silently
    // de-sync this shader from the Rust-side BindGroupLayout. Touching
    // each binding once (with no side effects) keeps them in.
    let drone_idx = gid.x;
    if (drone_idx >= params.drone_count) {
        return;
    }
    let pos = drone_positions[drone_idx];
    let orient = drone_orientations[drone_idx];
    let color = drone_colors[drone_idx];
    let scan = drone_scan[drone_idx];
    let ray = ray_dirs[scan.ray_offset];
    let bitset_word = ground_bitset[0];
    let node_word = bvh_nodes[0];
    let prim_idx = bvh_primitive_indices[0];
    let vert = bvh_triangle_vertices[0];

    // Touch every read_write binding so the layout includes them.
    let occ = atomicLoad(&local_occupancy[0]);
    let glb = atomicLoad(&global_occupancy[0]);
    let pc = atomicLoad(&point_count[0]);
    let lac = atomicLoad(&local_active_count[0]);
    let gac = atomicLoad(&global_active_count[0]);
    let pb = point_buffer[0];
    let lac_cell = local_active_cells[0];
    let gac_cell = global_active_cells[0];

    // Combine into a single guarded write that depends on the BVH
    // bindings (so they survive layout stripping) but never actually
    // executes in normal use — `params.drone_count` is at most 50.
    if (drone_idx == 0xFFFFFFFFu) {
        let sentinel = pos.x + orient.x + color.x + ray.x + f32(bitset_word) + f32(node_word)
            + f32(prim_idx) + vert.x + f32(occ) + f32(glb) + f32(pc) + f32(lac)
            + f32(gac) + pb.x + f32(lac_cell) + f32(gac_cell);
        if (sentinel == 0.0) {
            atomicStore(&point_count[0], 0u);
        }
    }
}
