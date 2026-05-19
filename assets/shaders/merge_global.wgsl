// Stage 9Ea: reduce per-drone occupancy into a single global occupancy
// SSBO. One thread per word index. The thread OR-folds each drone's
// word at that index and stores the union, so Free / Occupied state
// bits accumulate sticky across drones the same way they do inside one
// drone.

struct BuildParams {
    dims: vec4<u32>,
    drone_count: u32,
    voxel_size: f32,
    scale_factor: f32,
    max_instances: u32,
    drone_mask_lo: u32,
    drone_mask_hi: u32,
    connected_mask_lo: u32,
    connected_mask_hi: u32,
}

@group(0) @binding(0) var<storage, read> local_occupancy: array<u32>;
@group(0) @binding(1) var<storage, read> params: BuildParams;
@group(0) @binding(2) var<storage, read_write> global_occupancy: array<u32>;

@compute @workgroup_size(256, 1, 1)
fn merge_global(@builtin(global_invocation_id) gid: vec3<u32>) {
    let w = gid.x;
    let cells_per_drone = params.dims.x * params.dims.y * params.dims.z;
    let words_per_drone = (cells_per_drone + 15u) / 16u;
    if (w >= words_per_drone) {
        return;
    }

    var acc: u32 = 0u;
    for (var d: u32 = 0u; d < params.drone_count; d = d + 1u) {
        // Comms gate: only drones reachable from base contribute to the
        // merged central map. CPU-side BFS fills connected_mask each
        // frame; disabled mode leaves it all-ones so every drone
        // contributes (legacy behavior).
        let mask = select(params.connected_mask_lo, params.connected_mask_hi, d >= 32u);
        if (((mask >> (d % 32u)) & 1u) == 0u) {
            continue;
        }
        acc |= local_occupancy[d * words_per_drone + w];
    }
    global_occupancy[w] = acc;
}
