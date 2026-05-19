// Stage 9B compute pass. Reads the per-drone occupancy SSBO (2 bits per
// cell) and emits one InstanceData per Occupied cell into a shared
// VERTEX|STORAGE buffer. Each entry is two `vec4<f32>`s — `pos_scale`
// (cube center + size) and `color` (drone tint) — matching the existing
// instanced voxel vertex layout.
//
// Dispatch shape: 1D workgroup of 64 threads. Outer dispatch dims map
// `(gid.x, gid.y) = (cell_flat, drone_idx)`.

struct BuildParams {
    dims: vec4<u32>,         // (x, y, z, _pad)
    drone_count: u32,
    voxel_size: f32,
    scale_factor: f32,
    max_instances: u32,
}

@group(0) @binding(0) var<storage, read> local_occupancy: array<u32>;
@group(0) @binding(1) var<storage, read> params: BuildParams;
@group(0) @binding(2) var<storage, read> drone_colors: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read_write> instance_count: atomic<u32>;
@group(0) @binding(4) var<storage, read_write> instance_buffer: array<vec4<f32>>;

@compute @workgroup_size(64, 1, 1)
fn build(@builtin(global_invocation_id) gid: vec3<u32>) {
    let cell_flat = gid.x;
    let drone_idx = gid.y;
    let cells_per_drone = params.dims.x * params.dims.y * params.dims.z;
    if (cell_flat >= cells_per_drone || drone_idx >= params.drone_count) {
        return;
    }

    let words_per_drone = (cells_per_drone + 15u) / 16u;
    let word_idx = drone_idx * words_per_drone + cell_flat / 16u;
    let bit_offset = (cell_flat % 16u) * 2u;
    let state = (local_occupancy[word_idx] >> bit_offset) & 3u;
    // Only Occupied cells (state == 2) become instances.
    if (state != 2u) {
        return;
    }

    let dx = params.dims.x;
    let dy = params.dims.y;
    let plane = dx * dy;
    let z = cell_flat / plane;
    let rem = cell_flat % plane;
    let y = rem / dx;
    let x = rem % dx;
    let half = params.voxel_size * 0.5;
    let pos = vec3<f32>(f32(x), f32(y), f32(z)) * params.voxel_size + vec3<f32>(half);
    let size = params.voxel_size * params.scale_factor;

    let slot = atomicAdd(&instance_count, 1u);
    if (slot >= params.max_instances) {
        return;
    }
    let base = slot * 2u;
    instance_buffer[base] = vec4<f32>(pos, size);
    instance_buffer[base + 1u] = drone_colors[drone_idx];
}
