// Stage 9Eb: build the central-map instance buffer from the global
// occupancy SSBO. One thread per cell; Occupied cells emit one
// InstanceData entry with the shared global-map color.

struct BuildParams {
    dims: vec4<u32>,
    drone_count: u32,
    voxel_size: f32,
    scale_factor: f32,
    max_instances: u32,
    drone_mask_lo: u32,
    drone_mask_hi: u32,
    _pad0: u32,
    _pad1: u32,
}

@group(0) @binding(0) var<storage, read> global_occupancy: array<u32>;
@group(0) @binding(1) var<storage, read> params: BuildParams;
@group(0) @binding(2) var<storage, read_write> instance_count: atomic<u32>;
@group(0) @binding(3) var<storage, read_write> instance_buffer: array<vec4<f32>>;

const GLOBAL_COLOR: vec4<f32> = vec4<f32>(0.1, 0.85, 1.0, 0.7);
const GLOBAL_SCALE_FACTOR: f32 = 1.01;
// Max instances the buffer holds (matches the CPU-side allocation).
const GLOBAL_MAX_INSTANCES: u32 = 1000000u;

@compute @workgroup_size(256, 1, 1)
fn build_global(@builtin(global_invocation_id) gid: vec3<u32>) {
    let cell_flat = gid.x;
    let cells_per_drone = params.dims.x * params.dims.y * params.dims.z;
    if (cell_flat >= cells_per_drone) {
        return;
    }

    let word_idx = cell_flat / 16u;
    let bit_offset = (cell_flat % 16u) * 2u;
    let state = (global_occupancy[word_idx] >> bit_offset) & 3u;
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
    let size = params.voxel_size * GLOBAL_SCALE_FACTOR;

    let slot = atomicAdd(&instance_count, 1u);
    if (slot >= GLOBAL_MAX_INSTANCES) {
        return;
    }
    let base = slot * 2u;
    instance_buffer[base] = vec4<f32>(pos, size);
    instance_buffer[base + 1u] = GLOBAL_COLOR;
}
