// Central-map instance builder. Reads the global active-cell list
// (cell flat-indices appended by `lidar_compute` the first time ANY
// comms-connected drone flips the global Occupied bit for a cell)
// and emits one InstanceData per active cell. Dispatch shape
// (MAX_GLOBAL_ACTIVE / 256, 1, 1) workgroups of (256, 1, 1) threads;
// slots past the live count early-return.

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

@group(0) @binding(0) var<storage, read> params: BuildParams;
@group(0) @binding(1) var<storage, read_write> instance_count: atomic<u32>;
@group(0) @binding(2) var<storage, read_write> instance_buffer: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> active_cells: array<u32>;
@group(0) @binding(4) var<storage, read_write> active_count: atomic<u32>;

const GLOBAL_COLOR: vec4<f32> = vec4<f32>(0.35, 0.7, 0.85, 0.85);
const GLOBAL_POINT_PX: f32 = 2.0;
const GLOBAL_MAX_INSTANCES: u32 = 1000000u;
const MAX_GLOBAL_ACTIVE: u32 = 500000u;

@compute @workgroup_size(256, 1, 1)
fn build_global(@builtin(global_invocation_id) gid: vec3<u32>) {
    let slot = gid.x;
    let count = atomicLoad(&active_count);
    let cap = min(count, MAX_GLOBAL_ACTIVE);
    if (slot >= cap) {
        return;
    }
    let cell_flat = active_cells[slot];

    let dx = params.dims.x;
    let dy = params.dims.y;
    let plane = dx * dy;
    let z = cell_flat / plane;
    let rem = cell_flat % plane;
    let y = rem / dx;
    let x = rem % dx;
    let half = params.voxel_size * 0.5;
    let pos = vec3<f32>(f32(x), f32(y), f32(z)) * params.voxel_size + vec3<f32>(half);
    let size = GLOBAL_POINT_PX;

    let inst_slot = atomicAdd(&instance_count, 1u);
    if (inst_slot >= GLOBAL_MAX_INSTANCES) {
        return;
    }
    let base = inst_slot * 2u;
    instance_buffer[base] = vec4<f32>(pos, size);
    instance_buffer[base + 1u] = GLOBAL_COLOR;
}
