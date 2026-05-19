// Per-drone local-map instance builder.
//
// Reads the per-drone active-cell list (a flat list of cell flat-
// indices populated by `lidar_compute` on each Unknown -> Occupied
// transition) and emits one InstanceData per active cell into the
// shared VERTEX|STORAGE buffer. Each instance is two `vec4<f32>`s —
// `pos_scale` (point center + screen-pixel radius) and `color` (drone
// tint).
//
// Dispatch shape: (MAX_LOCAL_ACTIVE_PER_DRONE / 256, MAX_DRONES, 1)
// workgroups of (256, 1, 1) threads. Each thread reads its slot from
// the active list and early-returns if the slot is past the per-drone
// live count. This replaces the old "1 thread per native cell" sweep
// (~491 M invocations at 50 drones × 9.83 M cells) with ~10 M
// pre-allocated invocations, most of which early-return cheaply.

struct BuildParams {
    dims: vec4<u32>,         // (x, y, z, _pad)
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
@group(0) @binding(1) var<storage, read> drone_colors: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> instance_count: atomic<u32>;
@group(0) @binding(3) var<storage, read_write> instance_buffer: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read> active_cells: array<u32>;
@group(0) @binding(5) var<storage, read_write> active_count: array<atomic<u32>>;

const MAX_LOCAL_ACTIVE_PER_DRONE: u32 = 200000u;

@compute @workgroup_size(256, 1, 1)
fn build(@builtin(global_invocation_id) gid: vec3<u32>) {
    let slot = gid.x;
    let drone_idx = gid.y;
    if (drone_idx >= params.drone_count) {
        return;
    }
    // Per-drone visibility mask: bit `drone_idx` must be set, else
    // this drone contributes no instances this frame.
    let mask_word = select(params.drone_mask_lo, params.drone_mask_hi, drone_idx >= 32u);
    if (((mask_word >> (drone_idx % 32u)) & 1u) == 0u) {
        return;
    }
    let count = atomicLoad(&active_count[drone_idx]);
    let cap = min(count, MAX_LOCAL_ACTIVE_PER_DRONE);
    if (slot >= cap) {
        return;
    }

    let cell_flat = active_cells[drone_idx * MAX_LOCAL_ACTIVE_PER_DRONE + slot];

    let dx = params.dims.x;
    let dy = params.dims.y;
    let plane = dx * dy;
    let z = cell_flat / plane;
    let rem = cell_flat % plane;
    let y = rem / dx;
    let x = rem % dx;
    let half = params.voxel_size * 0.5;
    let pos = vec3<f32>(f32(x), f32(y), f32(z)) * params.voxel_size + vec3<f32>(half);
    let size = params.scale_factor;

    let inst_slot = atomicAdd(&instance_count, 1u);
    if (inst_slot >= params.max_instances) {
        return;
    }
    let base = inst_slot * 2u;
    instance_buffer[base] = vec4<f32>(pos, size);
    instance_buffer[base + 1u] = drone_colors[drone_idx];
}
