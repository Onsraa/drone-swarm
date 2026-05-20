// Writes dispatch_indirect args for build_local + build_global by
// reading the per-drone and global active-cell counts left over from
// `lidar_compute`. One workgroup is enough: 64 threads cooperatively
// find max(local_active_count[0..MAX_DRONES]) into shared memory, the
// first thread divides it (and the global count) by WORKGROUP_SIZE_X
// and writes the two `DispatchIndirectArgs` records into
// `build_indirect`. Both build passes then `dispatch_indirect` from
// slot 0 and slot 1.

const MAX_DRONES: u32 = 50u;
const WORKGROUP_SIZE_X: u32 = 256u; // matches build_local + build_global @workgroup_size(256)

@group(0) @binding(0) var<storage, read> local_count: array<atomic<u32>>;
@group(0) @binding(1) var<storage, read> global_count: atomic<u32>;
@group(0) @binding(2) var<storage, read_write> build_indirect: array<u32>;

var<workgroup> shared_max: array<u32, 64>;

@compute @workgroup_size(64, 1, 1)
fn prepare(
    @builtin(local_invocation_id) lid: vec3<u32>,
    @builtin(global_invocation_id) gid: vec3<u32>,
) {
    let lane = lid.x;
    // Each lane scans a strided slice of the 50 drone slots, keeping
    // its running max. With 50 drones and 64 lanes most lanes do 1
    // load; a few do 0.
    var local_max: u32 = 0u;
    var i: u32 = lane;
    loop {
        if (i >= MAX_DRONES) { break; }
        let v = atomicLoad(&local_count[i]);
        if (v > local_max) { local_max = v; }
        i = i + 64u;
    }
    shared_max[lane] = local_max;
    workgroupBarrier();

    // Tree-reduce 64 -> 1 in shared memory.
    var stride: u32 = 32u;
    loop {
        if (stride == 0u) { break; }
        if (lane < stride) {
            let a = shared_max[lane];
            let b = shared_max[lane + stride];
            shared_max[lane] = select(a, b, b > a);
        }
        workgroupBarrier();
        stride = stride >> 1u;
    }

    if (lane == 0u) {
        let max_local = shared_max[0];
        let local_groups = (max_local + WORKGROUP_SIZE_X - 1u) / WORKGROUP_SIZE_X;
        // build_local args (slot 0): dispatches a 2D grid of
        // (local_groups, MAX_DRONES, 1). The shader's gid.y indexes the
        // drone; threads past that drone's actual active count
        // early-return.
        build_indirect[0] = local_groups;
        build_indirect[1] = MAX_DRONES;
        build_indirect[2] = 1u;
        build_indirect[3] = 0u; // pad

        let g = atomicLoad(&global_count);
        let global_groups = (g + WORKGROUP_SIZE_X - 1u) / WORKGROUP_SIZE_X;
        // build_global args (slot 1): 1D dispatch of (global_groups, 1, 1).
        build_indirect[4] = global_groups;
        build_indirect[5] = 1u;
        build_indirect[6] = 1u;
        build_indirect[7] = 0u; // pad
    }
}
