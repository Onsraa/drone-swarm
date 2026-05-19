// Stage 2 sanity shader. Counts set bits in the ground-truth bitset via
// atomic-add and writes the total to `output[0]`. The CPU-side compute
// node clears the buffer before each dispatch so the count is fresh per
// frame. Replaced by an Amanatides-Woo lidar traversal in Stage 3.

@group(0) @binding(0)
var<storage, read> ground_bitset: array<u32>;

@group(0) @binding(1)
var<storage, read_write> output: array<atomic<u32>>;

@compute @workgroup_size(64)
fn count(@builtin(global_invocation_id) gid: vec3<u32>) {
    let word_idx = gid.x;
    if (word_idx >= arrayLength(&ground_bitset)) {
        return;
    }
    let bits = ground_bitset[word_idx];
    atomicAdd(&output[0], countOneBits(bits));
}
