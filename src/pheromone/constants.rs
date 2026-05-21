/// Pheromone-field downsample relative to native voxel grid.
/// At voxel_size = 1 m and DOWNSAMPLE = 8, each pheromone cell covers
/// an 8 m × 8 m × 8 m block. For a 640×24×640 world that's an 80×3×80
/// scalar grid ≈ 19 K cells per channel × 4 bytes = 76 KB per channel.
pub const DOWNSAMPLE: u32 = 8;

/// Continuous decay rate in 1/s. The per-frame factor used by
/// `decay_pheromone` is `exp(-DECAY_RATE * dt)`. Half-life ≈ 6 s.
pub const DECAY_RATE: f32 = 0.115;

/// Per-frame Laplacian diffusion rate. `new = (1-rate)*here + rate*mean6`
/// applied each frame. At 60 Hz this softens edges over ~0.5 s without
/// erasing the trail.
pub const DIFFUSION_RATE: f32 = 0.015;

/// Pheromone deposited per frame into the drone's current cell, per
/// channel. Scouts mark "I came through here" so other scouts repel and
/// mappers follow. Mappers mark "I've detail-mapped this region" so
/// peer mappers can avoid duplication. Anchors hover, depositing would
/// pile up at one cell.
pub const DEPOSIT_SCOUT_PER_FRAME: f32 = 5.0;
pub const DEPOSIT_MAPPER_PER_FRAME: f32 = 3.0;
pub const DEPOSIT_ANCHOR_PER_FRAME: f32 = 0.0;

/// Fraction of the drone's deposit that bleeds into each of the six
/// face-neighbour cells. Combined with per-frame diffusion this gives
/// smooth gradients without one-frame deposit spikes.
pub const DEPOSIT_NEIGHBOR_FRACTION: f32 = 0.25;
