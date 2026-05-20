/// Pheromone-field downsample relative to native voxel grid.
/// At voxel_size = 1 m and DOWNSAMPLE = 8, each pheromone cell covers
/// an 8 m × 8 m × 8 m block. For a 640×24×640 world that's an 80×3×80
/// scalar grid ≈ 19 K cells × 4 bytes = 76 KB. Cheap to decay + deposit
/// every frame on CPU.
pub const DOWNSAMPLE: u32 = 8;

/// Continuous decay rate in 1/s. The per-frame factor used by
/// `decay_pheromone` is `exp(-DECAY_RATE * dt)`. At dt = 1/120 s the
/// factor is ≈ 0.999, half-life ≈ 6 s. Slow enough that trails persist
/// long enough for a slow Mapper to follow them; fast enough that the
/// colony "forgets" stale paths within ~20 s.
pub const DECAY_RATE: f32 = 0.115;

/// Pheromone deposited per frame in the drone's current cell. Higher
/// on Scouts so they leave strong trails. Mappers don't deposit —
/// they're consumers, not producers. Anchors don't move so depositing
/// would just pile up at their hover position.
pub const DEPOSIT_SCOUT_PER_FRAME: f32 = 5.0;
pub const DEPOSIT_MAPPER_PER_FRAME: f32 = 0.0;
pub const DEPOSIT_ANCHOR_PER_FRAME: f32 = 0.0;

/// Fraction of the drone's deposit that bleeds into each of the six
/// face-neighbour cells. Softens the trail edge so Mappers don't lose
/// the gradient on cell-boundary crossings.
pub const DEPOSIT_NEIGHBOR_FRACTION: f32 = 0.25;
