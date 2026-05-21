// Reactive avoidance — terrain cube scan + peer separation.
pub const AVOID_RADIUS_M: f32 = 4.0;
/// Personal-space radius for peer drones. Bigger than terrain radius
/// + non-linear falloff under the hard "personal bubble" radius
/// (`PEER_BUBBLE_RADIUS_M`) keeps drones from physically overlapping.
pub const AVOID_RADIUS_PEER_M: f32 = 10.0;
/// Inside this radius the peer-repulsion force ramps up as the
/// inverse of distance (1 / d clamped) rather than the quadratic
/// (1 - d/R)² falloff used outside it. Effect: drones cannot
/// inter-penetrate because the force diverges at zero distance.
pub const PEER_BUBBLE_RADIUS_M: f32 = 3.0;

// Trail (gizmo viz)
/// Points retained per-drone in the trail buffer. 120 samples × 0.2 s
/// interval = ~24 s of recent travel. Visualization-only, so the
/// memory + draw cost is fine at 50 drones.
pub const TRAIL_MAX_POINTS: usize = 120;
/// Seconds between trail samples. Sampling on every Update tick wastes
/// gizmo draw budget on points that are 1 mm apart at low FPS; the
/// 0.2 s gate keeps the line smooth without flooding.
pub const TRAIL_SAMPLE_INTERVAL_SECS: f32 = 0.2;

/// Mapper steering reads a weighted two-channel gradient:
/// `α · ∇scout − β · ∇mapper`. α pulls mappers up Scout trails
/// (pro-Scout). β pushes them away from regions other mappers have
/// already detailed (anti-Mapper, prevents duplication).
pub const MAPPER_GRADIENT_ALPHA: f32 = 1.0;
pub const MAPPER_GRADIENT_BETA: f32 = 0.6;

/// Scout heading is a first-order low-pass over the fresh anti-gradient
/// direction. Each frame `dir = lerp(stored, fresh, SCOUT_EMA_ALPHA)`.
/// α = 0.25 → ~4-frame time constant, kills self-deposit oscillation
/// without making the scout slow to react to real gradient shifts.
pub const SCOUT_EMA_ALPHA: f32 = 0.25;

/// Frontier-attraction weight in role steering. The role's local
/// direction (anti-pheromone for Scout, two-channel gradient for
/// Mapper) is blended with a unit vector pointing at the assigned
/// frontier cluster centroid:
/// `dir = (frontier * w + local * (1 - w)).normalize()`. Higher = more
/// goal-directed; lower = more reactive to local field. Scouts lean
/// harder on the frontier (long-range exploration); Mappers stay
/// closer to scout trails (detail work along the way).
pub const SCOUT_FRONTIER_WEIGHT: f32 = 0.7;
pub const MAPPER_FRONTIER_WEIGHT: f32 = 0.5;
