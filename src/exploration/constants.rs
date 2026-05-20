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
