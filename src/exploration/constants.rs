// Frontier scan + clustering
pub const FRONTIER_REACHED_DIST: f32 = 6.0;
/// Cap on candidate cells the time-sliced frontier scan collects in a
/// single sweep. Smaller cap = cheaper `build_clusters` finalize spike
/// (the per-cell flood-fill scales linearly). 5 K cells is plenty for
/// the assignment + crowding signal at typical exploration densities.
pub const MAX_FRONTIER_CANDIDATES: usize = 5_000;
pub const MIN_CLUSTER_SIZE: usize = 4;

// Planner
pub const PLANNER_DOWNSAMPLE: u32 = 8;
pub const PLANNER_FREE_COST: f32 = 1.0;
pub const PLANNER_UNKNOWN_COST_MULT: f32 = 3.0;
pub const PLANNER_DEEP_UNKNOWN_MULT: f32 = 5.0;

// Steering
pub const PATH_FOLLOW_LERP_RATE: f32 = 3.0;
pub const AVOID_RADIUS_M: f32 = 4.0;
pub const AVOID_RADIUS_PEER_M: f32 = 6.0;

// Stuck detection
pub const STUCK_VEL_MPS: f32 = 0.5;
pub const STUCK_SECS: f32 = 3.0;
pub const STUCK_ESCALATION_WINDOW_SECS: f32 = 20.0;

// Scoring (role-agnostic Phase 1 defaults; per-role weights live in RoleParams)
pub const SCORE_INFO_WEIGHT: f32 = 1.0;
pub const SCORE_DISTANCE_WEIGHT: f32 = 1.0;
pub const SCORE_DISTANCE_BIAS: f32 = 1.0;
pub const SCORE_CROWDING_WEIGHT: f32 = 1.0;
pub const SCORE_UPGRADE_RATIO: f32 = 1.5;

// Trail (gizmo viz)
/// Points retained per-drone in the trail buffer. 120 samples × 0.2 s
/// interval = ~24 s of recent travel. Visualization-only, so the
/// memory + draw cost is fine at 50 drones.
pub const TRAIL_MAX_POINTS: usize = 120;
/// Seconds between trail samples. Sampling on every Update tick wastes
/// gizmo draw budget on points that are 1 mm apart at low FPS; the
/// 0.2 s gate keeps the line smooth without flooding.
pub const TRAIL_SAMPLE_INTERVAL_SECS: f32 = 0.2;
