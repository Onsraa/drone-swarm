pub const RAYS_PER_SCAN: usize = 64;

/// Half-angle of the forward-facing lidar cone. 30 deg gives a 60 deg FOV
/// fan along the drone's heading. Rays inside the cap are distributed via
/// a fibonacci spiral so 64 of them spread evenly without clumping.
pub const LIDAR_CONE_HALF_ANGLE_DEGREES: f32 = 30.0;
