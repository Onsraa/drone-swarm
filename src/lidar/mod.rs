mod components;
mod constants;
mod resources;
mod sampling;
mod scan;
mod traversal;

use bevy::prelude::*;

pub use components::LastScanRays;

use constants::SCAN_INTERVAL_SECS;
use resources::ScanTimer;
use sampling::LidarRayDirs;
use scan::lidar_scan;

pub struct LidarPlugin;

impl Plugin for LidarPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ScanTimer(Timer::from_seconds(
            SCAN_INTERVAL_SECS,
            TimerMode::Repeating,
        )))
        .insert_resource(LidarRayDirs::default_for_scan())
        .add_systems(Update, lidar_scan);
    }
}
