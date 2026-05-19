use std::path::Path;

use bevy::prelude::*;

use crate::world::GroundTruthMap;
use crate::world::constants::{
    CLUSTER_A_HI, CLUSTER_A_LO, CLUSTER_B_HI, CLUSTER_B_LO, CLUSTER_C_HI, CLUSTER_C_LO,
    CLUSTER_D_HI, CLUSTER_D_LO, CLUSTER_E_HI, CLUSTER_E_LO, CLUSTER_F_HI, CLUSTER_F_LO, FLOOR_Y,
};

use super::asset::encode_dvm;

const VOXEL_SIZE: f32 = 1.0;
const DIMS: UVec3 = UVec3::new(640, 24, 640);

/// On a fresh checkout the `assets/maps/` directory may be empty. This
/// runs at startup and seeds it with four procedurally-built `.dvm`
/// files so the registry has something to load. Idempotent: skips any
/// file that already exists on disk.
pub fn bootstrap_default_maps() {
    let dir = Path::new("assets/maps");
    if let Err(e) = std::fs::create_dir_all(dir) {
        warn!("failed to create {}: {}", dir.display(), e);
        return;
    }

    write_if_missing(dir, "clusters.dvm", build_clusters());
    write_if_missing(dir, "empty.dvm", build_empty());
    write_if_missing(dir, "tight_corridor.dvm", build_tight_corridor());
    write_if_missing(dir, "tower.dvm", build_towers());
}

fn write_if_missing(dir: &Path, name: &str, map: GroundTruthMap) {
    let path = dir.join(name);
    if path.exists() {
        return;
    }
    let bitset = map.pack_bitset();
    let bytes = encode_dvm(map.dims, VOXEL_SIZE, &bitset);
    match std::fs::write(&path, &bytes) {
        Ok(()) => info!(
            "wrote {} ({} occupied cells, {} bytes)",
            path.display(),
            map.count_occupied(),
            bytes.len()
        ),
        Err(e) => warn!("failed to write {}: {}", path.display(), e),
    }
}

fn empty_world() -> GroundTruthMap {
    let mut map = GroundTruthMap::new(DIMS);
    for x in 0..DIMS.x as i32 {
        for z in 0..DIMS.z as i32 {
            map.set(IVec3::new(x, FLOOR_Y, z), true);
        }
    }
    map
}

fn build_empty() -> GroundTruthMap {
    empty_world()
}

fn build_clusters() -> GroundTruthMap {
    let mut map = empty_world();
    for (lo, hi) in [
        (CLUSTER_A_LO, CLUSTER_A_HI),
        (CLUSTER_B_LO, CLUSTER_B_HI),
        (CLUSTER_C_LO, CLUSTER_C_HI),
        (CLUSTER_D_LO, CLUSTER_D_HI),
        (CLUSTER_E_LO, CLUSTER_E_HI),
        (CLUSTER_F_LO, CLUSTER_F_HI),
    ] {
        fill_box(&mut map, lo, hi);
    }
    map
}

fn build_tight_corridor() -> GroundTruthMap {
    let mut map = empty_world();
    let z_a = 280;
    let z_b = 360;
    for x in 40..600 {
        for y in 1..18 {
            map.set(IVec3::new(x, y, z_a), true);
            map.set(IVec3::new(x, y, z_b), true);
        }
    }
    map
}

fn build_towers() -> GroundTruthMap {
    let mut map = empty_world();
    let towers = [
        (IVec3::new(120, 1, 120), IVec3::new(140, 22, 140)),
        (IVec3::new(280, 1, 200), IVec3::new(300, 22, 220)),
        (IVec3::new(440, 1, 320), IVec3::new(460, 22, 340)),
        (IVec3::new(180, 1, 440), IVec3::new(200, 22, 460)),
        (IVec3::new(380, 1, 500), IVec3::new(400, 22, 520)),
        (IVec3::new(520, 1, 100), IVec3::new(540, 22, 120)),
    ];
    for (lo, hi) in towers {
        fill_box(&mut map, lo, hi);
    }
    map
}

fn fill_box(map: &mut GroundTruthMap, lo: IVec3, hi: IVec3) {
    for x in lo.x..hi.x {
        for y in lo.y..hi.y {
            for z in lo.z..hi.z {
                map.set(IVec3::new(x, y, z), true);
            }
        }
    }
}
