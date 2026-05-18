use bevy::prelude::*;

use super::config::WorldConfig;
use super::constants::{
    CLUSTER_A_HI, CLUSTER_A_LO, CLUSTER_B_HI, CLUSTER_B_LO, CLUSTER_C_HI, CLUSTER_C_LO,
    CLUSTER_D_HI, CLUSTER_D_LO, CLUSTER_E_HI, CLUSTER_E_LO, CLUSTER_F_HI, CLUSTER_F_LO, FLOOR_Y,
};
use super::ground_truth::GroundTruthMap;

pub fn build_test_scene(mut commands: Commands, config: Res<WorldConfig>) {
    let mut map = GroundTruthMap::new(config.size);

    fill_floor(&mut map, config.size);
    fill_box(&mut map, CLUSTER_A_LO, CLUSTER_A_HI);
    fill_box(&mut map, CLUSTER_B_LO, CLUSTER_B_HI);
    fill_box(&mut map, CLUSTER_C_LO, CLUSTER_C_HI);
    fill_box(&mut map, CLUSTER_D_LO, CLUSTER_D_HI);
    fill_box(&mut map, CLUSTER_E_LO, CLUSTER_E_HI);
    fill_box(&mut map, CLUSTER_F_LO, CLUSTER_F_HI);

    info!("ground truth: {} occupied cells", map.count_occupied());
    commands.insert_resource(map);
}

fn fill_floor(map: &mut GroundTruthMap, dims: UVec3) {
    for x in 0..dims.x as i32 {
        for z in 0..dims.z as i32 {
            map.set(IVec3::new(x, FLOOR_Y, z), true);
        }
    }
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
