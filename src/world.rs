use bevy::prelude::*;

#[derive(Resource, Clone)]
pub struct WorldConfig {
    pub size: UVec3,
    pub voxel_size: f32,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            // Bevy Y-up: X = width, Y = height, Z = depth.
            // Ground footprint 32 x 32 (X by Z), vertical height 16 (Y).
            size: UVec3::new(32, 16, 32),
            voxel_size: 1.0,
        }
    }
}

impl WorldConfig {
    pub fn world_size(&self) -> Vec3 {
        self.size.as_vec3() * self.voxel_size
    }

    pub fn center(&self) -> Vec3 {
        self.world_size() * 0.5
    }
}

#[derive(Resource)]
pub struct GroundTruthMap {
    pub dims: UVec3,
    cells: Vec<bool>,
}

impl GroundTruthMap {
    pub fn new(dims: UVec3) -> Self {
        let n = (dims.x * dims.y * dims.z) as usize;
        Self {
            dims,
            cells: vec![false; n],
        }
    }

    pub fn idx(&self, p: IVec3) -> Option<usize> {
        if p.x < 0 || p.y < 0 || p.z < 0 {
            return None;
        }
        let (x, y, z) = (p.x as u32, p.y as u32, p.z as u32);
        if x >= self.dims.x || y >= self.dims.y || z >= self.dims.z {
            return None;
        }
        Some((x + y * self.dims.x + z * self.dims.x * self.dims.y) as usize)
    }

    pub fn get(&self, p: IVec3) -> bool {
        self.idx(p).map(|i| self.cells[i]).unwrap_or(false)
    }

    pub fn set(&mut self, p: IVec3, v: bool) {
        if let Some(i) = self.idx(p) {
            self.cells[i] = v;
        }
    }

    pub fn iter_occupied(&self) -> impl Iterator<Item = IVec3> + '_ {
        let dx = self.dims.x as i32;
        let dy = self.dims.y as i32;
        let dz = self.dims.z as i32;
        (0..dz)
            .flat_map(move |z| (0..dy).flat_map(move |y| (0..dx).map(move |x| IVec3::new(x, y, z))))
            .filter(move |p| self.get(*p))
    }
}

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(WorldConfig::default())
            .add_systems(Startup, build_test_scene);
    }
}

pub fn build_test_scene(mut commands: Commands, config: Res<WorldConfig>) {
    let mut map = GroundTruthMap::new(config.size);

    // Floor at y=0 across full XZ footprint.
    for x in 0..config.size.x as i32 {
        for z in 0..config.size.z as i32 {
            map.set(IVec3::new(x, 0, z), true);
        }
    }

    // Cluster A: low box near front-left.
    fill_box(&mut map, IVec3::new(6, 1, 6), IVec3::new(10, 6, 10));
    // Cluster B: tall pillar at back-right.
    fill_box(&mut map, IVec3::new(22, 1, 22), IVec3::new(26, 12, 26));
    // Cluster C: short wall.
    fill_box(&mut map, IVec3::new(16, 1, 14), IVec3::new(22, 4, 18));

    let occupied = map.cells.iter().filter(|c| **c).count();
    info!("ground truth: {} occupied cells", occupied);
    commands.insert_resource(map);
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
