use bevy::prelude::*;
use bevy::render::extract_resource::ExtractResource;
use bevy::render::render_resource::{BufferUsages, ShaderType};
use bevy::render::storage::ShaderStorageBuffer;

use crate::world::GroundTruthMap;

use super::super::constants::RAYS_PER_SCAN;
use super::super::sampling::{build_role_ray_buffer, RoleConeRange};
use super::per_drone_scan::{allocate_buffer as alloc_scan_params, DroneScanParamsBuffer};

#[derive(Resource, Clone, Copy, Debug)]
pub struct RoleConeRanges(pub [RoleConeRange; 3]);

pub const MAX_STEPS_PER_RAY: u32 = 96;
pub const MAX_DRONES_GPU: u32 = 50;

/// Soft cap on points the lidar point buffer can hold. The shader
/// soft-truncates writes past this slot via `slot >= max_points`.
/// Sized for the sticky-spray mode: 2 M points × 32 bytes per entry
/// (pos + color) = 64 MB. Live (non-sticky) mode only writes
/// drone_count * rays_per_scan per frame so the cap is overkill there.
pub const MAX_LIDAR_POINTS: u32 = 2_000_000;

/// Cap on cells each drone may have in its active-Occupied list. The
/// list is append-only across frames; the lidar shader appends a cell
/// flat-index the first time it flips that cell's Occupied bit. Build
/// passes iterate the list instead of every cell × every drone — 50×
/// fewer GPU thread invocations at 50 drones × 9.83 M cells. 200 K
/// cells × 4 B × 50 drones = 40 MB GPU.
pub const MAX_LOCAL_ACTIVE_PER_DRONE: u32 = 200_000;

/// Cap on cells the global active list holds. One entry per cell
/// where SOMEONE has flipped the global Occupied bit. 500 K × 4 B =
/// 2 MB GPU.
pub const MAX_GLOBAL_ACTIVE: u32 = 500_000;

/// Stage 9B output buffer capacity: max number of Occupied-cell instances
/// the build pass can emit across all drones in a single dispatch. At
/// 640×24×640 with 50 drones, steady-state local-map coverage can hit
/// millions of cells in aggregate; 2M slots is a soft cap that gracefully
/// truncates visual output via the shader's `slot >= max_instances` check.
pub const MAX_LOCAL_INSTANCES: u32 = 2_000_000;

/// Stage 9Eb output capacity. The merged central map has at most one
/// instance per cell. At 640×24×640 the world holds ~9.8M cells; in
/// practice only a few hundred thousand are Occupied at any time (floor
/// + clusters). 1M slots is generous headroom.
pub const MAX_GLOBAL_INSTANCES: u32 = 1_000_000;

/// Per-drone local-map occupancy is packed as 2 bits per cell into a
/// `u32` storage buffer: bit 0 = Free flag, bit 1 = Occupied flag. Both
/// flags are sticky under `atomicOr`. `Unknown` is the all-zero default.
pub fn occupancy_words_per_drone(dims: UVec3) -> usize {
    let cells = (dims.x * dims.y * dims.z) as usize;
    cells.div_ceil(16)
}

/// Mirrors the WGSL `LidarParams` struct. Stage 3 uses a stub drone count
/// of 1 to validate the traversal shader in isolation; Stage 4 replaces
/// these values with per-tick uploads driven by the real drone query.
/// `voxel_size` is needed by the lidar shader so it can convert
/// cell-space hit positions back into world-space points for the
/// point-cloud render channel.
#[derive(ShaderType, Clone, Copy, Debug)]
pub struct LidarParams {
    pub dims: UVec4,
    pub max_steps: u32,
    pub rays_per_scan: u32,
    pub drone_count: u32,
    pub voxel_size: f32,
    pub drone_mask_lo: u32,
    pub drone_mask_hi: u32,
    pub max_points: u32,
    pub connected_mask_lo: u32,
    pub connected_mask_hi: u32,
    pub _pad0: u32,
    pub _pad1: u32,
}

/// Mirrors the WGSL `BuildParams` struct used by `build_local_instances`,
/// `merge_global`, and `build_global_instances`. All three shaders bind
/// the same buffer; the mask fields are only consumed by
/// `build_local_instances` (and any future point-cloud shader). The
/// trailing `_pad` keeps the struct at a 16-byte boundary in WGSL.
#[derive(ShaderType, Clone, Copy, Debug)]
pub struct BuildLocalParams {
    pub dims: UVec4,
    pub drone_count: u32,
    pub voxel_size: f32,
    pub scale_factor: f32,
    pub max_instances: u32,
    pub drone_mask_lo: u32,
    pub drone_mask_hi: u32,
    pub connected_mask_lo: u32,
    pub connected_mask_hi: u32,
}

#[derive(Resource, ExtractResource, Clone)]
pub struct GroundTruthBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LidarParamsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct DronePositionsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct DroneOrientationsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct RayDirsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalOccupancyBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalOccupancyBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct BuildLocalParamsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct DroneColorsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalInstanceCountBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalInstanceVecBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalInstanceCountBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalInstanceVecBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LidarPointCountBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LidarPointVecBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalActiveCellsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct LocalActiveCountBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalActiveCellsBuffer(pub Handle<ShaderStorageBuffer>);

#[derive(Resource, ExtractResource, Clone)]
pub struct GlobalActiveCountBuffer(pub Handle<ShaderStorageBuffer>);

/// One indirect dispatch args block per build pass. Layout is the wgpu
/// `DispatchIndirectArgs` triple (x, y, z) plus a u32 pad to keep it
/// 16-byte aligned. Slot 0 = build_local, slot 1 = build_global. The
/// `prepare_build_indirect` compute pass writes `x =
/// ceil(max(active_count) / 256)` into each slot every frame; both
/// build passes then `dispatch_workgroups_indirect` from this same
/// buffer. Total size 32 bytes.
#[derive(Resource, ExtractResource, Clone)]
pub struct BuildIndirectBuffer(pub Handle<ShaderStorageBuffer>);

/// CWBVH8 node table. 20 × u32 per node (80 bytes), bytemuck-cast
/// directly from `obvhs::cwbvh::node::CwBvhNode`. Allocated empty at
/// startup; `upload_bvh_buffers` fills it when `WorldBvh` is built.
#[derive(Resource, ExtractResource, Clone)]
pub struct BvhNodesBuffer(pub Handle<ShaderStorageBuffer>);

/// Primitive index table from `CwBvh.primitive_indices`. `array<u32>`.
/// Leaf nodes reference primitives through this indirection.
#[derive(Resource, ExtractResource, Clone)]
pub struct BvhPrimitiveIndicesBuffer(pub Handle<ShaderStorageBuffer>);

/// Triangle vertex positions, unindexed: 3 × `vec4<f32>` per triangle.
/// `xyz` is the world-space vertex, `w` is padding for 16-byte align.
#[derive(Resource, ExtractResource, Clone)]
pub struct BvhTriangleVerticesBuffer(pub Handle<ShaderStorageBuffer>);

/// One-shot startup: packs the CPU ground truth and allocates every
/// lidar input/output buffer. Positions and params start zeroed; the
/// per-frame `upload_drone_positions` system fills them with real data.
pub fn setup_gpu_lidar_assets(
    mut commands: Commands,
    mut buffers: ResMut<Assets<ShaderStorageBuffer>>,
    ground: Res<GroundTruthMap>,
) {
    let bitset = ground.pack_bitset();
    let mut ground_buf = ShaderStorageBuffer::from(bitset);
    ground_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let ground_handle = buffers.add(ground_buf);

    let params = LidarParams {
        dims: UVec4::new(ground.dims.x, ground.dims.y, ground.dims.z, 0),
        max_steps: MAX_STEPS_PER_RAY,
        rays_per_scan: RAYS_PER_SCAN as u32,
        drone_count: 0,
        voxel_size: 1.0,
        drone_mask_lo: u32::MAX,
        drone_mask_hi: u32::MAX,
        max_points: MAX_LIDAR_POINTS,
        connected_mask_lo: u32::MAX,
        connected_mask_hi: u32::MAX,
        _pad0: 0,
        _pad1: 0,
    };
    let mut params_buf = ShaderStorageBuffer::from(params);
    params_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let params_handle = buffers.add(params_buf);

    let drone_positions: Vec<Vec4> = vec![Vec4::ZERO; MAX_DRONES_GPU as usize];
    let mut positions_buf = ShaderStorageBuffer::from(drone_positions);
    positions_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let positions_handle = buffers.add(positions_buf);

    // Identity quaternion = (0, 0, 0, 1).
    let drone_orientations: Vec<Vec4> =
        vec![Vec4::new(0.0, 0.0, 0.0, 1.0); MAX_DRONES_GPU as usize];
    let mut orientations_buf = ShaderStorageBuffer::from(drone_orientations);
    orientations_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let orientations_handle = buffers.add(orientations_buf);

    // Build the role-specific concatenated ray buffer.
    let (ray_dirs_vec, role_ranges) = build_role_ray_buffer();
    let max_ray_slots = crate::lidar::constants::MAX_RAYS_PER_SCAN as usize;
    let mut ray_dirs: Vec<Vec4> = vec![Vec4::ZERO; max_ray_slots.max(ray_dirs_vec.len())];
    for (i, d) in ray_dirs_vec.iter().enumerate() {
        if i >= ray_dirs.len() {
            break;
        }
        ray_dirs[i] = Vec4::new(d.x, d.y, d.z, 0.0);
    }
    let mut dirs_buf = ShaderStorageBuffer::from(ray_dirs);
    dirs_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let dirs_handle = buffers.add(dirs_buf);

    let words_per_drone = occupancy_words_per_drone(ground.dims);
    let occupancy_len = (MAX_DRONES_GPU as usize) * words_per_drone;
    let mut occupancy_buf = ShaderStorageBuffer::from(vec![0u32; occupancy_len]);
    occupancy_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let occupancy_handle = buffers.add(occupancy_buf);

    let mut global_occupancy_buf = ShaderStorageBuffer::from(vec![0u32; words_per_drone]);
    global_occupancy_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let global_occupancy_handle = buffers.add(global_occupancy_buf);

    let build_params = BuildLocalParams {
        dims: UVec4::new(ground.dims.x, ground.dims.y, ground.dims.z, 0),
        drone_count: 0,
        voxel_size: 1.0,
        scale_factor: 1.0,
        max_instances: MAX_LOCAL_INSTANCES,
        drone_mask_lo: u32::MAX,
        drone_mask_hi: u32::MAX,
        connected_mask_lo: u32::MAX,
        connected_mask_hi: u32::MAX,
    };
    let mut build_params_buf = ShaderStorageBuffer::from(build_params);
    build_params_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let build_params_handle = buffers.add(build_params_buf);

    let drone_colors: Vec<Vec4> = vec![Vec4::ZERO; MAX_DRONES_GPU as usize];
    let mut drone_colors_buf = ShaderStorageBuffer::from(drone_colors);
    drone_colors_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let drone_colors_handle = buffers.add(drone_colors_buf);

    let mut count_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    count_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let count_handle = buffers.add(count_buf);

    // Two vec4 per instance (pos_scale + color). Also flag for vertex use
    // so Stage 9C's render pipeline can bind the same buffer.
    let instance_vec_len = (MAX_LOCAL_INSTANCES as usize) * 2;
    let mut instance_vec_buf = ShaderStorageBuffer::from(vec![Vec4::ZERO; instance_vec_len]);
    instance_vec_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST | BufferUsages::VERTEX;
    let instance_vec_handle = buffers.add(instance_vec_buf);

    let mut global_count_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    global_count_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let global_count_handle = buffers.add(global_count_buf);

    let global_instance_vec_len = (MAX_GLOBAL_INSTANCES as usize) * 2;
    let mut global_instance_vec_buf =
        ShaderStorageBuffer::from(vec![Vec4::ZERO; global_instance_vec_len]);
    global_instance_vec_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST | BufferUsages::VERTEX;
    let global_instance_vec_handle = buffers.add(global_instance_vec_buf);

    let mut point_count_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    point_count_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let point_count_handle = buffers.add(point_count_buf);

    let point_vec_len = (MAX_LIDAR_POINTS as usize) * 2;
    let mut point_vec_buf = ShaderStorageBuffer::from(vec![Vec4::ZERO; point_vec_len]);
    point_vec_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST | BufferUsages::VERTEX;
    let point_vec_handle = buffers.add(point_vec_buf);

    let scan_params_handle = alloc_scan_params(&mut buffers);

    let local_active_len = (MAX_DRONES_GPU as usize) * (MAX_LOCAL_ACTIVE_PER_DRONE as usize);
    let mut local_active_buf = ShaderStorageBuffer::from(vec![0u32; local_active_len]);
    local_active_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let local_active_handle = buffers.add(local_active_buf);

    let mut local_active_count_buf =
        ShaderStorageBuffer::from(vec![0u32; MAX_DRONES_GPU as usize]);
    local_active_count_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let local_active_count_handle = buffers.add(local_active_count_buf);

    let mut global_active_buf =
        ShaderStorageBuffer::from(vec![0u32; MAX_GLOBAL_ACTIVE as usize]);
    global_active_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let global_active_handle = buffers.add(global_active_buf);

    let mut global_active_count_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    global_active_count_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let global_active_count_handle = buffers.add(global_active_count_buf);

    // 2 slots × (x, y, z, _pad) u32 = 8 u32 = 32 bytes. Slot 0 =
    // build_local args, slot 1 = build_global args. INDIRECT usage
    // so both build passes can dispatch_workgroups_indirect from it.
    let mut build_indirect_buf = ShaderStorageBuffer::from(vec![0u32; 8]);
    build_indirect_buf.buffer_description.usage |=
        BufferUsages::COPY_SRC | BufferUsages::COPY_DST | BufferUsages::INDIRECT;
    let build_indirect_handle = buffers.add(build_indirect_buf);

    // BVH SSBOs — allocated empty (1-word fallback) so the bind group
    // can build before the scene mesh is loaded + BVH built. The
    // `upload_bvh_buffers` system replaces the contents via set_data
    // once `WorldBvh` is inserted.
    let mut bvh_nodes_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    bvh_nodes_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let bvh_nodes_handle = buffers.add(bvh_nodes_buf);

    let mut bvh_prim_idx_buf = ShaderStorageBuffer::from(vec![0u32; 1]);
    bvh_prim_idx_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let bvh_prim_idx_handle = buffers.add(bvh_prim_idx_buf);

    let mut bvh_verts_buf = ShaderStorageBuffer::from(vec![Vec4::ZERO]);
    bvh_verts_buf.buffer_description.usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
    let bvh_verts_handle = buffers.add(bvh_verts_buf);

    info!(
        "GPU lidar buffers allocated: {} drone slots, {} rays/scan, {} steps/ray, {} occupancy u32s ({} words/drone)",
        MAX_DRONES_GPU,
        params.rays_per_scan,
        params.max_steps,
        occupancy_len,
        words_per_drone,
    );

    commands.insert_resource(GroundTruthBuffer(ground_handle));
    commands.insert_resource(LidarParamsBuffer(params_handle));
    commands.insert_resource(DronePositionsBuffer(positions_handle));
    commands.insert_resource(DroneOrientationsBuffer(orientations_handle));
    commands.insert_resource(RayDirsBuffer(dirs_handle));
    commands.insert_resource(LocalOccupancyBuffer(occupancy_handle));
    commands.insert_resource(GlobalOccupancyBuffer(global_occupancy_handle));
    commands.insert_resource(BuildLocalParamsBuffer(build_params_handle));
    commands.insert_resource(DroneColorsBuffer(drone_colors_handle));
    commands.insert_resource(LocalInstanceCountBuffer(count_handle));
    commands.insert_resource(LocalInstanceVecBuffer(instance_vec_handle));
    commands.insert_resource(GlobalInstanceCountBuffer(global_count_handle));
    commands.insert_resource(GlobalInstanceVecBuffer(global_instance_vec_handle));
    commands.insert_resource(LidarPointCountBuffer(point_count_handle));
    commands.insert_resource(LidarPointVecBuffer(point_vec_handle));
    commands.insert_resource(DroneScanParamsBuffer(scan_params_handle));
    commands.insert_resource(LocalActiveCellsBuffer(local_active_handle));
    commands.insert_resource(LocalActiveCountBuffer(local_active_count_handle));
    commands.insert_resource(GlobalActiveCellsBuffer(global_active_handle));
    commands.insert_resource(GlobalActiveCountBuffer(global_active_count_handle));
    commands.insert_resource(BuildIndirectBuffer(build_indirect_handle));
    commands.insert_resource(BvhNodesBuffer(bvh_nodes_handle));
    commands.insert_resource(BvhPrimitiveIndicesBuffer(bvh_prim_idx_handle));
    commands.insert_resource(BvhTriangleVerticesBuffer(bvh_verts_handle));
    commands.insert_resource(RoleConeRanges(role_ranges));
}
