use bevy::camera::primitives::MeshAabb;
use bevy::prelude::*;

use super::components::PendingCenter;

/// Once GLB meshes finish loading, compute the union AABB of all descendant
/// meshes in the entity's local frame and shift translation by -center so
/// rotations pivot on the model's geometric center.
pub fn recenter_visuals(
    mut commands: Commands,
    mut pending_q: Query<(Entity, &mut Transform), With<PendingCenter>>,
    children_q: Query<&Children>,
    descendant_transform_q: Query<&Transform, Without<PendingCenter>>,
    mesh3d_q: Query<&Mesh3d>,
    meshes: Res<Assets<Mesh>>,
) {
    for (root, mut root_transform) in &mut pending_q {
        let Some(model_center) = compute_model_center(
            root,
            &children_q,
            &descendant_transform_q,
            &mesh3d_q,
            &meshes,
        ) else {
            continue;
        };
        root_transform.translation = -model_center;
        info!(
            "recentered drone GLB: mesh center {:?} in scene-root local space",
            model_center
        );
        commands.entity(root).remove::<PendingCenter>();
    }
}

fn compute_model_center(
    root: Entity,
    children_q: &Query<&Children>,
    transform_q: &Query<&Transform, Without<PendingCenter>>,
    mesh3d_q: &Query<&Mesh3d>,
    meshes: &Assets<Mesh>,
) -> Option<Vec3> {
    let mut bounds: Option<(Vec3, Vec3)> = None;
    let mut stack: Vec<(Entity, Mat4)> = vec![(root, Mat4::IDENTITY)];

    while let Some((entity, to_root)) = stack.pop() {
        if let Ok(mesh3d) = mesh3d_q.get(entity)
            && let Some(mesh) = meshes.get(&mesh3d.0)
            && let Some(aabb) = mesh.compute_aabb()
        {
            let center = Vec3::from(aabb.center);
            let half_extents = Vec3::from(aabb.half_extents);
            for i in 0..8u32 {
                let sign = Vec3::new(
                    if i & 1 != 0 { 1.0 } else { -1.0 },
                    if i & 2 != 0 { 1.0 } else { -1.0 },
                    if i & 4 != 0 { 1.0 } else { -1.0 },
                );
                let corner = to_root.transform_point3(center + sign * half_extents);
                bounds = Some(match bounds {
                    None => (corner, corner),
                    Some((lo, hi)) => (lo.min(corner), hi.max(corner)),
                });
            }
        }

        if let Ok(children) = children_q.get(entity) {
            for &child in children {
                let local = transform_q
                    .get(child)
                    .map(|t| t.to_matrix())
                    .unwrap_or(Mat4::IDENTITY);
                stack.push((child, to_root * local));
            }
        }
    }

    bounds.map(|(lo, hi)| (lo + hi) * 0.5)
}
