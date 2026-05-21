mod bvh;
mod components;
mod constants;
mod resources;
mod systems;
mod triangles;

use bevy::prelude::*;

pub use bvh::{ground_altitude, WorldBvh};
pub use resources::MeshGroundTruthConfig;

pub struct MeshGroundTruthPlugin;

impl Plugin for MeshGroundTruthPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MeshGroundTruthConfig>().add_systems(
            Update,
            (
                // Ordering matters: invalidate first so a same-frame
                // Apply tears the scene down before spawn re-fires.
                systems::invalidate_mesh_on_apply,
                systems::spawn_mesh_ground_truth,
                systems::apply_mesh_visibility,
                systems::build_bvh_when_scene_ready,
            )
                .chain(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::components::GroundTruthMesh;

    #[test]
    fn plugin_inserts_config_resource() {
        let mut app = App::new();
        app.add_plugins(MeshGroundTruthPlugin);
        app.update();
        assert!(
            app.world().contains_resource::<MeshGroundTruthConfig>(),
            "MeshGroundTruthPlugin must insert MeshGroundTruthConfig resource"
        );
    }

    #[test]
    fn config_visible_false_hides_marked_entity() {
        let mut app = App::new();
        app.add_plugins(MeshGroundTruthPlugin);
        let entity = app
            .world_mut()
            .spawn((GroundTruthMesh, Visibility::Visible))
            .id();
        app.world_mut()
            .resource_mut::<MeshGroundTruthConfig>()
            .visible = false;
        app.update();
        let v = app
            .world()
            .entity(entity)
            .get::<Visibility>()
            .expect("entity must have Visibility");
        assert_eq!(
            *v,
            Visibility::Hidden,
            "config.visible=false must hide GroundTruthMesh entities"
        );
    }

    #[test]
    fn config_visible_true_shows_marked_entity() {
        let mut app = App::new();
        app.add_plugins(MeshGroundTruthPlugin);
        let entity = app
            .world_mut()
            .spawn((GroundTruthMesh, Visibility::Hidden))
            .id();
        // Default config.visible = true.
        app.update();
        let v = app
            .world()
            .entity(entity)
            .get::<Visibility>()
            .expect("entity must have Visibility");
        assert_eq!(
            *v,
            Visibility::Visible,
            "config.visible=true must show GroundTruthMesh entities"
        );
    }
}
