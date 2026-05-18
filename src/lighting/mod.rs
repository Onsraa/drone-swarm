use bevy::prelude::*;

const SUN_ILLUMINANCE_LUX: f32 = 10_000.0;
const SUN_POSITION: Vec3 = Vec3::new(20.0, 50.0, 20.0);

pub struct LightingPlugin;

impl Plugin for LightingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_directional_light);
    }
}

fn setup_directional_light(mut commands: Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: SUN_ILLUMINANCE_LUX,
            shadows_enabled: true,
            ..Default::default()
        },
        Transform::from_translation(SUN_POSITION).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
