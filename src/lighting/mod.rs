//! Scene lighting. Five lights total for a layered look:
//!
//! - Key directional ("sun") from above-right-front, warm tint,
//!   shadows ON. Drives the primary highlight + drop-shadow direction.
//! - Fill directional from the opposite hemisphere, cool tint, shadows
//!   OFF. Lifts the unlit faces — simulates sky bounce.
//! - Ambient light resource for the absolute floor.
//! - Two warm/cool accent point lights at world-quarter positions to
//!   add visual depth + a "search-zone" mood.
//!
//! Accent positions + ranges scale with `WorldConfig.size`, recomputed
//! whenever a map swap changes the world dimensions.

use bevy::prelude::*;

use crate::world::WorldConfig;

const KEY_ILLUMINANCE_LUX: f32 = 10_000.0;
const FILL_ILLUMINANCE_LUX: f32 = 3_500.0;
const AMBIENT_BRIGHTNESS: f32 = 0.04;
const ACCENT_INTENSITY: f32 = 8_000_000.0;

#[derive(Component)]
struct AccentLight {
    /// Fraction (0..1) along each axis of `world_size` where this
    /// accent sits.
    anchor_fraction: Vec3,
}

pub struct LightingPlugin;

impl Plugin for LightingPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(bevy::light::GlobalAmbientLight {
            color: Color::WHITE,
            brightness: AMBIENT_BRIGHTNESS,
            affects_lightmapped_meshes: false,
        })
        .add_systems(Startup, spawn_lights)
        .add_systems(Update, reposition_accents_for_world);
    }
}

fn spawn_lights(mut commands: Commands) {
    // KEY — warm sunlight from above-right-front.
    commands.spawn((
        DirectionalLight {
            illuminance: KEY_ILLUMINANCE_LUX,
            color: Color::srgb(1.0, 0.96, 0.88),
            shadows_enabled: true,
            ..Default::default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(-0.6, -0.8, -0.4), Vec3::Y),
    ));

    // FILL — cool sky-bounce from the opposite hemisphere, no shadows.
    commands.spawn((
        DirectionalLight {
            illuminance: FILL_ILLUMINANCE_LUX,
            color: Color::srgb(0.78, 0.85, 1.0),
            shadows_enabled: false,
            ..Default::default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(0.4, -0.5, 0.6), Vec3::Y),
    ));

    // Accent point lights — positioned per-frame by
    // `reposition_accents_for_world` once WorldConfig is available.
    commands.spawn((
        AccentLight {
            anchor_fraction: Vec3::new(0.25, 0.65, 0.25),
        },
        PointLight {
            color: Color::srgb(1.0, 0.78, 0.55),
            intensity: ACCENT_INTENSITY,
            range: 200.0,
            shadows_enabled: false,
            ..Default::default()
        },
        Transform::IDENTITY,
    ));
    commands.spawn((
        AccentLight {
            anchor_fraction: Vec3::new(0.75, 0.65, 0.75),
        },
        PointLight {
            color: Color::srgb(0.55, 0.78, 1.0),
            intensity: ACCENT_INTENSITY,
            range: 200.0,
            shadows_enabled: false,
            ..Default::default()
        },
        Transform::IDENTITY,
    ));
}

/// Place accent point lights at their anchor-fraction × world_size.
/// Runs every frame but is a no-op once the lights already match the
/// current world dims, so the cost is trivial.
fn reposition_accents_for_world(
    world: Option<Res<WorldConfig>>,
    mut accents: Query<(&AccentLight, &mut Transform, &mut PointLight)>,
) {
    let Some(world) = world else { return; };
    let size = world.world_size();
    if size.length_squared() <= 0.0 {
        return;
    }
    let range = (size.x.max(size.z) * 0.6).max(50.0);
    for (anchor, mut transform, mut light) in &mut accents {
        let target = anchor.anchor_fraction * size;
        if transform.translation.distance(target) > 0.5 || (light.range - range).abs() > 0.5 {
            transform.translation = target;
            light.range = range;
        }
    }
}
