mod cave_noise;
mod cave_world;
mod chunk_colliders;
mod player_controller;

use avian3d::prelude::*;
use bevy::asset::AssetMetaCheck;
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};

use cave_world::CaveWorldPlugin;
use chunk_colliders::ChunkColliderPlugin;
use player_controller::PlayerControllerPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(EguiPlugin::default())
        .add_plugins(
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        )
        .add_plugins(CaveWorldPlugin)
        .add_plugins(PlayerControllerPlugin)
        .add_plugins(ChunkColliderPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        DirectionalLight {
            illuminance: 4_500.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.0, 0.0, 0.0).looking_at(Vec3::new(0.3, -1.0, 0.2), Vec3::Y),
    ));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.62, 0.68, 0.78),
        brightness: 260.0,
        affects_lightmapped_meshes: true,
    });
}
