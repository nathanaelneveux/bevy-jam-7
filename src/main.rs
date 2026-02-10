mod cave_noise;
mod cave_world;
mod chunk_colliders;
mod enemy_ai;
mod enemy_navigation;
mod player_controller;

use avian3d::prelude::*;
use bevy::asset::AssetMetaCheck;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};
use bevy_enhanced_input::prelude::EnhancedInputPlugin;
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};

use cave_world::CaveWorldPlugin;
use chunk_colliders::ChunkColliderPlugin;
use enemy_ai::EnemyAiPlugin;
use player_controller::PlayerControllerPlugin;

#[derive(Resource, Default)]
pub(crate) struct InspectorMode {
    pub(crate) enabled: bool,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .init_resource::<InspectorMode>()
        .add_plugins(EnhancedInputPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(EguiPlugin::default())
        .add_plugins(WorldInspectorPlugin::default().run_if(inspector_mode_active))
        .add_plugins(CaveWorldPlugin)
        .add_plugins(PlayerControllerPlugin)
        .add_plugins(ChunkColliderPlugin)
        .add_plugins(EnemyAiPlugin)
        .add_systems(Update, toggle_inspector_mode)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut cursor_options: Single<&mut CursorOptions, With<PrimaryWindow>>,
) {
    apply_cursor_mode(&mut cursor_options, false);

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

fn inspector_mode_active(inspector_mode: Res<InspectorMode>) -> bool {
    inspector_mode.enabled
}

fn toggle_inspector_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut inspector_mode: ResMut<InspectorMode>,
    mut cursor_options: Single<&mut CursorOptions, With<PrimaryWindow>>,
) {
    if keys.just_pressed(KeyCode::Escape) {
        inspector_mode.enabled = !inspector_mode.enabled;
        apply_cursor_mode(&mut cursor_options, inspector_mode.enabled);
    }
}

fn apply_cursor_mode(cursor_options: &mut CursorOptions, inspector_enabled: bool) {
    if inspector_enabled {
        cursor_options.visible = true;
        cursor_options.grab_mode = CursorGrabMode::None;
    } else {
        cursor_options.visible = false;
        cursor_options.grab_mode = CursorGrabMode::Locked;
    }
}
